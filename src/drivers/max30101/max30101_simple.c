#include "max30101_simple.h"

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "relife_protocol.h"

LOG_MODULE_REGISTER(max30101_simple, LOG_LEVEL_INF);

#define MAX30101_ADDR        0x57
#define MAX_REG_INT_STATUS1  0x00
#define MAX_REG_INT_ENABLE1  0x02
#define MAX_REG_FIFO_WR_PTR  0x04
#define MAX_REG_FIFO_OVF     0x05
#define MAX_REG_FIFO_RD_PTR  0x06
#define MAX_REG_FIFO_DATA    0x07
#define MAX_REG_FIFO_CFG     0x08
#define MAX_REG_MODE_CFG     0x09
#define MAX_REG_SPO2_CFG     0x0A
#define MAX_REG_LED1_PA      0x0C
#define MAX_REG_LED2_PA      0x0D
#define MAX_REG_LED3_PA      0x0E
#define MAX_REG_LED4_PA      0x0F
#define MAX_REG_MULTI_LED1   0x11
#define MAX_REG_MULTI_LED2   0x12
#define MAX_REG_PART_ID      0xFF

#define AVG_SIZE             4
#define SAMPLE_RATE_HZ       100
#define MIN_IBI_MS           200
#define MAX_IBI_MS           3000

struct max_algorithm_state {
    int32_t avg_buf[AVG_SIZE];
    int avg_idx;
    int32_t prev_sample;
    int32_t prev_prev_sample;
    int64_t last_beat_time;
    int32_t last_ibi_ms;
    int32_t peak_threshold;
    int32_t noise_level;
    int32_t signal_level;
    int32_t dc_est;
    int32_t hr_smoothed_x10;
    int32_t dc_red;
    int32_t dc_ir;
    int32_t red_max;
    int32_t red_min;
    int32_t ir_max;
    int32_t ir_min;
    int beat_count_spo2;
    uint8_t latest_hr;
    uint8_t latest_spo2;
    bool hr_valid;
    bool spo2_valid;
};

static const struct device *s_i2c;
static bool s_ready;

static int reg_write(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };

    return i2c_write(s_i2c, buf, sizeof(buf), MAX30101_ADDR);
}

static int reg_read(uint8_t reg, uint8_t *value)
{
    return i2c_write_read(s_i2c, MAX30101_ADDR, &reg, 1, value, 1);
}

static int fifo_read(uint8_t *buf, size_t len)
{
    uint8_t reg = MAX_REG_FIFO_DATA;

    return i2c_write_read(s_i2c, MAX30101_ADDR, &reg, 1, buf, len);
}

static uint8_t available_samples(void)
{
    uint8_t wr = 0;
    uint8_t rd = 0;

    reg_read(MAX_REG_FIFO_WR_PTR, &wr);
    reg_read(MAX_REG_FIFO_RD_PTR, &rd);
    return (wr - rd) & 0x1FU;
}

static void algorithm_reset(struct max_algorithm_state *state)
{
    memset(state, 0, sizeof(*state));
    state->red_max = INT32_MIN;
    state->red_min = INT32_MAX;
    state->ir_max = INT32_MIN;
    state->ir_min = INT32_MAX;
    state->peak_threshold = 500;
}

static bool process_beat_detection(struct max_algorithm_state *state, int32_t sample, int32_t *hr_out)
{
    const int64_t now = k_uptime_get();
    bool beat_detected = false;

    if ((state->prev_sample > state->prev_prev_sample) &&
        (state->prev_sample > sample) &&
        (state->prev_sample > state->peak_threshold)) {
        if (state->last_beat_time == 0) {
            state->last_beat_time = now;
            goto update_levels;
        }

        const int64_t ibi = now - state->last_beat_time;
        if (ibi < MIN_IBI_MS) {
            goto update_levels;
        }

        if (ibi > MAX_IBI_MS) {
            state->last_beat_time = now;
            state->last_ibi_ms = 0;
            goto update_levels;
        }

        state->last_ibi_ms = (int32_t)ibi;
        state->last_beat_time = now;
        *hr_out = 60000 / state->last_ibi_ms;
        beat_detected = true;
    }

update_levels:
    state->noise_level = (state->noise_level * 7 + abs(sample)) / 8;
    if (beat_detected) {
        state->signal_level = (state->signal_level * 7 + state->prev_sample) / 8;
    } else {
        state->peak_threshold = (state->peak_threshold * 31) / 32;
    }
    state->peak_threshold = state->noise_level + (state->signal_level - state->noise_level) / 2;
    state->prev_prev_sample = state->prev_sample;
    state->prev_sample = sample;

    return beat_detected;
}

static void process_spo2_slot(struct max_algorithm_state *state, uint32_t red, uint32_t ir)
{
    const int32_t red_i = (int32_t)red;
    const int32_t ir_i = (int32_t)ir;
    int32_t ac_red;
    int32_t ac_ir;

    state->dc_red = (state->dc_red * 63 + red_i) / 64;
    state->dc_ir = (state->dc_ir * 63 + ir_i) / 64;

    ac_red = red_i - state->dc_red;
    ac_ir = ir_i - state->dc_ir;

    if (ac_red > state->red_max) {
        state->red_max = ac_red;
    }
    if (ac_red < state->red_min) {
        state->red_min = ac_red;
    }
    if (ac_ir > state->ir_max) {
        state->ir_max = ac_ir;
    }
    if (ac_ir < state->ir_min) {
        state->ir_min = ac_ir;
    }
}

static void process_ppg_slot(struct max_algorithm_state *state, uint32_t slot)
{
    int32_t hr = 0;
    int32_t sum = 0;
    int32_t smoothed;
    int32_t ac;

    state->avg_buf[state->avg_idx] = (int32_t)slot;
    state->avg_idx = (state->avg_idx + 1) % AVG_SIZE;
    for (int i = 0; i < AVG_SIZE; i++) {
        sum += state->avg_buf[i];
    }
    smoothed = sum / AVG_SIZE;

    state->dc_est = (state->dc_est * 31 + smoothed) / 32;
    ac = smoothed - state->dc_est;

    if (process_beat_detection(state, -ac, &hr)) {
        const int32_t hr_x10 = hr * 10;

        if (state->hr_smoothed_x10 == 0) {
            state->hr_smoothed_x10 = hr_x10;
        } else {
            state->hr_smoothed_x10 = (2 * hr_x10 + 8 * state->hr_smoothed_x10) / 10;
        }

        state->latest_hr = (uint8_t)CLAMP(state->hr_smoothed_x10 / 10, 30, 220);
        state->hr_valid = true;
        state->beat_count_spo2++;

        if (state->beat_count_spo2 >= 8) {
            const int32_t ac_red = state->red_max - state->red_min;
            const int32_t ac_ir = state->ir_max - state->ir_min;

            state->red_max = INT32_MIN;
            state->red_min = INT32_MAX;
            state->ir_max = INT32_MIN;
            state->ir_min = INT32_MAX;
            state->beat_count_spo2 = 0;

            if ((ac_red > 0) && (ac_ir > 0) && (state->dc_red > 0) && (state->dc_ir > 0)) {
                const int32_t ratio_x1000 =
                    (int32_t)(((int64_t)ac_red * state->dc_ir * 1000LL) / ((int64_t)ac_ir * state->dc_red));
                int32_t spo2 = 102 - (15 * ratio_x1000) / 1000;

                spo2 = CLAMP(spo2, 70, 100);
                state->latest_spo2 = (uint8_t)spo2;
                state->spo2_valid = true;
            }
        }
    }
}

static void process_fifo_samples(struct max_algorithm_state *state)
{
    const uint8_t sample_count = available_samples();

    for (uint8_t i = 0; i < sample_count; i++) {
        uint8_t buf[12];
        uint32_t slot1;
        uint32_t slot2;
        uint32_t slot3;
        uint32_t slot4;

        if (fifo_read(buf, sizeof(buf))) {
            return;
        }

        slot1 = (((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | (uint32_t)buf[2]) & 0x3FFFFU;
        slot2 = (((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 8) | (uint32_t)buf[5]) & 0x3FFFFU;
        slot3 = (((uint32_t)buf[6] << 16) | ((uint32_t)buf[7] << 8) | (uint32_t)buf[8]) & 0x3FFFFU;
        slot4 = (((uint32_t)buf[9] << 16) | ((uint32_t)buf[10] << 8) | (uint32_t)buf[11]) & 0x3FFFFU;

        process_ppg_slot(state, slot1);
        process_ppg_slot(state, slot2);
        process_spo2_slot(state, slot3, slot4);
    }
}

static int max30101_configure(void)
{
    int err;

    err = reg_write(MAX_REG_MODE_CFG, 0x40);
    if (err) {
        return err;
    }
    k_msleep(100);

    err |= reg_write(MAX_REG_FIFO_WR_PTR, 0x00);
    err |= reg_write(MAX_REG_FIFO_OVF, 0x00);
    err |= reg_write(MAX_REG_FIFO_RD_PTR, 0x00);
    err |= reg_write(MAX_REG_FIFO_CFG, 0x4F);
    err |= reg_write(MAX_REG_MODE_CFG, 0x07);
    err |= reg_write(MAX_REG_SPO2_CFG, 0x0F);
    err |= reg_write(MAX_REG_MULTI_LED1, 0x33);
    err |= reg_write(MAX_REG_MULTI_LED2, 0x21);
    err |= reg_write(MAX_REG_LED1_PA, 8);
    err |= reg_write(MAX_REG_LED2_PA, 17);
    err |= reg_write(MAX_REG_LED3_PA, 125);
    err |= reg_write(MAX_REG_LED4_PA, 125);
    err |= reg_write(MAX_REG_INT_ENABLE1, 0x80);

    return err ? -EIO : 0;
}

int max30101_simple_init(void)
{
    uint8_t part_id = 0;
    int err;

    s_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(s_i2c)) {
        s_ready = false;
        return -ENODEV;
    }

    err = reg_read(MAX_REG_PART_ID, &part_id);
    if (err || (part_id != 0x15U)) {
        s_ready = false;
        return -ENODEV;
    }

    err = max30101_configure();
    if (err) {
        s_ready = false;
        return err;
    }

    s_ready = true;
    LOG_INF("MAX30101 ready");
    return 0;
}

int max30101_simple_capture(uint8_t *heart_rate_bpm, bool *hr_valid, uint8_t *spo2_percent, bool *spo2_valid)
{
    struct max_algorithm_state state;
    const int64_t deadline = k_uptime_get() + RELIFE_PPG_CAPTURE_MS;
    uint8_t throwaway = 0;
    int err;

    if (!s_ready || !heart_rate_bpm || !hr_valid || !spo2_percent || !spo2_valid) {
        return -EINVAL;
    }

    algorithm_reset(&state);
    err = max30101_configure();
    if (err) {
        return err;
    }

    reg_read(MAX_REG_INT_STATUS1, &throwaway);
    k_msleep(RELIFE_SENSOR_STABILIZE_MS);

    while (k_uptime_get() < deadline) {
        process_fifo_samples(&state);
        k_msleep(1000 / SAMPLE_RATE_HZ);
    }

    *heart_rate_bpm = state.latest_hr;
    *spo2_percent = state.latest_spo2;
    *hr_valid = state.hr_valid;
    *spo2_valid = state.spo2_valid;

    return (state.hr_valid || state.spo2_valid) ? 0 : -ENODATA;
}

bool max30101_simple_is_ready(void)
{
    return s_ready;
}
