#include "bma400_simple.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "bma400.h"

LOG_MODULE_REGISTER(bma400_simple, LOG_LEVEL_INF);

#define BMA400_I2C_ADDR 0x14

static const struct device *s_i2c;
static struct bma400_dev s_bma;
static uint8_t s_addr = BMA400_I2C_ADDR;
static bool s_ready;

static const uint8_t s_step_counter_wrist_params[] = {
    1, 45, 123, 212, 68, 1, 59, 122,
    219, 123, 63, 108, 205, 39, 25, 150,
    160, 195, 14, 12, 60, 240, 0, 247,
};

static BMA400_INTF_RET_TYPE bma_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);

    struct i2c_msg msgs[2] = {
        {
            .buf = &reg_addr,
            .len = 1,
            .flags = I2C_MSG_WRITE,
        },
        {
            .buf = reg_data,
            .len = length,
            .flags = I2C_MSG_READ | I2C_MSG_STOP,
        },
    };

    return (BMA400_INTF_RET_TYPE)i2c_transfer(s_i2c, msgs, 2, s_addr);
}

static BMA400_INTF_RET_TYPE bma_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    uint8_t tx[32];

    ARG_UNUSED(intf_ptr);

    if (length > (sizeof(tx) - 1U)) {
        return (BMA400_INTF_RET_TYPE)-EINVAL;
    }

    tx[0] = reg_addr;
    memcpy(&tx[1], reg_data, length);

    return (BMA400_INTF_RET_TYPE)i2c_write(s_i2c, tx, length + 1U, s_addr);
}

static void bma_delay_us(uint32_t period, void *intf_ptr)
{
    ARG_UNUSED(intf_ptr);
    k_busy_wait(period);
}

int bma400_simple_init(void)
{
    struct bma400_sensor_conf accel_conf = {
        .type = BMA400_ACCEL,
    };
    struct bma400_sensor_conf step_conf = {
        .type = BMA400_STEP_COUNTER_INT,
    };
    struct bma400_int_enable step_int = {
        .type = BMA400_STEP_COUNTER_INT_EN,
        .conf = BMA400_ENABLE,
    };
    int8_t ret;

    s_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(s_i2c)) {
        s_ready = false;
        return -ENODEV;
    }

    memset(&s_bma, 0, sizeof(s_bma));
    s_bma.intf = BMA400_I2C_INTF;
    s_bma.read = bma_i2c_read;
    s_bma.write = bma_i2c_write;
    s_bma.delay_us = bma_delay_us;
    s_bma.intf_ptr = &s_addr;

    ret = bma400_init(&s_bma);
    if (ret != BMA400_OK) {
        s_ready = false;
        return -EIO;
    }

    ret = bma400_soft_reset(&s_bma);
    if (ret != BMA400_OK) {
        s_ready = false;
        return -EIO;
    }

    accel_conf.param.accel.odr = BMA400_ODR_100HZ;
    accel_conf.param.accel.range = BMA400_RANGE_4G;
    accel_conf.param.accel.data_src = BMA400_DATA_SRC_ACCEL_FILT_1;
    accel_conf.param.accel.filt1_bw = BMA400_ACCEL_FILT1_BW_1;

    ret = bma400_set_sensor_conf(&accel_conf, 1, &s_bma);
    if (ret != BMA400_OK) {
        s_ready = false;
        return -EIO;
    }

    ret = bma400_set_step_counter_param(s_step_counter_wrist_params, &s_bma);
    if (ret != BMA400_OK) {
        s_ready = false;
        return -EIO;
    }

    step_conf.param.step_cnt.int_chan = BMA400_INT_CHANNEL_2;
    ret = bma400_set_sensor_conf(&step_conf, 1, &s_bma);
    if (ret != BMA400_OK) {
        s_ready = false;
        return -EIO;
    }

    ret = bma400_enable_interrupt(&step_int, 1, &s_bma);
    if (ret != BMA400_OK) {
        s_ready = false;
        return -EIO;
    }

    ret = bma400_set_power_mode(BMA400_MODE_NORMAL, &s_bma);
    if (ret != BMA400_OK) {
        s_ready = false;
        return -EIO;
    }

    s_ready = true;
    LOG_INF("BMA400 ready");
    return 0;
}

int bma400_simple_get_steps(uint32_t *steps_total, uint8_t *activity)
{
    int8_t ret;

    if (!s_ready || (steps_total == NULL) || (activity == NULL)) {
        return -EINVAL;
    }

    ret = bma400_get_steps_counted(steps_total, activity, &s_bma);
    if (ret != BMA400_OK) {
        return -EIO;
    }

    return 0;
}

bool bma400_simple_is_ready(void)
{
    return s_ready;
}
