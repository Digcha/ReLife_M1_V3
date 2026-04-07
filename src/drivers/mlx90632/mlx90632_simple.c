#include "mlx90632_simple.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "mlx90632.h"

LOG_MODULE_REGISTER(mlx90632_simple, LOG_LEVEL_INF);

#define MLX90632_ADDR  0x3A

struct mlx90632_calibration {
    int32_t p_t;
    int32_t p_r;
    int32_t p_g;
    int32_t p_o;
    int16_t gb;
    int16_t ka;
    int32_t ea;
    int32_t eb;
    int32_t ga;
    int32_t fa;
    int32_t fb;
    int16_t ha;
    int16_t hb;
};

static const struct device *s_i2c;
static struct mlx90632_calibration s_cal;
static bool s_ready;

int32_t mlx90632_i2c_read(int16_t register_address, uint16_t *value)
{
    uint8_t tx[2];
    uint8_t rx[2];
    int err;

    tx[0] = (uint8_t)((register_address >> 8) & 0xFF);
    tx[1] = (uint8_t)(register_address & 0xFF);

    err = i2c_write_read(s_i2c, MLX90632_ADDR, tx, sizeof(tx), rx, sizeof(rx));
    if (err) {
        return err;
    }

    *value = (uint16_t)(((uint16_t)rx[0] << 8) | rx[1]);
    return 0;
}

int32_t mlx90632_i2c_write(int16_t register_address, uint16_t value)
{
    uint8_t tx[4];

    tx[0] = (uint8_t)((register_address >> 8) & 0xFF);
    tx[1] = (uint8_t)(register_address & 0xFF);
    tx[2] = (uint8_t)((value >> 8) & 0xFF);
    tx[3] = (uint8_t)(value & 0xFF);

    return i2c_write(s_i2c, tx, sizeof(tx), MLX90632_ADDR);
}

void usleep(int min_range, int max_range)
{
    ARG_UNUSED(max_range);
    k_usleep(min_range);
}

void msleep(int msecs)
{
    k_msleep(msecs);
}

static int read_calibration_s32(int16_t reg, int32_t *value)
{
    uint16_t lo = 0;
    uint16_t hi = 0;
    int err;

    err = mlx90632_i2c_read(reg, &lo);
    err |= mlx90632_i2c_read((int16_t)(reg + 1), &hi);
    if (err) {
        return -EIO;
    }

    *value = (int32_t)(((uint32_t)hi << 16) | lo);
    return 0;
}

static int read_calibration_s16(int16_t reg, int16_t *value)
{
    uint16_t raw = 0;
    int err = mlx90632_i2c_read(reg, &raw);

    if (err) {
        return -EIO;
    }

    *value = (int16_t)raw;
    return 0;
}

static int load_calibration(void)
{
    int err = 0;

    err |= read_calibration_s32(MLX90632_EE_P_T, &s_cal.p_t);
    err |= read_calibration_s32(MLX90632_EE_P_R, &s_cal.p_r);
    err |= read_calibration_s32(MLX90632_EE_P_G, &s_cal.p_g);
    err |= read_calibration_s32(MLX90632_EE_P_O, &s_cal.p_o);
    err |= read_calibration_s16(MLX90632_EE_Gb, &s_cal.gb);
    err |= read_calibration_s16(MLX90632_EE_Ka, &s_cal.ka);
    err |= read_calibration_s32(MLX90632_EE_Ea, &s_cal.ea);
    err |= read_calibration_s32(MLX90632_EE_Eb, &s_cal.eb);
    err |= read_calibration_s32(MLX90632_EE_Ga, &s_cal.ga);
    err |= read_calibration_s32(MLX90632_EE_Fa, &s_cal.fa);
    err |= read_calibration_s32(MLX90632_EE_Fb, &s_cal.fb);
    err |= read_calibration_s16(MLX90632_EE_Ha, &s_cal.ha);
    err |= read_calibration_s16(MLX90632_EE_Hb, &s_cal.hb);

    return err ? -EIO : 0;
}

int mlx90632_simple_init(void)
{
    int ret;

    s_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(s_i2c)) {
        s_ready = false;
        return -ENODEV;
    }

    k_msleep(80);

    ret = mlx90632_init();
    if (ret < 0) {
        s_ready = false;
        return ret;
    }

    ret = mlx90632_set_meas_type(MLX90632_MTYP_MEDICAL);
    if (ret < 0) {
        s_ready = false;
        return ret;
    }

    ret = load_calibration();
    if (ret) {
        s_ready = false;
        return ret;
    }

    s_ready = true;
    LOG_INF("MLX90632 ready");
    return 0;
}

int mlx90632_simple_read_centi_degrees(int16_t *temp_centi_c)
{
    int16_t ambient_new_raw;
    int16_t ambient_old_raw;
    int16_t object_new_raw;
    int16_t object_old_raw;
    double pre_ambient;
    double pre_object;
    double object_temp;
    int ret;

    if (!s_ready || (temp_centi_c == NULL)) {
        return -EINVAL;
    }

    ret = mlx90632_read_temp_raw(&ambient_new_raw, &ambient_old_raw, &object_new_raw, &object_old_raw);
    if (ret < 0) {
        return ret;
    }

    pre_ambient = mlx90632_preprocess_temp_ambient(ambient_new_raw, ambient_old_raw, s_cal.gb);
    pre_object = mlx90632_preprocess_temp_object(
        object_new_raw,
        object_old_raw,
        ambient_new_raw,
        ambient_old_raw,
        s_cal.ka
    );
    object_temp = mlx90632_calc_temp_object(
        pre_object,
        pre_ambient,
        s_cal.ea,
        s_cal.eb,
        s_cal.ga,
        s_cal.fa,
        s_cal.fb,
        s_cal.ha,
        s_cal.hb
    );

    *temp_centi_c = (int16_t)(object_temp >= 0.0 ? (object_temp * 100.0 + 0.5) : (object_temp * 100.0 - 0.5));
    return 0;
}

bool mlx90632_simple_is_ready(void)
{
    return s_ready;
}
