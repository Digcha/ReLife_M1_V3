#include "rv3028_simple.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(rv3028_simple, LOG_LEVEL_INF);

#define RV3028_ADDR         0x52
#define RV3028_REG_SECONDS  0x00
#define RV3028_REG_MINUTES  0x01
#define RV3028_REG_HOURS    0x02
#define RV3028_REG_WEEKDAY  0x03
#define RV3028_REG_DATE     0x04
#define RV3028_REG_MONTH    0x05
#define RV3028_REG_YEAR     0x06
#define RV3028_REG_ID       0x28

struct rv3028_datetime {
    int year;
    int month;
    int day;
    int hour;
    int minute;
    int second;
};

static const struct device *s_i2c;
static bool s_ready;

static uint8_t bcd_to_dec(uint8_t value)
{
    return (uint8_t)(((value >> 4) * 10U) + (value & 0x0FU));
}

static uint8_t dec_to_bcd(uint8_t value)
{
    return (uint8_t)((((value / 10U) & 0x0FU) << 4) | (value % 10U));
}

static bool is_leap_year(int year)
{
    return ((year % 4) == 0) && (((year % 100) != 0) || ((year % 400) == 0));
}

static int64_t days_from_civil(int year, unsigned int month, unsigned int day)
{
    const int adjusted_month = (month > 2U) ? (int)month - 3 : (int)month + 9;

    year -= (month <= 2U);
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned int yoe = (unsigned int)(year - era * 400);
    const unsigned int doy = (153U * (unsigned int)adjusted_month + 2U) / 5U + day - 1U;
    const unsigned int doe = yoe * 365U + yoe / 4U - yoe / 100U + doy;

    return (int64_t)era * 146097LL + (int64_t)doe - 719468LL;
}

static void civil_from_days(int64_t z, struct rv3028_datetime *dt)
{
    z += 719468LL;
    const int era = (z >= 0 ? z : z - 146096) / 146097;
    const unsigned int doe = (unsigned int)(z - era * 146097);
    const unsigned int yoe = (doe - doe / 1460U + doe / 36524U - doe / 146096U) / 365U;
    const int y = (int)yoe + era * 400;
    const unsigned int doy = doe - (365U * yoe + yoe / 4U - yoe / 100U);
    const unsigned int mp = (5U * doy + 2U) / 153U;
    const unsigned int d = doy - (153U * mp + 2U) / 5U + 1U;
    const int m = (mp < 10U) ? (int)mp + 3 : (int)mp - 9;

    dt->year = y + (m <= 2U);
    dt->month = m;
    dt->day = (int)d;
}

static bool rv3028_datetime_valid(const struct rv3028_datetime *dt)
{
    static const uint8_t days_per_month[] = {
        31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31,
    };

    if ((dt->year < 2024) || (dt->year > 2099)) {
        return false;
    }
    if ((dt->month < 1) || (dt->month > 12)) {
        return false;
    }
    if ((dt->hour < 0) || (dt->hour > 23) ||
        (dt->minute < 0) || (dt->minute > 59) ||
        (dt->second < 0) || (dt->second > 59)) {
        return false;
    }

    int max_day = days_per_month[dt->month - 1];
    if ((dt->month == 2) && is_leap_year(dt->year)) {
        max_day = 29;
    }

    return (dt->day >= 1) && (dt->day <= max_day);
}

static int rv3028_read_datetime(struct rv3028_datetime *dt)
{
    uint8_t reg = RV3028_REG_SECONDS;
    uint8_t raw[7];
    int err;

    if (!s_ready) {
        return -ENODEV;
    }

    err = i2c_write_read(s_i2c, RV3028_ADDR, &reg, 1, raw, sizeof(raw));
    if (err) {
        return err;
    }

    dt->second = bcd_to_dec(raw[0] & 0x7FU);
    dt->minute = bcd_to_dec(raw[1] & 0x7FU);
    dt->hour = bcd_to_dec(raw[2] & 0x3FU);
    dt->day = bcd_to_dec(raw[4] & 0x3FU);
    dt->month = bcd_to_dec(raw[5] & 0x1FU);
    dt->year = 2000 + bcd_to_dec(raw[6]);
    return 0;
}

int rv3028_init(void)
{
    uint8_t reg = RV3028_REG_ID;
    uint8_t id = 0;
    int err;

    s_i2c = DEVICE_DT_GET(DT_NODELABEL(i2c0));
    if (!device_is_ready(s_i2c)) {
        s_ready = false;
        return -ENODEV;
    }

    err = i2c_write_read(s_i2c, RV3028_ADDR, &reg, 1, &id, 1);
    if (err) {
        s_ready = false;
        return err;
    }

    s_ready = true;
    LOG_INF("RV3028 ready (ID=0x%02x)", id);
    return 0;
}

int rv3028_get_unix_time(uint32_t *unix_time)
{
    struct rv3028_datetime dt;
    int64_t days;
    int64_t seconds;
    int err;

    if (unix_time == NULL) {
        return -EINVAL;
    }

    err = rv3028_read_datetime(&dt);
    if (err) {
        return err;
    }
    if (!rv3028_datetime_valid(&dt)) {
        return -ERANGE;
    }

    days = days_from_civil(dt.year, (unsigned int)dt.month, (unsigned int)dt.day);
    seconds = (days * 86400LL) + (int64_t)dt.hour * 3600LL + (int64_t)dt.minute * 60LL + (int64_t)dt.second;
    if (seconds < 0 || seconds > UINT32_MAX) {
        return -ERANGE;
    }

    *unix_time = (uint32_t)seconds;
    return 0;
}

int rv3028_set_unix_time(uint32_t unix_time)
{
    struct rv3028_datetime dt = { 0 };
    uint8_t raw[8];
    int64_t days;
    int err;

    if (!s_ready) {
        return -ENODEV;
    }

    days = (int64_t)(unix_time / 86400U);
    civil_from_days(days, &dt);
    dt.hour = (int)((unix_time % 86400U) / 3600U);
    dt.minute = (int)((unix_time % 3600U) / 60U);
    dt.second = (int)(unix_time % 60U);

    raw[0] = RV3028_REG_SECONDS;
    raw[1] = dec_to_bcd((uint8_t)dt.second);
    raw[2] = dec_to_bcd((uint8_t)dt.minute);
    raw[3] = dec_to_bcd((uint8_t)dt.hour);
    raw[4] = dec_to_bcd((uint8_t)(((days + 4LL) % 7LL) + 1LL));
    raw[5] = dec_to_bcd((uint8_t)dt.day);
    raw[6] = dec_to_bcd((uint8_t)dt.month);
    raw[7] = dec_to_bcd((uint8_t)(dt.year - 2000));

    err = i2c_write(s_i2c, raw, sizeof(raw), RV3028_ADDR);
    if (!err) {
        LOG_INF("RTC synced to %u", unix_time);
    }
    return err;
}

bool rv3028_is_ready(void)
{
    return s_ready;
}

bool rv3028_has_valid_time(void)
{
    struct rv3028_datetime dt;

    if (rv3028_read_datetime(&dt)) {
        return false;
    }

    return rv3028_datetime_valid(&dt);
}
