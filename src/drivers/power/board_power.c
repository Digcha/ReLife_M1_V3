#include "board_power.h"

#include <errno.h>
#include <stdbool.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(board_power, LOG_LEVEL_INF);

#define EN_1V8_I2C_PIN   16
#define EN_TEMP_PIN      20
#define EN_VLED_PIN      21
#define EN_I2CSHIFT_PIN  22

static const struct device *s_gpio0;
static bool s_ready;

int board_power_init(void)
{
    int err;

    s_gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(s_gpio0)) {
        s_ready = false;
        return -ENODEV;
    }

    err = gpio_pin_configure(s_gpio0, EN_1V8_I2C_PIN, GPIO_OUTPUT_INACTIVE);
    err |= gpio_pin_configure(s_gpio0, EN_TEMP_PIN, GPIO_OUTPUT_INACTIVE);
    err |= gpio_pin_configure(s_gpio0, EN_VLED_PIN, GPIO_OUTPUT_INACTIVE);
    err |= gpio_pin_configure(s_gpio0, EN_I2CSHIFT_PIN, GPIO_OUTPUT_INACTIVE);
    if (err) {
        s_ready = false;
        return err;
    }

    s_ready = true;
    return 0;
}

int board_power_enable_sensor_rails(void)
{
    if (!s_ready) {
        return -ENODEV;
    }

    gpio_pin_set(s_gpio0, EN_1V8_I2C_PIN, 1);
    k_msleep(20);
    gpio_pin_set(s_gpio0, EN_I2CSHIFT_PIN, 1);
    k_msleep(20);
    gpio_pin_set(s_gpio0, EN_TEMP_PIN, 1);
    k_msleep(20);
    gpio_pin_set(s_gpio0, EN_VLED_PIN, 1);
    k_msleep(60);

    LOG_INF("Sensor rails enabled");
    return 0;
}

bool board_power_is_ready(void)
{
    return s_ready;
}
