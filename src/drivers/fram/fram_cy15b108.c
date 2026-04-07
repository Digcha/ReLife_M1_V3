#include "fram_cy15b108.h"

#include <errno.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(fram_drv, LOG_LEVEL_INF);

#define FRAM_CS_PIN         8
#define FRAM_CMD_RDSR       0x05
#define FRAM_CMD_WREN       0x06
#define FRAM_CMD_READ       0x03
#define FRAM_CMD_WRITE      0x02
#define FRAM_CMD_RDID       0x9F
#define FRAM_CHUNK_SIZE     240U

static const struct device *s_spi;
static const struct device *s_gpio0;
static bool s_ready;

static const struct spi_config s_spi_cfg = {
    .frequency = 1000000U,
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER,
    .slave = 0U,
};

static inline void fram_cs_assert(void)
{
    gpio_pin_set(s_gpio0, FRAM_CS_PIN, 0);
}

static inline void fram_cs_deassert(void)
{
    gpio_pin_set(s_gpio0, FRAM_CS_PIN, 1);
}

static int fram_spi_write(const uint8_t *data, size_t len)
{
    struct spi_buf tx_buf = {
        .buf = (void *)data,
        .len = len,
    };
    struct spi_buf_set tx_set = {
        .buffers = &tx_buf,
        .count = 1,
    };

    return spi_write(s_spi, &s_spi_cfg, &tx_set);
}

static int fram_spi_transceive(const uint8_t *tx, uint8_t *rx, size_t len)
{
    struct spi_buf tx_buf = {
        .buf = (void *)tx,
        .len = len,
    };
    struct spi_buf rx_buf = {
        .buf = rx,
        .len = len,
    };
    struct spi_buf_set tx_set = {
        .buffers = &tx_buf,
        .count = 1,
    };
    struct spi_buf_set rx_set = {
        .buffers = &rx_buf,
        .count = 1,
    };

    return spi_transceive(s_spi, &s_spi_cfg, &tx_set, &rx_set);
}

static int fram_write_enable(void)
{
    const uint8_t cmd = FRAM_CMD_WREN;
    int err;

    fram_cs_assert();
    err = fram_spi_write(&cmd, sizeof(cmd));
    fram_cs_deassert();

    return err;
}

static int fram_read_status(uint8_t *status)
{
    uint8_t tx[2] = { FRAM_CMD_RDSR, 0x00 };
    uint8_t rx[2] = { 0 };
    int err;

    fram_cs_assert();
    err = fram_spi_transceive(tx, rx, sizeof(tx));
    fram_cs_deassert();
    if (err) {
        return err;
    }

    *status = rx[1];
    return 0;
}

int fram_drv_init(void)
{
    uint8_t tx[10] = { FRAM_CMD_RDID, 0 };
    uint8_t rx[10] = { 0 };
    uint8_t status = 0;
    int err;

    s_spi = DEVICE_DT_GET(DT_NODELABEL(spi1));
    s_gpio0 = DEVICE_DT_GET(DT_NODELABEL(gpio0));
    if (!device_is_ready(s_spi) || !device_is_ready(s_gpio0)) {
        s_ready = false;
        return -ENODEV;
    }

    err = gpio_pin_configure(s_gpio0, FRAM_CS_PIN, GPIO_OUTPUT | GPIO_OUTPUT_INIT_HIGH);
    if (err) {
        s_ready = false;
        return err;
    }

    fram_cs_assert();
    err = fram_spi_transceive(tx, rx, sizeof(tx));
    fram_cs_deassert();
    if (err) {
        s_ready = false;
        return err;
    }

    err = fram_read_status(&status);
    if (err) {
        s_ready = false;
        return err;
    }

    if ((status & 0x40U) == 0U) {
        s_ready = false;
        return -EIO;
    }

    s_ready = true;
    LOG_INF("FRAM ready");
    return 0;
}

int fram_drv_read(uint32_t addr, uint8_t *data, size_t len)
{
    size_t remaining = len;
    size_t copied = 0U;

    if (!s_ready || ((len > 0U) && (data == NULL))) {
        return -EINVAL;
    }

    while (remaining > 0U) {
        const size_t chunk = MIN(remaining, FRAM_CHUNK_SIZE);
        uint8_t tx[4 + FRAM_CHUNK_SIZE];
        uint8_t rx[4 + FRAM_CHUNK_SIZE];
        int err;

        memset(tx, 0, sizeof(tx));
        tx[0] = FRAM_CMD_READ;
        tx[1] = (uint8_t)((addr >> 16) & 0xFFU);
        tx[2] = (uint8_t)((addr >> 8) & 0xFFU);
        tx[3] = (uint8_t)(addr & 0xFFU);

        fram_cs_assert();
        err = fram_spi_transceive(tx, rx, 4U + chunk);
        fram_cs_deassert();
        if (err) {
            return err;
        }

        memcpy(&data[copied], &rx[4], chunk);
        copied += chunk;
        remaining -= chunk;
        addr += chunk;
    }

    return 0;
}

int fram_drv_write(uint32_t addr, const uint8_t *data, size_t len)
{
    size_t remaining = len;
    size_t written = 0U;

    if (!s_ready || ((len > 0U) && (data == NULL))) {
        return -EINVAL;
    }

    while (remaining > 0U) {
        const size_t chunk = MIN(remaining, FRAM_CHUNK_SIZE);
        uint8_t tx[4 + FRAM_CHUNK_SIZE];
        int err;

        err = fram_write_enable();
        if (err) {
            return err;
        }

        tx[0] = FRAM_CMD_WRITE;
        tx[1] = (uint8_t)((addr >> 16) & 0xFFU);
        tx[2] = (uint8_t)((addr >> 8) & 0xFFU);
        tx[3] = (uint8_t)(addr & 0xFFU);
        memcpy(&tx[4], &data[written], chunk);

        fram_cs_assert();
        err = fram_spi_write(tx, 4U + chunk);
        fram_cs_deassert();
        if (err) {
            return err;
        }

        written += chunk;
        remaining -= chunk;
        addr += chunk;
    }

    return 0;
}

bool fram_drv_is_ready(void)
{
    return s_ready;
}
