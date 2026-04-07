#ifndef FRAM_CY15B108_H
#define FRAM_CY15B108_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

int fram_drv_init(void);
int fram_drv_read(uint32_t addr, uint8_t *data, size_t len);
int fram_drv_write(uint32_t addr, const uint8_t *data, size_t len);
bool fram_drv_is_ready(void);

#endif
