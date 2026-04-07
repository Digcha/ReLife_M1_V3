#ifndef BMA400_SIMPLE_H
#define BMA400_SIMPLE_H

#include <stdbool.h>
#include <stdint.h>

int bma400_simple_init(void);
int bma400_simple_get_steps(uint32_t *steps_total, uint8_t *activity);
bool bma400_simple_is_ready(void);

#endif
