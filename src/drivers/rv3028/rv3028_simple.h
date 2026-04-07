#ifndef RV3028_SIMPLE_H
#define RV3028_SIMPLE_H

#include <stdbool.h>
#include <stdint.h>

int rv3028_init(void);
int rv3028_get_unix_time(uint32_t *unix_time);
int rv3028_set_unix_time(uint32_t unix_time);
bool rv3028_is_ready(void);
bool rv3028_has_valid_time(void);

#endif
