#ifndef MAX30101_SIMPLE_H
#define MAX30101_SIMPLE_H

#include <stdbool.h>
#include <stdint.h>

int max30101_simple_init(void);
int max30101_simple_capture(uint8_t *heart_rate_bpm, bool *hr_valid, uint8_t *spo2_percent, bool *spo2_valid);
bool max30101_simple_is_ready(void);

#endif
