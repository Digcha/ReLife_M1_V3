#ifndef BOARD_POWER_H
#define BOARD_POWER_H

#include <stdbool.h>

int board_power_init(void);
int board_power_enable_sensor_rails(void);
bool board_power_is_ready(void);

#endif
