#ifndef MLX90632_SIMPLE_H
#define MLX90632_SIMPLE_H

#include <stdbool.h>
#include <stdint.h>

int mlx90632_simple_init(void);
int mlx90632_simple_read_centi_degrees(int16_t *temp_centi_c);
bool mlx90632_simple_is_ready(void);

#endif
