cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ReLife_M1_V3)

target_include_directories(app PRIVATE
    include
    include/bma400
    include/mlx90632
    src
)

target_sources(app PRIVATE
    src/main.c
    src/drivers/power/board_power.c
    src/drivers/fram/fram_cy15b108.c
    src/drivers/rv3028/rv3028_simple.c
    src/drivers/bma400/bma400_simple.c
    src/drivers/max30101/max30101_simple.c
    src/drivers/mlx90632/mlx90632_simple.c
    src/drivers/mlx90632/mlx90632_lib.c
    src/drivers/mlx90632/mlx90632_extended_meas.c
    src/shell_commands.c
    include/bma400/bma400.c
)

target_link_libraries(app PRIVATE m)
