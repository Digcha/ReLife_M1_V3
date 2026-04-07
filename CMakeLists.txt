cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(ReLife_M1_V3)

string(TIMESTAMP RELIFE_BUILD_UNIX "%s" UTC)
configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/src/relife_build_time.h.in
    ${CMAKE_CURRENT_BINARY_DIR}/generated/relife_build_time.h
    @ONLY
)

target_include_directories(app PRIVATE
    include
    include/bma400
    include/mlx90632
    src
    ${CMAKE_CURRENT_BINARY_DIR}/generated
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
