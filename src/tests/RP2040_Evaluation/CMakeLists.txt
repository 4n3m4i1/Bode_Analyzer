cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(RP2040_ADAPTIVE_FILTER_EVAL C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(
                RP2040_ADAPTIVE_FILTER_EVAL
                main.c
                core_0_exec.c
                core_1_exec.c
                )

# Link Libraries
target_link_libraries(
                        RP2040_ADAPTIVE_FILTER_EVAL
                        pico_stdlib
                        pico_multicore
                        hardware_i2c
                        hardware_timer
                        hardware_pwm
                        )

pico_enable_stdio_usb(RP2040_ADAPTIVE_FILTER_EVAL 1)
pico_enable_stdio_uart(RP2040_ADAPTIVE_FILTER_EVAL 0)

# Create EXEC binaries like uf2
pico_add_extra_outputs(RP2040_ADAPTIVE_FILTER_EVAL)