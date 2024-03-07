cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(ADS7253_PIO C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()


add_executable(
                ${PROJECT_NAME}
                main.c
                )

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/ads7253_pio.pio)

# Link Libraries
target_link_libraries(
                        ${PROJECT_NAME}
                        pico_stdlib
                        pico_multicore
                        hardware_i2c
                        hardware_timer
                        hardware_pwm
                        hardware_dma
                        hardware_pio
                        )

pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Create EXEC binaries like uf2
pico_add_extra_outputs(${PROJECT_NAME})