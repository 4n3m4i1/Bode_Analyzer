cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(ADS_PIO_SPI_LIB C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()

add_executable(${PROJECT_NAME}
        main.c
        PIO_ADS7253/PIO_ADS7253.c
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/PIO_ADS7253/pio_ads7253_spi.pio)

target_link_libraries(${PROJECT_NAME} 
        pico_stdlib
        hardware_pio
)

#target_sources(${PROJECT_NAME} PRIVATE
#                PIO_ADS7253.c
#                PIO_ADS7253.h
#                )

add_library(PIOSPI PIO_ADS7253/PIO_ADS7253.c PIO_ADS7253/PIO_ADS7253.h)
target_link_libraries(PIOSPI
                        pico_stdlib
                        hardware_pio
                        )

pico_add_extra_outputs(${PROJECT_NAME})

# For USB printf, disable UART
pico_enable_stdio_usb(${PROJECT_NAME} 1)