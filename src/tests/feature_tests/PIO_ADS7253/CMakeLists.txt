cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(spitester C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

pico_sdk_init()

add_executable(${PROJECT_NAME}
        main.c
)

pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/spi.pio)

target_link_libraries(${PROJECT_NAME} 
        pico_stdlib
        hardware_pio
)

target_sources(${PROJECT_NAME} PRIVATE
                pio_spi.c
                pio_spi.h
                )

#add_library(PIOSPI pio_spi.c pio_spi.h)
#target_link_libraries(PIOSPI
#                        pico_stdlib
#                        hardware_pio
#                        )

pico_add_extra_outputs(${PROJECT_NAME})

# For USB printf, disable UART
pico_enable_stdio_usb(${PROJECT_NAME} 1)