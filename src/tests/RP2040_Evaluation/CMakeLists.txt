cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(RP2040_ADAPTIVE_FILTER_EVAL C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

set(PICO_EXAMPLES_PATH ${PROJECT_SOURCE_DIR})

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(
                ${PROJECT_NAME}
                main.c
                core_0_exec.c
                core_1_exec.c

                FFT/fft_half.c
                circular_buffers/RP2040_Circ_Buffer.c
                )

# Link Libraries
target_link_libraries(
                        ${PROJECT_NAME}
                        pico_stdlib
                        pico_multicore
                        hardware_i2c
                        hardware_spi
                        hardware_timer
                        hardware_pwm
                        )



add_library(FFT FFT/fft_half.c FFT/fft_half.h)
add_library(circular_buffers circular_buffers/RP2040_Circ_Buffer.c circular_buffers/RP2040_Circ_Buffer.h)

target_link_libraries(FFT
    pico_stdlib
)

target_link_libraries(circular_buffers
    pico_stdlib
    stdlib
    pico_malloc
)

pico_enable_stdio_usb(RP2040_ADAPTIVE_FILTER_EVAL 1)
pico_enable_stdio_uart(RP2040_ADAPTIVE_FILTER_EVAL 0)

# Create EXEC binaries like uf2
pico_add_extra_outputs(RP2040_ADAPTIVE_FILTER_EVAL)

# Important Memory Management!!!!!!!!!!!!
## Okay the .. is a total hack but it works so... idk man I'm not fighting cmake this much
# I think it derives from some weird "build is the current working directory" thing
# even tho cmake source dir should be the right place
pico_set_linker_script(${PROJECT_NAME}  ../linking_scripts/memmap_blocked_ram.ld)

