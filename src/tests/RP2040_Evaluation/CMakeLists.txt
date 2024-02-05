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

                FFT/fft_half.c
                circular_buffers/RP2040_Circ_Buffer.c
                FIR/FIR_Fast_Fixed.c
                LMS/LMS_Fixed.c
                )

#pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/pio_val_2_pin.pio)
pico_generate_pio_header(${PROJECT_NAME} ${CMAKE_CURRENT_LIST_DIR}/PIO_PROG/pio_val_2_pin.pio)

# Link Libraries
target_link_libraries(
                        ${PROJECT_NAME}
                        pico_stdlib
                        pico_multicore
                        hardware_i2c
                        hardware_spi
                        hardware_timer
                        hardware_pwm
                        hardware_dma
                        hardware_pio
                        )


add_library(FFT FFT/fft_half.c FFT/fft_half.h)
add_library(circular_buffers circular_buffers/RP2040_Circ_Buffer.c circular_buffers/RP2040_Circ_Buffer.h)
add_library(FIR FIR/FIR_Fast_Fixed.c FIR/FIR_Fast_Fixed.h)
add_library(LMS LMS/LMS_Fixed.c LMS/LMS_Fixed.h)


target_link_libraries(FFT
    pico_stdlib
)

target_link_libraries(circular_buffers
    pico_stdlib
    stdlib
    pico_malloc
)

target_link_libraries(FIR
    pico_stdlib
)

target_link_libraries(LMS
    pico_stdlib
    FIR
)

pico_enable_stdio_usb(RP2040_ADAPTIVE_FILTER_EVAL 1)
pico_enable_stdio_uart(RP2040_ADAPTIVE_FILTER_EVAL 0)

# Create EXEC binaries like uf2
pico_add_extra_outputs(RP2040_ADAPTIVE_FILTER_EVAL)

# Important Memory Management!!!!!!!!!!!!
## Okay the .. is a total hack but it works so... idk man I'm not fighting cmake this much
# I think it derives from some weird "build is the current working directory" thing
# even tho cmake source dir should be the right place
#pico_set_linker_script(${PROJECT_NAME}  ../linking_scripts/memmap_blocked_ram.ld)
pico_set_linker_script(${PROJECT_NAME}  ../linking_scripts/memmap_custom_blocks.ld)

