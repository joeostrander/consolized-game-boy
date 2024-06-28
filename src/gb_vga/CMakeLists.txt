cmake_minimum_required(VERSION 3.12)

# Pull in PICO SDK (must be before project).
include(../pico-sdk/external/pico_sdk_import.cmake)

# We also need PICO EXTRAS.
include(../pico-extras/external/pico_extras_import.cmake)

project(gb_vga C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the Pico SDK.
pico_sdk_init()

if (TARGET pico_scanvideo_dpi)
    add_executable(gb_vga
            osd.c
            colors.c
            eeprom.c
            )

    target_sources(gb_vga PRIVATE gb_vga.c)

    target_compile_definitions(gb_vga PRIVATE
        -DPICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=500
        )

    # RGB222
    add_compile_definitions(PICO_SCANVIDEO_COLOR_PIN_COUNT=6)    # scanvideo_base.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_RSHIFT=4)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_GSHIFT=2)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_BSHIFT=0)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_RCOUNT=2)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_GCOUNT=2)   # scanvideo.h
    add_compile_definitions(PICO_SCANVIDEO_DPI_PIXEL_BCOUNT=2)   # scanvideo.h

    target_link_libraries(gb_vga PRIVATE
            pico_multicore
            pico_stdlib
            pico_scanvideo_dpi
            hardware_i2c
            hardware_flash
            )

endif ()
    
pico_set_linker_script(gb_vga ${CMAKE_SOURCE_DIR}/memmap_custom.ld)

pico_set_program_name(gb_vga "gb_vga")
pico_set_program_version(gb_vga "2.4.1")

# UART & USB
# pico_enable_stdio_usb(gb_vga 1)
# pico_enable_stdio_uart(gb_vga 0)

pico_add_extra_outputs(gb_vga)