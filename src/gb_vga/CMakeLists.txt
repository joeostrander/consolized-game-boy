cmake_minimum_required(VERSION 3.12)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

project(gb_vga C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
option(USE_RGB332_PCB "Use RGB332 output format instead of RGB222" OFF)
option(USE_NES_CLASSIC_OR_WII "Use I2C NES Classic/Wii controller instead of the shift-register NES controller" OFF)

# Initialize the Pico SDK.
pico_sdk_init()

if (USE_RGB332_PCB)
    set(VIDEO_OUTPUT_FORMAT "RGB332")
else()
    set(VIDEO_OUTPUT_FORMAT "RGB222")
endif()

if (USE_NES_CLASSIC_OR_WII)
    set(CONTROLLER_INPUT_FORMAT "NES Classic/Wii")
else()
    set(CONTROLLER_INPUT_FORMAT "NES")
endif()

message(STATUS "Video output format: ${VIDEO_OUTPUT_FORMAT}")
message(STATUS "Controller input: ${CONTROLLER_INPUT_FORMAT}")

if (NOT TARGET pico_scanvideo_dpi)
    message(FATAL_ERROR "pico_scanvideo_dpi target is unavailable. Ensure Pico Extras is available via PICO_EXTRAS_PATH or as a sibling of PICO_SDK_PATH.")
endif ()

add_executable(gb_vga
        osd.c
        colors.c
        eeprom.c
        )

target_sources(gb_vga PRIVATE gb_vga.c)

pico_generate_pio_header(gb_vga ${CMAKE_CURRENT_LIST_DIR}/video_capture.pio)

target_compile_definitions(gb_vga PRIVATE
    PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS=500
    USE_RGB332_PCB=${USE_RGB332_PCB}
    USE_NES_CLASSIC_OR_WII=${USE_NES_CLASSIC_OR_WII}
    )

if (USE_RGB332_PCB)
    # RGB332
    target_compile_definitions(gb_vga PRIVATE
        PICO_SCANVIDEO_COLOR_PIN_COUNT=8
        PICO_SCANVIDEO_DPI_PIXEL_RSHIFT=5
        PICO_SCANVIDEO_DPI_PIXEL_GSHIFT=2
        PICO_SCANVIDEO_DPI_PIXEL_BSHIFT=0
        PICO_SCANVIDEO_DPI_PIXEL_RCOUNT=3
        PICO_SCANVIDEO_DPI_PIXEL_GCOUNT=3
        PICO_SCANVIDEO_DPI_PIXEL_BCOUNT=2
        )
else()
    # RGB222
    target_compile_definitions(gb_vga PRIVATE
        PICO_SCANVIDEO_COLOR_PIN_COUNT=6
        PICO_SCANVIDEO_DPI_PIXEL_RSHIFT=4
        PICO_SCANVIDEO_DPI_PIXEL_GSHIFT=2
        PICO_SCANVIDEO_DPI_PIXEL_BSHIFT=0
        PICO_SCANVIDEO_DPI_PIXEL_RCOUNT=2
        PICO_SCANVIDEO_DPI_PIXEL_GCOUNT=2
        PICO_SCANVIDEO_DPI_PIXEL_BCOUNT=2
        )
endif()

target_link_libraries(gb_vga PRIVATE
        pico_multicore
        pico_stdlib
        pico_scanvideo_dpi
        hardware_i2c
        hardware_dma
        hardware_flash
        hardware_clocks
        hardware_vreg
        hardware_pio
        )
    
# pico_set_linker_script(gb_vga ${CMAKE_SOURCE_DIR}/memmap_custom.ld)

pico_set_program_name(gb_vga "gb_vga")

# UART & USB
# pico_enable_stdio_usb(gb_vga 1)
# pico_enable_stdio_uart(gb_vga 0)

pico_add_extra_outputs(gb_vga)