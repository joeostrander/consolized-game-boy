# Replace TMDS with 10 bit UART (same baud rate):
# add_definitions(-DDVI_SERIAL_DEBUG=1)
# add_definitions(-DRUN_FROM_CRYSTAL)

add_executable(dmg_dvi
    main.c
    analog_microphone.c
    emusound.c
    shared_dma_handler.c
    colors.c
    eeprom.c
    osd.c
    font_5x7.c
)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})

target_compile_options(dmg_dvi PRIVATE -Wall)

# Set resolution mode (choose 0/1/2)
# 0 = 640x480, horizontally scaled x4, vertically x3 --> 640x480 Full screen
# 1 = 800x600, horizontally scaled x4, vertically x4 --> 640x576 window
# 2 = 640x480, horizontally scaled x2, vertically x2 --> 320x288 window
set(RESOLUTION_MODE "0" CACHE STRING "Resolution mode: 0=640x480 x4/x3, 1=800x600 x4/x4, 2=640x480 x2/x2")
set_property(CACHE RESOLUTION_MODE PROPERTY STRINGS 0 1 2)
set(USE_NES_CLASSIC_OR_WII "1" CACHE STRING "Controller input: 0=NES shift-register, 1=NES Classic/Wii I2C")
set_property(CACHE USE_NES_CLASSIC_OR_WII PROPERTY STRINGS 0 1)

if(RESOLUTION_MODE STREQUAL "1")
    set(DVI_VERTICAL_REPEAT_VALUE 4)
elseif(RESOLUTION_MODE STREQUAL "2")
    set(DVI_VERTICAL_REPEAT_VALUE 2)
else()
    set(DVI_VERTICAL_REPEAT_VALUE 3)
endif()

if(USE_NES_CLASSIC_OR_WII STREQUAL "1")
    set(CONTROLLER_INPUT_FORMAT "NES Classic/Wii")
else()
    set(CONTROLLER_INPUT_FORMAT "NES")
endif()

message(STATUS "Controller input: ${CONTROLLER_INPUT_FORMAT}")

target_compile_definitions(dmg_dvi PRIVATE
    DVI_DEFAULT_SERIAL_CONFIG=${DVI_DEFAULT_SERIAL_CONFIG}
    DVI_VERTICAL_REPEAT=${DVI_VERTICAL_REPEAT_VALUE}
    DVI_SYMBOLS_PER_WORD=2
    RESOLUTION_MODE=${RESOLUTION_MODE}
    USE_NES_CLASSIC_OR_WII=${USE_NES_CLASSIC_OR_WII}
    PICO_FLASH_SIZE_BYTES=0x200000  # (2097152, 2MB) - I need to define this or it may set to 4MB by default
)

target_link_libraries(dmg_dvi
    pico_stdlib
    pico_stdio_uart
    pico_multicore
    pico_util
    libdvi
    libsprite
    hardware_pio
    hardware_i2c
    hardware_uart
    hardware_irq
    hardware_adc
    hardware_dma
    hardware_flash
)

# Disable all stdio output
pico_enable_stdio_uart(dmg_dvi 0)
pico_enable_stdio_usb(dmg_dvi 0)

# Build pio programs
pico_generate_pio_header(dmg_dvi ${CMAKE_CURRENT_LIST_DIR}/video_capture.pio)

target_include_directories(dmg_dvi PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# create map/bin/hex file etc.
pico_add_extra_outputs(dmg_dvi)
