#ifndef VIDEO_DEFS_H
#define VIDEO_DEFS_H

#include "dvi.h"

// Game Boy capture geometry (2bpp packed)
#define DMG_PIXELS_X                160
#define DMG_PIXELS_Y                144
#define DMG_PIXEL_COUNT             (DMG_PIXELS_X * DMG_PIXELS_Y)

// Packed DMA buffers - 4 pixels per byte (2 bits each)
#define PACKED_FRAME_SIZE           (DMG_PIXEL_COUNT / 4)
#define PACKED_LINE_STRIDE_BYTES    (DMG_PIXELS_X / 4)


#define RESOLUTION_MODE_640x480_x4x3 0   // stretch x4,x3, window = 640x432
#define RESOLUTION_MODE_800x600 1        // stretch x4,x4, window = 640x576
#define RESOLUTION_MODE_640x480_x2x2 2   // stretch x2,x2, window = 320x288

#if RESOLUTION_MODE == RESOLUTION_MODE_640x480_x4x3  // 640x480, stretch x4,x3
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define HORIZONTAL_SCALE 4

#define VREG_VSEL VREG_VOLTAGE_1_10  // 252 MHz is comfortable at lower voltage
#define DVI_TIMING dvi_timing_640x480p_60hz
#elif RESOLUTION_MODE == RESOLUTION_MODE_800x600    // 800x600, stretch x4,x4
#define FRAME_WIDTH 800
#define FRAME_HEIGHT 600

#define HORIZONTAL_SCALE 4

static const struct dvi_timing __not_in_flash_func(dvi_timing_800x600p_60hz_280K) = {
    .h_sync_polarity   = false,
    .h_front_porch     = 44,
    .h_sync_width      = 128,
    .h_back_porch      = 88,
    .h_active_pixels   = 800,

    .v_sync_polarity   = false,
    .v_front_porch     = 2,        // increased from 1
    .v_sync_width      = 4,
    .v_back_porch      = 22,
    .v_active_lines    = 600,

    .bit_clk_khz       = 280000
};

#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_800x600p_60hz_280K
#else  // RESOLUTION_MODE_640x480_x2x2
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define HORIZONTAL_SCALE 2

#define VREG_VSEL VREG_VOLTAGE_1_10  // 252 MHz is comfortable at lower voltage
#define DVI_TIMING dvi_timing_640x480p_60hz
#endif

#define SCANLINE_COUNT    (FRAME_HEIGHT / DVI_VERTICAL_REPEAT)
#define VERTICAL_OFFSET   (((SCANLINE_COUNT - DMG_PIXELS_Y) / 2) - 2)  // center vertically, adjust for pre-pushed lines

#endif // VIDEO_DEFS_H
