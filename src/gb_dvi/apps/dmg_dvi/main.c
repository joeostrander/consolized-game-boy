/*
TODO:
- Add support for NES shift-register based controller

*/

// Joe Ostrander
// 2026.03.28
// gb_dvi

#pragma GCC optimize("O3")

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <hardware/watchdog.h>
#include "hardware/vreg.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/clocks.h"
#include "hardware/regs/intctrl.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"
#include "pico/stdio.h"
#include "pico/stdio_uart.h"
#include "hardware/uart.h"
#include "hardware/adc.h"
#include "hardware/pwm.h"

#include "eeprom.h" // emulated with flash :)
#include "hardware/flash.h"
#include "pico/bootrom.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"
#include "tmds_encode.h"
#include "audio_ring.h"  // For get_write_size and get_read_size


#include "analog_microphone.h"
#include "emusound.h"
#include "colors.h"
#include "mario.h"
#include "video_defs.h"
#include "osd.h"

#include "video_capture.pio.h"  // PIO-based video capture
#include "shared_dma_handler.h"

// Set to 1 to enable NES Classic / Wii I2C-based controller support
// Set to 0 to use old-school shift register controller support
#ifndef USE_NES_CLASSIC_OR_WII
#define USE_NES_CLASSIC_OR_WII      1
#endif

#define HOME_RESETS_TO_BOOTLOADER   0  // Set to 1 to enable HOME button to reset into USB mass storage mode for easier programming

#define ENABLE_AUDIO                1  // Set to 1 to enable audio, 0 to disable all audio code
#define ENABLE_VIDEO_CAPTURE        1
#define ENABLE_OSD                  1  // Set to 1 to enable OSD code, 0 to disable
#define AUDIO_ON_CORE1              0  // Set to 1 to run audio processing on Core 1 (alongside DVI) to reduce contention with VSYNC GPIO handling on Core 0; set to 0 to run audio on Core 0 with timer IRQ (default)

#define BIT_IS_CLEAR(value, bit)    (((value) & (1U << (bit))) == 0)

#if ENABLE_AUDIO
static const int hdmi_n[6] = {4096, 6272, 6144, 3136, 4096, 6144};  // 32k, 44.1k, 48k, 22.05k, 16k, 24k
static uint16_t rate = SAMPLE_FREQ;
// #define AUDIO_BUFFER_SIZE   (0x1<<8) // Must be power of 2
audio_sample_t audio_buffer[AUDIO_BUFFER_SIZE];

#define AUDIO_GAIN                  1.0f
#define AUDIO_HPF_HZ                0.0f
#define AUDIO_LPF_HZ                0.0f

#define PIN_AUDIO_IN                28 // ADC2
#endif

// #define DEBUG_BUTTON_PRESS

#define ONBOARD_LED_PIN             25

#define DVI_SERIAL_CONFIG           adafruit_hstx_cfg

// GAMEBOY VIDEO INPUT (From level shifter)
#define HSYNC_PIN                   8
#define DATA_1_PIN                 11
#define DATA_0_PIN                 10
#define PIXEL_CLOCK_PIN             9
#define VSYNC_PIN                   7
#define DMG_READING_BUTTONS_PIN    21       // P15 (input, DMG will set HIGH when reading buttons)
#define DMG_READING_DPAD_PIN       20       // P14 (input, DMG will set HIGH when reading D-PAD)
#define DMG_OUTPUT_RIGHT_A_PIN      6       // P10 (output, button 'A' or 'RIGHT' state, depending on DMG_READING pins)
#define DMG_OUTPUT_UP_SELECT_PIN   26       // P12 (output, button 'SELECT' or 'UP' state, depending on DMG_READING pins)
#define DMG_OUTPUT_DOWN_START_PIN  22       // P13 (output, button 'START' or 'DOWN' state, depending on DMG_READING pins)
#define DMG_OUTPUT_LEFT_B_PIN      27       // P11 (output, button 'B' or 'LEFT' state, depending on DMG_READING pins)

#if USE_NES_CLASSIC_OR_WII
// I2C Pins, etc. -- for I2C controller
#define NES_CONTROLLER_INIT_DELAY_MS    1000u
#define NES_CONTROLLER_REINIT_DELAY_MS  1000u
#define SDA_PIN                         2
#define SCL_PIN                         3
#define I2C_ADDRESS                     0x52
i2c_inst_t* i2cHandle = i2c1;
#else
// NES Controller Pins -- for shift register controller
#define PIN_NES_LATCH                   0
#define PIN_NES_PULSE                   1
#define PIN_NES_DATA                    5
#endif

#define SPLASH_DURATION_MS          3000u

//********************************************************************************
// TYPEDEFS AND STRUCTS
//********************************************************************************
typedef enum
{
    BUTTON_A = 0,
    BUTTON_B,
    BUTTON_SELECT,
    BUTTON_START,
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_LEFT,
    BUTTON_RIGHT,
    BUTTON_HOME,
    BUTTON_COUNT
} controller_button_t;

typedef enum
{
    OSD_LINE_COLOR_SCHEME = 0,
    OSD_LINE_FRAME_BLENDING,
    OSD_LINE_AUDIO_GAIN,
    OSD_LINE_RESET_DEVICE,
    OSD_LINE_SAVE_SETTINGS,
    OSD_LINE_EXIT,
    OSD_LINE_COUNT
} osd_line_t;

typedef enum
{
    BUTTON_STATE_PRESSED = 0,
    BUTTON_STATE_UNPRESSED
} button_state_t;


typedef enum
{
    SAVE_INDEX_SCHEME = 0,
    SAVE_INDEX_FRAME_BLENDING
} save_position_t;

typedef enum
{
    DISABLE_MASK_NONE = 0,
    DISABLE_MASK_MASS_STORAGE,
    DISABLE_MASK_PICOBOOT
} interface_disable_mask_t;

typedef enum
{
    RESTART_NORMAL = 0,
    RESTART_MASS_STORAGE,
} restart_option_t;


//********************************************************************************
// PRIVATE VARIABLES
//********************************************************************************
// Packed DMA buffers - 4 pixels per byte (2 bits each)
// This is the native format from the Game Boy (2 bits per pixel)
// Used by BOTH 640x480 and 800x600 modes for DMA capture AND display
static uint8_t packed_buffer_0[PACKED_FRAME_SIZE] = {0};
static uint8_t packed_buffer_1[PACKED_FRAME_SIZE] = {0};
static uint8_t packed_buffer_previous[PACKED_FRAME_SIZE] = {0};  // For frame blending

// TMDS encoder handles palette conversion and horizontal scaling
static volatile uint8_t* packed_display_ptr = packed_buffer_0;
// static uint8_t* packed_render_ptr = packed_buffer_1;

// Frame blending - blends previous frame with current for sprite overlay effects
static volatile bool frame_blending_enabled = false;

// Frame blending lookup tables for ultra-fast processing
// Precomputed lookup table for brightening effect (256 bytes)
// Only store_lut is needed; blend calculation is done inline to save 64KB of RAM
static uint8_t store_lut[256];             // What to store for next frame

// PIO video capture
// PIO NOTES:
// - Each PIO instance has a 32 instruction limit
// - Pico allows for 4 state machines per PIO instance
// - the 32 instructions are shared among all state machines in that PIO instance
// PIO1:
// - Video capture PIO uses 31 instructions, so we can't have any more PIO programs on PIO1
// PIO0:
// - DVI uses SM0, SM1, SM2, but all 3 use the same program (instruction count = 2)
// - I think we can use SM3 for DMG buttons PIO

static PIO pio_video = pio1;  // Exclusively for video capture
static uint video_sm = 0;
static uint video_offset = 0;

static uint8_t button_states[BUTTON_COUNT];
static uint8_t button_states_previous[BUTTON_COUNT];

struct dvi_inst dvi0;

// RGB888 palettes - shared by both 640x480 and 800x600 modes
// Store in flash to save RAM
const uint32_t palette__dmg_nso[4] = {
    0x8cad28,  // GB 0 = White (DMG green lightest)
    0x6c9421,  // GB 1 = Light gray
    0x426b29,  // GB 2 = Dark gray
    0x214231   // GB 3 = Black (DMG green darkest)
};

const uint32_t palette__gbp_nso[4] = {
    0xb5c69c,  // GB 0 = White (GBP lightest)
    0x8d9c7b,  // GB 1 = Light gray
    0x6c7251,  // GB 2 = Dark gray
    0x303820   // GB 3 = Black (GBP darkest)
};

const uint32_t* game_palette_rgb888 = palette__gbp_nso;

uint8_t line_buffer[DMG_PIXELS_X / 4] = {0};  // 40 bytes for 160 pixels packed

#if ENABLE_AUDIO
// configuration
const struct analog_microphone_config mic_config = {
    // GPIO to use for input, must be ADC compatible (GPIO 26 - 28)
    .gpio = PIN_AUDIO_IN,

    // bias voltage of microphone in volts
    .bias_voltage = 1.65f,

    // sample rate in Hz - match HDMI audio
    .sample_rate = SAMPLE_FREQ,

    // ADC fills 128 samples every 4ms, timer reads 128 samples every 4ms
    // Cuts DMA IRQ frequency vs the original 64-sample cadence without long-latency buzz
    .sample_buffer_size = ADC_CHUNK_SIZE,
#if PICO_RP2350 && AUDIO_ON_CORE1
    .dma_irq = DMA_IRQ_3    // POC - not really needed
#else
    .dma_irq = -1  // Use default DMA IRQ
#endif
};
// variables - use double buffering to avoid race conditions
int16_t sample_buffer_a[ADC_CHUNK_SIZE];
int16_t sample_buffer_b[ADC_CHUNK_SIZE];
volatile int16_t* adc_write_buffer = sample_buffer_a;  // ADC writes here
volatile int16_t* adc_read_buffer = sample_buffer_b;   // timer reads from here

// Fixed buffer that audio always reads from (ADC copies to this)
int16_t sample_buffer_for_audio[ADC_CHUNK_SIZE];
#endif // ENABLE_AUDIO

static volatile bool dma_irq_ready_core1 = false;

static restart_option_t restart_option = RESTART_NORMAL;

// Duplicated from tmds_encode.c
static const __unused uint32_t __scratch_x("tmds_table") tmds_table[] = {
#include "tmds_table.h"
};

// Adafruit HSTX
static const struct dvi_serialiser_cfg adafruit_hstx_cfg = {
	.pio = DVI_DEFAULT_PIO_INST,
	.sm_tmds = {0, 1, 2},
	.pins_tmds = {18, 16, 12},
	.pins_clk = 14,
	.invert_diffpairs = false
};

//********************************************************************************
// PRIVATE FUNCTION PROTOTYPES
//********************************************************************************
static void core1_main(void);
static void __no_inline_not_in_flash_func(prepare_scanline_2bpp_gameboy)(struct dvi_inst *inst, const uint8_t *packed_scanbuf);
static void __not_in_flash_func(tmds_encode_2bpp_packed_gameboy)(const uint8_t *packed_pixbuf,
                                    uint32_t *symbuf_r,
                                    uint32_t *symbuf_g,
                                    uint32_t *symbuf_b,
                                    size_t output_words,
                                    uint32_t horizontal_repeat,
                                    size_t input_pixels,
                                    const uint32_t *palette_rgb888);
static void __no_inline_not_in_flash_func(core1_scanline_callback)(uint scanline);
static void init_frame_blending_luts(void);
static void set_game_palette(int index);
static void initialize_gpio(void);
static bool __no_inline_not_in_flash_func(game_controller)(void);
static bool button_is_pressed(controller_button_t button);
static bool button_was_released(controller_button_t button);
static bool command_check(void);
static void button_state_save_previous(void);
static void reset_button_states(void);
static void save_settings(void);
static void reset_pico(restart_option_t restart_option);
static void load_settings(void);

#if ENABLE_AUDIO
static void on_analog_samples_ready(void);
// static void __not_in_flash_func(on_analog_samples_ready)(void);
#endif // ENABLE_AUDIO

static void __no_inline_not_in_flash_func(gpio_callback)(uint gpio, uint32_t events);
static void update_osd(void);
static void change_audio_gain(float delta);

//********************************************************************************
// PRIVATE FUNCTIONS
//********************************************************************************
static void core1_main(void)
{
    // Initialize shared DMA handler on Core 1 so audio DMA IRQs stay off Core 0
    // use dma_irq from config
    SHARED_DMA_Init(mic_config.dma_irq);
    printf("Shared DMA IRQ handler installed on Core 1\n");

    dma_irq_ready_core1 = true;
    // Notify core0 that DMA IRQ handler is armed
    multicore_fifo_push_blocking(0xDACEB00C);  // arbitrary value, just a signal
    dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
    dvi_start(&dvi0);
    
#if ENABLE_AUDIO && AUDIO_ON_CORE1
    absolute_time_t next_audio_tick = delayed_by_ms(get_absolute_time(), AUDIO_TICK_MS);
#endif

    while (true)
    {
        const uint8_t *scanbuf = NULL;
        if (queue_try_remove_u32(&dvi0.q_colour_valid, (uint32_t*)&scanbuf))
        {
            prepare_scanline_2bpp_gameboy(&dvi0, scanbuf);
            queue_add_blocking_u32(&dvi0.q_colour_free, (uint32_t*)&scanbuf);
        }

#if ENABLE_AUDIO && AUDIO_ON_CORE1
        // Run audio chunking on Core 1 to reduce contention with VSYNC GPIO handling on Core 0
        if (time_reached(next_audio_tick)) 
        {
            emu_audio_manual_tick();
            next_audio_tick = delayed_by_ms(next_audio_tick, AUDIO_TICK_MS);
        }
#endif
    }
}

static void __no_inline_not_in_flash_func(prepare_scanline_2bpp_gameboy)(struct dvi_inst *inst, const uint8_t *packed_scanbuf)
{
    static uint scanline_idx = 0;

    uint32_t *tmdsbuf = NULL;
    queue_remove_blocking_u32(&inst->q_tmds_free, &tmdsbuf);
    uint pixwidth = inst->timing->h_active_pixels;             // e.g., 800
    uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;  // e.g., 400 when SPW=2

    static const uint32_t default_palette[4] = {
        0xb5c69c, 0x8d9c7b, 0x6c7251, 0x303820
    };
    const uint32_t *palette = (const uint32_t*)inst->blank_settings.palette_rgb888;
    if (palette == NULL) {
        palette = default_palette;
    }

    const uint current_scanline = scanline_idx;
    scanline_idx = (scanline_idx + 1) % SCANLINE_COUNT;

    const bool in_active_window =
        (current_scanline >= VERTICAL_OFFSET) &&
        (current_scanline < (DMG_PIXELS_Y + VERTICAL_OFFSET));

    if (!in_active_window || packed_scanbuf == NULL)
    {
        // Force full black using TMDS zero symbol (not palette-derived) for porches/blank lines
        const uint32_t black_word = tmds_table[0];
        for (uint word_idx = 0; word_idx < words_per_channel; ++word_idx) {
            tmdsbuf[2 * words_per_channel + word_idx] = black_word;
            tmdsbuf[1 * words_per_channel + word_idx] = black_word;
            tmdsbuf[0 * words_per_channel + word_idx] = black_word;
        }
    } 
    else 
    {
        tmds_encode_2bpp_packed_gameboy(
            packed_scanbuf,
            tmdsbuf + 2 * words_per_channel,  // Red
            tmdsbuf + 1 * words_per_channel,  // Green
            tmdsbuf + 0 * words_per_channel,  // Blue
            words_per_channel,
            HORIZONTAL_SCALE,
            DMG_PIXELS_X,
            palette);
    }

    queue_add_blocking_u32(&inst->q_tmds_valid, &tmdsbuf);
}
                                     
// Flexible 2bpp packed encoder with runtime RGB888 palette support
// Input: packed 2bpp data (4 pixels per byte, 40 bytes = 160 pixels per scanline)
// Output: RGB TMDS symbols with centered 4× horizontal scaling (160→640 pixels) + optional borders
// Works for any resolution: calculates borders automatically based on output_words
//   - 640x480: output_words=320 → 0px borders, 640 game pixels (160×4)
//   - 800x600: output_words=400 → 80px borders each side, 640 game pixels (160×4)
// With DVI_SYMBOLS_PER_WORD=2: pixels = output_words × 2
// 2bpp packed encoder for 800x600 resolution with RGB888 palette support
// Input: 40 bytes (160 GB pixels packed as 2bpp)
// Output: 800 pixels (80 black border + 640 game area + 80 black border)
// With DVI_SYMBOLS_PER_WORD=2: 800 pixels = 400 words per channel
static void __not_in_flash_func(tmds_encode_2bpp_packed_gameboy)(
    const uint8_t *packed_pixbuf,    // Input: packed pixels (e.g., 40 bytes = 160 pixels)
    uint32_t *symbuf_r,              // Output: Red channel TMDS symbols
    uint32_t *symbuf_g,              // Output: Green channel TMDS symbols
    uint32_t *symbuf_b,              // Output: Blue channel TMDS symbols
    size_t output_words,             // Number of output words per channel (pixels = output_words × DVI_SYMBOLS_PER_WORD)
    uint32_t horizontal_repeat,      // Horizontal scale factor (e.g., 4 for x4, 2 for x2)
    size_t input_pixels,             // Number of source pixels in the line (e.g., 160)
    const uint32_t *palette_rgb888   // Palette: 4 RGB888 colors (0xRRGGBB format)
)
{
    // Build TMDS symbol lookup tables from RGB888 palette at runtime
    // This happens once per scanline, but it's only 12 lookups total (4 colors × 3 channels)
    uint32_t tmds_palette_red[4];
    uint32_t tmds_palette_green[4];
    uint32_t tmds_palette_blue[4];

    for (int i = 0; i < 4; i++)
    {
        uint32_t color = palette_rgb888[i];
        uint8_t r8 = (color >> 16) & 0xFF;
        uint8_t g8 = (color >> 8) & 0xFF;
        uint8_t b8 = color & 0xFF;
        
        // Convert 8-bit to 6-bit indices for tmds_table lookup
        tmds_palette_red[i]   = tmds_table[r8 >> 2];
        tmds_palette_green[i] = tmds_table[g8 >> 2];
        tmds_palette_blue[i]  = tmds_table[b8 >> 2];
    }
  
    // Get black color for borders (darkest color in palette)
    const uint32_t black_word = tmds_table[0];
  
    const uint8_t *src = packed_pixbuf;
    const size_t packed_bytes = input_pixels / 4;  // 4 pixels per packed byte
    const uint32_t words_per_pixel = horizontal_repeat / DVI_SYMBOLS_PER_WORD;  // each word = 2 pixels
  
  // Calculate horizontal layout based on output_words
    // Compute active area from input pixel count and horizontal repeat factor
    const size_t game_words = (input_pixels * horizontal_repeat) / DVI_SYMBOLS_PER_WORD;
  
    // Calculate border width on each side (centered)
    // For 640x480: output_words=320, border=0
    // For 800x600: output_words=400, border=40 words (80 pixels)
    size_t border_words = (output_words > game_words) ? (output_words - game_words) / 2 : 0;
    
    size_t word_idx = 0;

    // LEFT BORDER (if any)
    for (size_t i = 0; i < border_words; i++)
    {
        symbuf_r[word_idx] = black_word;
        symbuf_g[word_idx] = black_word;
        symbuf_b[word_idx] = black_word;
        word_idx++;
    }
  
    // GAME AREA: input_pixels × horizontal_repeat
    // Process each input byte (contains 4 packed pixels)
    // Each pixel gets replicated horizontal_repeat× for horizontal scaling
    for (size_t byte_idx = 0; byte_idx < packed_bytes; byte_idx++) 
    {
        uint8_t packed_byte = src[byte_idx];
      
        // Extract and process each of the 4 pixels in this byte
        for (int pixel_in_byte = 0; pixel_in_byte < 4; pixel_in_byte++)
        {
            // Extract 2-bit pixel value (MSB first: bits 7-6, 5-4, 3-2, 1-0)
            uint shift = (3 - pixel_in_byte) * 2;
            uint8_t pixel_2bpp = (packed_byte >> shift) & 0x03;
            
            // Get TMDS symbol pair for this color
            uint32_t word_r = tmds_palette_red[pixel_2bpp];
            uint32_t word_g = tmds_palette_green[pixel_2bpp];
            uint32_t word_b = tmds_palette_blue[pixel_2bpp];
            
            // Replicate this pixel horizontally: two TMDS symbols per word
            for (uint32_t repeat = 0; repeat < words_per_pixel; repeat++)
            {
                symbuf_r[word_idx] = word_r;
                symbuf_g[word_idx] = word_g;
                symbuf_b[word_idx] = word_b;
                word_idx++;
            }
        }
    }
  
    // RIGHT BORDER: Fill remaining words with black
    while (word_idx < output_words)
    {
        symbuf_r[word_idx] = black_word;
        symbuf_g[word_idx] = black_word;
        symbuf_b[word_idx] = black_word;
        word_idx++;
    }
}

static void __no_inline_not_in_flash_func(core1_scanline_callback)(uint scanline)
{
    const bool in_active_window =
        (scanline >= VERTICAL_OFFSET) && (scanline < (DMG_PIXELS_Y + VERTICAL_OFFSET));

    const uint8_t* packed_fb = (const uint8_t*)packed_display_ptr;
    const uint32_t *bufptr = NULL;
    if (in_active_window && (packed_fb != NULL))
    {
        uint dmg_line_idx = scanline - VERTICAL_OFFSET;
        const uint8_t* packed_line = packed_fb + (dmg_line_idx * DMG_PIXELS_X / 4);  // 40 bytes per line
        memcpy(line_buffer, packed_line, sizeof(line_buffer));  // Copy 40 bytes
        bufptr = (uint32_t*)line_buffer;
    }

    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
    
    while (queue_try_remove_u32(&dvi0.q_colour_free, &bufptr))
    ;
}

// Initialize frame blending lookup tables for ultra-fast processing
// Called once at startup to precompute all 256×256 byte combinations
// This implements the exact logic from old_code.c:
//   Blend: new_value == 0 ? new_value|*pixel_old : new_value
//   Store: new_value > 0 ? 2 : 0  (brightens ghosts by storing gray)
// Used by BOTH 640x480 and 800x600 modes
static void init_frame_blending_luts(void)
{
    // Build the store_lut first (what to save for next frame's ghost)
    // This implements: *pixel_old++ = new_value > 0 ? 2 : 0;
    // For each byte, convert: non-white pixels → gray (2), white → white (0)
    // The "2" (gray) value creates the "brightened" ghost effect
    for (int curr = 0; curr < 256; curr++) {
        uint8_t result = 0;
        for (int pixel = 0; pixel < 4; pixel++) {
            int shift = (3 - pixel) * 2;
            uint8_t p = (curr >> shift) & 0x03;
            // Non-white (1,2,3) becomes gray (2), white (0) stays white (0)
            // This is the "brighten up the previous frame" logic!
            uint8_t store_p = (p > 0) ? 2 : 0;
            result |= (store_p << shift);
        }
        store_lut[curr] = result;
    }
      // Note: blend_lut removed to save 64KB RAM (65,536 bytes)
    // Blending is now calculated inline in the frame processing loop
}

// Palette support for both 640x480 and 800x600 modes
static void set_game_palette(int index)
{
    set_scheme_index(index);
    game_palette_rgb888 = (uint32_t*)get_scheme();

    // Set RGB888 palette pointer for 2bpp palette mode
    // Works for both 640x480 (no borders) and 800x600 (with borders)
    dvi_get_blank_settings(&dvi0)->palette_rgb888 = game_palette_rgb888;
}

static void initialize_gpio(void)
{    
    //Onboard LED
    gpio_init(ONBOARD_LED_PIN);
    gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);
    gpio_put(ONBOARD_LED_PIN, 0);

    // Gameboy video signal inputs
    gpio_init(VSYNC_PIN);
    gpio_init(PIXEL_CLOCK_PIN);
    gpio_init(DATA_0_PIN);
    gpio_init(DATA_1_PIN);
    gpio_init(HSYNC_PIN);
    gpio_set_dir(VSYNC_PIN, GPIO_IN);
    gpio_set_dir(PIXEL_CLOCK_PIN, GPIO_IN);
    gpio_set_dir(HSYNC_PIN, GPIO_IN);

#if USE_NES_CLASSIC_OR_WII
    //Initialize I2C port at 400 kHz
    i2c_init(i2cHandle, 400 * 1000);

    // Initialize I2C pins
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);
#else
    // Configure GPIO for actual NES controller
    /* Clock, normally HIGH */
    gpio_init(PIN_NES_PULSE);
    gpio_set_dir(PIN_NES_PULSE, GPIO_OUT);
    gpio_put(PIN_NES_PULSE, 1);

    /* Latch, normally LOW */
    gpio_init(PIN_NES_LATCH);
    gpio_set_dir(PIN_NES_LATCH, GPIO_OUT);
    gpio_put(PIN_NES_LATCH, 0);

    /* Data, reads normally high */
    gpio_init(PIN_NES_DATA);
    gpio_set_dir(PIN_NES_DATA, GPIO_IN);
    // Optionally add pullup
    gpio_pull_up(PIN_NES_DATA);
#endif

    gpio_init(DMG_OUTPUT_RIGHT_A_PIN);
    gpio_set_dir(DMG_OUTPUT_RIGHT_A_PIN, GPIO_OUT);
    gpio_put(DMG_OUTPUT_RIGHT_A_PIN, 1);

    gpio_init(DMG_OUTPUT_LEFT_B_PIN);
    gpio_set_dir(DMG_OUTPUT_LEFT_B_PIN, GPIO_OUT);
    gpio_put(DMG_OUTPUT_LEFT_B_PIN, 1);

    gpio_init(DMG_OUTPUT_UP_SELECT_PIN);
    gpio_set_dir(DMG_OUTPUT_UP_SELECT_PIN, GPIO_OUT);
    gpio_put(DMG_OUTPUT_UP_SELECT_PIN, 1);

    gpio_init(DMG_OUTPUT_DOWN_START_PIN);
    gpio_set_dir(DMG_OUTPUT_DOWN_START_PIN, GPIO_OUT);
    gpio_put(DMG_OUTPUT_DOWN_START_PIN, 1);

    gpio_init(DMG_READING_DPAD_PIN);
    gpio_set_dir(DMG_READING_DPAD_PIN, GPIO_IN);

    gpio_init(DMG_READING_BUTTONS_PIN);
    gpio_set_dir(DMG_READING_BUTTONS_PIN, GPIO_IN);
}

#if USE_NES_CLASSIC_OR_WII
static bool __no_inline_not_in_flash_func(game_controller)(void)
{
    static uint32_t last_micros = 0;
    static bool initialized = false;
    static uint8_t i2c_buffer[8] = {0};
    static absolute_time_t next_init_time = {0};
    static bool waiting_for_init = false;
    static uint32_t pending_init_delay_ms = NES_CONTROLLER_INIT_DELAY_MS;
    static bool controller_armed = false; // don't report until we've seen a clean idle

    uint32_t current_micros = time_us_32();
    if (current_micros - last_micros < 5000)   // NES Classic queries about every 5ms
        return false;

    if (!initialized)
    {
        absolute_time_t now = get_absolute_time();
        if (!waiting_for_init)
        {
            next_init_time = delayed_by_us(now, pending_init_delay_ms * 1000u);
            waiting_for_init = true;
            return false;
        }

        if (time_reached(next_init_time) == false)
        {
            return false;
        }

        waiting_for_init = false;
        pending_init_delay_ms = NES_CONTROLLER_REINIT_DELAY_MS;
        controller_armed = false;

        i2c_buffer[0] = 0xF0;
        i2c_buffer[1] = 0x55;
        (void)i2c_write_blocking(i2cHandle, I2C_ADDRESS, i2c_buffer, 2, false);

        i2c_buffer[0] = 0xFB;
        i2c_buffer[1] = 0x00;
        (void)i2c_write_blocking(i2cHandle, I2C_ADDRESS, i2c_buffer, 2, false);

        initialized = true;
        last_micros = time_us_32();

        reset_button_states();

        return false;
    }

    last_micros = current_micros;

    i2c_buffer[0] = 0x00;
    (void)i2c_write_blocking(i2cHandle, I2C_ADDRESS, i2c_buffer, 1, false);   // false - finished with bus
    sleep_us(300);  // NES Classic uses about 330uS

    // Clear buffer to avoid stale bits if the device returns fewer bytes.
    for (int j = 0; j < 8; ++j) {
        i2c_buffer[j] = 0xFF;
    }

    // Get button data - only read 6 bytes
    int ret = i2c_read_blocking(i2cHandle, I2C_ADDRESS, i2c_buffer, 6, false);
    if (ret < 0)
    {
        last_micros = time_us_32();
        return false;
    }
        
    bool valid = false;
    uint8_t i;
    // Validate the buffer - Check the first 4 bytes
    for (i = 0; i < 4; i++)
    {
        if (i2c_buffer[i] != 0xFF)
            valid = true;
    }

    if (valid)
    {
        // Reject frames that are clearly bogus: all zeros on payload bytes.
        const uint8_t b4 = i2c_buffer[4];
        const uint8_t b5 = i2c_buffer[5];
        if (b4 == 0x00 && b5 == 0x00) {
            return false;
        }

        button_states[BUTTON_START] = BIT_IS_CLEAR(b4, 2) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;
        button_states[BUTTON_SELECT] = BIT_IS_CLEAR(b4, 4) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;
        button_states[BUTTON_DOWN] = BIT_IS_CLEAR(b4, 6) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;
        button_states[BUTTON_RIGHT] = BIT_IS_CLEAR(b4, 7) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;
        button_states[BUTTON_HOME] = BIT_IS_CLEAR(b4, 3) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;

        button_states[BUTTON_UP] = BIT_IS_CLEAR(b5, 0) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;
        button_states[BUTTON_LEFT] = BIT_IS_CLEAR(b5, 1) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;
        button_states[BUTTON_A] = BIT_IS_CLEAR(b5, 4) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;
        button_states[BUTTON_B] = BIT_IS_CLEAR(b5, 6) ? BUTTON_STATE_PRESSED : BUTTON_STATE_UNPRESSED;

#ifdef DEBUG_BUTTON_PRESS
        static uint8_t button_states_debug[BUTTON_COUNT] = {0};
        if (memcmp(button_states, button_states_debug, sizeof(button_states)) != 0)
        {
            memcpy(button_states_debug, button_states, sizeof(button_states));
            printf("Button states:  ");
            for (uint8_t i = 0; i < BUTTON_COUNT; i++)
                printf("%d ", button_states_debug[i]);
            printf("\n");
        }
#endif

    }

    if (!valid )
    {
        initialized = false;
        waiting_for_init = false;
        pending_init_delay_ms = NES_CONTROLLER_REINIT_DELAY_MS;
        controller_armed = false;
        last_micros = time_us_32();
        return false;
    }

    bool any_pressed = false;
    for (i = 0; i < BUTTON_COUNT; i++)
    {
        if (button_states[i] == BUTTON_STATE_PRESSED)
        {
            any_pressed = true;
            break;
        }
    }
    gpio_put(ONBOARD_LED_PIN, any_pressed);


    if (!controller_armed)
    {
        if (any_pressed)
        {
            reset_button_states();
            printf("Waiting for clean button report (no presses)...\n");
            return false;
        }

        controller_armed = true;
        printf("Controller armed\n");
    }


    return true;
}
#else
// Using NES original controller!
static bool __no_inline_not_in_flash_func(game_controller)(void)
{
    static uint32_t last_micros = 0;
    uint32_t current_micros = time_us_32();
    if (current_micros - last_micros < 20000)
        return false;

    last_micros = current_micros;

    gpio_put(PIN_NES_LATCH, 1);
    sleep_us(5);
    gpio_put(PIN_NES_LATCH, 0);
    sleep_us(1);
    button_states[0] = gpio_get(PIN_NES_DATA) ? BUTTON_STATE_UNPRESSED : BUTTON_STATE_PRESSED;
    sleep_us(4);

    for (uint i = 1; i < 8; i++) 
    {
        sleep_us(8);
        gpio_put(PIN_NES_PULSE, 0);
        sleep_us(1);
        gpio_put(PIN_NES_PULSE, 1);
        sleep_us(8);
        button_states[i] = gpio_get(PIN_NES_DATA) ? BUTTON_STATE_UNPRESSED : BUTTON_STATE_PRESSED;
    }

    // Debounce/arm: require several consecutive idle frames before honoring input.
    bool any_pressed = false;
    for (uint i = 0; i < BUTTON_COUNT; i++)
    {
        if (button_states[i] == BUTTON_STATE_PRESSED)
        {
            any_pressed = true;
            break;
        }
    }
    gpio_put(ONBOARD_LED_PIN, any_pressed);

    return true;
}
#endif

static bool button_is_pressed(controller_button_t button)
{
    return button_states[button] == BUTTON_STATE_PRESSED;
}

static bool button_was_released(controller_button_t button)
{
    return button_states[button] == BUTTON_STATE_UNPRESSED && button_states_previous[button] == BUTTON_STATE_PRESSED;
}

static bool command_check(void)
{
    // No change in states, just return false
    if (memcmp(button_states, button_states_previous, sizeof(button_states)) == 0)
        return false;

    // // Ignore any button presses for the first 5 seconds
    // if (time_us_32() < 5000000)
    //     return false;

    bool result = false;

    if (button_is_pressed(BUTTON_SELECT))
    {
        // SELECT + START - TODO: Change to OSD menu
        if (button_was_released(BUTTON_START))
        {
            result = true;
            printf("Hotkey: SELECT+START\n");
            OSD_toggle();
        }
    }
    else
    {
        // select not pressed

        if (button_was_released(BUTTON_HOME))
        {
            result = true;
#if HOME_RESETS_TO_BOOTLOADER
            // HOME - reset into USB mass storage mode for easier programming
            reset_pico(RESTART_MASS_STORAGE);
#else
            OSD_toggle();
#endif
        }
        else
        {
            if (OSD_is_enabled())
            {
                if (button_was_released(BUTTON_DOWN))
                {
                    result = true;
                    OSD_change_active_line(1);
                }
                else if (button_was_released(BUTTON_UP))
                {
                    result = true;
                    OSD_change_active_line(-1);
                }
                else if (button_was_released(BUTTON_RIGHT) 
                        || button_was_released(BUTTON_LEFT)
                        || button_was_released(BUTTON_A))
                {
                    result = true;
                    controller_button_t button = button_was_released(BUTTON_A) ? BUTTON_A : button_was_released(BUTTON_RIGHT) ? BUTTON_RIGHT : BUTTON_LEFT;
                    switch (OSD_get_active_line())
                    {
                        case OSD_LINE_COLOR_SCHEME:
                            set_game_palette(button == BUTTON_RIGHT ? get_scheme_index() + 1 : get_scheme_index() - 1);
                            update_osd();
                            break;
                        case OSD_LINE_FRAME_BLENDING:
                            frame_blending_enabled = !frame_blending_enabled;
                            printf("Frame blending: %s\n", frame_blending_enabled ? "ENABLED" : "DISABLED");
                            if (!frame_blending_enabled) {
                                // Clear previous frame buffer when disabling
                                memset(packed_buffer_previous, 0x00, PACKED_FRAME_SIZE);  // 0x00 = all white pixels
                            }

                            update_osd();
                            break;
                        case OSD_LINE_AUDIO_GAIN:
                            if (button == BUTTON_RIGHT)
                            {
                                change_audio_gain(0.5f);
                            }
                            else
                            {
                                change_audio_gain(-0.5f);
                            }
                            update_osd();
                            break;
                        case OSD_LINE_RESET_DEVICE:
                            if (button == BUTTON_A)
                            {
                                reset_pico(restart_option);
                            }
                            else
                            {
                                restart_option = restart_option == RESTART_NORMAL ? RESTART_MASS_STORAGE : RESTART_NORMAL;
                                update_osd();
                            }
                            break;
                        case OSD_LINE_SAVE_SETTINGS:
                            if (button == BUTTON_A)    
                                save_settings();
                            break;
                        case OSD_LINE_EXIT:
                            if (button == BUTTON_A)
                                OSD_toggle();
                            break;
                    }
                }
                else if (button_was_released(BUTTON_B))
                {
                    result = true;
                    OSD_toggle();
                }
            }
        }
    }

    return result;
}

static void button_state_save_previous(void)
{
    for (int i = 0; i < BUTTON_COUNT; i++) 
    {
        button_states_previous[i] = button_states[i];
    }
}

static void reset_button_states(void)
{
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        button_states[i] = BUTTON_STATE_UNPRESSED;
        button_states_previous[i] = BUTTON_STATE_UNPRESSED;
    }
}

static void save_settings(void)
{
    eeprom_result_t result;
    result = EEPROM_write(SAVE_INDEX_SCHEME, get_scheme_index());
    if (result == EEPROM_SUCCESS)
    {
        // We have to disable DVI to safely commit EEPROM changes
        dvi_serialiser_enable(&dvi0.ser_cfg, false);
        result = EEPROM_commit();
        dvi_serialiser_enable(&dvi0.ser_cfg, true);

        if (result == EEPROM_SUCCESS)
        {
            printf("Saved scheme index to EEPROM: %d\n", get_scheme_index());
        }
        else
        {
            printf("Failed to commit scheme index to EEPROM: %d\n", get_scheme_index());
        }
    }
    else
    {
        printf("Failed to save scheme index to EEPROM: %d\n", get_scheme_index());
    }
}

static void reset_pico(restart_option_t restart_option)
{
    if (restart_option == RESTART_NORMAL)
    {
        // Reset the Pico
        printf("Resetting Pico...\n");
        watchdog_reboot(0, 0, 0);
    }
    else
    {
        // Reset into USB boot mode
        printf("Resetting Pico into USB boot mode...\n");
        reset_usb_boot(0, DISABLE_MASK_NONE);
    }

    while (1)
    {
        tight_loop_contents();
    }
}


static void load_settings(void)
{
    uint8_t scheme = (uint8_t)SCHEME_SGB_4H;
    if (EEPROM_read(SAVE_INDEX_SCHEME, &scheme) == EEPROM_SUCCESS)
    {
        printf("Loaded palette from EEPROM: %d\n", scheme);
    }
    else
    {
        printf("Failed to load palette from EEPROM, using default: %d\n", scheme);
    }
    set_game_palette((int)scheme);

    // set_scheme_index((int)EEPROM_read(SAVE_INDEX_SCHEME));
    // frame_blending_enabled = EEPROM_read(SAVE_INDEX_FRAME_BLENDING) == 1;
}

#if ENABLE_AUDIO
// static void __not_in_flash_func(on_analog_samples_ready)(void)
static void on_analog_samples_ready(void)
{

    // ADC has filled the write buffer with fresh samples
    // Read them into the current write buffer
    int samples_read = analog_microphone_read((int16_t*)adc_write_buffer, ADC_CHUNK_SIZE);

    #ifndef AUDIO_DEBUG
    (void)samples_read;
    #endif
    
    // Copy the just-read samples to the fixed audio buffer.
    memcpy(sample_buffer_for_audio, (void*)adc_write_buffer, ADC_CHUNK_SIZE * sizeof(int16_t));
    
    // Atomically swap buffers - get ready for next ADC capture
    volatile int16_t* temp = adc_write_buffer;
    adc_write_buffer = adc_read_buffer;
    adc_read_buffer = temp;
    
    // Debug output every 1000 callbacks (enable by defining AUDIO_DEBUG)
    #ifdef AUDIO_DEBUG
    static uint32_t callback_count = 0;
    if (++callback_count % 1000 == 0) {
        printf("ADC callback #%lu: read %d samples, sample[0]=%d, sample_buffer_for_audio[32]=%d\n",
               callback_count, samples_read, sample_buffer_for_audio[0], sample_buffer_for_audio[32]);
    }
    #endif
}
#endif

static void __no_inline_not_in_flash_func(gpio_callback)(uint gpio, uint32_t events)
{
#if ENABLE_VIDEO_CAPTURE
    // Handle VSYNC IRQ for video capture (GPIO 7) - ONLY IN IRQ MODE
    if (gpio == VSYNC_PIN) 
    {
        video_capture_handle_vsync_irq(events);
        return;  // VSYNC handled, done
    }
#endif

    // Prevent controller input to game if OSD is visible
#if ENABLE_OSD
    if (OSD_is_enabled())
        return;
#endif // ENABLE_OSD

    if(gpio==DMG_READING_DPAD_PIN)
    {
        if (events & GPIO_IRQ_EDGE_FALL)   // Send DPAD states on low
        {
            if (gpio_get(DMG_READING_BUTTONS_PIN) == 1)
            {

                gpio_put(DMG_OUTPUT_RIGHT_A_PIN, button_states[BUTTON_RIGHT]);
                gpio_put(DMG_OUTPUT_LEFT_B_PIN, button_states[BUTTON_LEFT]);
                gpio_put(DMG_OUTPUT_UP_SELECT_PIN, button_states[BUTTON_UP]);
                gpio_put(DMG_OUTPUT_DOWN_START_PIN, button_states[BUTTON_DOWN]);
            }
        }

        if (events & GPIO_IRQ_EDGE_RISE)   // Send BUTTON states on low
        {
            // When P15 pin goes high, read cycle is complete, send all high
            if(gpio_get(DMG_READING_BUTTONS_PIN) == 1)
            {
                gpio_put(DMG_OUTPUT_RIGHT_A_PIN, 1);
                gpio_put(DMG_OUTPUT_LEFT_B_PIN, 1);
                gpio_put(DMG_OUTPUT_UP_SELECT_PIN, 1);
                gpio_put(DMG_OUTPUT_DOWN_START_PIN, 1);
            }
        }
    }

    if(gpio==DMG_READING_BUTTONS_PIN)
    {
        if (events & GPIO_IRQ_EDGE_FALL)   // Send BUTTON states on low
        {
            gpio_put(DMG_OUTPUT_RIGHT_A_PIN, button_states[BUTTON_A]);
            gpio_put(DMG_OUTPUT_LEFT_B_PIN, button_states[BUTTON_B]);
            gpio_put(DMG_OUTPUT_UP_SELECT_PIN, button_states[BUTTON_SELECT]);
            gpio_put(DMG_OUTPUT_DOWN_START_PIN, button_states[BUTTON_START]);

            // Prevent in-game reset lockup
            // If A,B,Select and Start are all pressed, release them!
            if ((button_states[BUTTON_A] | button_states[BUTTON_B] | button_states[BUTTON_SELECT]| button_states[BUTTON_START])==0)
            {
                button_states[BUTTON_A] = 1;
                button_states[BUTTON_B] = 1;
                button_states[BUTTON_SELECT] = 1;
                button_states[BUTTON_START] = 1;
            }
        }
    }
}

static void update_osd(void)
{
    char buff[32];
    sprintf(buff, "COLOR SCHEME:%8d", get_scheme_index());
    OSD_set_line_text(OSD_LINE_COLOR_SCHEME, buff);

    // sprintf(buff, "BORDER COLOR:%8d", get_border_color_index());
    // OSD_set_line_text(OSD_LINE_BORDER_COLOR, buff);


    
    float gain = emu_audio_get_gain();
    sprintf(buff, "AUDIO GAIN:%10.1f", gain);
    OSD_set_line_text(OSD_LINE_AUDIO_GAIN, buff);

    sprintf(buff, "FRAME BLEND:%9s", frame_blending_enabled ? "ON" : "OFF");
    OSD_set_line_text(OSD_LINE_FRAME_BLENDING, buff);
    
    sprintf(buff, "RESET DEVICE:%8s", restart_option == RESTART_MASS_STORAGE ? "USB" : "NORM");
    OSD_set_line_text(OSD_LINE_RESET_DEVICE, buff);

    OSD_set_line_text(OSD_LINE_SAVE_SETTINGS, "SAVE SETTINGS");

    OSD_set_line_text(OSD_LINE_EXIT, "EXIT");
}

static void change_audio_gain(float delta)
{
    float gain = emu_audio_get_gain();
    gain += delta;
    emu_audio_set_gain(gain);

    gain = emu_audio_get_gain();  // Read back clamped value
    char buff[32];
    sprintf(buff, "AUDIO GAIN:%5.2f", gain);
    OSD_set_line_text(OSD_LINE_AUDIO_GAIN, buff);
    printf("Audio gain set to %.2f\n", gain);
}

//********************************************************************************
// PUBLIC FUNCTIONS
//********************************************************************************
int main(void)
{
    vreg_set_voltage(VREG_VSEL);
    sleep_ms(10);
    set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);
    reset_button_states();

    // Initialize frame blending lookup tables (one-time computation)
    // Both modes use packed buffers for capture, so both need LUTs
    init_frame_blending_luts();
    
    // Mario image is now pre-packed in 2bpp format
    // Simply copy the packed data directly to the display buffers
    memcpy(packed_buffer_0, mario_packed_160x144, PACKED_FRAME_SIZE);
    memcpy(packed_buffer_1, packed_buffer_0, PACKED_FRAME_SIZE);

    // Both modes use packed buffer directly (TMDS encoder handles palette and scaling)
    packed_display_ptr = packed_buffer_0;
    // TODO packed_render_ptr = packed_buffer_1;

    // Initialize OSD overlays (disabled by default)
    OSD_init(DMG_PIXELS_X, DMG_PIXELS_Y);
    OSD_clear();
    OSD_set_enabled(false);

    dvi0.timing = &DVI_TIMING;
    dvi0.ser_cfg = DVI_SERIAL_CONFIG;
    //dvi0.scanline_callback = (dvi_callback_t*)core1_scanline_callback;
    dvi0.scanline_callback = core1_scanline_callback;
    dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

    load_settings();

    uint32_t *bufptr = (uint32_t*)line_buffer;
    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);
    queue_add_blocking_u32(&dvi0.q_colour_valid, &bufptr);

    // HDMI Audio related
    // Support 16000, 22050, 24000, 32000, 44100, and 48000 Hz sample rates
#if ENABLE_AUDIO
    int offset;
    if (rate == 48000) {
        offset = 2;
    } else if (rate == 44100) {
        offset = 1;
    } else if (rate == 24000) {
        offset = 5;  // 24kHz - well supported
    } else if (rate == 22050) {
        offset = 3;  // Half of 44.1k
    } else if (rate == 16000) {
        offset = 4;  // Half of 32k
    } else {
        offset = 0;  // Default for other rates (32000)
    }
    int cts = dvi0.timing->bit_clk_khz*hdmi_n[offset]/(rate/100)/128;
    dvi_get_blank_settings(&dvi0)->top    = 0;
    dvi_get_blank_settings(&dvi0)->bottom = 0;
    dvi_audio_sample_buffer_set(&dvi0, audio_buffer, AUDIO_BUFFER_SIZE);
    dvi_set_audio_freq(&dvi0, rate, cts, hdmi_n[offset]);
    // Note: dvi_set_audio_freq() automatically calls dvi_enable_data_island()
    
    // Pre-fill buffer to 25% to allow for bursty DVI consumption patterns
    increase_write_pointer(&dvi0.audio_ring, AUDIO_BUFFER_SIZE / 4);
    printf("Audio buffer pre-filled to %d samples (25%%)\n", AUDIO_BUFFER_SIZE / 4);
#endif

    // OPTIMIZED ORDER for audio + video:
    // 1. Start audio timer (500Hz chunk processing)
    // 2. Start Core1 (DVI output starts consuming audio)
    // 3. Initialize ADC microphone
    // 4. Initialize GPIO/PIO for DMG controller
    // 5. Register VSYNC interrupt (video capture begins)

#if ENABLE_AUDIO
    if (AUDIO_ON_CORE1) {
        emu_audio_set_manual_tick(true);
        printf("Audio: manual tick on Core1 every %d ms\n", AUDIO_TICK_MS);
    } else {
        emu_audio_set_manual_tick(false);
        printf("Starting audio timer (500Hz chunk rate)...\n");
    }
	emu_sndInit(false, false, &dvi0.audio_ring, sample_buffer_for_audio);
    emu_audio_set_gain(AUDIO_GAIN);
    emu_audio_set_highpass(AUDIO_HPF_HZ);
    emu_audio_set_lowpass(AUDIO_LPF_HZ);
    printf("Audio system initialized\n");
    
#endif

#if ENABLE_VIDEO_CAPTURE

    printf("Initializing PIO video capture...\n");
    video_offset = pio_add_program(pio_video, &video_capture_irq_program);
    video_capture_program_init(pio_video, video_sm, video_offset);

    // Video uses polled completion; pass -1 to avoid enabling any DMA IRQ
    int video_dma_chan = video_capture_dma_init(pio_video, video_sm, -1, packed_buffer_0, PACKED_FRAME_SIZE);
    if (video_dma_chan < 0)
    {
        printf("ERROR: Video capture DMA initialization failed!\n");
        while (1) { tight_loop_contents(); }
    }
    // Video uses polled completion (no IRQ) to avoid contention
    printf("  -> DMA initialized (packed format: %d bytes, polled completion)\n", PACKED_FRAME_SIZE);
    // stdio_flush();
#endif

    // Always start DVI output on Core 1, regardless of audio
    printf("Starting Core 1 (DVI output)...\n");
    multicore_launch_core1(core1_main);
    printf("Core 1 running - now consuming video!\n");

    // Wait for Core1 to arm DMA IRQ handler before starting analog mic DMA
    if (AUDIO_ON_CORE1) {
        while (!dma_irq_ready_core1) {
            // Also drain FIFO token if core1 already signaled
            if (multicore_fifo_rvalid()) {
                multicore_fifo_pop_blocking();
            }
            tight_loop_contents();
        }
        // Drain any pending FIFO token
        if (multicore_fifo_rvalid()) multicore_fifo_pop_blocking();
    }

#if ENABLE_AUDIO
    printf("Initializing analog microphone on GPIO %d at %d Hz...\n", mic_config.gpio, mic_config.sample_rate);
    if (analog_microphone_init(&mic_config) < 0) {
        printf("ERROR: analog microphone initialization failed!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("Analog microphone initialized successfully\n");

    analog_microphone_set_samples_ready_handler(on_analog_samples_ready);
    printf("ADC callback handler registered\n");

    printf("Starting analog microphone DMA capture...\n");
    if (analog_microphone_start() < 0) {
        printf("ERROR: PDM microphone start failed!\n");
        while (1) { tight_loop_contents(); }
    }
    printf("Analog microphone started successfully - DMA should be running\n");
#endif

    initialize_gpio();

    for (int i = 0; i < 5; i++) {
        printf("\n\n=== PicoDVI-DMG_EMU Starting (attempt %d) ===\n", i + 1);
        sleep_ms(100);
    }

    printf("Firmware build: %s %s\n", __DATE__, __TIME__);

    // Set up shared GPIO callback - this handles BOTH DMG buttons AND VSYNC
    gpio_set_irq_enabled_with_callback(DMG_READING_DPAD_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled(DMG_READING_BUTTONS_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);  // Callback already registered

    // Enable VSYNC interrupt (GPIO 7) after callback registration
#if ENABLE_VIDEO_CAPTURE
    irq_set_priority(IO_IRQ_BANK0, 0x00);  // Max priority to avoid missing VSYNC edges
    gpio_set_irq_enabled(VSYNC_PIN, GPIO_IRQ_EDGE_RISE, true);
    printf("VSYNC GPIO interrupt enabled (rising edge)\n");
#endif

    // Track which packed buffer is being captured (both modes use packed buffers)
    uint8_t* packed_capture = packed_buffer_0;
    // uint8_t* packed_next = packed_buffer_1;
    bool video_capture_active = false;
    bool video_capture_started = false;

    // Hold off controller polling briefly after boot to avoid early contention with VSYNC/audio
    const absolute_time_t controller_poll_enable_time = delayed_by_ms(get_absolute_time(), 1000);
    
    absolute_time_t splash_until = delayed_by_ms(get_absolute_time(), SPLASH_DURATION_MS);
    bool splash_done = false;

    update_osd();

    // Main loop - PIO video + good audio
    while (true) 
    {
        static uint32_t loop_counter = 0;
        static uint32_t frames_captured = 0;

#if ENABLE_VIDEO_CAPTURE

        // Skip arming capture until splash time has elapsed
        if (!splash_done) 
        {
            if (time_reached(splash_until))
            {
                printf("Splash screen done, starting video capture\n");
                splash_done = true;
            }
        }

        if (splash_done) 
        {
            // Start video capture when we're past initial delay and the throttle window has elapsed
            // if (!video_capture_active && (!video_capture_started || time_reached(next_capture_time)))
            if (!video_capture_active && (!video_capture_started || time_reached(splash_until)))
            {
                video_capture_start_frame(pio_video, video_sm, packed_capture, PACKED_FRAME_SIZE);
                video_capture_active = true;
                video_capture_started = true;
            }
        }
        else
        {
            // While showing splash, ensure we don't arm captures
            video_capture_active = false;
            video_capture_started = false;
        }

        if (video_capture_active) 
        {
            // Poll for DMA completion (no IRQ) and then check if frame is ready (non-blocking)
            (void)video_capture_poll_complete();

            if (video_capture_frame_ready()) 
            {
                // Swap capture buffers now and immediately arm next capture to minimize VSYNC miss window
                uint8_t* completed_packed = video_capture_get_frame();
                uint8_t* next_buf = (completed_packed == packed_buffer_0) ? packed_buffer_1 : packed_buffer_0;
                packed_capture = next_buf;
    
                video_capture_start_frame(pio_video, video_sm, packed_capture, PACKED_FRAME_SIZE);
                video_capture_active = true;
                video_capture_started = true;

                // Apply frame blending if enabled (packed 2bpp data)
                if (frame_blending_enabled)
                {
                    // Ultra-fast frame blending using precomputed lookup tables
                    // Logic from old_code.c:
                    //   - Blend: white pixels (0) OR with previous frame
                    //   - Store: non-white pixels become gray (2) to "brighten up the previous frame"                
                    // This creates visible ghost trails that fade after one frame
                    // Blend calculation done inline to save 64KB RAM (instead of blend_lut)
                    
                    for (size_t i = 0; i < PACKED_FRAME_SIZE; i++)
                    {
                        uint8_t current = completed_packed[i];
                        uint8_t previous = packed_buffer_previous[i];

                        // Calculate blended result inline: white (0) pixels OR with previous (ghost effect)
                        uint8_t blended = 0;
                        for (int pixel = 0; pixel < 4; pixel++)
                        {
                            int shift = (3 - pixel) * 2;
                            uint8_t p_curr = (current >> shift) & 0x03;
                            uint8_t p_prev = (previous >> shift) & 0x03;
                            uint8_t p_blend = (p_curr == 0) ? (p_curr | p_prev) : p_curr;
                            blended |= (uint8_t)(p_blend << shift);
                        }
                        completed_packed[i] = blended;

                        // Single lookup for brightened ghost (non-white→gray, white→white)
                        // This matches: *pixel_old++ = new_value > 0 ? 2 : 0;
                        packed_buffer_previous[i] = store_lut[current];
                    }
                }
    
                // Overlay OSD text, if enabled
                OSD_render((uint8_t*)completed_packed);

                // Swap display buffer to the completed frame (packed 2bpp)
                __dmb();
                packed_display_ptr = (volatile uint8_t*)completed_packed;
                __dmb();
    
                frames_captured++;
            }
        }


#endif // ENABLE_VIDEO_CAPTURE

        loop_counter++;
        
        // Poll controller at a low rate to reduce I2C/CPU load that can steal VSYNC time
        static absolute_time_t next_controller_poll = {0};
        absolute_time_t now = get_absolute_time();
        if (time_reached(controller_poll_enable_time) && time_reached(next_controller_poll)) 
        {
            game_controller();
            (void)command_check();
            button_state_save_previous();
            next_controller_poll = delayed_by_ms(now, 5);  // ~200 Hz
        }
    }
    __builtin_unreachable();
}