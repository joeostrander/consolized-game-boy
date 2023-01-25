/* 
    Authors: 
        Andy West (original code)
        Joe Ostrander

    Version: 2.0.0

    https://github.com/joeostrander/consolized-game-boy
*/

#include <stdio.h>
#include <stdlib.h>
#include "time.h"
#include "pico.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/scanvideo.h"
#include "pico/scanvideo/composable_scanline.h"
#include "pico/sync.h"
#include "hardware/vreg.h"
#include "pico/stdio.h"
#include "osd.h"
#include "hardware/i2c.h"

#define SDA_PIN     12
#define SCL_PIN     13
#define I2C_ADDRESS 0x52
i2c_inst_t* i2cHandle = i2c0;

#define VGA_MODE vga_mode_640x480_60
#define MIN_RUN 3

#define ONBOARD_LED_PIN         25

// GAMEBOY VIDEO INPUT (From level shifter)
#define VSYNC_PIN               18
#define HSYNC_PIN               17
#define PIXEL_CLOCK_PIN         16
#define DATA_1_PIN              15
#define DATA_0_PIN              14

#define BUTTONS_DPAD_PIN        19      // P14
#define BUTTONS_OTHER_PIN       20      // P15
#define BUTTONS_LEFT_B_PIN      26      // P11
#define BUTTONS_DOWN_START_PIN  21      // P13
#define BUTTONS_UP_SELECT_PIN   22      // P12
#define BUTTONS_RIGHT_A_PIN     27      // P10

#define GAMEBOY_RESET_PIN       28

#define PIXELS_X                (160)
#define PIXELS_Y                (144)

// Game area will be 480x432 
#define PIXEL_SCALE             (3)
#define BORDER_HORZ             (80)    
#define BORDER_VERT             (24)

#define PIXEL_COUNT             (PIXELS_X*PIXELS_Y)

#define RGB888_TO_RGB222(r, g, b) ((((b)>>6u)<<PICO_SCANVIDEO_PIXEL_BSHIFT)|(((g)>>6u)<<PICO_SCANVIDEO_PIXEL_GSHIFT)|(((r)>>6u)<<PICO_SCANVIDEO_PIXEL_RSHIFT))

static uint8_t border_colors[] = {
    RGB888_TO_RGB222(0x00, 0x00, 0x00), // BLACK
    RGB888_TO_RGB222(0x00, 0x00, 0xFF), // BLUE
    RGB888_TO_RGB222(0xFF, 0xFF, 0xFF), // WHITE
    RGB888_TO_RGB222(0x80, 0x80, 0x80), // LIGHT GREY
    RGB888_TO_RGB222(0x40, 0x40, 0x40), // DARK GREY
    RGB888_TO_RGB222(0xFF, 0x00, 0x00), // RED
    RGB888_TO_RGB222(0x00, 0xFF, 0x00), // GREEN
    RGB888_TO_RGB222(0xFF, 0xFF, 0x00), // YELLOW
    RGB888_TO_RGB222(0xFF, 0x00, 0xFF), // PURPLE
};

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
    VIDEO_EFFECT_NONE = 0,
    VIDEO_EFFECT_PIXEL_EFFECT,
    VIDEO_EFFECT_SCANLINES,
    VIDEO_EFFECT_COUNT
} video_effect_t;

typedef enum
{
    OSD_LINE_COLOR_SCHEME = 0,
    OSD_LINE_BORDER_COLOR,
    OSD_LINE_EFFECTS,
    OSD_LINE_FX_SCHEME,
    OSD_LINE_RESET_GAMEBOY,
    OSD_LINE_EXIT,
    OSD_LINE_COUNT
} osd_line_t;

static semaphore_t video_initted;
static volatile uint8_t button_states[BUTTON_COUNT];
static uint8_t button_states_previous[BUTTON_COUNT];
static volatile uint8_t buttons_state = 0xFF;
static int scheme_offset = 0;
static int scanline_color_offset = 0;
static int video_effect = VIDEO_EFFECT_NONE;

static uint8_t framebuffer[PIXEL_COUNT];
static uint8_t osd_framebuffer[OSD_HEIGHT*OSD_WIDTH] = {0};

// map gb pixel to screen pixel
static uint8_t indexes_x[PIXELS_X*PIXEL_SCALE];
static uint8_t indexes_y[PIXELS_Y*PIXEL_SCALE];

static int8_t border_color_index = 0;
static uint16_t scanline_color = RGB888_TO_RGB222(0x00, 0x00, 0x00);

static uint16_t colors[] = {
    // Black and white
    RGB888_TO_RGB222(0xF7, 0xF3, 0xF7),
    RGB888_TO_RGB222(0xB5, 0xB2, 0xB5),
    RGB888_TO_RGB222(0x4E, 0x4C, 0x4E),
    RGB888_TO_RGB222(0x00, 0x00, 0x00),

    // Inverted
    RGB888_TO_RGB222(0x00, 0x00, 0x00),
    RGB888_TO_RGB222(0x4E, 0x4C, 0x4E),
    RGB888_TO_RGB222(0xB5, 0xB2, 0xB5),
    RGB888_TO_RGB222(0xF7, 0xF3, 0xF7),

    // DMG
    RGB888_TO_RGB222(0x7B, 0x82, 0x10),
    RGB888_TO_RGB222(0x5A, 0x79, 0x42),
    RGB888_TO_RGB222(0x39, 0x59, 0x4A),
    RGB888_TO_RGB222(0x29, 0x41, 0x39),

    // Game Boy Pocket
    RGB888_TO_RGB222(0xC6, 0xCB, 0xA5),
    RGB888_TO_RGB222(0x8C, 0x92, 0x6B),
    RGB888_TO_RGB222(0x4A, 0x51, 0x39),
    RGB888_TO_RGB222(0x18, 0x18, 0x18),

    // Game Boy Light
    RGB888_TO_RGB222(0x00, 0xB2, 0x84),
    RGB888_TO_RGB222(0x8C, 0x92, 0x6B),
    RGB888_TO_RGB222(0x00, 0x69, 0x4A),
    RGB888_TO_RGB222(0x00, 0x51, 0x39),

    // SGB 1A
    RGB888_TO_RGB222(0xF7, 0xE3, 0xC6),
    RGB888_TO_RGB222(0xD6, 0x92, 0x4A),
    RGB888_TO_RGB222(0xA5, 0x28, 0x21),
    RGB888_TO_RGB222(0x31, 0x18, 0x52),

    // SGB 2A
    RGB888_TO_RGB222(0xEF, 0xC3, 0x9C),
    RGB888_TO_RGB222(0xBD, 0x8A, 0x4A),
    RGB888_TO_RGB222(0x29, 0x79, 0x00),
    RGB888_TO_RGB222(0x00, 0x00, 0x00),

    // SGB 3A
    RGB888_TO_RGB222(0xF7, 0xCB, 0x94),
    RGB888_TO_RGB222(0x73, 0xBA, 0xBD),
    RGB888_TO_RGB222(0xF7, 0x61, 0x29),
    RGB888_TO_RGB222(0x31, 0x49, 0x63),

    // SGB 4A
    RGB888_TO_RGB222(0xEF, 0xA2, 0x6B),
    RGB888_TO_RGB222(0x7B, 0xA2, 0xF7),
    RGB888_TO_RGB222(0xCE, 0x00, 0xCE),
    RGB888_TO_RGB222(0x00, 0x00, 0x7B),

    // SGB 1B
    RGB888_TO_RGB222(0xD6, 0xD3, 0xBD),
    RGB888_TO_RGB222(0xC6, 0xAA, 0x73),
    RGB888_TO_RGB222(0xAD, 0x51, 0x10),
    RGB888_TO_RGB222(0x00, 0x00, 0x00),

    // SGB 2B
    RGB888_TO_RGB222(0xF7, 0xF3, 0xF7),
    RGB888_TO_RGB222(0xF7, 0xE3, 0x52),
    RGB888_TO_RGB222(0xF7, 0x30, 0x00),
    RGB888_TO_RGB222(0x52, 0x00, 0x5A),

    // SGB 3B
    RGB888_TO_RGB222(0xD6, 0xD3, 0xBD),
    RGB888_TO_RGB222(0xDE, 0x82, 0x21),
    RGB888_TO_RGB222(0x00, 0x51, 0x00),
    RGB888_TO_RGB222(0x00, 0x10, 0x10),

    // SGB 4B
    RGB888_TO_RGB222(0xEF, 0xE3, 0xEF),
    RGB888_TO_RGB222(0xE7, 0x9A, 0x63),
    RGB888_TO_RGB222(0x42, 0x79, 0x39),
    RGB888_TO_RGB222(0x18, 0x08, 0x08),

    // SGB 1C
    RGB888_TO_RGB222(0xF7, 0xBA, 0xF7),
    RGB888_TO_RGB222(0xE7, 0x92, 0x52),
    RGB888_TO_RGB222(0x94, 0x38, 0x63),
    RGB888_TO_RGB222(0x39, 0x38, 0x94),

    // SGB 2C
    RGB888_TO_RGB222(0xF7, 0xF3, 0xF7),
    RGB888_TO_RGB222(0xE7, 0x8A, 0x8C),
    RGB888_TO_RGB222(0x7B, 0x30, 0xE7),
    RGB888_TO_RGB222(0x29, 0x28, 0x94),

    // SGB 3C
    RGB888_TO_RGB222(0xDE, 0xA2, 0xC6),
    RGB888_TO_RGB222(0xF7, 0xF3, 0x7B),
    RGB888_TO_RGB222(0x00, 0xB2, 0xF7),
    RGB888_TO_RGB222(0x21, 0x20, 0x5A),

    // SGB 4C
    RGB888_TO_RGB222(0xF7, 0xDB, 0xDE),
    RGB888_TO_RGB222(0xF7, 0xF3, 0x7B),
    RGB888_TO_RGB222(0x94, 0x9A, 0xDE),
    RGB888_TO_RGB222(0x08, 0x00, 0x00),

    // SGB 1D
    RGB888_TO_RGB222(0xF7, 0xF3, 0xA5),
    RGB888_TO_RGB222(0xBD, 0x82, 0x4A),
    RGB888_TO_RGB222(0xF7, 0x00, 0x00),
    RGB888_TO_RGB222(0x52, 0x18, 0x00),

    // SGB 2D
    RGB888_TO_RGB222(0xF7, 0xF3, 0x9C),
    RGB888_TO_RGB222(0x00, 0xF3, 0x00),
    RGB888_TO_RGB222(0xF7, 0x30, 0x00),
    RGB888_TO_RGB222(0x00, 0x00, 0x52),

    // SGB 3D
    RGB888_TO_RGB222(0xEF, 0xF3, 0xB5),
    RGB888_TO_RGB222(0xDE, 0xA2, 0x7B),
    RGB888_TO_RGB222(0x96, 0xAD, 0x52),
    RGB888_TO_RGB222(0x00, 0x00, 0x00),

    // SGB 4D
    RGB888_TO_RGB222(0xF7, 0xF3, 0xB5),
    RGB888_TO_RGB222(0x94, 0xC3, 0xC6),
    RGB888_TO_RGB222(0x4A, 0x69, 0x7B),
    RGB888_TO_RGB222(0x08, 0x20, 0x4A),

    // SGB 1E
    RGB888_TO_RGB222(0xF7, 0xD3, 0xAD),
    RGB888_TO_RGB222(0x7B, 0xBA, 0x7B),
    RGB888_TO_RGB222(0x6B, 0x8A, 0x42),
    RGB888_TO_RGB222(0x5A, 0x38, 0x21),

    // SGB 2E
    RGB888_TO_RGB222(0xF7, 0xC3, 0x84),
    RGB888_TO_RGB222(0x94, 0xAA, 0xDE),
    RGB888_TO_RGB222(0x29, 0x10, 0x63),
    RGB888_TO_RGB222(0x10, 0x08, 0x10),

    // SGB 3E
    RGB888_TO_RGB222(0xF7, 0xF3, 0xBD),
    RGB888_TO_RGB222(0xDE, 0xAA, 0x6B),
    RGB888_TO_RGB222(0xAD, 0x79, 0x21),
    RGB888_TO_RGB222(0x52, 0x49, 0x73),

    // SGB 4E
    RGB888_TO_RGB222(0xF7, 0xD3, 0xA5),
    RGB888_TO_RGB222(0xDE, 0xA2, 0x7B),
    RGB888_TO_RGB222(0x7B, 0x59, 0x8C),
    RGB888_TO_RGB222(0x00, 0x20, 0x31),

    // SGB 1F
    RGB888_TO_RGB222(0xD6, 0xE3, 0xF7),
    RGB888_TO_RGB222(0xDE, 0x8A, 0x52),
    RGB888_TO_RGB222(0xA5, 0x00, 0x00),
    RGB888_TO_RGB222(0x00, 0x41, 0x10),

    // SGB 2F
    RGB888_TO_RGB222(0xCE, 0xF3, 0xF7),
    RGB888_TO_RGB222(0xF7, 0x92, 0x52),
    RGB888_TO_RGB222(0x9C, 0x00, 0x00),
    RGB888_TO_RGB222(0x18, 0x00, 0x00),

    // SGB 3F
    RGB888_TO_RGB222(0x7B, 0x79, 0xC6),
    RGB888_TO_RGB222(0xF7, 0x69, 0xF7),
    RGB888_TO_RGB222(0xF7, 0xCB, 0x00),
    RGB888_TO_RGB222(0x42, 0x41, 0x42),

    // SGB 4F
    RGB888_TO_RGB222(0xB5, 0xCB, 0xCE),
    RGB888_TO_RGB222(0xD6, 0x82, 0xD6),
    RGB888_TO_RGB222(0x84, 0x00, 0x9C),
    RGB888_TO_RGB222(0x39, 0x00, 0x00),

    // SGB 1G
    RGB888_TO_RGB222(0x00, 0x00, 0x52),
    RGB888_TO_RGB222(0x00, 0x9A, 0xE7),
    RGB888_TO_RGB222(0x7B, 0x79, 0x00),
    RGB888_TO_RGB222(0xF7, 0xF3, 0x5A),

    // SGB 2G
    RGB888_TO_RGB222(0x6B, 0xB2, 0x39),
    RGB888_TO_RGB222(0xDE, 0x51, 0x42),
    RGB888_TO_RGB222(0xDE, 0xB2, 0x84),
    RGB888_TO_RGB222(0x00, 0x18, 0x00),

    // SGB 3G
    RGB888_TO_RGB222(0x63, 0xD3, 0x52),
    RGB888_TO_RGB222(0xF7, 0xF3, 0xF7),
    RGB888_TO_RGB222(0xC6, 0x30, 0x39),
    RGB888_TO_RGB222(0x39, 0x00, 0x00),

    // SGB 4G
    RGB888_TO_RGB222(0xAD, 0xDB, 0x18),
    RGB888_TO_RGB222(0xB5, 0x20, 0x5A),
    RGB888_TO_RGB222(0x29, 0x10, 0x00),
    RGB888_TO_RGB222(0x00, 0x82, 0x63),

    // SGB 1H
    RGB888_TO_RGB222(0xF7, 0xE3, 0xDE),
    RGB888_TO_RGB222(0xF7, 0xB2, 0x8C),
    RGB888_TO_RGB222(0x84, 0x41, 0x00),
    RGB888_TO_RGB222(0x31, 0x18, 0x00),

    // SGB 2H
    RGB888_TO_RGB222(0xF7, 0xF3, 0xF7),
    RGB888_TO_RGB222(0xB5, 0xB2, 0xB5),
    RGB888_TO_RGB222(0x73, 0x71, 0x73),
    RGB888_TO_RGB222(0x00, 0x00, 0x00),

    // SGB 3H
    RGB888_TO_RGB222(0xDE, 0xF3, 0x9C),
    RGB888_TO_RGB222(0x7B, 0xC3, 0x39),
    RGB888_TO_RGB222(0x4A, 0x8A, 0x18),
    RGB888_TO_RGB222(0x08, 0x18, 0x00),

    // SGB 4H
    RGB888_TO_RGB222(0xF7, 0xF3, 0xC6),
    RGB888_TO_RGB222(0xB5, 0xBA, 0x5A),
    RGB888_TO_RGB222(0x84, 0x8A, 0x42),
    RGB888_TO_RGB222(0x42, 0x51, 0x29)
};

static void core1_func(void);
static void render_scanline(scanvideo_scanline_buffer_t *buffer);
static void initialize_gpio(void);
static void video_stuff(void);
static void nes_classic_controller(void);
static void gpio_callback(uint gpio, uint32_t events);
static void change_scheme_offset(int direction);
static void change_border_color_index(int direction);
static void change_video_effect(int increment);
static void change_scanline_color(int increment);
static void command_check(void);
static bool button_is_pressed(controller_button_t button);
static bool button_was_released(controller_button_t button);
static long map(long x, long in_min, long in_max, long out_min, long out_max);
static void set_indexes(void);
static void update_osd(void);
static void gameboy_reset(void);

int32_t single_solid_line(uint32_t *buf, size_t buf_length, uint16_t color);
int32_t single_scanline(uint32_t *buf, size_t buf_length, uint8_t mapped_y);

int main(void) 
{
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    sleep_ms(10);

    set_sys_clock_khz(300000, true);

    // Create a semaphore to be posted when video init is complete.
    sem_init(&video_initted, 0, 1);

    // Launch all the video on core 1.
    multicore_launch_core1(core1_func);

    // Wait for initialization of video to be complete.
    sem_acquire_blocking(&video_initted);

    initialize_gpio();

    // Clear all button states
    for (int i = 0; i < BUTTON_COUNT; i++) 
    {
        button_states[i] = 1;
        button_states_previous[i] = 1;
    }

    set_indexes();

    change_scanline_color(0);
    
    OSD_init(osd_framebuffer);
    update_osd();
    
    while (true) 
    {
        video_stuff();
        nes_classic_controller();
        command_check();
    }
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void video_stuff(void)
{
    static bool vsync_reset;
    static bool vsync;

    uint8_t *p = framebuffer; 

    while (gpio_get(VSYNC_PIN) == 0);

    vsync_reset = false;

    for (int y = 0; y < PIXELS_Y; y++) {
        while (gpio_get(HSYNC_PIN) == 0);
        while (gpio_get(HSYNC_PIN) == 1);

        *p++ = (gpio_get(DATA_0_PIN) << 1) + gpio_get(DATA_1_PIN);
        
        for (int x = 0; x < (PIXELS_X-1); x++) {
            while (gpio_get(PIXEL_CLOCK_PIN) == 0);
            while (gpio_get(PIXEL_CLOCK_PIN) == 1);

            *p++ = (gpio_get(DATA_0_PIN) << 1) + gpio_get(DATA_1_PIN);
        }

        vsync = gpio_get(VSYNC_PIN);
        if (!vsync) { vsync_reset = true; }
        if (vsync && vsync_reset) { break; }
    }
}

int32_t single_scanline(uint32_t *buf, size_t buf_length, uint8_t mapped_y)
{
    uint16_t* p16 = (uint16_t *) buf;
    uint16_t* first_pixel;

    // LEFT BORDER
    *p16++ = COMPOSABLE_COLOR_RUN;
    *p16++ = border_colors[border_color_index];
    *p16++ = BORDER_HORZ - MIN_RUN - 1;

    // PLAY AREA
    *p16++ = COMPOSABLE_RAW_RUN;
    first_pixel = p16;
    *p16++ = RGB888_TO_RGB222(0x00, 0xFF, 0x00); // replaced later - first pixel
    *p16++ = PIXELS_X*PIXEL_SCALE - MIN_RUN;
    
    uint8_t *pbuff = &framebuffer[mapped_y * PIXELS_X];

    int pos = 0;
    int x,i;
    uint16_t color = 0;
    uint8_t osd_start_x = (PIXELS_X - OSD_get_width())/2;
    uint8_t osd_end_x = osd_start_x + OSD_get_width();
    uint8_t osd_start_y = (PIXELS_Y - OSD_get_height())/2;
    uint8_t osd_end_y = osd_start_y + OSD_get_height();
    uint osd_char_index = 0;
    uint8_t pixels = 0;
    bool in_osd = false;
    int osd_pos = 0;
    bool osd_row = OSD_is_enabled() & (mapped_y >= osd_start_y) & (mapped_y < osd_end_y);

    uint16_t nnn = (mapped_y - osd_start_y) * OSD_WIDTH;
    for (x = 0; x < PIXELS_X; x++)
    {
        if (osd_row && (osd_pos >= 0))
        {
            in_osd = x >= osd_start_x && x < osd_end_x;
        }

        for (i = 0; i < PIXEL_SCALE; i++)
        {
            if (x == 0 && i == 0)
            {
                *first_pixel = colors[*(pbuff) + scheme_offset];
            }
            else
            {
                if (in_osd )
                {
                    color = (uint16_t)(osd_framebuffer[nnn + osd_pos]);
                }
                else
                {
                    // if pixel-effect enabled & 3rd pixel...
                    if ((video_effect == VIDEO_EFFECT_PIXEL_EFFECT) && i == 2)
                    {
                        color = scanline_color;
                    }
                    else
                    {
                        color = colors[*(pbuff) + (uint8_t)scheme_offset]; 
                    }
                }

                *p16++ = color; 
            }
        }
        if (in_osd)
        {
            osd_pos++;
        }

        pbuff++;
    }
   
    // RIGHT BORDER
    *p16++ = COMPOSABLE_COLOR_RUN;
    *p16++ = border_colors[border_color_index]; 
    *p16++ = BORDER_HORZ - MIN_RUN;

    // black pixel to end line
    *p16++ = COMPOSABLE_RAW_1P;
    *p16++ = 0;

    *p16++ = COMPOSABLE_EOL_ALIGN;

    return ((uint32_t *) p16) - buf;
}

int32_t single_solid_line(uint32_t *buf, size_t buf_length, uint16_t color)
{
    uint16_t *p16 = (uint16_t *) buf;

    // LEFT BORDER
    *p16++ = COMPOSABLE_COLOR_RUN;
    *p16++ = border_colors[border_color_index]; 
    *p16++ = BORDER_HORZ - MIN_RUN - 1;

    *p16++ = COMPOSABLE_COLOR_RUN;
    *p16++ = color; 
    *p16++ = PIXELS_X*PIXEL_SCALE - MIN_RUN;

    //RIGHT BORDER
    *p16++ = COMPOSABLE_COLOR_RUN;
    *p16++ = border_colors[border_color_index]; 
    *p16++ = BORDER_HORZ - MIN_RUN;

    // black pixel to end line
    *p16++ = COMPOSABLE_RAW_1P;
    *p16++ = 0;

    *p16++ = COMPOSABLE_EOL_ALIGN;
    
    return ((uint32_t *) p16) - buf;
}

static void render_scanline(scanvideo_scanline_buffer_t *dest) 
{
    uint32_t *buf = dest->data;
    size_t buf_length = dest->data_max;
    int line_num = scanvideo_scanline_number(dest->scanline_id);
       
    if (line_num < (BORDER_VERT) || line_num >= (PIXELS_Y*PIXEL_SCALE + BORDER_VERT))
    {
         dest->data_used = single_solid_line(buf, buf_length, border_colors[border_color_index]);
    }
    else
    {
        if ((video_effect == VIDEO_EFFECT_PIXEL_EFFECT || video_effect == VIDEO_EFFECT_SCANLINES)
            &&  line_num % PIXEL_SCALE == 0)
        {
            dest->data_used = single_solid_line(buf, buf_length, scanline_color);
        }
        else
        {
            uint8_t mapped_y = indexes_y[line_num-BORDER_VERT];
            dest->data_used = single_scanline(buf, buf_length, mapped_y);
        }
    }

    dest->status = SCANLINE_OK;
}


static void core1_func(void) 
{
    
    hard_assert(VGA_MODE.width + 4 <= PICO_SCANVIDEO_MAX_SCANLINE_BUFFER_WORDS * 2);    

    // Initialize video and interrupts on core 1.
    scanvideo_setup(&VGA_MODE);
    scanvideo_timing_enable(true);
    sem_release(&video_initted);

    gpio_set_irq_enabled_with_callback(BUTTONS_DPAD_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(BUTTONS_OTHER_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

    while (true) 
    {
        scanvideo_scanline_buffer_t *scanline_buffer = scanvideo_begin_scanline_generation(true);
        render_scanline(scanline_buffer);
        scanvideo_end_scanline_generation(scanline_buffer);
    }
}

static void initialize_gpio(void)
{    
    //Onboard LED
    gpio_init(ONBOARD_LED_PIN);
    gpio_set_dir(ONBOARD_LED_PIN, GPIO_OUT);
    gpio_put(ONBOARD_LED_PIN, 0);

    // Gameboy Reset
    gpio_init(GAMEBOY_RESET_PIN);
    gpio_set_dir(GAMEBOY_RESET_PIN, GPIO_OUT);
    gpio_put(GAMEBOY_RESET_PIN, 1);

    // Gameboy video signal inputs
    gpio_init(VSYNC_PIN);
    gpio_init(PIXEL_CLOCK_PIN);
    gpio_init(DATA_0_PIN);
    gpio_init(DATA_1_PIN);
    gpio_init(HSYNC_PIN);

    // UART, for testing
    //stdio_init_all();

    // //Initialize I2C port at 400 kHz
    i2c_init(i2cHandle, 400 * 1000);

    // Initialize I2C pins
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);

    gpio_init(BUTTONS_RIGHT_A_PIN);
    gpio_set_dir(BUTTONS_RIGHT_A_PIN, GPIO_OUT);
    gpio_put(BUTTONS_RIGHT_A_PIN, 1);

    gpio_init(BUTTONS_LEFT_B_PIN);
    gpio_set_dir(BUTTONS_LEFT_B_PIN, GPIO_OUT);
    gpio_put(BUTTONS_LEFT_B_PIN, 1);

    gpio_init(BUTTONS_UP_SELECT_PIN);
    gpio_set_dir(BUTTONS_UP_SELECT_PIN, GPIO_OUT);
    gpio_put(BUTTONS_UP_SELECT_PIN, 1);

    gpio_init(BUTTONS_DOWN_START_PIN);
    gpio_set_dir(BUTTONS_DOWN_START_PIN, GPIO_OUT);
    gpio_put(BUTTONS_DOWN_START_PIN, 1);

    gpio_init(BUTTONS_DPAD_PIN);
    gpio_set_dir(BUTTONS_DPAD_PIN, GPIO_IN);

    gpio_init(BUTTONS_OTHER_PIN);
    gpio_set_dir(BUTTONS_OTHER_PIN, GPIO_IN);
}

static void nes_classic_controller(void)
{
    static uint32_t last_micros = 0;
    uint32_t current_micros = time_us_32();

    if (current_micros - last_micros < 20000)
        return;
    
    static bool initialized = false;
    static uint8_t i2c_buffer[16] = {0};

    if (!initialized)
    {
        sleep_ms(2000);

        i2c_buffer[0] = 0xF0;
        i2c_buffer[1] = 0x55;
        (void)i2c_write_blocking(i2cHandle, I2C_ADDRESS, i2c_buffer, 2, false);
        sleep_ms(10);

        i2c_buffer[0] = 0xFB;
        i2c_buffer[1] = 0x00;
        (void)i2c_write_blocking(i2cHandle, I2C_ADDRESS, i2c_buffer, 2, false);
        sleep_ms(20);

        initialized = true;
    }

    last_micros = current_micros;

    i2c_buffer[0] = 0x00;
    (void)i2c_write_blocking(i2cHandle, I2C_ADDRESS, i2c_buffer, 1, false);   // false - finished with bus
    sleep_ms(1);
    int ret = i2c_read_blocking(i2cHandle, I2C_ADDRESS, i2c_buffer, 8, false);
    if (ret < 0)
    {
        last_micros = time_us_32();
        return;
    }
        
    bool valid = false;
    uint8_t i;
    for (i = 0; i < 8; i++)
    {
        if ((i < 4) && (i2c_buffer[i] != 0xFF))
            valid = true;

        if (valid)
        {
            if (i == 4)
            {
                button_states[BUTTON_START] = (~i2c_buffer[i] & (1<<2)) > 0 ? 0 : 1;
                button_states[BUTTON_SELECT] = (~i2c_buffer[i] & (1<<4)) > 0 ? 0 : 1;
                button_states[BUTTON_DOWN] = (~i2c_buffer[i] & (1<<6)) > 0 ? 0 : 1;
                button_states[BUTTON_RIGHT] = (~i2c_buffer[i] & (1<<7)) > 0 ? 0 : 1;

                button_states[BUTTON_HOME] = (~i2c_buffer[i] & (1<<3)) > 0 ? 0 : 1;
            }
            else if (i == 5)
            {
                button_states[BUTTON_UP] = (~i2c_buffer[i] & (1<<0)) > 0 ? 0 : 1;
                button_states[BUTTON_LEFT] = (~i2c_buffer[i] & (1<<1)) > 0 ? 0 : 1;
                button_states[BUTTON_A] = (~i2c_buffer[i] & (1<<4)) > 0 ? 0 : 1;
                button_states[BUTTON_B] = (~i2c_buffer[i] & (1<<6)) > 0 ? 0 : 1;
            }
        }
    }

    if (!valid )
    {
        initialized = false;
        sleep_ms(1000);
        last_micros = time_us_32();
    }

    uint8_t buttondown = 0;
    for (i = 0; i < BUTTON_COUNT; i++)
    {
        if (button_states[i] == 0)
        {
            buttondown = 1;
        }
    }
    gpio_put(ONBOARD_LED_PIN, buttondown);
}

static void gpio_callback(uint gpio, uint32_t events) 
{
    // Prevent controller input to game if OSD is visible
    if (OSD_is_enabled())
        return;

    if(gpio==BUTTONS_DPAD_PIN)
    {
        if (events & (1<<2))   // EDGE LOW -- Send DPAD on falling
        {
            gpio_put(BUTTONS_RIGHT_A_PIN, button_states[BUTTON_RIGHT]);
            gpio_put(BUTTONS_LEFT_B_PIN, button_states[BUTTON_LEFT]);
            gpio_put(BUTTONS_UP_SELECT_PIN, button_states[BUTTON_UP]);
            gpio_put(BUTTONS_DOWN_START_PIN, button_states[BUTTON_DOWN]);
        }
        
        if (events & (1<<3)) // EDGE HIGH -- Send BUTTONS on rising
        {
            gpio_put(BUTTONS_RIGHT_A_PIN, button_states[BUTTON_A]);
            gpio_put(BUTTONS_LEFT_B_PIN, button_states[BUTTON_B]);
            gpio_put(BUTTONS_UP_SELECT_PIN, button_states[BUTTON_SELECT]);
            gpio_put(BUTTONS_DOWN_START_PIN, button_states[BUTTON_START]);

            // Prevent Tetris in-game reset lockup
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

    // When *other* pin goes high read cycle complete, send all high
    if(gpio==BUTTONS_OTHER_PIN && (events & (1<<3)))
    {
        gpio_put(BUTTONS_RIGHT_A_PIN, 1);
        gpio_put(BUTTONS_LEFT_B_PIN, 1);
        gpio_put(BUTTONS_UP_SELECT_PIN, 1);
        gpio_put(BUTTONS_DOWN_START_PIN, 1);
    }
}

static void change_scheme_offset(int direction)
{
    int max_offset = sizeof(colors)/sizeof(colors[0]) - 4;
    scheme_offset += direction * 4;
    scheme_offset = scheme_offset > max_offset ? 0 : scheme_offset;
    scheme_offset = scheme_offset < 0 ? max_offset : scheme_offset;
}

static void change_border_color_index(int direction)
{
    border_color_index += direction;
    border_color_index = border_color_index < 0 ? (sizeof(border_colors)-1) : border_color_index;
    border_color_index = border_color_index >= sizeof(border_colors) ? 0 : border_color_index;
}

static void change_video_effect(int increment)
{
    video_effect += increment;
    video_effect = video_effect >= VIDEO_EFFECT_COUNT ? VIDEO_EFFECT_NONE : video_effect;
    video_effect = video_effect < 0 ? VIDEO_EFFECT_COUNT-1 : video_effect;
}

static void change_scanline_color(int increment)
{
    scanline_color_offset += increment;
    scanline_color_offset = scanline_color_offset > 3 ? 0 : scanline_color_offset;
    scanline_color_offset = scanline_color_offset < 0 ? 3 : scanline_color_offset;
    scanline_color = colors[scheme_offset + scanline_color_offset];
}

static bool button_is_pressed(controller_button_t button)
{
    return button_states[button] == 0;
}

static bool button_was_released(controller_button_t button)
{
    return button_states[button] == 1 && button_states_previous[button] == 0;
}

static void command_check(void)
{
    // Home pressed
    if (button_was_released(BUTTON_HOME))
    {
        OSD_toggle();
    }
    else
    {
        if (OSD_is_enabled())
        {
            if (button_was_released(BUTTON_DOWN))
            {
                OSD_change_line(1);
            }
            else if (button_was_released(BUTTON_UP))
            {
                OSD_change_line(-1);
            }
            else if (button_was_released(BUTTON_RIGHT) 
                    || button_was_released(BUTTON_LEFT)
                    || button_was_released(BUTTON_A))
            {
                bool leftbtn = button_was_released(BUTTON_LEFT);
                switch (OSD_get_active_line())
                {
                    case OSD_LINE_COLOR_SCHEME:
                        change_scheme_offset(leftbtn ? -1 : 1);
                        update_osd();
                        break;
                    case OSD_LINE_BORDER_COLOR:
                        change_border_color_index(leftbtn ? -1 : 1);
                        update_osd();
                        break;
                    case OSD_LINE_EFFECTS:
                        change_video_effect(leftbtn ? -1 : 1);
                        update_osd();
                        break;
                    case OSD_LINE_FX_SCHEME:
                        change_scanline_color(leftbtn ? -1 : 1);
                        update_osd();
                        break;
                    case OSD_LINE_RESET_GAMEBOY:
                        gameboy_reset();
                        break;
                    case OSD_LINE_EXIT:
                        OSD_toggle();
                        break;
                }
            }
        }
    }

    
    for (int i = 0; i < BUTTON_COUNT; i++) 
    {
        button_states_previous[i] = button_states[i];
    }
}

static void set_indexes(void)
{
    int i;
    uint16_t n = 0;

    uint16_t x;
    for (x = 0; x < PIXELS_X; x++)
    {
        for (i = 0; i < PIXEL_SCALE; i++) 
        {
            indexes_x[n++] = x;
        }
    }

    n = 0;
    uint16_t y;
    for (y = 0; y < PIXELS_Y; y++)
    {
        for (i = 0; i < PIXEL_SCALE; i++) 
        {
            indexes_y[n++] = y;
        }
    }
}

static void update_osd(void)
{
    char buff[32];
    sprintf(buff, "COLOR SCHEME:% 5d", scheme_offset/4);
    OSD_set_line_text(OSD_LINE_COLOR_SCHEME, buff);

    sprintf(buff, "BORDER COLOR:% 5d", border_color_index);
    OSD_set_line_text(OSD_LINE_BORDER_COLOR, buff);

    if (video_effect==VIDEO_EFFECT_SCANLINES)
    {
        sprintf(buff, "EFFECTS: SCANLINES");
    }
    else if (video_effect==VIDEO_EFFECT_PIXEL_EFFECT)
    {
        sprintf(buff, "EFFECTS:    PIXELS");
    }
    else
    {
        sprintf(buff, "EFFECTS:      NONE");
    }
    OSD_set_line_text(OSD_LINE_EFFECTS, buff);

    sprintf(buff, "FX SCHEME:% 8d", scanline_color_offset);
    OSD_set_line_text(OSD_LINE_FX_SCHEME, buff);

    OSD_set_line_text(OSD_LINE_RESET_GAMEBOY, "RESET GAMEBOY");
    OSD_set_line_text(OSD_LINE_EXIT, "EXIT");

    OSD_update_framebuffer();
}

static void gameboy_reset(void)
{
    gpio_put(GAMEBOY_RESET_PIN, 0);
    sleep_ms(50);
    gpio_put(GAMEBOY_RESET_PIN, 1);
}
