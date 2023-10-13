/* 
    Authors: 
        Andy West (original code)
        Joe Ostrander

    Version: 2.2

    https://github.com/joeostrander/consolized-game-boy
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h> // for memcmp
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
#include "colors.h"

// #define USE_NES_CONTROLLER   // Uncomment to use old-school NES controller
// #define DEBUG_BUTTON_PRESS   // Illuminate LED on button presses

// Shift register pins -- for old-school NES controller
#define DATA_PIN                8
#define LATCH_PIN               9
#define PULSE_PIN               10

// I2C Pins, etc. -- for I2C controller
#define SDA_PIN     12
#define SCL_PIN     13
#define I2C_ADDRESS 0x52
i2c_inst_t* i2cHandle = i2c0;

#define MIN_RUN 3

#define ONBOARD_LED_PIN             25

// GAMEBOY VIDEO INPUT (From level shifter)
#define VSYNC_PIN                   18
#define HSYNC_PIN                   17
#define PIXEL_CLOCK_PIN             16
#define DATA_1_PIN                  15
#define DATA_0_PIN                  14

#define DMG_READING_DPAD_PIN        19      // P14
#define DMG_READING_BUTTONS_PIN     20      // P15
#define DMG_OUTPUT_LEFT_B_PIN       26      // P11
#define DMG_OUTPUT_DOWN_START_PIN   21      // P13
#define DMG_OUTPUT_UP_SELECT_PIN    22      // P12
#define DMG_OUTPUT_RIGHT_A_PIN      27      // P10

#define GAMEBOY_RESET_PIN           28

// at 3x Game area will be 480x432 
#define DMG_PIXELS_X                160
#define DMG_PIXELS_Y                144
#define DMG_PIXEL_COUNT             (DMG_PIXELS_X*DMG_PIXELS_Y)

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
    OSD_LINE_BORDER_COLOR,
    OSD_LINE_REVERSE_RGB_BITS,
    OSD_LINE_RESET_GAMEBOY,
    OSD_LINE_EXIT,
    OSD_LINE_COUNT
} osd_line_t;

typedef enum
{
    BUTTON_STATE_PRESSED = 0,
    BUTTON_STATE_UNPRESSED
} button_state_t;

typedef struct rectangle_t
{
    uint16_t x;
    uint16_t y;
    uint16_t width;
    uint16_t height;
} rectangle_t;

const scanvideo_timing_t vga_timing_640x480_gb =
        {
                .clock_freq = 25000000,

                .h_active = 640,
                .v_active = 480,

                .h_front_porch = 16,
                .h_pulse = 64,
                .h_total = 800,
                .h_sync_polarity = 1,

                .v_front_porch = 1,
                .v_pulse = 2,
                .v_total = 523,
                .v_sync_polarity = 1,

                .enable_clock = 0,
                .clock_polarity = 0,

                .enable_den = 0
        };

const scanvideo_mode_t vga_mode_640x480_3x_scale =
{
        .default_timing = &vga_timing_640x480_gb,
        .pio_program = &video_24mhz_composable,
        .width = 640,
        .height = 480,
        .xscale = 3,
        .yscale = 3,
};

#define VGA_MODE vga_mode_640x480_3x_scale
#define LINE_LENGTH     ((uint16_t)(((VGA_MODE.width * 100.0)/VGA_MODE.xscale) + 50) / 100)

static semaphore_t video_initted;
static uint8_t button_states[BUTTON_COUNT];
static uint8_t button_states_previous[BUTTON_COUNT];
static color_scheme_t* color_scheme;

static uint8_t framebuffer[DMG_PIXEL_COUNT];
static uint8_t* osd_framebuffer = NULL;

static rectangle_t rect_gamewindow;
static rectangle_t rect_osd;
static uint16_t background_color;

static void core1_func(void);
static void render_scanline(scanvideo_scanline_buffer_t *buffer);
static void initialize_gpio(void);
static bool nes_controller(void);
static bool nes_classic_controller(void);
static void gpio_callback(uint gpio, uint32_t events);
static void gpio_callback_VIDEO(uint gpio, uint32_t events);
static void __no_inline_not_in_flash_func(command_check)(void);
static bool button_is_pressed(controller_button_t button);
static bool button_was_released(controller_button_t button);
static long map(long x, long in_min, long in_max, long out_min, long out_max);
static void update_osd(void);
static void set_orientation(void);
static void gameboy_reset(void);

int32_t single_solid_line(uint32_t *buf, size_t buf_length, uint16_t color);
int32_t single_scanline(uint32_t *buf, size_t buf_length, uint8_t line_index);

int main(void) 
{
    hw_set_bits(&vreg_and_chip_reset_hw->vreg, VREG_AND_CHIP_RESET_VREG_VSEL_BITS);
    sleep_ms(10);

    // should be a multiple of VGA_MODE clock_freq
    //set_sys_clock_khz(300000, true);
    //set_sys_clock_khz(240000, true);
    set_sys_clock_khz(225000, true);    

    // Create a semaphore to be posted when video init is complete.
    sem_init(&video_initted, 0, 1);

    // Launch all the video on core 1.
    multicore_launch_core1(core1_func);

    // Wait for initialization of video to be complete.
    sem_acquire_blocking(&video_initted);

    initialize_gpio();

    // prevent false trigger of OSD on start -- set all previous button states to 1 (unpressed)
    for (int i = 0; i < BUTTON_COUNT; i++)
    {
        button_states[i] = BUTTON_STATE_UNPRESSED;
        button_states_previous[i] = BUTTON_STATE_UNPRESSED;
    }

    set_background_color(COLOR_BLACK);
    background_color = rgb888_to_rgb222(get_background_color());
    color_scheme = get_scheme();
    
    osd_framebuffer = OSD_get_framebuffer();
    update_osd();

    set_orientation();
    
    gpio_set_irq_enabled_with_callback(VSYNC_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback_VIDEO);

    while (true) 
    {
#ifdef USE_NES_CONTROLLER
        if (nes_controller())
        {
            command_check();
        }
#else
        if (nes_classic_controller())
        {
            command_check();
        }
#endif
    }
}

static long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void gpio_callback_VIDEO(uint gpio, uint32_t events)
{
//                  +-----------------------------------------+     
// VSYNC -----------+                                         +-----------------------
//                    +------+                                     +------+
// HSYNC -------------+      +-------------------------------------+      +-----------
//                      +-+     +-+ +-+ +-+ +-+ +-+ +-+ +-+ +-+      +-+     +-+ +-+ +
// CLOCK ---------------+ +-----+ +-+ +-+ +-+ +-+ +-+ +-+ +-+ +------+ +-----+ +-+ +-+
//       -------------+   +--+  ++ ++ +--+ ++ ++ +---------------+   +--+  ++ ++ +--+ 
// DATA 0/1           +---+  +--++-++-+  +-++-++-+               +---+  +--++-++-+  +-


    if(gpio==VSYNC_PIN)
    {
        // ignore falling edge interrupt
        if (events & GPIO_IRQ_EDGE_FALL)
            return;
    }

    uint16_t x = 0;
    uint16_t y = 0;
    uint32_t pos = 0;

    for (y = 0; y < DMG_PIXELS_Y; y++)
    {
        // wait for HSYNC edge to fall
        while (gpio_get(HSYNC_PIN) == 0);
        while (gpio_get(HSYNC_PIN) == 1);
        for (x = 0; x < DMG_PIXELS_X; x++)   //DMG_PIXELS_X
        {
            pos = x + (y * rect_gamewindow.width);

            if (pos < DMG_PIXEL_COUNT)
                framebuffer[pos] = (gpio_get(DATA_0_PIN) << 1) + gpio_get(DATA_1_PIN);

            // wait for clock pulse to fall
            while (gpio_get(PIXEL_CLOCK_PIN) == 0);
            while (gpio_get(PIXEL_CLOCK_PIN) == 1);
        }
    }
}

int32_t single_scanline(uint32_t *buf, size_t buf_length, uint8_t line_index)
{
    uint16_t* p16 = (uint16_t *) buf;
    uint16_t* first_pixel;
    uint16_t pixel_count = 0;

    uint16_t border = rect_gamewindow.x;

    // LEFT BORDER
    *p16++ = COMPOSABLE_COLOR_RUN;
    *p16++ = background_color;
    *p16++ = border - MIN_RUN;
    pixel_count += border;

    // GAME WINDOW
    *p16++ = COMPOSABLE_RAW_RUN;
    first_pixel = p16;
    *p16++ = 0; // replaced later - first pixel
    *p16++ = rect_gamewindow.width - MIN_RUN;

    uint8_t *pbuff = &framebuffer[line_index * rect_gamewindow.width];

    uint16_t pos = 0;
    uint16_t x;
    uint16_t x_osd;
    uint16_t y_osd;
    uint16_t i = 0;
    uint16_t color = 0;

    uint16_t idx;
    for (x = 0; x < rect_gamewindow.width; x++)
    {
        if (x == 0 && i == 0)
        {
            *first_pixel = rgb888_to_rgb222( *((uint32_t*)color_scheme + *pbuff) ) ;
        }
        else
        {
            if ( OSD_is_enabled() 
                && (line_index >= rect_osd.y)
                && (line_index <= (rect_osd.y+rect_osd.height))
                && (x >= rect_osd.x)
                && (x < (rect_osd.x+rect_osd.width)))
            {
                x_osd = x - rect_osd.x;
                y_osd = line_index - rect_osd.y;
                idx = x_osd + (y_osd * OSD_WIDTH);
                if (osd_framebuffer != NULL && idx < (OSD_WIDTH*OSD_HEIGHT))
                {
                    if (idx < (OSD_WIDTH*OSD_HEIGHT))
                        color = (uint16_t)(osd_framebuffer[idx]);
                }
            }
            else
            {
                color = rgb888_to_rgb222( *((uint32_t*)color_scheme + *pbuff) ) ;
            }

            *p16++ = color; 
        }

        pbuff++;
        pixel_count++;
    }
  
    if (pixel_count*VGA_MODE.xscale < VGA_MODE.width)
    {
        uint16_t remaining = (VGA_MODE.width - (pixel_count*VGA_MODE.xscale))/VGA_MODE.xscale;
        
        // //TESTING!!!
        // while (remaining % 3 != 0)
        // {
        //     remaining++;
        // }

        // RIGHT BORDER
        if (remaining > MIN_RUN)
        {
            *p16++ = COMPOSABLE_COLOR_RUN;
            *p16++ = background_color; 
            *p16++ = remaining - MIN_RUN;
        }
    }

    // black pixel to end line
    *p16++ = COMPOSABLE_RAW_1P;
    *p16++ = 0;

    *p16++ = COMPOSABLE_EOL_ALIGN;  // TODO... how to determine when to do skip align

    return ((uint32_t *) p16) - buf;
}

int32_t single_solid_line(uint32_t *buf, size_t buf_length, uint16_t color)
{
    uint16_t *p16 = (uint16_t *) buf;

    *p16++ = COMPOSABLE_COLOR_RUN;
    *p16++ = color; 
    *p16++ = LINE_LENGTH - MIN_RUN;

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
    int line_start = rect_gamewindow.y;
    int line_end = rect_gamewindow.y + rect_gamewindow.height - 1;
    
    if (line_num < line_start || line_num > line_end)
    {
        dest->data_used = single_solid_line(buf, buf_length, background_color);
    }
    else
    {
        dest->data_used = single_scanline(buf, buf_length, (uint8_t)(line_num - rect_gamewindow.y));
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

    gpio_set_irq_enabled_with_callback(DMG_READING_DPAD_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    gpio_set_irq_enabled_with_callback(DMG_READING_BUTTONS_PIN, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true, &gpio_callback);

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

#ifdef USE_NES_CONTROLLER

    /* NES Controller - start */

    /* Clock, normally HIGH */
    gpio_init(PULSE_PIN);
    gpio_set_dir(PULSE_PIN, GPIO_OUT);
    gpio_put(PULSE_PIN, 1);

    /* Latch, normally LOW */
    gpio_init(LATCH_PIN);
    gpio_set_dir(LATCH_PIN, GPIO_OUT);
    gpio_put(LATCH_PIN, 0);

    /* Data, reads normally high */
    gpio_init(DATA_PIN);
    gpio_set_dir(DATA_PIN, GPIO_IN);

    /* NES Controller - end */
#else
    //Initialize I2C port at 400 kHz
    i2c_init(i2cHandle, 400 * 1000);

    // Initialize I2C pins
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SCL_PIN);
    gpio_pull_up(SDA_PIN);
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

static bool nes_controller(void)
{
    static uint32_t last_micros = 0;
    uint32_t current_micros = time_us_32();
    if (current_micros - last_micros < 20000)
        return false;

    last_micros = current_micros;

    gpio_put(LATCH_PIN, 1);
    sleep_us(5);
    gpio_put(LATCH_PIN, 0);
    sleep_us(1);
    button_states[0] = gpio_get(DATA_PIN) ? BUTTON_STATE_UNPRESSED : BUTTON_STATE_PRESSED;
    sleep_us(4);

    for (uint i = 1; i < 8; i++) 
    {
        sleep_us(8);
        gpio_put(PULSE_PIN, 0);
        sleep_us(1);
        gpio_put(PULSE_PIN, 1);
        sleep_us(8);
        button_states[i] = gpio_get(DATA_PIN) ? BUTTON_STATE_UNPRESSED : BUTTON_STATE_PRESSED;
    }

#ifdef DEBUG_BUTTON_PRESS
    uint8_t buttondown = 0;
    for (int i = 0; i < BUTTON_HOME; i++)
    {
        if (button_states[i] == BUTTON_STATE_PRESSED)
        {
            buttondown = 1;
        }
    }
    gpio_put(ONBOARD_LED_PIN, buttondown);
#endif

    return true;
}

static bool nes_classic_controller(void)
{
    static uint32_t last_micros = 0;
    uint32_t current_micros = time_us_32();

    if (current_micros - last_micros < 5000)
        return false;
    
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
        return false;
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

#ifdef DEBUG_BUTTON_PRESS
    uint8_t buttondown = 0;
    for (i = 0; i < BUTTON_COUNT; i++)
    {
        if (button_states[i] == BUTTON_STATE_PRESSED)
        {
            buttondown = 1;
        }
    }
    gpio_put(ONBOARD_LED_PIN, buttondown);
#endif

    return true;
}

static void gpio_callback(uint gpio, uint32_t events) 
{
    // Prevent controller input to game if OSD is visible
    if (OSD_is_enabled())
        return;

    if(gpio==DMG_READING_DPAD_PIN)
    {
        if (events & GPIO_IRQ_EDGE_FALL)   // Send DPAD states on low
        {
            gpio_put(DMG_OUTPUT_RIGHT_A_PIN, button_states[BUTTON_RIGHT]);
            gpio_put(DMG_OUTPUT_LEFT_B_PIN, button_states[BUTTON_LEFT]);
            gpio_put(DMG_OUTPUT_UP_SELECT_PIN, button_states[BUTTON_UP]);
            gpio_put(DMG_OUTPUT_DOWN_START_PIN, button_states[BUTTON_DOWN]);
        }

        //TODO: it might be best to read BUTTONS when this goes high
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

        // When P15 pin goes high, read cycle is complete, send all high
        if(events & GPIO_IRQ_EDGE_RISE)
        {
            gpio_put(DMG_OUTPUT_RIGHT_A_PIN, 1);
            gpio_put(DMG_OUTPUT_LEFT_B_PIN, 1);
            gpio_put(DMG_OUTPUT_UP_SELECT_PIN, 1);
            gpio_put(DMG_OUTPUT_DOWN_START_PIN, 1);
        }
    }
}

static bool button_is_pressed(controller_button_t button)
{
    return button_states[button] == BUTTON_STATE_PRESSED;
}

static bool button_was_released(controller_button_t button)
{
    return button_states[button] == BUTTON_STATE_UNPRESSED && button_states_previous[button] == BUTTON_STATE_PRESSED;
}

static void __no_inline_not_in_flash_func(command_check)(void)
{
    if (memcmp(button_states, button_states_previous, sizeof(button_states)) == 0)
        return;


    if (button_is_pressed(BUTTON_SELECT))
    {
        // select pressed
        if (button_was_released(BUTTON_START))
        {
            OSD_toggle();
        }
    }
    else
    {
        // select not pressed

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
                            change_color_scheme_index(leftbtn ? -1 : 1);
                            color_scheme = get_scheme();
                            update_osd();
                            break;
                        case OSD_LINE_BORDER_COLOR:
                            change_border_color_index(leftbtn ? -1 : 1);
                            background_color = rgb888_to_rgb222(get_background_color());
                            update_osd();
                            break;
                        case OSD_LINE_REVERSE_RGB_BITS:
                            reverse_rgb_bits_toggle();
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

        
    }
    
    for (int i = 0; i < BUTTON_COUNT; i++) 
    {
        button_states_previous[i] = button_states[i];
    }
}

// static void set_indexes(void)
// {
//     int i;
//     uint16_t n = 0;

//     uint16_t x;
//     for (x = 0; x < PIXELS_X; x++)
//     {
//         for (i = 0; i < PIXEL_SCALE; i++) 
//         {
//             indexes_x[n++] = x;
//         }
//     }

//     n = 0;
//     uint16_t y;
//     for (y = 0; y < PIXELS_Y; y++)
//     {
//         for (i = 0; i < PIXEL_SCALE; i++) 
//         {
//             indexes_y[n++] = y;
//         }
//     }
// }

static void update_osd(void)
{
    char buff[32];
    sprintf(buff, "COLOR SCHEME:% 5d", get_scheme_index());
    OSD_set_line_text(OSD_LINE_COLOR_SCHEME, buff);

    sprintf(buff, "BORDER COLOR:% 5d", get_border_color_index());
    OSD_set_line_text(OSD_LINE_BORDER_COLOR, buff);

    sprintf(buff, "RGB BIT FLIP:% 5s", rgb_bit_reverse_state() ? "ON" :"OFF");
    OSD_set_line_text(OSD_LINE_REVERSE_RGB_BITS, buff);

    OSD_set_line_text(OSD_LINE_RESET_GAMEBOY, "RESET GAMEBOY");
    OSD_set_line_text(OSD_LINE_EXIT, "EXIT");

    OSD_update();
}


static void set_orientation(void)
{
//uint16_t border = ((VGA_MODE.width/VGA_MODE.xscale) - rect_gamewindow.width)/2;

    rect_gamewindow.width = DMG_PIXELS_X;
    rect_gamewindow.height = DMG_PIXELS_Y;
    rect_gamewindow.x = ((VGA_MODE.width/VGA_MODE.xscale) - rect_gamewindow.width)/2;
    rect_gamewindow.y = ((VGA_MODE.height/VGA_MODE.yscale) - rect_gamewindow.height)/2;
    rect_osd.height = OSD_HEIGHT;
    rect_osd.width = OSD_WIDTH;

    rect_osd.x = (rect_gamewindow.width - rect_osd.width)/2;
    rect_osd.y = (rect_gamewindow.height - rect_osd.height)/2;
}

static void gameboy_reset(void)
{
    gpio_put(GAMEBOY_RESET_PIN, 0);
    sleep_ms(50);
    gpio_put(GAMEBOY_RESET_PIN, 1);
}
