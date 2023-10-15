#ifndef COLORS_H
#define COLORS_H

//#include "pico.h"
//#include "pico/scanvideo.h"
//#include <stdlib.h>
#include "pico/stdlib.h"

//#define RGB888_TO_RGB222(r, g, b) ((((b)>>6u)<<PICO_SCANVIDEO_PIXEL_BSHIFT)|(((g)>>6u)<<PICO_SCANVIDEO_PIXEL_GSHIFT)|(((r)>>6u)<<PICO_SCANVIDEO_PIXEL_RSHIFT))

typedef enum
{
    SCHEME_BLACK_AND_WHITE = 0,
    SCHEME_INVERTED,
    SCHEME_DMG,
    SCHEME_GAME_BOY_POCKET,
    SCHEME_GAME_BOY_LIGHT,
    SCHEME_SGB_1A,
    SCHEME_SGB_1B,
    SCHEME_SGB_1C,
    SCHEME_SGB_1D,
    SCHEME_SGB_1E,
    SCHEME_SGB_1F,
    SCHEME_SGB_1G,
    SCHEME_SGB_1H,
    SCHEME_SGB_2A,
    SCHEME_SGB_2B,
    SCHEME_SGB_2C,
    SCHEME_SGB_2D,
    SCHEME_SGB_2E,
    SCHEME_SGB_2F,
    SCHEME_SGB_2G,
    SCHEME_SGB_2H,
    SCHEME_SGB_3A,
    SCHEME_SGB_3B,
    SCHEME_SGB_3C,
    SCHEME_SGB_3D,
    SCHEME_SGB_3E,
    SCHEME_SGB_3F,
    SCHEME_SGB_3G,
    SCHEME_SGB_3H,
    SCHEME_SGB_4A,
    SCHEME_SGB_4B,
    SCHEME_SGB_4C,
    SCHEME_SGB_4D,
    SCHEME_SGB_4E,
    SCHEME_SGB_4F,
    SCHEME_SGB_4G,
    SCHEME_SGB_4H,

    NUMBER_OF_SCHEMES
} COLOR_SCHEMES;



// TODO:  name variables based on what the color is...
typedef struct color_scheme_t
{
    uint32_t c1;
    uint32_t c2;
    uint32_t c3;
    uint32_t c4;
} color_scheme_t;

typedef enum
{
    COLOR_BLACK = 0,
    COLOR_BLUE,
    COLOR_WHITE,
    COLOR_LIGHT_GREY,
    COLOR_DARK_GREY,
    COLOR_RED,
    COLOR_GREEN,
    COLOR_YELLOW,
    COLOR_PURPLE,
    NUMBER_OF_COLORS
} COLORS;

uint32_t get_basic_color(uint8_t index);
uint32_t get_background_color(void);
int get_border_color_index(void);
void set_border_color_index(int index);
void increase_border_color_index(int direction);
void increase_color_scheme_index(int direction);
color_scheme_t* get_scheme(void);
int get_scheme_index(void);
void set_scheme_index(int index);
uint16_t rgb888_to_rgb222(uint32_t color);
void reverse_rgb_bits_toggle(void);
bool rgb_bit_reverse_state(void);

#endif // COLORS_H