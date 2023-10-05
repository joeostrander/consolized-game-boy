#include "colors.h"
#include "pico/scanvideo.h"

#define REVERSE_VGA_BITS    // I had the high bit/low bit backwards on Pico.. will fix in next PCB rev

static color_scheme_t color_schemes[NUMBER_OF_SCHEMES] = 
{
    [SCHEME_BLACK_AND_WHITE] =  { 0xF7F3F7, 0xB5B2B5, 0x4E4C4E, 0x000000 },
    [SCHEME_INVERTED] =         { 0x000000, 0x4E4C4E, 0xB5B2B5, 0xF7F3F7 },
    [SCHEME_DMG] =              { 0x7B8210, 0x5A7942, 0x39594A, 0x294139 },
    [SCHEME_GAME_BOY_POCKET] =  { 0xC6CBA5, 0x8C926B, 0x4A5139, 0x181818 },
    [SCHEME_GAME_BOY_LIGHT] =   { 0x00B284, 0x8C926B, 0x00694A, 0x005139 },
    [SCHEME_SGB_1A] =           { 0xF7E3C6, 0xD6924A, 0xA52821, 0x311852 },
    [SCHEME_SGB_2A] =           { 0xEFC39C, 0xBD8A4A, 0x297900, 0x000000 },
    [SCHEME_SGB_3A] =           { 0xF7CB94, 0x73BABD, 0xF76129, 0x314963 },
    [SCHEME_SGB_4A] =           { 0xEFA26B, 0x7BA2F7, 0xCE00CE, 0x00007B },
    [SCHEME_SGB_1B] =           { 0xD6D3BD, 0xC6AA73, 0xAD5110, 0x000000 },
    [SCHEME_SGB_2B] =           { 0xF7F3F7, 0xF7E352, 0xF73000, 0x52005A },
    [SCHEME_SGB_3B] =           { 0xD6D3BD, 0xDE8221, 0x005100, 0x001010 },
    [SCHEME_SGB_4B] =           { 0xEFE3EF, 0xE79A63, 0x427939, 0x180808 },
    [SCHEME_SGB_1C] =           { 0xF7BAF7, 0xE79252, 0x943863, 0x393894 },
    [SCHEME_SGB_2C] =           { 0xF7F3F7, 0xE78A8C, 0x7B30E7, 0x292894 },
    [SCHEME_SGB_3C] =           { 0xDEA2C6, 0xF7F37B, 0x00B2F7, 0x21205A },
    [SCHEME_SGB_4C] =           { 0xF7DBDE, 0xF7F37B, 0x949ADE, 0x080000 },
    [SCHEME_SGB_1D] =           { 0xF7F3A5, 0xBD824A, 0xF70000, 0x521800 },
    [SCHEME_SGB_2D] =           { 0xF7F39C, 0x00F300, 0xF73000, 0x000052 },
    [SCHEME_SGB_3D] =           { 0xEFF3B5, 0xDEA27B, 0x96AD52, 0x000000 },
    [SCHEME_SGB_4D] =           { 0xF7F3B5, 0x94C3C6, 0x4A697B, 0x08204A },
    [SCHEME_SGB_1E] =           { 0xF7D3AD, 0x7BBA7B, 0x6B8A42, 0x5A3821 },
    [SCHEME_SGB_2E] =           { 0xF7C384, 0x94AADE, 0x291063, 0x100810 },
    [SCHEME_SGB_3E] =           { 0xF7F3BD, 0xDEAA6B, 0xAD7921, 0x524973 },
    [SCHEME_SGB_4E] =           { 0xF7D3A5, 0xDEA27B, 0x7B598C, 0x002031 },
    [SCHEME_SGB_1F] =           { 0xD6E3F7, 0xDE8A52, 0xA50000, 0x004110 },
    [SCHEME_SGB_2F] =           { 0xCEF3F7, 0xF79252, 0x9C0000, 0x180000 },
    [SCHEME_SGB_3F] =           { 0x7B79C6, 0xF769F7, 0xF7CB00, 0x424142 },
    [SCHEME_SGB_4F] =           { 0xB5CBCE, 0xD682D6, 0x84009C, 0x390000 },
    [SCHEME_SGB_1G] =           { 0x000052, 0x009AE7, 0x7B7900, 0xF7F35A },
    [SCHEME_SGB_2G] =           { 0x6BB239, 0xDE5142, 0xDEB284, 0x001800 },
    [SCHEME_SGB_3G] =           { 0x63D352, 0xF7F3F7, 0xC63039, 0x390000 },
    [SCHEME_SGB_4G] =           { 0xADDB18, 0xB5205A, 0x291000, 0x008263 },
    [SCHEME_SGB_1H] =           { 0xF7E3DE, 0xF7B28C, 0x844100, 0x311800 },
    [SCHEME_SGB_2H] =           { 0xF7F3F7, 0xB5B2B5, 0x737173, 0x000000 },
    [SCHEME_SGB_3H] =           { 0xDEF39C, 0x7BC339, 0x4A8A18, 0x081800 },
    [SCHEME_SGB_4H] =           { 0xF7F3C6, 0xB5BA5A, 0x848A42, 0x425129 }
};

static uint32_t basic_colors[NUMBER_OF_COLORS] = 
{
    [COLOR_BLACK]       = 0x000000,
    [COLOR_BLUE]        = 0x0000FF,
    [COLOR_WHITE]       = 0xFFFFFF,
    [COLOR_DARK_GREY]   = 0x808080,
    [COLOR_LIGHT_GREY]  = 0x404040,
    [COLOR_RED]         = 0xFF0000,
    [COLOR_GREEN]       = 0x00FF00,
    [COLOR_YELLOW]      = 0xFFFF00,
    [COLOR_PURPLE]      = 0xFF00FF
};

static int border_color_index = 0;
static int color_scheme_index = SCHEME_BLACK_AND_WHITE;    // TODO... color "scheme" offset?

uint32_t get_basic_color(uint8_t index)
{
    return basic_colors[index];
}

uint32_t get_background_color(void)
{
    return basic_colors[border_color_index];
}

int get_border_color_index(void)
{
    return border_color_index;
}

void change_border_color_index(int direction)
{
    set_background_color(border_color_index + direction);
}

void set_background_color(int index)
{
    uint8_t max = sizeof(basic_colors)/sizeof(uint32_t) -1;
    border_color_index = index;
    border_color_index = border_color_index < 0 ? max : border_color_index;
    border_color_index = border_color_index > max ? 0 : border_color_index;
}

void change_color_scheme_index(int direction)
{
    color_scheme_index += direction;
    color_scheme_index = color_scheme_index >= NUMBER_OF_SCHEMES ? 0 : color_scheme_index;
    color_scheme_index = color_scheme_index < 0 ? (NUMBER_OF_SCHEMES-1) : color_scheme_index;
}

color_scheme_t* get_scheme(void)
{
    return &color_schemes[color_scheme_index];
}

int get_scheme_index(void)
{
    return color_scheme_index;
}

#ifdef REVERSE_VGA_BITS
uint16_t rgb888_to_rgb222(uint32_t color)
{
        uint32_t ret = 0;
        ret = ((color & (1<<6)) >> 5);          //b1
        ret = ret | ((color & (1<<7)) >> 7);    //b0
        ret = ret | ((color & (1<<14)) >> 11);  //g1
        ret = ret | ((color & (1<<15)) >> 13);  //g0
        ret = ret | ((color & (1<<22)) >> 17);  //r1
        ret = ret | ((color & (1<<23)) >> 19);  //r0
        return (uint16_t)ret;
}
#else
uint16_t rgb888_to_rgb222(uint32_t color)
{
    uint32_t red = (color & 0xC00000) >> 22;
    uint32_t green = (color & 0xC000) >> 14;
    uint32_t blue = (color & 0xC0) >> 6;
    return (uint16_t)( ( blue<<PICO_SCANVIDEO_PIXEL_BSHIFT ) |( green<<PICO_SCANVIDEO_PIXEL_GSHIFT ) |( red<<PICO_SCANVIDEO_PIXEL_RSHIFT ) );
}
}
#endif