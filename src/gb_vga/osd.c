#include "osd.h"
#include <string.h>
#include <stdint.h>

static uint8_t osd_pixel_buff[OSD_CHARS_PER_LINE];   // TODO:  OSD_get_pixel_buff?
static char osd_text[OSD_LINES][OSD_CHARS_PER_LINE+1];
static bool osd_enabled = false;
static int active_line = 0;
//static uint8_t* framebuffer = NULL;
static uint8_t framebuffer[OSD_HEIGHT*OSD_WIDTH] = {0};

typedef struct 
{
    char osd_char;
    uint8_t data[OSD_CHAR_HEIGHT];
} osd_char_data_t;

//TODO: add more!
static osd_char_data_t osd_letters[] = 
{    
    { 
        .osd_char = ' ', 
        .data = 
        {
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000,
            0b00000000
        }
    },
    { 
        .osd_char = 'A', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100010,
            0b00100010,
            0b00111110,
            0b00100010,
            0b00100010,
            0b00000000
        }
    },
    { 
        .osd_char = 'B', 
        .data = 
        {
            0b00000000,
            0b00111100,
            0b00100010,
            0b00111100,
            0b00100010,
            0b00100010,
            0b00111110,
            0b00000000
        }
    },
    { 
        .osd_char = 'C', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100010,
            0b00100000,
            0b00100000,
            0b00100010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = 'D', 
        .data = 
        {
            0b00000000,
            0b00111100,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00111100,
            0b00000000
        }
    },
    { 
        .osd_char = 'E', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00100000,
            0b00111000,
            0b00100000,
            0b00100000,
            0b00111110,
            0b00000000
        }
    },
    { 
        .osd_char = 'F', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00100000,
            0b00111000,
            0b00100000,
            0b00100000,
            0b00100000,
            0b00000000
        }
    },
    { 
        .osd_char = 'G', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100010,
            0b00100000,
            0b00100110,
            0b00100010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = 'H', 
        .data = 
        {
            0b000000000,
            0b00100010,
            0b00100010,
            0b00111110,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00000000
        }
    },
    { 
        .osd_char = 'I', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00001000,
            0b00001000,
            0b00001000,
            0b00001000,
            0b00111110,
            0b00000000
        }
    },
    { 
        .osd_char = 'J', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00000100,
            0b00000100,
            0b00000100,
            0b00100100,
            0b00111100,
            0b00000000
        }
    },
    { 
        .osd_char = 'K', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00100100,
            0b00111000,
            0b00101000,
            0b00100100,
            0b00100010,
            0b00000000
        }
    },
    { 
        .osd_char = 'L', 
        .data = 
        {
            0b00000000,
            0b00100000,
            0b00100000,
            0b00100000,
            0b00100000,
            0b00100000,
            0b00111110,
            0b00000000
        }
    },
    { 
        .osd_char = 'M', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00110110,
            0b00101010,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00000000
        }
    },
    { 
        .osd_char = 'N', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00110010,
            0b00101010,
            0b00100110,
            0b00100010,
            0b00100010,
            0b00000000
        }
    },
    { 
        .osd_char = 'O', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = 'P', 
        .data = 
        {
            0b00000000,
            0b00111100,
            0b00100010,
            0b00100010,
            0b00111100,
            0b00100000,
            0b00100000,
            0b00000000
        }
    },
    { 
        .osd_char = 'Q', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00100110,
            0b00011101,
            0b00000000
        }
    },
    { 
        .osd_char = 'R', 
        .data = 
        {
            0b00000000,
            0b00111100,
            0b00100010,
            0b00111110,
            0b00100100,
            0b00100010,
            0b00100010,
            0b00000000
        }
    },
    { 
        .osd_char = 'S', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100000,
            0b00011000,
            0b00001100,
            0b00000010,
            0b00111100,
            0b00000000
        }
    },
    { 
        .osd_char = 'T', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00001000,
            0b00001000,
            0b00001000,
            0b00001000,
            0b00001000,
            0b00000000
        }
    },
    { 
        .osd_char = 'U', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = 'V', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00010100,
            0b00001000,
            0b00001000,
            0b00000000
        }
    },
        { 
        .osd_char = 'W', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00100010,
            0b00100010,
            0b00101010,
            0b00110110,
            0b00100010,
            0b00000000
        }
    },
        { 
        .osd_char = 'X', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00010100,
            0b00001000,
            0b00001000,
            0b00010100,
            0b00100010,
            0b00000000
        }
    },
    { 
        .osd_char = 'Y', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00100010,
            0b00010100,
            0b00001000,
            0b00001000,
            0b00001000,
            0b00000000
        }
    },
    { 
        .osd_char = 'Z', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00000100,
            0b00001000,
            0b00010000,
            0b00100000,
            0b00111110,
            0b00000000
        }
    },
    { 
        .osd_char = '0', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100110,
            0b00101010,
            0b00101010,
            0b00110010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = '1', 
        .data = 
        {
            0b00000000,
            0b00001000,
            0b00011000,
            0b00101000,
            0b00001000,
            0b00001000,
            0b00111110,
            0b00000000
        }
    },
    { 
        .osd_char = '2', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100010,
            0b00000100,
            0b00011000,
            0b00100000,
            0b00111110,
            0b00000000
        }
    },
    { 
        .osd_char = '3', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00000010,
            0b00011100,
            0b00000010,
            0b00100010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = '4', 
        .data = 
        {
            0b00000000,
            0b00100010,
            0b00100010,
            0b00111110,
            0b00000010,
            0b00000010,
            0b00000010,
            0b00000000
        }
    },
    { 
        .osd_char = '5', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00100000,
            0b00111100,
            0b00000010,
            0b00100010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = '6', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100000,
            0b00111100,
            0b00100010,
            0b00100010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = '7', 
        .data = 
        {
            0b00000000,
            0b00111110,
            0b00000010,
            0b00000100,
            0b00001000,
            0b00010000,
            0b00010000,
            0b00000000
        }
    },
    { 
        .osd_char = '8', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100010,
            0b00011100,
            0b00100010,
            0b00100010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = '9', 
        .data = 
        {
            0b00000000,
            0b00011100,
            0b00100010,
            0b00100010,
            0b00011110,
            0b00000010,
            0b00011100,
            0b00000000
        }
    },
    { 
        .osd_char = '!', 
        .data = 
        {
            0b00000000,
            0b00001000,
            0b00001000,
            0b00001000,
            0b00001000,
            0b00000000,
            0b00001000,
            0b00000000
        }
    },
    { 
        .osd_char = ':', 
        .data = 
        {
            0b00000000,
            0b00000000,
            0b00000000,
            0b00010000,
            0b00000000,
            0b00010000,
            0b00000000,
            0b00000000
        }
    },
    { .osd_char = '\0', .data = 0 }  // Keep this
};

//**********************************************************************************************
// PRIVATE FUNCTION PROTOTYPES
//**********************************************************************************************
static uint8_t* get_char_data(char lookup_char);

//**********************************************************************************************
// PUBLIC FUNCTIONS
//**********************************************************************************************
bool OSD_is_enabled(void)
{
    return osd_enabled;
}

void OSD_toggle(void)
{
    osd_enabled = !osd_enabled;
}

void OSD_set_line_text(uint8_t line_index, const char* text)
{
    if (line_index >= OSD_LINES)
        return;

    for (int i = 0; i < OSD_CHARS_PER_LINE; i++)
    {
        osd_text[line_index][i] = i < strlen(text) ? text[i] : ' ';
    }
    osd_text[line_index][OSD_CHARS_PER_LINE] = '\0';
}

void OSD_update(void)
{
    int pos = 0;
    uint8_t color1 = 0x00;
    uint8_t color2 = 0x3C;
    for (int y = 0; y < OSD_LINES; y++)
    {
        for (int n = 0; n < OSD_CHAR_HEIGHT; n++)
        {
            for (int x = 0; x < OSD_CHARS_PER_LINE; x++)
            {
                char myChar = osd_text[y][x];
                uint8_t* char_data = get_char_data(myChar);
                for (int o = OSD_CHAR_WIDTH-1; o >= 0; o--)
                {
                    if (y == active_line)
                    {
                        framebuffer[pos++] = (((char_data[n] >> o) & 1) == 0) ? color2 : color1;
                    }
                    else
                    {
                        framebuffer[pos++] = (((char_data[n] >> o) & 1) == 0) ? color1 : color2;
                    }
                }
            }
        }
    }
}

// uint8_t OSD_get_width(void)
// {
//     return OSD_WIDTH;
// }

// uint8_t OSD_get_height(void)
// {
//     return OSD_HEIGHT;
// }

// uint8_t OSD_get_char_width(void)
// {
//     return OSD_CHAR_WIDTH;
// }

// uint8_t OSD_get_char_height(void)
// {
//     return OSD_CHAR_HEIGHT;
// }

// uint8_t OSD_get_line_count(void)
// {
//     return OSD_LINES;
// }

void OSD_change_line(int direction)
{
    active_line += direction;
    active_line = active_line >= OSD_LINES ? 0 : active_line;
    active_line = active_line < 0 ? OSD_LINES-1 : active_line;

    OSD_update();
}

uint8_t OSD_get_active_line(void)
{
    return (uint8_t)active_line;
}

uint8_t OSD_get_pixel(uint8_t x, uint8_t y)
{
    int pos = x + (y * OSD_WIDTH);
    if (pos >= sizeof(framebuffer))
        return 0;

    return framebuffer[pos];
}

uint8_t* OSD_get_framebuffer(void)
{
    return framebuffer;
}

//**********************************************************************************************
// PRIVATE FUNCTIONS
//**********************************************************************************************
static uint8_t* get_char_data(char lookup_char)
{
    int i = 0;
    while (osd_letters[i].osd_char != '\0')
    {
        if (osd_letters[i].osd_char == lookup_char)
            return osd_letters[i].data;
        
        i++;
    }
    
    // not found, return blank (first element)
    return osd_letters[0].data;
}

