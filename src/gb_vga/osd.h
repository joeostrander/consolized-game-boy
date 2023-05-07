#ifndef OSD_H
#define OSD_H

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

#define OSD_CHAR_WIDTH      (7)
#define OSD_CHAR_HEIGHT     (8)
#define OSD_LINES           (4)
#define OSD_CHARS_PER_LINE  (18)
#define OSD_HEIGHT          (OSD_LINES*OSD_CHAR_HEIGHT)
#define OSD_WIDTH           (OSD_CHAR_WIDTH*OSD_CHARS_PER_LINE)

bool OSD_is_enabled(void);
void OSD_toggle(void);
void OSD_set_line_text(uint8_t line_index, const char* text);
void OSD_update(void);
// uint8_t OSD_get_width(void);
// uint8_t OSD_get_height(void);
// uint8_t OSD_get_char_width(void);
// uint8_t OSD_get_char_height(void);
//uint8_t OSD_get_line_count(void);
void OSD_change_line(int direction);
uint8_t OSD_get_active_line(void);
uint8_t OSD_get_pixel(uint8_t x, uint8_t y);
uint8_t* OSD_get_framebuffer(void);

#endif // OSD_H