#ifndef OSD_H
#define OSD_H

#include <stdbool.h>
#include <stdint.h>

#define OSD_MAX_LINES   10
#define OSD_MAX_CHARS   21

void OSD_init(uint16_t fb_width, uint16_t fb_height);
void OSD_set_enabled(bool enable);
void OSD_toggle(void);
bool OSD_is_enabled(void);
void OSD_set_line_text(int line, const char *text);
void OSD_set_active_line(int line);
void OSD_change_active_line(int delta);
int  OSD_get_active_line(void);
void OSD_clear(void);
void OSD_render(uint8_t *packed_buf);

#endif // OSD_H