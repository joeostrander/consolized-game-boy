#include "osd.h"
#include "video_defs.h"
#include "font_5x7.h"
#include <string.h>
#include <stdint.h>

static inline void set_pixel_2bpp(uint8_t *buf, int x, int y, uint8_t color)
{
    if ((buf == NULL) || (x < 0) || (x >= DMG_PIXELS_X) || (y < 0) || (y >= DMG_PIXELS_Y)) {
        return;
    }

    // Treat 0xFF as transparent (skip write)
    if (color == 0xFF) {
        return;
    }

    size_t idx = (size_t)y * PACKED_LINE_STRIDE_BYTES + (size_t)(x >> 2);
    uint32_t shift = (3u - (uint32_t)(x & 3)) * 2u;
    uint8_t mask = (uint8_t)(0x03u << shift);
    buf[idx] = (uint8_t)((buf[idx] & ~mask) | ((color & 0x03u) << shift));
}

// static void fill_buffer_2bpp(uint8_t *buf, uint8_t color)
// {
//     if (buf == NULL) {
//         return;
//     }

//     uint8_t packed = (uint8_t)(((color & 0x03u) << 6) | ((color & 0x03u) << 4) | ((color & 0x03u) << 2) | (color & 0x03u));
//     memset(buf, packed, PACKED_FRAME_SIZE);
// }

static void draw_glyph_5x7(uint8_t *buf, int x, int y, const glyph_5x7_t *glyph, uint8_t fg, uint8_t bg)
{
    if ((buf == NULL) || (glyph == NULL)) {
        return;
    }

    for (int row = 0; row < 7; ++row) {
        uint8_t bits = glyph->rows[row];
        for (int col = 0; col < 5; ++col) {
            const bool on = (bits & (0x10 >> col)) != 0;
            set_pixel_2bpp(buf, x + col, y + row, on ? fg : bg);
        }
        set_pixel_2bpp(buf, x + 5, y + row, bg); // 1px spacing
    }
}

static void draw_text_line(uint8_t *buf, int x, int y, const char *text, uint8_t fg, uint8_t bg)
{
    if ((buf == NULL) || (text == NULL)) {
        return;
    }

    // Render exactly OSD_MAX_CHARS glyph slots; pad with spaces
    int cursor_x = x;
    for (int i = 0; i < OSD_MAX_CHARS; ++i) {
        char c = text[i];
        if (c == '\0') c = ' ';
        const glyph_5x7_t *glyph = font5x7_lookup(c);
        draw_glyph_5x7(buf, cursor_x, y, glyph, fg, bg);
        cursor_x += 6; // 5 pixels plus spacing
    }
}

static bool osd_enabled = false;
static uint16_t fb_w = DMG_PIXELS_X;
static uint16_t fb_h = DMG_PIXELS_Y;
static int osd_x = 0;
static int osd_y = 0;
static int osd_width = 0;
static int osd_height = 0;
static int osd_border = 2;
static int osd_padding = 1;
static int osd_active_line = 0;
static char osd_lines[OSD_MAX_LINES][OSD_MAX_CHARS + 1];

static int osd_visible_lines(void)
{
    int last = -1;
    for (int i = 0; i < OSD_MAX_LINES; ++i) {
        if (osd_lines[i][0] != '\0') {
            last = i;
        }
    }
    return last + 1; // 0 if nothing to show
}

// Public API
void OSD_init(uint16_t fb_width, uint16_t fb_height)
{
    fb_w = fb_width;
    fb_h = fb_height;
    osd_width  = 6 * OSD_MAX_CHARS + osd_border * 2 + osd_padding * 2;   // glyphs + spacing + border + padding
    osd_height = 0; // will be set dynamically per render
    osd_x = 0;
    osd_y = 0;
    OSD_clear();
}

void OSD_set_enabled(bool enable)
{
    osd_enabled = enable;
}

void OSD_toggle(void)
{
    osd_enabled = !osd_enabled;
}

bool OSD_is_enabled(void)
{
    return osd_enabled;
}

void OSD_set_line_text(int line, const char *text)
{
    if (line < 0 || line >= OSD_MAX_LINES || text == NULL) {
        return;
    }
    strncpy(osd_lines[line], text, OSD_MAX_CHARS);
    osd_lines[line][OSD_MAX_CHARS] = '\0';
}

void OSD_set_active_line(int line)
{
    int count = osd_visible_lines();
    if (count <= 0) {
        osd_active_line = 0;
        return;
    }
    if (line < 0) line = 0;
    if (line >= count) line = count - 1;
    osd_active_line = line;
}

void OSD_change_active_line(int delta)
{
    int count = osd_visible_lines();
    if (count <= 0) {
        osd_active_line = 0;
        return;
    }
    int next = (osd_active_line + delta) % count;
    if (next < 0) next += count;
    osd_active_line = next;
}

int OSD_get_active_line(void)
{
    return osd_active_line;
}

void OSD_clear(void)
{
    for (int i = 0; i < OSD_MAX_LINES; ++i) {
        osd_lines[i][0] = '\0';
    }
    osd_active_line = 0;
}

void OSD_render(uint8_t *packed_buf)
{
    if (!osd_enabled || packed_buf == NULL) {
        return;
    }

    int line_count = osd_visible_lines();
    if (line_count <= 0) {
        return;
    }

    if (osd_active_line >= line_count) {
        osd_active_line = line_count - 1;
    }

    osd_height = line_count * 8 + osd_border * 2 + osd_padding * 2;
    osd_x = (int)(fb_w - osd_width) / 2;
    osd_y = (int)(fb_h - osd_height) / 2;

    const uint8_t bg = 3;             // dark fill for text background
    const uint8_t border = 2;         // slightly lighter border
    const uint8_t highlight_bg = 1;   // lighter fill for active line
    const uint8_t fg = 0;             // bright text

    // Outer border box
    for (int y = 0; y < osd_height; ++y) {
        for (int x = 0; x < osd_width; ++x) {
            set_pixel_2bpp(packed_buf, osd_x + x, osd_y + y, border);
        }
    }

    // Inner fill
    int inner_w = osd_width - osd_border * 2;
    int inner_h = osd_height - osd_border * 2;
    int inner_x = osd_x + osd_border;
    int inner_y = osd_y + osd_border;
    for (int y = 0; y < inner_h; ++y) {
        for (int x = 0; x < inner_w; ++x) {
            set_pixel_2bpp(packed_buf, inner_x + x, inner_y + y, bg);
        }
    }

    // Render lines inside border; 8px stride fills inter-line gap with bg
    int text_origin_y = inner_y + osd_padding;
    int text_origin_x = inner_x + osd_padding;
    int text_width = 6 * OSD_MAX_CHARS;
    for (int i = 0; i < line_count; ++i) {
        int y = text_origin_y + i * 8; // 7px glyph height + 1px spacing
        // Highlight active line across glyph height (leave spacing as bg)
        if (i == osd_active_line) {
            for (int row = 0; row < 7; ++row) {
                for (int x = 0; x < text_width; ++x) {
                    set_pixel_2bpp(packed_buf, text_origin_x + x, y + row, highlight_bg);
                }
            }
        }
        draw_text_line(packed_buf, text_origin_x, y, osd_lines[i], fg, (i == osd_active_line) ? highlight_bg : bg);
    }
}