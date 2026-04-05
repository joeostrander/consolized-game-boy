#ifndef FONT_5X7_H
#define FONT_5X7_H

#include <stddef.h>
#include <stdint.h>

typedef struct {
    char ch;
    uint8_t rows[7];
} glyph_5x7_t;

extern const glyph_5x7_t font5x7_default[];
extern const size_t font5x7_default_count;

const glyph_5x7_t *font5x7_lookup(char c);

#endif // FONT_5X7_H
