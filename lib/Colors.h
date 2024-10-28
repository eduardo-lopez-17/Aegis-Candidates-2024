/**
 * Colors.h - Library used to define and simplify colors identification.
 * Autor: Angel Eduardo López.
 */

#ifndef COLORS_H
#define COLORS_H

#include <stdint.h>

enum Colors
{
    RED = 0,
    ORANGE,
    YELLOW,
    GREEN,
    BLUE,
    PURPLE,
    PINK,
    BROWN,
    GRAY,
    BLACK,
    WHITE
};

struct Color
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

#endif