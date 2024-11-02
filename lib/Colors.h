/**
 * colors.h - Library used to define and simplify colors identification.
 * Autor: Angel Eduardo López.
 */

#ifndef COLORS_H
#define COLORS_H

#include <stdint.h>
#include "Arduino.h"

/// Definitions

// Enums

enum Colors
{
    RED = 0,
    ORANGE,
    YELLOW,
    GREEN,
    BLUE,
    MAGENTA,
    PURPLE,
    PINK,
    BROWN,
    GRAY,
    BLACK,
    WHITE,
    NO_COLOR
};

// Structs

struct Color
{
    uint8_t red;
    uint8_t green;
    uint8_t blue;
};

/// Function Prototypes

bool isInRange(uint8_t value, uint8_t ref);
bool checkColor(Color color, Color refColor);
Colors identifyColor(Color color);

// Color definitions

static Color RED_RGB;
static Color GREEN_RGB;
static Color YELLOW_RGB;
static Color MAGENTA_RGB;
static Color PURPLE_RGB;

bool isInRange(uint8_t value, uint8_t ref)
{
    // In %
    const float ERROR_MARGIN = 0.1;
    const uint8_t HIGH_MARGIN = (uint8_t)constrain(float(ref * (1 + ERROR_MARGIN)), 0, 255);
    const uint8_t LOW_MARGIN = (uint8_t)constrain(float(ref * (1 - ERROR_MARGIN)), 0, 255);
    
    return LOW_MARGIN <= value & value <= HIGH_MARGIN;
}

bool checkColor(Color color, Color ref)
{
    if (!isInRange(color.red, ref.red)) return false;
    if (!isInRange(color.green, ref.green)) return false;
    if (!isInRange(color.blue, ref.blue)) return false;
    
    return true;
}

Colors identifyColor(Color color)
{
    if (checkColor(color, RED_RGB))
    {
        return RED;
    }
    else if (checkColor(color, GREEN_RGB))
    {
        return GREEN;
    }
    else if (checkColor(color, YELLOW_RGB))
    {
        return YELLOW;
    }
    else if (checkColor(color, MAGENTA_RGB))
    {
        return MAGENTA;
    }
    else if (checkColor(color, PURPLE_RGB))
    {
        return PURPLE;
    }
    
    return NO_COLOR;
}

#endif
