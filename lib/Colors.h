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

void initializeColorDefinitions()
{
    RED_RGB.red = 190;
    RED_RGB.green = 108;
    RED_RGB.blue = 97;
    
    GREEN_RGB.red = 118;
    GREEN_RGB.green = 142;
    GREEN_RGB.blue = 90;
    
    YELLOW_RGB.red = 142;
    YELLOW_RGB.green = 115;
    YELLOW_RGB.blue = 66;
    
    MAGENTA_RGB.red = 166;
    MAGENTA_RGB.green = 103;
    MAGENTA_RGB.blue = 124;
    
    PURPLE_RGB.red = 135;
    PURPLE_RGB.green = 119;
    PURPLE_RGB.blue = 130;
}

bool isInRange(uint8_t value, uint8_t ref)
{
    // In %
    const float ERROR_MARGIN = 0.2;
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
