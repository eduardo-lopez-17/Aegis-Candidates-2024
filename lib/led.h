#ifndef LED_H
#define LED_H

#include "pin_definitions.h"
#include "colors.h"

/// LED

// Returns true if the color was defined, otherwise false
bool turnOnLED(Colors color)
{
    // Asociate the color identificated with a certain partern of the LED
    switch (color)
    {
    case RED:
    case MAGENTA:
        digitalWrite(LED_RED_PIN, HIGH);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_BLUE_PIN, LOW);
        break;
    case GREEN:
    case PURPLE:
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, HIGH);
        digitalWrite(LED_BLUE_PIN, LOW);
        break;
    case BLUE:
    case YELLOW:
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_BLUE_PIN, HIGH);
        break;
    case WHITE:
        digitalWrite(LED_RED_PIN, HIGH);
        digitalWrite(LED_GREEN_PIN, HIGH);
        digitalWrite(LED_BLUE_PIN, HIGH);
        break;
    
    default:
        digitalWrite(LED_RED_PIN, LOW);
        digitalWrite(LED_GREEN_PIN, LOW);
        digitalWrite(LED_BLUE_PIN, LOW);
        return false;
    }
    
    return true;
}

inline void turnOffLED()
{
    turnOnLED(BLACK);
}

void initializeLED()
{
    // LED
    
    pinMode(LED_RED_PIN, OUTPUT);
    pinMode(LED_GREEN_PIN, OUTPUT);
    pinMode(LED_BLUE_PIN, OUTPUT);
    
    digitalWrite(LED_RED_PIN, LOW);
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
}

#endif
