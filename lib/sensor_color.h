#ifndef SENSOR_COLOR_H
#define SENSOR_COLOR_H

#include "Adafruit_TCS34725.h"
#include "colors.h"

// Color

Adafruit_TCS34725 sensorColor = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_540MS,
    TCS34725_GAIN_4X);

/// Color sensor

Colors readColor()
{
    float red, green, blue = 0;
    
    sensorColor.getRGB(&red, &green, &blue);
    
    Color color;
    color.red = red;
    color.green = green;
    color.blue = blue;
    
    Serial.println("Colors");
    Serial.println(color.red);
    Serial.println(color.green);
    Serial.println(color.blue); 
    
    return identifyColor(color);
}

void initializeSensorColor()
{
    sensorColor.begin();
}

#endif
