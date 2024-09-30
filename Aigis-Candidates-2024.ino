#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

#include "Servo.h"
#include "NewPing.h"
#include "Adafruit_TCS34725.h"

/// Functions

// Color

void readColor();

// Motor

void stepforward();
void stepBack();

void turnRight();
void turnLeft();

// Zones

void zoneA();
void zoneB();
void zoneC();
void seesaw();

/// Variables

// Ultrasonic

uint8_t frontUltrasonicPin = 10;
uint8_t leftUltrasonicPin = 11;
uint8_t rightUltrasonicPin = 12;

NewPing frontUltrasonic();
NewPing leftUltrasonic();
NewPing rightUltrasonic();

// Color

Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Infrared

uint8_t westIRPin = 13;
uint8_t northWestIRPin = A0;
uint8_t northIRPin = A1;
uint8_t northEastIRPin = A2;
uint8_t eastIRPin = A3;

// Servo

uint8_t gripperPin = 9;

Servo gripper;

// Motor

uint8_t iN1 = 3;
uint8_t iN2 = 4;
uint8_t iN3 = 5;
uint8_t iN4 = 6;

uint8_t iNA = 7;
uint8_t iNB = 8;

/// Main program

void setup()
{
    
}

void loop()
{
    
}

