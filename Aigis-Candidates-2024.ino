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

static const uint8_t FRONT_ULTRASONIC_PIN = 10;
static const uint8_t LEFT_ULTRASONIC_PIN = 11;
static const uint8_t RIGHT_ULTRASONIC_PIN = 12;

NewPing frontUltrasonic();
NewPing leftUltrasonic();
NewPing rightUltrasonic();

// Color

Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Infrared

static const uint8_t WEST_IR_PIN = 13;
static const uint8_t NORT_WEST_IR_PIN = A0;
static const uint8_t NORTH_IR_PIN = A1;
static const uint8_t NORTH_EAST_IR_PIN = A2;
static const uint8_t EAST_IR_PIN = A3;

// Servo

static const uint8_t GRIPPER_PIN = 9;

Servo gripper;

// Motor

static const uint8_t IN1 = 3;
static const uint8_t IN2 = 4;
static const uint8_t IN3 = 5;
static const uint8_t IN4 = 6;

static const uint8_t INA = 7;
static const uint8_t INB = 8;

/// Main program

void setup()
{
    
}

void loop()
{
    
}

