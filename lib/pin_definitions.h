/**
 * Azimuth.h - Library used to define pins.
 * Autor: Angel Eduardo López.
 */

#ifndef PIN_DEFINITIONS
#define PIN_DEFINITIONS

#include "Arduino.h"

#include <stdint.h>
#include <stdbool.h>

// Ultrasonic

static const uint8_t FRONT_ULTRASONIC_PIN = 10;
static const uint8_t LEFT_ULTRASONIC_PIN = 11;
static const uint8_t RIGHT_ULTRASONIC_PIN = 12;

// Infrared

static const uint8_t WEST_IR_PIN = A0;
static const uint8_t NORTH_IR_PIN = A1;
static const uint8_t EAST_IR_PIN = A2;

// Servo

static const uint8_t GRIPPER_PIN = 9;

// Motor

static const uint8_t IN1 = 2;
static const uint8_t IN2 = 3;
static const uint8_t IN3 = 4;
static const uint8_t IN4 = 7;

static const uint8_t ENA = 5;
static const uint8_t ENB = 6;

// Encoder

// It is supposed that we are not using these
static const uint8_t LEFT_ENCODER_PIN = 2;
static const uint8_t RIGHT_ENCODER_PIN = 3;

// LED

static const uint8_t LED_RED_PIN = 8;
static const uint8_t LED_GREEN_PIN = 13;
static const uint8_t LED_BLUE_PIN = A3;

#endif
