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

NewPing frontUltrasonic = NewPing(FRONT_ULTRASONIC_PIN, FRONT_ULTRASONIC_PIN);
NewPing leftUltrasonic = NewPing(LEFT_ULTRASONIC_PIN, LEFT_ULTRASONIC_PIN);
NewPing rightUltrasonic = NewPing(RIGHT_ULTRASONIC_PIN, RIGHT_ULTRASONIC_PIN);

// Color

Adafruit_TCS34725 color = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// Infrared

static const uint8_t WEST_IR_PIN = A0;
static const uint8_t NORTH_IR_PIN = A1;
static const uint8_t EAST_IR_PIN = A2;

// Servo

static const uint8_t GRIPPER_PIN = 9;
static const uint8_t GRIPPER_ARM_PIN = 10;

Servo gripper;
Servo gripper_arm;

// Motor

static const uint8_t IN1 = 3;
static const uint8_t IN2 = 4;
static const uint8_t IN3 = 5;
static const uint8_t IN4 = 6;

static const uint8_t ENA = 7;
static const uint8_t ENB = 8;

/// Main program

void setup()
{
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    pinMode(WEST_IR_PIN, INPUT);
    pinMode(NORTH_IR_PIN, INPUT);
    pinMode(EAST_IR_PIN, INPUT);
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    
    gripper.attach(GRIPPER_PIN);
    Serial.begin(9600);
    
    color.begin();
}

void loop()
{
    // Serial.print("Detects object at ");
    // Serial.println(leftUltrasonic.ping_cm());
    float r, g, b = 0;
    color.getRGB(&r, &g, &b);
    Serial.print("Colors: ");
    Serial.println(r);
    Serial.println(g);
    Serial.println(b);
    
    // stepforward();
    // delay(1000);
    // stepBack();
    
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(100);
    // digitalWrite(LED_BUILTIN, LOW);
    // delay(100);
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(100);
    // digitalWrite(LED_BUILTIN, LOW);
    
    // delay(1000);
    // digitalWrite(LED_BUILTIN, digitalRead(NORT_WEST_IR_PIN));
    // delay(1000);
    
    // digitalWrite(LED_BUILTIN, HIGH);
    // delay(100);
    // digitalWrite(LED_BUILTIN, LOW);
    
    // delay(1000);
    // digitalWrite(LED_BUILTIN, digitalRead(NORTH_IR_PIN));
    // delay(1000);
    
    // gripper.write(0);
    // delay(1000);
    // gripper.write(90);
    // delay(1000);
    
    
    
    
    
}

void stepforward()
{
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    delay(1000);
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void stepBack()
{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    delay(1000);
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
}

void turnRight()
{
    
}

void turnLeft()
{
    
}