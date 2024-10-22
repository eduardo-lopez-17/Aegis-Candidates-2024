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

static const uint8_t WALL_OFFSET = 15;

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

enum Direction {
    Forward = 0,
    Backwards,
    Left,
    Right,
    Standing
};

Direction direction = Standing;
static const uint8_t BASE_SPEED = 10;

// Encoder

static const uint8_t LEFT_ENCODER_PIN = 2;
static const uint8_t RIGHT_ENCODER_PIN = 3;

static uint16_t leftEncoderCounter = 0;
static uint16_t rightEncoderCounter = 0;

static const uint8_t NUMBER_OF_TEETH = 16;
static const uint16_t DISTANCE_OF_CENTER_TO_WHEEL = 10;
static const uint8_t WHEEL_RADIUS = 13;
static const uint16_t SIDE_OF_UNIT = 30;

/// Main program

void setup()
{
    // Motor
    
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    turnOffMotors();
    
    // Encoder
    
    pinMode(LEFT_ENCODER_PIN, INPUT);
    pinMode(RIGHT_ENCODER_PIN, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoder, RISING);
    
    // IR
    
    pinMode(WEST_IR_PIN, INPUT);
    pinMode(NORTH_IR_PIN, INPUT);
    pinMode(EAST_IR_PIN, INPUT);
    
    // For debugging purposes
    
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Servo
    
    gripper.attach(GRIPPER_PIN);
    Serial.begin(9600);
    
    // Color sensor
    
    color.begin();
}

void loop()
{
    readColor();
    delay(1000);
}

/// Motor

void stepforward()
{
    direction = Forward;
    
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED);
    
    delay(1000);
    
    float distanceElapsed = 0;
    
    while (distanceElapsed >= SIDE_OF_UNIT)
    {
        float turns = leftEncoderCounter / NUMBER_OF_TEETH;
        float diameter = WHEEL_RADIUS * 2 * PI;
        distanceElapsed = turns * diameter;
        
        delayMicroseconds(10);
    }
    
    turnOffMotors();
}

void stepBack()
{
    direction = Backwards;
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED);
    
    delay(1000);
    
    turnOffMotors();
}

void turnRight()
{
    direction = Right;
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED);
    
    delay(1000);
    
    turnOffMotors();
}

void turnLeft()
{
    direction = Left;
    
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    
    analogWrite(ENA, BASE_SPEED);
    analogWrite(ENB, BASE_SPEED);
    
    delay(1000);
    
    turnOffMotors();
}

void turnOffMotors()
{
    direction = Standing;
    
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
    
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

/// Encoder

void leftEncoder()
{
    leftEncoderCounter++;
}

void rightEncoder()
{
    rightEncoderCounter++;
}

/// Color sensor

void readColor()
{
    float r, g, b = 0;
    color.getRGB(&r, &g, &b);
    
    Serial.println("Colors: ");
    Serial.println(r);
    Serial.println(g);
    Serial.println(b);
}

/// Ultrasonic

bool isWallInLeft()
{
    return leftUltrasonic.ping_cm() <= WALL_OFFSET;
}

bool isWallInFront()
{
    return frontUltrasonic.ping_cm() <= WALL_OFFSET;
}

bool isWallInRight()
{
    return rightUltrasonic.ping_cm() <= WALL_OFFSET;
}

/// Servo

void openGripper()
{
    gripper.write(0);
}

void closeGripper()
{
    gripper.write(90);
}

/// Zones

void zoneA()
{
    
}

void zoneB()
{
    
}

void zoneC()
{
    
}

void seesaw()
{
    
}
