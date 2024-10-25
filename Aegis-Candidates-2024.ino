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

void stepForward();
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

static Servo gripper;
static Servo gripper_arm;

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

static Direction direction = Standing;
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

void move(Direction dir, uint8_t speed = BASE_SPEED)
{
    direction = dir;
    
    switch (direction)
    {
        case Forward:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            break;
        case Backwards:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            break;
        case Left:
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, HIGH);
            break;
        case Right:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, HIGH);
            digitalWrite(IN3, HIGH);
            digitalWrite(IN4, LOW);
            break;
        
        default:
            digitalWrite(IN1, LOW);
            digitalWrite(IN2, LOW);
            digitalWrite(IN3, LOW);
            digitalWrite(IN4, LOW);
            break;
    }
    
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void stepForward()
{
    leftEncoderCounter = 0;
    rightEncoderCounter = 0;
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoder, RISING);
    
    move(Forward);
    
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
    move(Backwards);
    
    delay(1000);
    
    turnOffMotors();
}

void turnRight()
{
    move(Right);
    
    delay(1000);
    
    turnOffMotors();
}

void turnLeft()
{
    move(Left);
    
    delay(1000);
    
    turnOffMotors();
}

inline void turnOffMotors()
{
    move(Standing, 0);
}

/// Encoder

void startUsingEncoder()
{
    leftEncoderCounter = 0;
    rightEncoderCounter = 0;
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoder, RISING);
}

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
    // stepForward();
}

void zoneB()
{
    while (true)
    {
        const byte WEST_IR_BIT = digitalRead(WEST_IR_PIN) << 2;
        const byte NORTH_IR_BIT = digitalRead(NORTH_IR_PIN) << 1;
        const byte EAST_IR_BIT = digitalRead(EAST_IR_PIN) << 0;
        
        // Concatenate bits
        const byte IR_BYTES = WEST_IR_BIT | NORTH_IR_BIT | EAST_IR_BIT;
        
        switch (IR_BYTES)
        {
            // All IRs detect a black line
            case 0x000:
                move(Forward);
                break;
            // Left and center IRs detect a black line
            case 0x001:
                move(Left);
                break;
            // Left and right IRs detect a black line
            case 0x010:
                move(Forward);
                break;
            // Left IR detects a black line
            case 0x011:
                move(Left);
                break;
            // Center and right IRs detect a black line
            case 0x100:
                move(Right);
                break;
            // Center IR detects a black line
            case 0x101:
                move(Forward);
                break;
            // Right IR detects a black line
            case 0x110:
                move(Right);
                break;
            // No line detected
            case 0x111:
                // It is supposed that we need to stop here
                // However, I am not sure if we have the correct appproach.
                turnOffMotors();
                break;
        }
    }
    
}

void zoneC()
{
    
}

void seesaw()
{
    
}
