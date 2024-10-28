#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

#include "Servo.h"
#include "NewPing.h"
#include "Adafruit_TCS34725.h"

#include "lib/Colors.h"

/// Defines

#define DEBUG true
#define ENABLE_ENCODER false

/// Functions

// Color

Color readColor();

// Motor

void stepForward();
void stepBack();

void turnRight();
void turnLeft();

// Zones

void detectZone();
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

Adafruit_TCS34725 sensorColor = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_600MS, TCS34725_GAIN_4X);

static const uint8_t RED_MIN = 95;
static const uint8_t RED_MAX = 126;
static const uint8_t GREEN_MIN = 93;
static const uint8_t GREEN_MAX = 143;
static const uint8_t BLUE_MIN = 63;
static const uint8_t BLUE_MAX = 120;

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

static const uint8_t IN1 = 2;
static const uint8_t IN2 = 3;
static const uint8_t IN3 = 4;
static const uint8_t IN4 = 7;

static const uint8_t ENA = 5;
static const uint8_t ENB = 6;

enum Direction {
    Forward = 0,
    Backwards,
    Left,
    Right,
    Standing
};

static Direction direction = Standing;
static const uint8_t BASE_SPEED = 50;

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
    
    #if ENABLE_ENCODER
    
    pinMode(LEFT_ENCODER_PIN, INPUT);
    pinMode(RIGHT_ENCODER_PIN, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoder, RISING);
    
    #endif
    
    // IR
    
    pinMode(WEST_IR_PIN, INPUT);
    pinMode(NORTH_IR_PIN, INPUT);
    pinMode(EAST_IR_PIN, INPUT);
    
    // Servo
    
    gripper.attach(GRIPPER_PIN);
    
    // Color sensor
    
    sensorColor.begin();
    
    Serial.begin(9600);
    Serial.println("Working");
}

void loop()
{
    detectZone();
    
    // readColor();
    // delay(1000);
}

/// Motor

void move(Direction dir, uint8_t speed = BASE_SPEED)
{
    if (direction != dir)
    {
        direction = dir;
        
        switch (direction)
        {
            case Forward:
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, HIGH);
                digitalWrite(IN4, LOW);
                Serial.println("Forward");
                break;
            case Backwards:
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, HIGH);
                digitalWrite(IN3, LOW);
                digitalWrite(IN4, HIGH);
                Serial.println("Backwards");
                break;
            case Left:
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, LOW);
                digitalWrite(IN4, HIGH);
                Serial.println("Left");
                break;
            case Right:
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, HIGH);
                digitalWrite(IN3, HIGH);
                digitalWrite(IN4, LOW);
                Serial.println("Right");
                break;
            
            default:
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, LOW);
                digitalWrite(IN4, LOW);
                Serial.println("Not moving");
                break;
        }
    }
    
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
}

void stepForward()
{
    #if ENABLE_ENCODER
    
    leftEncoderCounter = 0;
    rightEncoderCounter = 0;
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoder, RISING);
    
    #endif
    
    move(Forward);
    
    #if ENABLE_ENCODER
    
    float distanceElapsed = 0;
    
    while (distanceElapsed >= SIDE_OF_UNIT / 2)
    {
        float turns = leftEncoderCounter / NUMBER_OF_TEETH;
        float diameter = WHEEL_RADIUS * 2 * PI;
        distanceElapsed = turns * diameter;
        
        delayMicroseconds(10);
    }
    
    #endif
    
    delay(1000);
    
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

Color readColor()
{
    float red, green, blue = 0;
    // Take 5 samples
    for (uint8_t i = 0; i < 5; ++i)
    {
        float r, g, b = 0;
        sensorColor.getRGB(&r, &g, &b);
        red += r;
        green += g;
        blue += b;
    }
    
    // Average
    red /= 5;
    green /= 5;
    blue /= 5;
    
    const int16_t r = constrain(map(red, RED_MIN, RED_MAX, 0, 255), 0, 255);
    const int16_t g = constrain(map(green, GREEN_MIN, GREEN_MAX, 0, 255), 0, 255);
    const int16_t b = constrain(map(blue, BLUE_MIN, BLUE_MAX, 0, 255), 0, 255);
    
    Color color;
    color.red = r;
    color.green = g;
    color.blue = b;
    
    return color;
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

void detectZone()
{
    struct Color color = readColor();
    
    // Do some analysis
    
    // We mainly need to detect 2 colors blue and green I guess
    // If none then it should be zone B
    
    if (color.blue > color.green & color.blue > color.red)
    {
        zoneC();
    }
    else if (color.green > color.blue & color.green > color.red)
    {
        zoneA();
    }
    else
    {
        zoneB();
    }
}

void zoneA()
{
    // stepForward();
}

void zoneB()
{
    bool isEndReached = false;
    while (!isEndReached)
    {
        const byte WEST_IR_BIT = digitalRead(WEST_IR_PIN) << 2;
        const byte NORTH_IR_BIT = digitalRead(NORTH_IR_PIN) << 1;
        const byte EAST_IR_BIT = digitalRead(EAST_IR_PIN);
        
        // Concatenate bits
        const byte IR_BYTES = WEST_IR_BIT | NORTH_IR_BIT | EAST_IR_BIT;
        
        // Serial.println(IR_BYTES, BIN);
        
        switch (IR_BYTES)
        {
            // All IRs detect a black line
            case 0b000:
                move(Forward);
                break;
            // Left and center IRs detect a black line
            case 0b001:
                move(Left);
                break;
            // Left and right IRs detect a black line
            case 0b010:
                move(Forward);
                break;
            // Left IR detects a black line
            case 0b011:
                move(Left);
                break;
            // Center and right IRs detect a black line
            case 0b100:
                move(Right);
                break;
            // Center IR detects a black line
            case 0b101:
                move(Forward);
                break;
            // Right IR detects a black line
            case 0b110:
                move(Right);
                break;
            // No line detected
            case 0b111:
                
                Serial.println("Scanning");
                
                // It is supposed that we need to stop here
                // However, I am not sure if we have the correct appproach.
                turnOffMotors();
                
                Serial.println("Scanning Left");
                
                // I got it, we should do a little scan first.
                // If found, we procede
                // Otherwise, we stop
                bool lineIsLocatedLeft = false;
                move(Left);
                
                unsigned long initialTime = millis();
                // In miliseconds
                const unsigned long finalTime = 1000;
                
                while (millis() - initialTime <= finalTime)
                {
                    if (!digitalRead(WEST_IR_PIN))
                    {
                        lineIsLocatedLeft = true;
                        break;
                    }
                    
                    if (!digitalRead(NORTH_IR_PIN))
                    {
                        lineIsLocatedLeft = true;
                        break;
                    }
                    
                    if (!digitalRead(EAST_IR_PIN))
                    {
                        lineIsLocatedLeft = true;
                        break;
                    }
                }
                
                move(Standing);
                
                delay(100);
                
                if (lineIsLocatedLeft)
                {
                    // We should continue with the code
                    break;
                }
                
                // Move back
                move(Right);
                
                delay(finalTime);
                
                Serial.println("Scanning Right");
                
                bool lineIsLocatedRight = false;
                initialTime = millis();
                
                while (millis() - initialTime <= finalTime)
                {
                    if (!digitalRead(WEST_IR_PIN))
                    {
                        lineIsLocatedRight = true;
                    }
                    
                    if (!digitalRead(NORTH_IR_PIN))
                    {
                        lineIsLocatedRight = true;
                    }
                    
                    if (!digitalRead(EAST_IR_PIN))
                    {
                        lineIsLocatedRight = true;
                    }
                }
                
                move(Standing);
                
                delay(100);
                
                if (!lineIsLocatedRight)
                {
                    isEndReached = true;
                }
                
                move(Left);
                
                delay(finalTime);
                
                move(Standing);
                
                break;
        }
    }
    
    Serial.println("Zone B done");
    turnOffMotors();
    for (;;);
    
    // We are done
}

void zoneC()
{
    
}

void seesaw()
{
    
}
