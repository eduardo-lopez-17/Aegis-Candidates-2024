#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

#include "Servo.h"
#include "NewPing.h"
#include "Adafruit_TCS34725.h"

#include "lib/Colors.h"
#include "lib/Azimuth.h"

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

void halt();

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

Adafruit_TCS34725 sensorColor = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_600MS,
    TCS34725_GAIN_4X);

static const uint8_t RED_MIN = 80;
static const uint8_t RED_MAX = 255;
static const uint8_t GREEN_MIN = 80;
static const uint8_t GREEN_MAX = 255;
static const uint8_t BLUE_MIN = 80;
static const uint8_t BLUE_MAX = 255;

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

static const uint16_t TIME_TO_ADVANCE_HALF_TILE = 2000;
static const uint16_t TIME_TO_TURN = 2000;

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
    // detectZone();
    zoneB();
    // readColor();
    // delay(100);
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
    
    delay(TIME_TO_ADVANCE_HALF_TILE);
    
    turnOffMotors();
}

void stepBack()
{
    move(Backwards);
    
    delay(TIME_TO_ADVANCE_HALF_TILE);
    
    turnOffMotors();
}

void turnRight()
{
    move(Right);
    
    delay(TIME_TO_TURN);
    
    turnOffMotors();
}

void turnLeft()
{
    move(Left);
    
    delay(TIME_TO_TURN);
    
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
    ++leftEncoderCounter;
}

void rightEncoder()
{
    ++rightEncoderCounter;
}

/// Color sensor

Color readColor()
{
    float red, green, blue = 0;
    // Take 5 samples
    // for (uint8_t i = 0; i < 5; ++i)
    // {
    //     float r, g, b = 0;
    //     sensorColor.getRGB(&r, &g, &b);
    //     red += r;
    //     green += g;
    //     blue += b;
    // }
    
    // // Average
    // red /= 5;
    // green /= 5;
    // blue /= 5;
    
    sensorColor.getRGB(&red, &green, &blue);
    
    // const int16_t r = constrain(map(red, RED_MIN, RED_MAX, 0, 255), 0, 255);
    // const int16_t g = constrain(map(green, GREEN_MIN, GREEN_MAX, 0, 255), 0, 255);
    // const int16_t b = constrain(map(blue, BLUE_MIN, BLUE_MAX, 0, 255), 0, 255);
    
    
    
    Color color;
    // color.red = r;
    // color.green = g;
    // color.blue = b;
    
    color.red = red;
    color.green = green;
    color.blue = blue;
    
    Serial.println("Colors");
    Serial.println(color.red);
    Serial.println(color.green);
    Serial.println(color.red); 
    
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
    // I saw and we need a variable to remember where we were searching
    Direction lastTurn = Left;
    
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
                lastTurn = Left;
                move(Left);
                break;
            // Left and right IRs detect a black line
            case 0b010:
                move(Forward);
                break;
            // Left IR detects a black line
            case 0b011:
                lastTurn = Left;
                move(Left);
                break;
            // Center and right IRs detect a black line
            case 0b100:
                lastTurn = Right;
                move(Right);
                break;
            // Center IR detects a black line
            case 0b101:
                move(Forward);
                break;
            // Right IR detects a black line
            case 0b110:
                lastTurn = Right;
                move(Right);
                break;
            // No line detected
            case 0b111:
                
                isEndReached = scan(lastTurn);
                
                break;
        }
    }
    
    // We are done
    Serial.println("Zone B done");
    halt();
}

bool scan(Direction &lastTurn)
{
    Serial.println("Scanning");
    
    // I got it, we should do a little scan first.
    // If found, we procede
    // Otherwise, we stop
    
    Direction firstSide = Left;
    Direction SecondSide = Right;
    // But first we need to remember which side we were moving to
    
    // We kinda consider that we are following a straight line
    // So we say we need to turn back to the last time we turning
    // e. g. If we were turning right, and the middle sensor loses
    // track of the line, we turn left, since we might have go to
    // the right side of the line, so we need to go left to find it
    // again.
    // For the left and right sensors, we "interpolate" were the line
    // should be.
    
    // Hope this explains it right. I think I wasted more time typing
    // these comments other than typing the code. *sad*
    if (direction == Forward)
    {
        if (lastTurn == Left)
        {
            firstSide = Right;
            SecondSide = Left;
        }
        else if (lastTurn == Right)
        {
            firstSide = Left;
            SecondSide = Right;
        }
    }
    else
    {
        if (lastTurn == Left)
        {
            firstSide = Left;
            SecondSide = Right;
        }
        else if (lastTurn == Right)
        {
            firstSide = Right;
            SecondSide = Left;
        }
    }
    
    turnOffMotors();
    
    bool lineIsLocatedFirstSide = false;
    lastTurn = firstSide;
    move(firstSide);
    
    unsigned long initialTime = millis();
    // In miliseconds
    const unsigned long finalTime = 1000;
    
    while (millis() - initialTime <= finalTime)
    {
        if (!digitalRead(WEST_IR_PIN))
        {
            lineIsLocatedFirstSide = true;
            break;
        }
        
        if (!digitalRead(NORTH_IR_PIN))
        {
            lineIsLocatedFirstSide = true;
            break;
        }
        
        if (!digitalRead(EAST_IR_PIN))
        {
            lineIsLocatedFirstSide = true;
            break;
        }
    }
    
    move(Standing);
    
    delay(100);
    
    // We should continue with the code
    if (lineIsLocatedFirstSide) return false;
    
    // Move back
    lastTurn = SecondSide;
    move(SecondSide);
    
    delay(finalTime);
    
    Serial.println("Scanning Right");
    
    bool lineIsLocatedSecondSide = false;
    initialTime = millis();
    
    while (millis() - initialTime <= finalTime)
    {
        if (!digitalRead(WEST_IR_PIN))
        {
            lineIsLocatedSecondSide = true;
            break;
        }
        
        if (!digitalRead(NORTH_IR_PIN))
        {
            lineIsLocatedSecondSide = true;
            break;
        }
        
        if (!digitalRead(EAST_IR_PIN))
        {
            lineIsLocatedSecondSide = true;
            break;
        }
    }
    
    move(Standing);
    
    delay(100);
    
    lastTurn = firstSide;
    move(firstSide);
    
    delay(finalTime);
    
    move(Standing);
    
    // Yeah we are done with this zone
    if (!lineIsLocatedSecondSide) return true;
    
    return false;
}

void zoneC()
{
    // We are going to make a laberynth solver
    
    // Size of the map minus 1
    const uint8_t sizeX = 3; // Size of 3 really
    const uint8_t sizeY = 6; // Size of 6 really
    
    // Initial coordinates in the plane
    uint8_t coordinateX = 1;
    uint8_t coordinateY = 0;
    
    // Default facing
    uint8_t cardinalDirection = North;
    
    // Using the right hand method to solve the maze
    if (!isWallInRight())
    {
        // Go right
        
        // Rotate the view to 90 degrees to the right
        cardinalDirection = (cardinalDirection + 1) % 4;
        turnRight();
        
        // Move coordinate
        
        updateCoordinate(cardinalDirection, coordinateX, coordinateY);
        
        stepForward();
        stepForward();
    }
    else if (!isWallInFront())
    {
        // Go forward
        // And update coordinate
        
        updateCoordinate(cardinalDirection, coordinateX, coordinateY);
        
        stepForward();
        stepForward();
    }
    else
    {
        // Turn left
        
        // Rotate the view to 90 degrees to the left
        cardinalDirection = (cardinalDirection - 1) % 4;
        turnLeft();
    }
    
    if (coordinateX == 2 & coordinateY == 5)
    {
        // Done!
        halt();
    }
    
    // Color color = readColor();
    
    // if (color.red >= color.blue & color.red >= color.green)
    // {
    //     // We are finished
        
    //     halt();
    // }
    
}

void updateCoordinate(uint8_t cardinalDirection, uint8_t &x, uint8_t &y)
{
    switch (cardinalDirection)
    {
        case North:
            ++y;
            break;
        case South:
            --y;
            break;
        case West:
            --x;
            break;
        case East:
            ++x;
            break;
    }
}

void seesaw()
{
    // I honestly don't know how we can approach this without the gyro.
}

void halt()
{
    turnOffMotors();
    for (;;);
}
