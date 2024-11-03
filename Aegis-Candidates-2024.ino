#include <Arduino.h>

#include <stdint.h>
#include <stdbool.h>

#include "Servo.h"
#include "NewPing.h"
#include "Adafruit_TCS34725.h"

#include "lib/colors.h"
#include "lib/azimuth.h"
#include "lib/pin_definitions.h"

#include "lib/motor.h"
#include "lib/ultrasonic.h"
#include "lib/servo_controller.h"
#include "lib/ir.h"
#include "lib/led.h"
#include "lib/sensor_color.h"

/// Defines

#define DEBUG true
#define ENABLE_ENCODER false

/// Structs

struct Coordinate
{
    uint8_t x;
    uint8_t y;
};

/// Variables



#if ENABLE_ENCODER

// Encoder

static uint16_t leftEncoderCounter = 0;
static uint16_t rightEncoderCounter = 0;

static const uint8_t NUMBER_OF_TEETH = 16;
static const uint16_t DISTANCE_OF_CENTER_TO_WHEEL = 10;
static const uint8_t WHEEL_RADIUS = 13;
static const uint16_t SIDE_OF_UNIT = 30;

#endif

/// Function Prototype

// Color definitions
void initializeColorDefinitions();
// Motors
void move(Direction dir, uint8_t speed = BASE_SPEED);
inline void turnOffMotors();
void stepForward();
void stepBack();
void turnRight();
void turnLeft();
void initializeMotor();
// Encoder
void startUsingEncoder();
void leftEncoder();
void rightEncoder();
// Color sensor
void initializeSensorColor();
Colors readColor();
// Ultrasonic
bool isWallInLeft();
bool isWallInFront();
bool isWallInRight();
// IR
void initializeIR();
byte readIRs();
// Servo
void openGripper();
void closeGripper();
// LED
bool turnOnLED(Colors color);
inline void turnOffLED();
void initializeLED();
// Zones
void detectZone();
void zoneA();
void zoneB();
bool scan(Direction &lastTurn);
void zoneC();
void updateCoordinate(uint8_t cardinalDirection, uint8_t &x, uint8_t &y);
void seesaw();
// Turn off
void halt();

/// Main program

void setup()
{
    initializeColorDefinitions();
    
    initializeMotor();
    
    #if ENABLE_ENCODER
    
    // Encoder
    
    pinMode(LEFT_ENCODER_PIN, INPUT);
    pinMode(RIGHT_ENCODER_PIN, INPUT);
    
    attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN), leftEncoder, RISING);
    attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), rightEncoder, RISING);
    
    #endif
    
    initializeIR();
    
    // Servo
    
    gripper.attach(GRIPPER_PIN);
    
    initializeSensorColor();
    
    initializeLED();
    
    Serial.begin(9600);
    Serial.println("Working");
}

void loop()
{
    // stepForward();
    // stepForward();
    
    // delay(1000);
    
    // turnRight();
    zoneA();
    // zoneB();
    // zoneC();
    
    // stepForward();
    // stepForward();
    // stepForward();
    // stepForward();
    // stepForward();
    // stepForward();
    
    // turnOnLED(readColor());
    // halt();
    
    
    // turnOnLED(readColor());
    
    // Serial.println("Ultrasonic");
    // Serial.println(isWallInFront());
    // Serial.println(isWallInLeft());
    // Serial.println(isWallInRight());
    
    // detectZone();
    
    // delay(1000);
    // halt();
}

#if ENABLE_ENCODER

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

#endif

/// Zones

void detectZone()
{
    Colors color = readColor();
    
    // Do some analysis
    
    // We mainly need to detect 2 colors blue and green I guess
    // If none then it should be zone B
    
    switch (color)
    {
    case GREEN:
        zoneA();
        break;
    case BLUE:
        zoneC();
        break;
    
    default:
        zoneB();
        break;
    }
}

void zoneA()
{
    // We have to run around the center, then we procede to "grab the ball"
    // I don't think we can make the servo work, nor we have time for that.
    // But yeah whatever.
    
    // Size of the map
    const uint8_t sizeX = 3; // Size of 3
    const uint8_t sizeY = 5; // Size of 6
    
    // Initial coordinates in the plane
    Coordinate coordinate;
    coordinate.x = 1;
    coordinate.y = 0;
    
    // Initialize blacklist
    const uint8_t blacklistSize = 2;
    uint8_t blackListCount = 0;
    Coordinate blacklist[blacklistSize];
    for (uint8_t i = 0; i < blacklistSize; ++i)
    {
        blacklist[i].x = sizeX;
        blacklist[i].y = sizeY;
    }
    
    // Default facing
    uint8_t cardinalDirection = North;
    
    bool lookForEnd = false;
    
    updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
    stepForward();
    stepForward();
    
    cardinalDirection = (cardinalDirection - 1) % 4;
    turnLeft();
    
    // Using the right hand method to find the center
    if (!isWallInRight() & !isBlackTileInFront(cardinalDirection, Right, coordinate, blacklist))
    {
        // Go right
        
        bool isWallLeft = isWallInLeft();
        
        // Rotate the view to 90 degrees to the right
        cardinalDirection = (cardinalDirection + 1) % 4;
        turnRight();
        
        if (isWallLeft)
        {
            // Recalibrate orientation
            stepBack();
            stepForward();
        }
        
        // Move coordinate
        
        updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
        
        if (coordinate.x == 1 & coordinate.y == 2)
        {
            // Seems we have found the place of the ball
            stepForward();
            
            // Do some magic with the servo and get the ball
            closeGripper();
            delay(1000);
            
            updateCoordinate((cardinalDirection + 2) % 4, coordinate.x, coordinate.y);
            stepBack();
            
            cardinalDirection = (cardinalDirection - 1) % 4;
            turnLeft();
            
            // Release ball
            openGripper();
            delay(1000);
            
            cardinalDirection = (cardinalDirection + 1) % 4;
            turnRight();
            cardinalDirection = (cardinalDirection + 1) % 4;
            turnRight();
            
            updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
            
            stepForward();
            stepForward();
            
            // Now we have to look for the end zone now
            lookForEnd = true;
        }
        else
        {
            stepForward();
            
            // If we detect something black
            if (readIRs() != 0b111)
            {
                stepBack();
                // We should mark this place cannot be touched
                blacklist[blackListCount].x = coordinate.x;
                blacklist[blackListCount].y = coordinate.y;
                
                ++blackListCount;
                
                Serial.println("Black tile in right!");
                
                // Return to the last coordinate
                cardinalDirection = (cardinalDirection + 2) % 4;
                updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
                cardinalDirection = (cardinalDirection + 2) % 4;
            }
            else
            {
                stepForward();
            }
        }
    }
    else if (!isWallInFront() & !isBlackTileInFront(cardinalDirection, Right, coordinate, blacklist))
    {
        // Go forward
        // And update coordinate
        
        updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
        
        stepForward();
        
        // If we detect something black
        if (readIRs() != 0b111)
        {
            stepBack();
            // We should mark this place cannot be touched
            blacklist[blackListCount].x = coordinate.x;
            blacklist[blackListCount].y = coordinate.y;
            
            ++blackListCount;
            
            Serial.println("Black tile in forward!");
            
            // Return to the last coordinate
            cardinalDirection = (cardinalDirection + 2) % 4;
            updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
            cardinalDirection = (cardinalDirection + 2) % 4;
        }
        else
        {
            stepForward();
            
            Colors color = readColor();
            bool isLEDDisplayed = turnOnLED(color);
        }
    }
    else
    {
        // Turn left
        
        // Rotate the view to 90 degrees to the left
        cardinalDirection = (cardinalDirection - 1) % 4;
        turnLeft();
    }
    
    if (lookForEnd)
    {
        if (coordinate.x == 2 & coordinate.y == 4)
        {
            // Done!
            halt();
        }
    }
}

void zoneB()
{
    // I saw and we need a variable to remember where we were searching
    Direction lastTurn = Left;
    
    bool isEndReached = false;
    while (!isEndReached)
    {
        // const byte WEST_IR_BIT = digitalRead(WEST_IR_PIN) << 2;
        // const byte NORTH_IR_BIT = digitalRead(NORTH_IR_PIN) << 1;
        // const byte EAST_IR_BIT = digitalRead(EAST_IR_PIN);
        
        // // Concatenate bits
        // const byte IR_BYTE = WEST_IR_BIT | NORTH_IR_BIT | EAST_IR_BIT;
        
        const byte IR_BYTE = readIRs();
        
        switch (IR_BYTE)
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
    
    Direction side = firstSide;
    
    for (uint8_t i = 0; i < 2; ++i)
    {
        lastTurn = side;
        bool lineIsLocatedThisSide = false;
        move(side);
        
        unsigned long initialTime = millis();
        // In miliseconds
        const unsigned long finalTime = 1000;
        
        while (millis() - initialTime <= finalTime)
        {
            if (!digitalRead(WEST_IR_PIN)) return false;
            if (!digitalRead(NORTH_IR_PIN)) return false;
            if (!digitalRead(EAST_IR_PIN)) return false;
        }
        
        move(Standing);
        delay(100);
        
        // Move back
        side = (side == Left) ? Right : Left;
        
        move(side);
        delay(finalTime);
    }
    
    move(Standing);
    
    return true;
}

void zoneC()
{
    // We are going to make a laberynth solver
    
    // Size of the map
    const uint8_t sizeX = 3; // Size of 3 really
    const uint8_t sizeY = 6; // Size of 6 really
    
    // Initial coordinates in the plane
    Coordinate coordinate;
    coordinate.x = 1;
    coordinate.y = 1;
    
    // Initialize blacklist
    const uint8_t blacklistSize = 3;
    uint8_t blackListCount = 0;
    Coordinate blacklist[blacklistSize];
    for (uint8_t i = 0; i < blacklistSize; ++i)
    {
        blacklist[i].x = sizeX;
        blacklist[i].y = sizeY;
    }
    
    
    Serial.println("Zone C starting");
    // Default facing
    uint8_t cardinalDirection = North;
    // while (!(coordinate.x == 2 & coordinate.y == 5))
    while (true)
    {
        Serial.println("Analizyng!");
        // Using the right hand method to solve the maze
        if (!isWallInRight() & !isBlackTileInFront(cardinalDirection, Right, coordinate, blacklist))
        {
            // Go right
            Serial.println("Going right!");
            
            bool isWallLeft = isWallInLeft();
            
            // turnOnLED(WHITE);
            // Rotate the view to 90 degrees to the right
            cardinalDirection = (cardinalDirection + 1) % 4;
            turnRight();
            
            if (isWallLeft)
            {
                // Recalibrate orientation
                stepBack();
                stepForward();
            }
            
            // Move coordinate
            
            updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
            
            stepForward();
            
            // If we detect something black
            if (readIRs() != 0b111)
            {
                stepBack();
                // We should mark this place cannot be touched
                blacklist[blackListCount].x = coordinate.x;
                blacklist[blackListCount].y = coordinate.y;
                
                ++blackListCount;
                
                Serial.println("Black tile in right!");
                
                // Return to the last coordinate
                cardinalDirection = (cardinalDirection + 2) % 4;
                updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
                cardinalDirection = (cardinalDirection + 2) % 4;
            }
            else
            {
                stepForward();
                
                Colors color = readColor();
                turnOnLED(color);
            }
        }
        else if (!isWallInFront() & !isBlackTileInFront(cardinalDirection, Forward, coordinate, blacklist))
        {
            // Go forward
            // And update coordinate
            Serial.println("Going forward!");
            
            // turnOnLED(RED);
            updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
            
            stepForward();
            
            // If we detect something black
            if (readIRs() != 0b111)
            {
                stepBack();
                // We should mark this place cannot be touched
                blacklist[blackListCount].x = coordinate.x;
                blacklist[blackListCount].y = coordinate.y;
                
                ++blackListCount;
                
                Serial.println("Black tile in forward!");
                
                // Return to the last coordinate
                cardinalDirection = (cardinalDirection + 2) % 4;
                updateCoordinate(cardinalDirection, coordinate.x, coordinate.y);
                cardinalDirection = (cardinalDirection + 2) % 4;
            }
            else
            {
                stepForward();
                
                Colors color = readColor();
                turnOnLED(color);
            }
        }
        else
        {
            // Turn left
            Serial.println("Turning left");
            // Rotate the view to 90 degrees to the left
            cardinalDirection = (cardinalDirection - 1) % 4;
            turnLeft();
            
            stepBack();
            stepForward();
        }
    }
    
    
    Serial.println("Done zone C!");
    
    halt();
}

bool isBlackTileInFront(uint8_t direction, Direction turnDirection, Coordinate coord, Coordinate blacklist[])
{
    switch (turnDirection)
    {
    case Left:
        direction = (direction - 1) % 4;
        break;
    case Right:
        direction = (direction + 1) % 4;
        break;
    }
    
    switch (direction)
    {
    case North:
        ++coord.y;
        break;
    case South:
        --coord.y;
        break;
    case West:
        --coord.x;
        break;
    case East:
        ++coord.x;
        break;
    }
    
    for (uint8_t i = 0; i < 3; ++i)
    {
        // Default values must be omitted
        if (blacklist[i].x == 3 & blacklist[i].y == 6) continue;
        
        if (blacklist[i].x == coord.x & blacklist[i].y == coord.y)
        {
            // We found a black tile!
            // Step back!
            return true;
        }
    }
    
    // There is no black tile ahead
    return false;
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
    turnOffLED();
    for (;;);
}
