#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#include "pin_definitions.h"

// Motor

enum Direction {
    Forward = 0,
    Backwards,
    Left,
    Right,
    Standing
};

static Direction direction = Standing;
static const uint8_t BASE_SPEED = 70;

static const uint16_t TIME_TO_ADVANCE_HALF_TILE = 2000;
static const uint16_t TIME_TO_TURN = 1000;

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
                digitalWrite(IN1, LOW);
                digitalWrite(IN2, HIGH);
                digitalWrite(IN3, HIGH);
                digitalWrite(IN4, LOW);
                Serial.println("Left");
                break;
            case Right:
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                digitalWrite(IN3, LOW);
                digitalWrite(IN4, HIGH);
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

inline void turnOffMotors()
{
    move(Standing, 0);
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

void initializeMotor()
{
    // Motor
    
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    
    pinMode(ENA, OUTPUT);
    pinMode(ENB, OUTPUT);
    
    turnOffMotors();
}

#endif
