#ifndef SERVO_CONTROLLER_H
#define SERVO_CONTROLLER_H

#include "Servo.h"

// Servo

static Servo gripper;

/// Servo

void openGripper()
{
    gripper.write(0);
}

void closeGripper()
{
    gripper.write(90);
}

#endif
