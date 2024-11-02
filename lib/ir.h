#ifndef IR_H
#define IR_H

#include "pin_definitions.h"

/// IR

void initializeIR()
{
    pinMode(WEST_IR_PIN, INPUT);
    pinMode(NORTH_IR_PIN, INPUT);
    pinMode(EAST_IR_PIN, INPUT);
}

byte readIRs()
{
    const byte WEST_IR_BIT = digitalRead(WEST_IR_PIN) << 2;
    const byte NORTH_IR_BIT = digitalRead(NORTH_IR_PIN) << 1;
    const byte EAST_IR_BIT = digitalRead(EAST_IR_PIN);
    
    // Concatenate bits
    return WEST_IR_BIT | NORTH_IR_BIT | EAST_IR_BIT;
}

#endif
