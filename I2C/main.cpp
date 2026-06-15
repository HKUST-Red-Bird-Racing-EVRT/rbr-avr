/**
 * @file I2C_test.cpp
 * @author Planeson, Red Bird Racing
 * @brief Test the I2C class.
 * @version 0.1
 * @date 2026-05-29
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <Arduino.h>
#include "I2C.hpp"

bool mode_changed, button_pushed;
constexpr uint8_t old_tape_size = 5;
constexpr uint8_t new_tape_size = 12;

const I2cTransaction old_tape[old_tape_size];
const I2cTransaction new_tape[new_tape_size];


const I2cTransaction init_config;
const I2cTransaction button_i2c_action;

I2C<100, 8> i2c;

ISR(TWI_vect){
    i2c.handleIsr();
}

void setup() {
    sei(); // enable interrupts
    i2c.setRecurring(old_tape, old_tape_size);

    i2c.pushPriority(init_config);
}

void loop() {
    i2c.pump();

    if (mode_changed)
    {
        i2c.setRecurring(new_tape, new_tape_size);   
    }

    if (button_pushed)
    {
        i2c.pushPriority(button_i2c_action);
    }
}