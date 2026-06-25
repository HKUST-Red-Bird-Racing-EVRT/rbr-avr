/**
 * @file I2C_test.cpp
 * @author Planeson, Red Bird Racing
 * @brief Test the I2C class.
 * @version 0.3
 * @date 2026-06-25
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <Arduino.h>
#include "I2C.hpp"

volatile bool mode_changed, button_pushed;
uint8_t send_data[3] = {0x01, 0x02, 0x03};
uint8_t init_data[5] = {0x00, 0x01, 0x02, 0x03, 0x04};
uint8_t button_data[4] = {0x45, 0x26, 0x75, 0x86};

const I2cTransaction init_config(0x23, 5, init_data);
const I2cTransaction button_i2c_action(0x23, 4, button_data);

I2C<100, 8, 8, 4> i2c;

ISR(TWI_vect)
{
    i2c.handleIsr();
}

void setup()
{
    sei(); // enable interrupts

    while (!i2c.pushPriority(init_config))
        // keeping trying init until it is queued
        ;
}

void loop()
{
    i2c.pump();

    if (button_pushed)
    {
        if(i2c.pushPriority(button_i2c_action))
        {
            button_pushed = false;
        }
    }

    i2c.pushRecurring(0x69, 3, send_data);
}