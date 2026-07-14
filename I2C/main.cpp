/**
 * @file I2C_test.cpp
 * @author Planeson, Red Bird Racing (carson.cpk@proton.me)
 * @brief Test the I2C class.
 * @version 1.0
 * @date 2026-07-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <Arduino.h>
#include "I2C.hpp"

volatile bool mode_changed = false, button_pushed = false;
uint8_t send_data[3] = {0x01, 0x02, 0x03};
uint8_t read_data[3] = {0x00, 0x00, 0x00};
uint8_t init_data[5] = {0x01, 0x23, 0x45, 0x67, 0x89};
uint8_t button_data[4] = {0x45, 0x26, 0x75, 0x86};
uint8_t random_data[3] = {0x11, 0x45, 0x14};

const I2cTransaction init_config = I2cTransaction::makeWrite(0x20, 5, init_data);
const I2cTransaction button_action = I2cTransaction::makeWrite(0x21, 4, button_data);

const I2cTransaction send_action = I2cTransaction::makeWrite(0x22, 3, send_data);
const I2cTransaction read_action = I2cTransaction::makeRead(0x23, 3, read_data);
const I2cTransaction random_action = I2cTransaction::makeWrite(0x24, 3, random_data);

I2C<100, 4, 8, 4> i2c;

ISR(TWI_vect)
{
    i2c.handleIsr();
}

void setup()
{
    /*Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("I2C Test");*/
    sei(); // enable interrupts

    while (!i2c.pushRecurring(send_action))
        // keeping trying read until it is queued
        ;
    // while (!i2c.pushPriority(init_config))
    //  keeping trying init until it is queued
    ;
    while (!i2c.pushRecurring(send_action))
        // keeping trying read until it is queued
        ;
}

bool send = true;

void loop()
{
    i2c.pump();

    /*Serial.print(read_data[0], HEX);
    Serial.print("\t");
    Serial.print(read_data[1], HEX);
    Serial.print("\t");
    Serial.println(read_data[2], HEX);*/

    if (button_pushed)
    {
        if (i2c.pushPriority(button_action))
        {
            button_pushed = false;
        }
    }

    i2c.pushRecurring(init_config);   // 20
    i2c.pushRecurring(button_action); // 21
    i2c.pushRecurring(send_action);   // 22
    i2c.pushRecurring(read_action);   // 23
    i2c.pushRecurring(random_action); // 24

    /* you are suggested to group multiple operations with a single check for emptiness first

    if (!i2c.recurring_queue_locked)
    {
        i2c.pushRecurring(send_action); // 22
        i2c.pushRecurring(init_config); // 20
        i2c.pushRecurring(read_action); // 23
        i2c.pushRecurring(button_action); // 21
    }*/

    /*
    Serial.print(i2c.active_address_and_mode, HEX);
    Serial.print("\t");
    Serial.print(i2c.recurring_queue_locked ? "Locked" : "Unlocked");
    Serial.print("\t");
    Serial.print(i2c.recurring_index);
    Serial.print("\t");
    Serial.print(read_data[0], HEX);
    Serial.print("\t");
    Serial.print(read_data[1], HEX);
    Serial.print("\t");
    Serial.println(read_data[2], HEX);*/
}