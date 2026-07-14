/**
 * @file I2C_speed_test.cpp
 * @author Planeson, Red Bird Racing (carson.cpk@proton.me)
 * @brief Test the I2C class speed.
 * @version 1.0
 * @date 2026-07-14
 *
 * @copyright Copyright (c) 2026
 *
 */

#include <Arduino.h>
#include "I2C.hpp"

/** Change this bool to compare with and without I2C */
constexpr bool i2c_in_loop = false;

/** Change this macro to decide the number of times to increment the count per while loop */
#define ADD_COUNT 200

#define CONCAT_HIDDEN(a, b) a##b
#define SAFE_CONCAT(a, b) CONCAT_HIDDEN(a, b)
#define addcount SAFE_CONCAT(add, ADD_COUNT)

#define add500 add300 add100 add100
#define add300 add100 add100 add100
#define add200 add100 add100
#define add140 add100 add40
#define add115 add100 add15


#define add100 \
    ++count;   \
    add99
#define add99 \
    ++count;  \
    add98
#define add98 \
    ++count;  \
    add97
#define add97 \
    ++count;  \
    add96
#define add96 \
    ++count;  \
    add95
#define add95 \
    ++count;  \
    add94
#define add94 \
    ++count;  \
    add93
#define add93 \
    ++count;  \
    add92
#define add92 \
    ++count;  \
    add91
#define add91 \
    ++count;  \
    add90
#define add90 \
    ++count;  \
    add89
#define add89 \
    ++count;  \
    add88
#define add88 \
    ++count;  \
    add87
#define add87 \
    ++count;  \
    add86
#define add86 \
    ++count;  \
    add85
#define add85 \
    ++count;  \
    add84
#define add84 \
    ++count;  \
    add83
#define add83 \
    ++count;  \
    add82
#define add82 \
    ++count;  \
    add81
#define add81 \
    ++count;  \
    add80
#define add80 \
    ++count;  \
    add79
#define add79 \
    ++count;  \
    add78
#define add78 \
    ++count;  \
    add77
#define add77 \
    ++count;  \
    add76
#define add76 \
    ++count;  \
    add75
#define add75 \
    ++count;  \
    add74
#define add74 \
    ++count;  \
    add73
#define add73 \
    ++count;  \
    add72
#define add72 \
    ++count;  \
    add71
#define add71 \
    ++count;  \
    add70
#define add70 \
    ++count;  \
    add69
#define add69 \
    ++count;  \
    add68
#define add68 \
    ++count;  \
    add67
#define add67 \
    ++count;  \
    add66
#define add66 \
    ++count;  \
    add65
#define add65 \
    ++count;  \
    add64
#define add64 \
    ++count;  \
    add63
#define add63 \
    ++count;  \
    add62
#define add62 \
    ++count;  \
    add61
#define add61 \
    ++count;  \
    add60
#define add60 \
    ++count;  \
    add59
#define add59 \
    ++count;  \
    add58
#define add58 \
    ++count;  \
    add57
#define add57 \
    ++count;  \
    add56
#define add56 \
    ++count;  \
    add55
#define add55 \
    ++count;  \
    add54
#define add54 \
    ++count;  \
    add53
#define add53 \
    ++count;  \
    add52
#define add52 \
    ++count;  \
    add51
#define add51 \
    ++count;  \
    add50
#define add50 \
    ++count;  \
    add49
#define add49 \
    ++count;  \
    add48
#define add48 \
    ++count;  \
    add47
#define add47 \
    ++count;  \
    add46
#define add46 \
    ++count;  \
    add45
#define add45 \
    ++count;  \
    add44
#define add44 \
    ++count;  \
    add43
#define add43 \
    ++count;  \
    add42
#define add42 \
    ++count;  \
    add41
#define add41 \
    ++count;  \
    add40
#define add40 \
    ++count;  \
    add39
#define add39 \
    ++count;  \
    add38
#define add38 \
    ++count;  \
    add37
#define add37 \
    ++count;  \
    add36
#define add36 \
    ++count;  \
    add35
#define add35 \
    ++count;  \
    add34
#define add34 \
    ++count;  \
    add33
#define add33 \
    ++count;  \
    add32
#define add32 \
    ++count;  \
    add31
#define add31 \
    ++count;  \
    add30
#define add30 \
    ++count;  \
    add29
#define add29 \
    ++count;  \
    add28
#define add28 \
    ++count;  \
    add27
#define add27 \
    ++count;  \
    add26
#define add26 \
    ++count;  \
    add25
#define add25 \
    ++count;  \
    add24
#define add24 \
    ++count;  \
    add23
#define add23 \
    ++count;  \
    add22
#define add22 \
    ++count;  \
    add21
#define add21 \
    ++count;  \
    add20
#define add20 \
    ++count;  \
    add19
#define add19 \
    ++count;  \
    add18
#define add18 \
    ++count;  \
    add17
#define add17 \
    ++count;  \
    add16
#define add16 \
    ++count;  \
    add15
#define add15 \
    ++count;  \
    add14
#define add14 \
    ++count;  \
    add13
#define add13 \
    ++count;  \
    add12
#define add12 \
    ++count;  \
    add11
#define add11 \
    ++count;  \
    add10
#define add10 \
    ++count;  \
    add9
#define add9 \
    ++count; \
    add8
#define add8 \
    ++count; \
    add7
#define add7 \
    ++count; \
    add6
#define add6 \
    ++count; \
    add5
#define add5 \
    ++count; \
    add4
#define add4 \
    ++count; \
    add3
#define add3 \
    ++count; \
    add2
#define add2 \
    ++count; \
    add1
#define add1 ++count;

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
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("I2C Speed Test");
    sei(); // enable interrupts

    Serial.println("===============================");
    delay(1000);

    /*

    while (!i2c.pushRecurring(send_action))
        // keeping trying read until it is queued
        ;
    // while (!i2c.pushPriority(init_config))
    //  keeping trying init until it is queued
    ;
    while (!i2c.pushRecurring(send_action))
        // keeping trying read until it is queued
        ;
        */
}

volatile uint32_t count = 0;

void loop()
{

    i2c.pump(); // just here to ensure i2c doesn't break for whatever reason
    uint32_t start_time = millis();

    while (1)
    {
        if (i2c_in_loop)
        {
            i2c.pump();
            i2c.pushRecurring(init_config);   // 20
            i2c.pushRecurring(button_action); // 21
            i2c.pushRecurring(send_action);   // 22
            i2c.pushRecurring(read_action);   // 23
            i2c.pushRecurring(random_action); // 24
        }

        addcount;

        if (millis() - start_time > 1000)
        {
            if (i2c_in_loop)
            {
                Serial.print("Count with I2C on, add count at ");
            }
            else
            {
                Serial.print("Count with I2C off, add count at ");
            }
            Serial.print(ADD_COUNT);
            Serial.print(" per loop: ");
            Serial.println(count);
            while (1)
                ;
        }
    }
}