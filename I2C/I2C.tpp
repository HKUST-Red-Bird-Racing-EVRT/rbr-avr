#include "I2C.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
constexpr I2C<BITRATE_KBPS, PRIORITY_SIZE>::I2C()
    : priority_queue()
{
    init();
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
constexpr void I2C<BITRATE_KBPS, PRIORITY_SIZE>::init()
{
    static_assert(BITRATE_KBPS <= 400, "Max bitrate of the ATmega328p is 400kbps!");
    // check if "undoing" the operation gives the correct speed, if not then it means the rate is invalid
    static_assert(((((F_CPU / BITRATE_KBPS / 1000) - 16) / 2) * 2 + 16) * 1000 * BITRATE_KBPS == F_CPU, "This bitrate is impossible to achieve for this CPU speed!");
    TWBR = ((F_CPU / BITRATE_KBPS / 1000) - 16) / 2;
    TWSR &= ~(1 << TWPS1) & ~(1 << TWPS0);
}