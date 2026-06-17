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
bool I2C<BITRATE_KBPS, PRIORITY_SIZE>::pushPriority(const I2cTransaction &new_queuer)
{
    if (((priority_tail + 1) & PRIORITY_MASK) == priority_head)
    {
        return false;
    }
    priority_tail = ((priority_tail + 1) & PRIORITY_MASK);
    priority_queue[priority_tail] = new_queuer;
    return true;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
void I2C<BITRATE_KBPS, PRIORITY_SIZE>::setRecurring(const I2cTransaction *new_tape, const uint8_t size)
{
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        recurring_tape = new_tape;
        recurring_index = 0;
        recurring_size = size;
    }
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
void I2C<BITRATE_KBPS, PRIORITY_SIZE>::pump()
{
    ;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
void I2C<BITRATE_KBPS, PRIORITY_SIZE>::handleIsr()
{
    ;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
constexpr void I2C<BITRATE_KBPS, PRIORITY_SIZE>::init()
{
    static_assert(BITRATE_KBPS <= 400, "Max bitrate of the ATmega328p is 400kbps!");
    // check if "undoing" the operation gives the correct speed, if not then it means the rate is invalid
    static_assert(((((F_CPU / BITRATE_KBPS / 1000) - 16) / 2) * 2 + 16) * 1000 * BITRATE_KBPS == F_CPU, "This bitrate is impossible to achieve for this CPU speed!");

    // bitrate
    TWBR = ((F_CPU / BITRATE_KBPS / 1000) - 16) / 2;

    // prescaler
    TWSR &= ~(1 << TWPS1) & ~(1 << TWPS0);

    // enable TWI, disabling GPIO functionalities
    TWCR = (1 << TWEN);
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
void I2C<BITRATE_KBPS, PRIORITY_SIZE>::process(const I2cTransaction &)
{
    ;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
void I2C<BITRATE_KBPS, PRIORITY_SIZE>::recoverBus()
{
    switch (recovery_state)
    {
    case RecoveryState::Init:
    {

        break;
    }
    }
}
