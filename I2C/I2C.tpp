#include "I2C.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
constexpr I2C<BITRATE_KBPS, PRIORITY_SIZE, WATCHDOG_MAX_COUNT>::I2C()
    : priority_queue()
{
    init();
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
constexpr bool I2C<BITRATE_KBPS, PRIORITY_SIZE, WATCHDOG_MAX_COUNT>::pushPriority(const I2cTransaction &new_queuer)
{
    uint8_t new_write_index = (priority_write_index + 1) & PRIORITY_MASK;
    if (new_write_index == priority_read_index)
    { // queue full
        return false;
    }
    priority_queue[priority_write_index] = new_queuer;
    priority_write_index = new_write_index;
    return true;
}

extern void ERROR_I2C_mismatched_pointer_and_size() __attribute__((error(
    "I2C Error: Invalid pairing! A valid pointer requires a size > 0, and a nullptr requires a size of 0.")));

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
constexpr void I2C<BITRATE_KBPS, PRIORITY_SIZE, WATCHDOG_MAX_COUNT>::setRecurring(const I2cTransaction *const new_tape, const uint8_t size)
{
    if (__builtin_constant_p(new_tape) && __builtin_constant_p(size))
    {
        if ((new_tape == nullptr && size > 0) ||
            (new_tape && size == 0))
        {
            // This forces the compiler to flag an error before the binary is generated!
            ERROR_I2C_mismatched_pointer_and_size();
        }
    }

    const I2cTransaction *final_tape = new_tape;
    uint8_t final_size = size;

    if (new_tape == nullptr || size == 0)
    { // if either invalid, clear both
        final_tape = nullptr;
        final_size = 0;
    }
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
    {
        recurring_tape = final_tape;
        recurring_size = final_size;
        recurring_index = 0;
    }
    return;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
void I2C<BITRATE_KBPS, PRIORITY_SIZE, WATCHDOG_MAX_COUNT>::pump()
{
    switch (bus_state)
    {
    case I2cState::Idle:
    {
        if (priority_write_index != priority_read_index)
        { // used to not have any tasks, kickstart priority task
            watchdog_count = 0;
            bus_state = I2cState::Busy;
            active_job = &priority_queue[priority_read_index];
            active_job_is_priority = true;
            // set control register last to prevent another interrupt from not updating active_job
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE); 
        }
        else if (recurring_size > 0)
        { // used to not have any tasks, only have recurring tasks to start
            watchdog_count = 0;
            bus_state = I2cState::Busy;
            active_job = &recurring_tape[recurring_index];
            ++recurring_index;
            active_job_is_priority = false;
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
        }
        return;
    }
    case I2cState::Busy:
    {
        if (watchdog_pulsed)
        {
            watchdog_pulsed = false;
            watchdog_count = 0;
            return;
        }

        if (watchdog_count > WATCHDOG_MAX_COUNT)
        {
            bus_state = I2cState::Hung;
            recovery_state = RecoveryState::Init;
            watchdog_count = 0;
            recoverBus();
        }
        else
        {
            ++watchdog_count;
        }
        return;
    }
    case I2cState::Hung:
    {
        recoverBus();
        return;
    }
    default:
        __builtin_unreachable();
    }
    return;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
inline void I2C<BITRATE_KBPS, PRIORITY_SIZE, WATCHDOG_MAX_COUNT>::handleIsr()
{
    I2cStatus bus_status = TWSR & 0xF8;
    switch (bus_status)
    {
    case I2cStatus::Start:
    case I2cStatus::RepeatedStart:
    {
        active_index = 0;
        TWDR = active_job->address_and_mode;
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        return;
    }
    case I2cStatus::AddressWriteAck:
    case I2cStatus::DataSentAck:
    {
        if (active_index < active_job->length)
        {
            TWDR = active_job->data[active_index];
            ++active_index;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        {
            finishIsr();
        }
        return;
    }
    case I2cStatus::AddressReadAck:
    {
        if (active_job->length > 0)
        {   // next ACK, multi-byte read
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        {   // next NACK, only read 1 byte
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);        
        }
    }
    case I2cStatus::DataReadAck:
    {
        if (active_index < active_job->length)
        {   // next byte still need ACK
            active_job->data[active_index] = TWDR;
            ++active_index;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        {   // next byte need NACK to end
            active_job->data[active_index] = TWDR;
            ++active_index;
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
        }
        return;
    }
    case I2cStatus::ArbitrationLost:
    case I2cStatus::AddressWriteNack:
    case I2cStatus::AddressReadNack:
    case I2cStatus::DataSentNack:
    {
        restartIsr();
    }
    case I2cStatus::DataReadNack:
    {
        finishIsr();
    }
    }
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
constexpr void I2C<BITRATE_KBPS, PRIORITY_SIZE, WATCHDOG_MAX_COUNT>::init()
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
    bus_state = I2cState::Idle;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
void I2C<BITRATE_KBPS, PRIORITY_SIZE, WATCHDOG_MAX_COUNT>::process(const I2cTransaction &)
{
    ;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
void I2C<BITRATE_KBPS, PRIORITY_SIZE, WATCHDOG_MAX_COUNT>::recoverBus()
{
    switch (recovery_state)
    {
    case RecoveryState::Init:
    {

        break;
    }
    }
}
