#include "I2C.hpp"
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/atomic.h>

/**
 * @brief Constexpr constructor for I2cTransaction.
 * @note The length parameter for reads is stored as the number of ACKs expected, which is one less than the total byte count for reads. This allows the ISR to determine when to send ACK vs NACK without needing additional state tracking. For writes, the length is stored directly as provided.
 *
 * @param[in] address_and_mode The 7-bit I2C address combined with the read/write mode in the least significant bit (0 for write, 1 for read).
 * @param[in] length The number of bytes to transfer.
 * @param[in,out] data Pointer to the data buffer. For writes, this is the source buffer. For reads, this is the destination buffer where received data will be stored. Be sure that the buffer is large enough to hold the specified length of data.
 */
constexpr I2cTransaction::I2cTransaction(const uint8_t address_and_mode_, const uint8_t length_, uint8_t *const data_)
    : address_and_mode(address_and_mode_),
      length((address_and_mode & 0x01) ? length_ : length_ - 1),
      data(data_)
{
}

inline constexpr I2cTransaction I2cTransaction::makeWrite(const uint8_t address, const uint8_t length, const uint8_t *const source)
{
    return I2cTransaction(address << 1, length, const_cast<uint8_t *>(source));
}

inline constexpr I2cTransaction I2cTransaction::makeChainedWrite(const uint8_t address, const uint8_t length, const uint8_t *const source)
{
    return I2cTransaction(address << 1, length | I2cTransaction::REPEAT_MASK, const_cast<uint8_t *>(source));
}


extern void __ERROR_I2C_READ_LENGTH_MUST_BE_GREATER_THAN_ZERO__()
    __attribute__((error("I2C read length must be greater than zero!")));

inline constexpr I2cTransaction I2cTransaction::makeRead(const uint8_t address, const uint8_t length, uint8_t *const destination)
{
    if (__builtin_constant_p(length) && length == 0)
    {
        __ERROR_I2C_READ_LENGTH_MUST_BE_GREATER_THAN_ZERO__();
    }
    return I2cTransaction((address << 1) | 0x01, length - 1, destination);
}

/**
 * @brief Constexpr constructor for I2C. Initializes the TWI hardware with the specified bitrate and sets up internal state.
 * @note This must be called before pump.
 */
template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
constexpr I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::I2C()
{
    init();
}

/**
 * @brief Pushes a new priority transaction to the priority queue.
 * @note Overloads exist for both I2cTransaction objects and raw parameters for convenience.

 * @param[in] new_queuer new I2cTransaction to be added to the priority queue.
 * @return true if the transaction was successfully added to the queue, false if the queue is full.
 */
template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
constexpr bool I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::pushPriority(const I2cTransaction &new_queuer)
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

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
constexpr bool I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::pushRecurring(const I2cTransaction &new_queuer)
{
    if (recurring_queue_locked)
    {
        return false;
    }
    recurring_queue[recurring_count] = new_queuer;
    ++recurring_count;
    return true;
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
void I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::pump()
{
    switch (bus_state)
    {
    case I2cState::Idle:
    {
        if (priority_write_index != priority_read_index)
        { // used to not have any tasks, kickstart priority task
            watchdog_count = 0;
            bus_state = I2cState::Busy;
            setActiveJob(priority_queue[priority_read_index], true);
            // set control register last to prevent another interrupt from not updating active_job
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
        }
        else if (recurring_count > 0)
        { // used to not have any tasks, only have recurring tasks to start
            watchdog_count = 0;
            bus_state = I2cState::Busy;
            setActiveJob(recurring_queue[0], false);
            ++recurring_index;
            recurring_queue_locked = true;
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

/**
 * @brief handles the TWI interrupt service routine. Must be called from within the ISR(TWI_vect) block.
 */
template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
inline void I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::handleIsr()
{
    I2cStatus bus_status = static_cast<I2cStatus>(TWSR & 0xF8);
    switch (bus_status)
    {
    case I2cStatus::Start:
    case I2cStatus::RepeatedStart:
    {
        active_byte_index = 0;
        TWDR = active_address_and_mode;
        TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        return;
    }
    case I2cStatus::AddressWriteAck:
    case I2cStatus::DataSentAck:
    {
        if (active_byte_index < active_length)
        {
            TWDR = active_data_ptr[active_byte_index];
            ++active_byte_index;
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
        if (active_length > 0)
        { // next ACK, multi-byte read
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        { // next NACK, only read 1 byte
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWIE);
        }
        return;
    }
    case I2cStatus::DataReadAck:
    {
        if (active_byte_index < active_length)
        { // next byte still need ACK
            active_data_ptr[active_byte_index] = TWDR;
            ++active_byte_index;
            TWCR = (1 << TWINT) | (1 << TWEA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        { // next byte need NACK to end
            active_data_ptr[active_byte_index] = TWDR;
            ++active_byte_index;
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
        return;
    }
    case I2cStatus::DataReadNack:
    {
        finishIsr();
    }
    }
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
constexpr void I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::init()
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

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
inline void I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::finishIsr()
{
    if (active_is_priority)
    {
        priority_read_index = (priority_read_index + 1) & PRIORITY_MASK;
        if (priority_write_index != priority_read_index)
        {
            setActiveJob(priority_queue[priority_read_index], true);
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
        }
        else if (recurring_queue_locked)
        {
            setActiveJob(recurring_queue[recurring_index], false);
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        {
            TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
            bus_state = I2cState::Idle;
        }
    }
    else
    {
        recurring_index += 1;
        if (priority_write_index != priority_read_index)
        {
            setActiveJob(priority_queue[priority_read_index], true);
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
        }
        else if (recurring_index != recurring_count)
        {
            setActiveJob(recurring_queue[recurring_index], false);
            TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
        }
        else
        {
            TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
            bus_state = I2cState::Idle;
        }
    }
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
inline void I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::restartIsr()
{
    active_byte_index = 0;
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN) | (1 << TWIE);
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
void I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::recoverBus()
{
    return;
    switch (recovery_state)
    {
    case RecoveryState::Init:
    {

        break;
    }
    }
}

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
void I2C<BITRATE_KBPS, PRIORITY_SIZE, RECURRING_SIZE, WATCHDOG_MAX_COUNT>::setActiveJob(const I2cTransaction &job, bool is_priority)
{
    active_address_and_mode = job.address_and_mode;
    active_length = job.length & I2cTransaction::LENGTH_MASK;
    active_is_chained = job.length & I2cTransaction::REPEAT_MASK;
    active_data_ptr = job.data.read; // For writes, this will be cast to const uint8_t* in the ISR

    active_is_priority = is_priority;
    active_byte_index = 0;
}