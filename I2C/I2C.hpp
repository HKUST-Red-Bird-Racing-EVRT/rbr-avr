#ifndef I2C_HPP
#define I2C_HPP

#include <avr/io.h>
#include <avr/interrupt.h>


struct I2cTransaction
{
    const uint8_t address_and_mode;
    const uint8_t length; /**< Stores the length of the data to be written/read. @note For read data, the length stores the length of ACK bytes, i.e. total read byte count - 1.*/
    uint8_t *data;
};

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t WATCHDOG_MAX_COUNT>
class I2C
{
    static_assert((PRIORITY_SIZE & (PRIORITY_SIZE - 1)) == 0, "PRIORITY_SIZE must be a strict power of 2!");
    static_assert(PRIORITY_SIZE >= 2, "PRIORITY_SIZE must be at least 2");

public:
    constexpr I2C();

    constexpr bool pushPriority(const I2cTransaction &new_queuer);
    constexpr void setRecurring(const I2cTransaction *new_tape, const uint8_t size);

    void pump();

    inline void handleIsr();

private:
    constexpr void init();
    inline void finishIsr();
    void recoverBus();
    

    // priority queue
    I2cTransaction priority_queue[PRIORITY_SIZE];
    volatile uint8_t priority_write_index = 0;
    volatile uint8_t priority_read_index = 0;
    static constexpr uint8_t PRIORITY_MASK = PRIORITY_SIZE - 1;

    // recurring tape
    const I2cTransaction *recurring_tape = nullptr;
    volatile uint8_t recurring_size = 0;
    volatile uint8_t recurring_index = 0;

    // state machine tracking
    enum class I2cState : uint8_t
    {
        Idle = 0,
        Busy = 1,
        Hung = 2
    };
    I2cTransaction *active_job = nullptr;
    volatile uint8_t active_index = 0;
    volatile bool active_job_is_priority = false;
    volatile I2cState bus_state = I2cState::Idle;

    // watchdog
    volatile uint8_t watchdog_pulsed = false;
    uint8_t watchdog_count = 0;

    // stuck bus recovery
    enum class RecoveryState : uint8_t
    {
        Init = 0,
        ClockLow = 1,
        ClockHigh = 2,
        Stop = 3
    };

    RecoveryState recovery_state = RecoveryState::Init;
    uint8_t recovery_count = 0;

    enum class I2cStatus : uint8_t
    {
        Start = 0x08,
        RepeatedStart = 0x10,
        AddressWriteAck = 0x18,
        AddressWriteNack = 0x20,
        DataSentAck = 0x28,
        DataSentNack = 0x30,
        ArbitrationLost = 0x38,
        AddressReadAck = 0x40,
        AddressReadNack = 0x48,
        DataReadAck = 0x50,
        DataReadNack = 0x58
        // slave modes not implemented
    };
};

#include "I2C.tpp"
#endif // I2C_HPP