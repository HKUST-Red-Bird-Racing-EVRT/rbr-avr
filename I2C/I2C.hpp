#ifndef I2C_HPP
#define I2C_HPP

#include <avr/io.h>
#include <avr/interrupt.h>

enum class I2cMode : uint8_t
{
    Write = 0,
    Read = 1
};

enum class I2cState: uint8_t
{
    Idle = 0,
    Busy = 1,
    Hung = 2
};

struct I2cTransaction
{
    uint8_t address;
    I2cMode mode;
    uint8_t length;
    volatile uint8_t processed_count;
    const uint8_t *data;
};

template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE>
class I2C
{
    static_assert((PRIORITY_SIZE & (PRIORITY_SIZE - 1)) == 0, "PRIORITY_SIZE must be a strict power of 2!");
    static_assert(PRIORITY_SIZE >= 2, "PRIORITY_SIZE must be at least 2");

public:
    constexpr I2C();

    bool pushPriority(const I2cTransaction& new_queuer);
    void setRecurring(const I2cTransaction* new_tape, const uint8_t size);

    void pump();

    void handleIsr();

private:
    constexpr void init();
    void process(const I2cTransaction &);
    void recoverBus();

    // priority queue
    I2cTransaction priority_queue[PRIORITY_SIZE];
    volatile uint8_t priority_head = 0;
    volatile uint8_t priority_tail = 0;
    static constexpr uint8_t PRIORITY_MASK = PRIORITY_SIZE - 1; 

    // recurring tape
    const I2cTransaction* recurring_tape = nullptr;
    volatile uint8_t recurring_index = 0;
    volatile uint8_t recurring_size = 0;

    // state machine tracking
    I2cTransaction* active_job = nullptr;
    volatile uint8_t active_index = 0;
    volatile bool active_job_is_priority = false;
    volatile I2cState bus_state = I2cState::Idle;

    // watchdog
    uint8_t watchdog_count = 0;

    // stuck bus recovery
    enum class RecoveryState: uint8_t
    {
        Init = 0,
        ClockLow = 1,
        ClockHigh = 2,
        Stop = 3
    };

    RecoveryState recovery_state = RecoveryState::Init;
    uint8_t recovery_count = 0;
};

#include "I2C.tpp"
#endif // I2C_HPP