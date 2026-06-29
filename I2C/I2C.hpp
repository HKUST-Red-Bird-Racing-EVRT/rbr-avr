#ifndef I2C_HPP
#define I2C_HPP

#include <avr/io.h>
#include <avr/interrupt.h>

struct I2cTransaction
{
    uint8_t address_and_mode;
    uint8_t length; /**< Stores the length of the data to be written/read. @note For read data, the length stores the length of ACK bytes, i.e. total read byte count - 1.*/

    union DataUnion
    {
        constexpr DataUnion(const uint8_t *write_) : write(write_) {}
        constexpr DataUnion(uint8_t *read_) : read(read_) {}
        const uint8_t *write;
        uint8_t *read;
    } data;

    inline static constexpr I2cTransaction makeWrite(const uint8_t address, const uint8_t length, const uint8_t *const source);
    inline static constexpr I2cTransaction makeChainedWrite(const uint8_t address, const uint8_t length, const uint8_t *const source);
    inline static constexpr I2cTransaction makeRead(const uint8_t address, const uint8_t length, uint8_t *const destination);

    static constexpr uint8_t REPEAT_MASK = 0x80;
    static constexpr uint8_t LENGTH_MASK = ~(REPEAT_MASK);
private:
    constexpr I2cTransaction(const uint8_t address_and_mode_, const uint8_t length_, uint8_t *const data_);

    constexpr I2cTransaction() : address_and_mode(0), length(0), data(static_cast<uint8_t *const>(nullptr)) {}

    template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
    friend class I2C;
};

/**
 * @brief I2C Master Driver class template.
 * Provides a high-speed, interrupt-driven I2C master interface with priority and recurring transaction support.
 * @details This class is designed for AVR microcontrollers and uses the TWI hardware module.
 * It supports a priority queue for urgent transactions and a recurring queue for periodic tasks.
 * The driver handles bus recovery in case of stuck conditions and provides a watchdog mechanism to detect bus hangs.
 * Users create I2cTransaction objects and push them to the driver using pushPriority or pushRecurring.
 * The priority queue is a ring buffer fifo.
 * When a new transaction is chosen to be processed, the priority queue always asserts presidence over the recurring queue.
 * The recurring queue works as a buffer that is flushed in a round-robin fashion, and is only processed when the priority queue is empty.
 * The driver must be initialized with the init() method before use,
 * and the pump() method should be called regularly in the main loop to provide a heartbeat for the watchdog, and to kickstart the bus when idle.
 * The handleIsr() method must be called from the TWI_vect ISR to handle hardware events. See the example for usage.
 * @attention You must wrap the call to handleIsr() in an ISR(TWI_vect) block, and you must enable global interrupts with sei() before using the driver.
 *
 * @tparam BITRATE_KBPS Bitrate in kbps for the I2C bus. Must be achievable for the given CPU frequency, else a static_assert will trigger.
 * @tparam PRIORITY_SIZE Size of the priority queue. Must be a power of 2 and at least 2.
 * @tparam RECURRING_SIZE Size of the recurring queue. Must be a power of 2 and at least 2.
 * @tparam WATCHDOG_MAX_COUNT Maximum count for the watchdog timer before considering the bus hung. This is a measure of how many pump cycles can occur without activity before triggering recovery. If the main loop runs extremely quick, you should set this higher to avoid false positives.
 */
template <uint16_t BITRATE_KBPS, uint8_t PRIORITY_SIZE, uint8_t RECURRING_SIZE, uint8_t WATCHDOG_MAX_COUNT>
class I2C
{
    static_assert((PRIORITY_SIZE & (PRIORITY_SIZE - 1)) == 0, "PRIORITY_SIZE must be a strict power of 2!");
    static_assert(PRIORITY_SIZE >= 2, "PRIORITY_SIZE must be at least 2");
    static_assert((RECURRING_SIZE & (RECURRING_SIZE - 1)) == 0, "RECURRING_SIZE must be a strict power of 2!");
    static_assert(RECURRING_SIZE >= 2, "RECURRING_SIZE must be at least 2");

public:
    constexpr I2C();

    [[nodiscard]] constexpr bool pushPriority(const I2cTransaction &new_queuer);

    constexpr bool pushRecurring(const I2cTransaction &new_queuer);

    void pump();

    inline void handleIsr();

private:
    // =========================================================================
    // Typedefs
    // =========================================================================
    enum class I2cState : uint8_t
    {
        Idle = 0,
        Busy = 1,
        Hung = 2
    };
    enum class RecoveryState : uint8_t
    {
        Init = 0,
        ClockLow = 1,
        ClockHigh = 2,
        Stop = 3
    };
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

    // =========================================================================
    // Private Methods
    // =========================================================================

    constexpr void init();
    inline void finishIsr();
    inline void restartIsr();
    void recoverBus();

    inline void setActiveJob(const I2cTransaction &job, bool is_priority);

    // =========================================================================
    // Private Members
    // =========================================================================

    // priority queue
    I2cTransaction priority_queue[PRIORITY_SIZE] = {};
    volatile uint8_t priority_write_index = 0;
    volatile uint8_t priority_read_index = 0;
    static constexpr uint8_t PRIORITY_MASK = PRIORITY_SIZE - 1;

    // recurring queue
    I2cTransaction recurring_queue[RECURRING_SIZE] = {};
    uint8_t recurring_count = 0;
    volatile uint8_t recurring_index = 0;
    volatile bool recurring_queue_locked = false;

    // state machine tracking
    volatile uint8_t active_address_and_mode = 0;
    volatile uint8_t active_length = 0;
    volatile bool active_is_chained = false; // repeated start required after this
    volatile uint8_t *active_data_ptr = nullptr;
    volatile bool active_is_priority = false;
    volatile uint8_t active_byte_index = 0;
    
    volatile I2cState bus_state = I2cState::Idle;

    // watchdog
    volatile uint8_t watchdog_pulsed = false;
    uint8_t watchdog_count = 0;

    // stuck bus recovery

    RecoveryState recovery_state = RecoveryState::Init;
    uint8_t recovery_count = 0;
};

#include "I2C.tpp"
#endif // I2C_HPP