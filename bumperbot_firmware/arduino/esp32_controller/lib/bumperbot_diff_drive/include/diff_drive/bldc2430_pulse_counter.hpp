#ifndef BLDC2430_PULSE_COUNTER_HPP
#define BLDC2430_PULSE_COUNTER_HPP

#include <Arduino.h>
#include "driver/pcnt.h"

#include <cstdint>
#include <optional>

class BLDC2430Encoder;

/**
 * @brief Low-level BLDC2430 FG pulse counter built on the ESP32 PCNT peripheral.
 *
 * This class owns one PCNT hardware unit and contains all PCNT-specific setup,
 * event handling, reset, and teardown logic used by the BLDC2430 wheel code.
 * It deliberately does not estimate velocity and does not attach any semantic
 * meaning to the configured PCNT limits.
 *
 * The BLDC2430 FG output is sampled on falling edges only.  Rising edges are
 * ignored.  Counting a single edge polarity gives one counter increment per
 * complete FG period and avoids the duty-cycle-dependent timing error that
 * occurs when rising->falling and falling->rising half-periods are treated as
 * equivalent measurements.
 *
 * The motor direction command is connected to the PCNT control input.  PCNT
 * therefore increments toward the positive or negative limit according to the
 * commanded direction.  The optional invert_logic flag compensates for the
 * mirrored mounting of the left/right motors so both sides can use the same
 * logical forward/reverse convention.
 *
 * Typical uses:
 *
 *   - BLDC2430Encoder owns a counter configured with +1/-1 limits.  Every FG
 *     falling edge therefore generates a limit event, allowing the interval
 *     between consecutive falling edges to be measured for reciprocal-period
 *     velocity estimation.
 *
 *   - The wheel-rotation diagnostic owns a counter configured with
 *     +/-falling_edges_per_wheel_revolution (for example +306/-306).  Each PCNT
 *     limit event then represents one complete output-wheel revolution.
 *
 * @important This is not a quadrature encoder.  Direction is derived from the
 *            motor direction-command GPIO, not independently measured from FG.
 */
class BLDC2430PulseCounter {
 public:
    // using LimitEventCallback = void (*)(void* context, LimitEventStatus status);
    using PulseCountCallback = void (*)(void* context);

    /**
     * @brief PCNT high/low-limit event observed by the ISR dispatcher.
     *
     * Normally exactly one flag is set for each callback.  Both flags are kept
     * in the interface so no raw ESP-IDF PCNT status bits leak into users of
     * this class and so an unusual accumulated-status condition is not lost.
     */
    struct LimitEventStatus {
        bool high_limit{false};
        bool low_limit{false};
    };

    /**
     * @brief Creates a falling-edge PCNT counter for one BLDC2430 FG signal.
     *
     * @param direction_pin Arduino pin connected to the BLDC2430 direction input.
     * @param speed_state_pin Arduino pin connected to the open-collector FG output.
     * @param counter_h_limit Positive PCNT limit. Must be greater than zero.
     * @param counter_l_limit Negative PCNT limit. Must be less than zero.
     * @param invert_logic Inverts the command-direction/count-direction mapping
     *        for a mirrored wheel installation.
     */
    BLDC2430PulseCounter(uint8_t direction_pin,
                         uint8_t speed_state_pin,
                         int16_t counter_h_limit,
                         int16_t counter_l_limit,
                         bool invert_logic);

    ~BLDC2430PulseCounter();

    BLDC2430PulseCounter(const BLDC2430PulseCounter&) = delete;
    BLDC2430PulseCounter(BLDC2430PulseCounter&&) = delete;
    BLDC2430PulseCounter& operator=(const BLDC2430PulseCounter&) = delete;
    BLDC2430PulseCounter& operator=(BLDC2430PulseCounter&&) = delete;

    /**
     * @brief Starts PCNT counting and dispatches high/low limit events.
     *
     * The callback runs in ISR context. It must not block, allocate, perform
     * Serial I/O, or call non-ISR-safe APIs.
     */
    void begin(PulseCountCallback callback, void* context);

    /**
     * @brief Retrieve PCNT high/low-limit event status.
     *
     * Can be called in the PulseCountCallback context.
     */
    std::optional<LimitEventStatus> getLimitEventStatus() const {
        uint32_t raw_status{0U};
        if (pcnt_get_event_status(pcnt_unit_, &raw_status) != ESP_OK) {
            return std::nullopt;
        }

        return LimitEventStatus{
            .high_limit = (raw_status & PCNT_EVT_H_LIM) != 0U,
            .low_limit = (raw_status & PCNT_EVT_L_LIM) != 0U,
        };
    }

    /**
     * @brief Clears the hardware count and restarts counting from zero.
     *
     * The configured callback and PCNT limits are preserved.
     */
    void reset();

 private:
    // BLDC2430Encoder needs a short interrupt-free reset window in which it can
    // clear the PCNT hardware and its timestamp snapshot as one logical reset.
    friend class BLDC2430Encoder;

    void pauseAndDisableInterrupt();
    void clearCounter();
    void resumeAndEnableInterrupt();

    // Allocates one legacy PCNT hardware unit per counter instance.  The robot
    // constructs these objects statically, so units intentionally remain owned
    // for the lifetime of the firmware.
    static pcnt_unit_t allocatePcntUnit();

    // The ESP-IDF legacy PCNT ISR service is global.  Install it once before
    // registering per-unit handlers.
    static void installIsrServiceOnce();

    const pcnt_unit_t pcnt_unit_;
    const uint8_t speed_state_pin_;

    bool begun_{false};
};

#endif  // BLDC2430_PULSE_COUNTER_HPP