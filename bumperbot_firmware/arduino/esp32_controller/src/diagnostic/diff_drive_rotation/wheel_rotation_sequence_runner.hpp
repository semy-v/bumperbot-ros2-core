#ifndef WHEEL_ROTATION_SEQUENCE_RUNNER_HPP
#define WHEEL_ROTATION_SEQUENCE_RUNNER_HPP

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "diff_drive/bldc2430_encoder.hpp"
#include "diff_drive/bldc2430_motor.hpp"
#include "diff_drive/bldc2430_pulse_counter.hpp"
#include "diff_drive/wheel_velocity_estimator.hpp"

struct WheelRotationStep {
    int pwm_speed;
    std::size_t revolutions;
};

template <std::size_t StepsNum>
using WheelRotationSequence = std::array<WheelRotationStep, StepsNum>;

/**
 * @brief Executes a fixed PWM/revolution diagnostic sequence for one wheel.
 *
 * Two deliberately different FG abstractions are used:
 *
 *  - full_revolution_counter_ is a BLDC2430PulseCounter configured by main()
 *    with +/-falling_edges_per_wheel_revolution.  Its PCNT ISR therefore fires
 *    once per complete output-wheel revolution and wakes this diagnostic task.
 *
 *  - velocity_encoder_ is a BLDC2430Encoder.  It is permanently configured
 *    internally with +/-1 limits and publishes complete falling-edge FG periods
 *    for reciprocal-period velocity estimation.
 *
 * Keeping these responsibilities separate prevents custom revolution-sized
 * PCNT limits from changing the meaning of BLDC2430Encoder::delta_us.
 */
template <std::size_t StepsNum>
class WheelRotationSequenceRunner {
 public:
    struct MeasuredPwmVelocity {
        int pwm_speed;
        float velocity;
        uint32_t excess_revolution_events;
    };

    WheelRotationSequenceRunner(BLDC2430PulseCounter& full_revolution_counter,
                                BLDC2430Encoder& velocity_encoder,
                                BLDC2430Motor& motor,
                                uint32_t velocity_update_period_ms,
                                float velocity_ticks_per_revolution,
                                const WheelRotationSequence<StepsNum>& sequence)
        : full_revolution_counter_{full_revolution_counter},
          velocity_encoder_{velocity_encoder},
          motor_{motor},
          velocity_estimator_{velocity_ticks_per_revolution},
          velocity_update_period_ms_{velocity_update_period_ms},
          sequence_{sequence} {
        static_assert(StepsNum > 0U, "The rotation sequence must contain at least one step");
        static_assert(std::atomic<uint32_t>::is_always_lock_free,
                      "The ISR revolution counter must be lock-free");
    }

    /**
     * @brief Initializes motor, falling-edge velocity encoder, and full-revolution counter.
     *
     * event_task is stored before the full-revolution PCNT interrupt is enabled,
     * so handleRevolutionEvent() always has a valid task to notify.
     */
    void begin(TaskHandle_t event_task) {
        if (initialized_) {
            return;
        }

        configASSERT(event_task != nullptr);
        event_task_ = event_task;

        motor_.begin();
        velocity_encoder_.begin();
        full_revolution_counter_.begin(&WheelRotationSequenceRunner::handleRevolutionEvent, this);
        velocity_estimator_.configure(velocity_update_period_ms_);

        stopMotor();
        initialized_ = true;
    }

    /** @brief Starts or restarts the configured sequence from its first step. */
    void run() {
        configASSERT(initialized_);

        stopMotor();
        full_revolution_counter_.reset();

        pending_revolutions_.store(0U, std::memory_order_relaxed);
        completed_revolutions_ = 0U;
        current_step_index_ = 0U;
        current_velocity_ = 0.0F;
        last_step_result_.reset();

        // Ignore the first falling-edge interval after restart because it spans
        // the stopped interval rather than one representative moving FG period.
        velocity_estimator_.reset(velocity_encoder_.getLastEdgeTimeUs(), true);

        running_ = true;
        motor_.setPwmSpeed(currentStep().pwm_speed);
    }

    /** @brief Stops the motor and terminates the active sequence. */
    void stop() {
        stopMotor();
        pending_revolutions_.store(0U, std::memory_order_relaxed);
        running_ = false;
    }

    /**
     * @brief Processes revolution-limit events accumulated by the ISR.
     *
     * Call immediately after the task wakes.  Motor command changes remain in
     * task context; the ISR only counts completed revolutions and notifies.
     */
    void processPendingRevolutionEvents() {
        const uint32_t pending = pending_revolutions_.exchange(0U, std::memory_order_relaxed);

        if (!running_ || pending == 0U) {
            return;
        }

        const uint32_t required = static_cast<uint32_t>(currentStep().revolutions);
        const uint32_t remaining = required - completed_revolutions_;

        if (pending < remaining) {
            completed_revolutions_ += pending;
            return;
        }

        // Events captured beyond the requested revolution count still belong to
        // the PWM command active before the task had a chance to react.  Report
        // them instead of carrying them into the next sequence step.
        const uint32_t excess_events = pending - remaining;
        completeCurrentStep(excess_events);
    }

    /** @brief Refreshes the reciprocal-period wheel velocity estimate. */
    void updateVelocity() {
        if (!running_) {
            current_velocity_ = 0.0F;
            return;
        }

        current_velocity_ = velocity_estimator_.update(velocity_encoder_.getEdgeData());
    }

    [[nodiscard]] bool isRunning() const { return running_; }
    [[nodiscard]] bool isFinished() const { return initialized_ && !running_; }

    [[nodiscard]] std::size_t currentStepNumber() const {
        return running_ ? current_step_index_ + 1U : sequence_.size();
    }

    [[nodiscard]] float currentVelocity() const { return current_velocity_; }

    /** Returns the latest completed-step result and clears it in task context. */
    std::optional<MeasuredPwmVelocity> takeLastStepResult() {
        auto result = last_step_result_;
        last_step_result_.reset();
        return result;
    }

 private:
    /**
     * @brief Full-revolution PCNT ISR callback.
     *
     * The BLDC2430PulseCounter is configured so each high/low limit corresponds
     * to one complete wheel revolution.  Keep this ISR path minimal.
     */
    static void IRAM_ATTR handleRevolutionEvent(void* context) noexcept {
        auto* runner = static_cast<WheelRotationSequenceRunner*>(context);

        // check that limit event status available
        const auto opt_status = runner->full_revolution_counter_.getLimitEventStatus();
        if (!opt_status) {
            return;
        }

        // Normally one status bit is set.  Count both defensively if PCNT ever
        // reports accumulated high- and low-limit status in one dispatch.
        const auto& status = opt_status.value();
        const uint32_t revolution_events =
            (status.high_limit ? 1U : 0U) + (status.low_limit ? 1U : 0U);

        if (revolution_events == 0U) {
            return;
        }

        runner->pending_revolutions_.fetch_add(revolution_events, std::memory_order_relaxed);

        BaseType_t higher_priority_task_woken = pdFALSE;
        vTaskNotifyGiveFromISR(runner->event_task_, &higher_priority_task_woken);

        if (higher_priority_task_woken == pdTRUE) {
            portYIELD_FROM_ISR();
        }
    }

    [[nodiscard]] const WheelRotationStep& currentStep() const {
        return sequence_[current_step_index_];
    }

    void completeCurrentStep(uint32_t excess_revolution_events) {
        const WheelRotationStep completed_step = currentStep();
        const float completed_step_velocity = current_velocity_;

        ++current_step_index_;
        completed_revolutions_ = 0U;

        // Issue the next hardware command before preparing diagnostic output so
        // the revolution-boundary reaction latency remains minimal.
        if (current_step_index_ >= sequence_.size()) {
            stopMotor();
            running_ = false;
        } else {
            motor_.setPwmSpeed(currentStep().pwm_speed);
        }

        last_step_result_ = MeasuredPwmVelocity{
            .pwm_speed = completed_step.pwm_speed,
            .velocity = completed_step_velocity,
            .excess_revolution_events = excess_revolution_events,
        };
    }

    void stopMotor() { motor_.setPwmSpeed(0); }

    BLDC2430PulseCounter& full_revolution_counter_;
    BLDC2430Encoder& velocity_encoder_;
    BLDC2430Motor& motor_;
    WheelVelocityEstimator velocity_estimator_;

    const uint32_t velocity_update_period_ms_;
    const WheelRotationSequence<StepsNum>& sequence_;

    std::atomic<uint32_t> pending_revolutions_{0U};
    TaskHandle_t event_task_{nullptr};

    std::size_t current_step_index_{0U};
    uint32_t completed_revolutions_{0U};
    float current_velocity_{0.0F};
    std::optional<MeasuredPwmVelocity> last_step_result_;

    bool initialized_{false};
    bool running_{false};
};

#endif  // WHEEL_ROTATION_SEQUENCE_RUNNER_HPP