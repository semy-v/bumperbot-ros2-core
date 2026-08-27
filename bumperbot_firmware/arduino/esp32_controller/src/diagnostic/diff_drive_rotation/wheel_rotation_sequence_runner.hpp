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
#include "diff_drive/wheel_velocity_estimator.hpp"

struct WheelRotationStep {
    int pwm_speed;
    std::size_t revolutions;
};

template <std::size_t StepsNum>
using WheelRotationSequence = std::array<WheelRotationStep, StepsNum>;

template <std::size_t StepsNum>
class WheelRotationSequenceRunner {
 public:
    struct MeasuredPwmVelocity {
        int pwm_speed;
        float velocity;
        uint32_t excess_revolution_events;
    };

    WheelRotationSequenceRunner(BLDC2430Encoder& revolution_encoder,
                                BLDC2430Encoder& velocity_encoder,
                                BLDC2430Motor& motor,
                                uint32_t velocity_update_period_ms,
                                float ticks_per_revolution,
                                const WheelRotationSequence<StepsNum>& sequence)
        : revolution_encoder_{revolution_encoder},
          velocity_encoder_{velocity_encoder},
          motor_{motor},
          velocity_estimator_{ticks_per_revolution},
          velocity_update_period_ms_{velocity_update_period_ms},
          sequence_{sequence} {
        static_assert(StepsNum > 0U, "The rotation sequence must contain at least one step");
        static_assert(std::atomic<uint32_t>::is_always_lock_free,
                      "The ISR revolution counter must be lock-free");
    }

    /**
     * Initializes the motor and both encoder instances.
     *
     * The task handle is stored before the revolution encoder interrupt is
     * enabled, so the ISR always has a valid task to notify.
     */
    void begin(TaskHandle_t event_task) {
        if (initialized_) {
            return;
        }

        configASSERT(event_task != nullptr);
        event_task_ = event_task;

        motor_.begin();
        velocity_encoder_.begin();
        revolution_encoder_.begin(&WheelRotationSequenceRunner::handleRevolutionEvent, this);
        velocity_estimator_.configure(velocity_update_period_ms_);

        stopMotor();
        initialized_ = true;
    }

    /** Starts or restarts the configured sequence from its first step. */
    void run() {
        configASSERT(initialized_);

        stopMotor();
        revolution_encoder_.reset();

        pending_revolutions_.store(0U, std::memory_order_relaxed);
        completed_revolutions_ = 0U;
        current_step_index_ = 0U;
        current_velocity_ = 0.0F;
        last_step_result_.reset();

        // The first velocity edge after restart may span the stopped interval.
        velocity_estimator_.reset(velocity_encoder_.getLastEdgeTimeUs(), true);

        running_ = true;
        motor_.setPwmSpeed(currentStep().pwm_speed);
    }

    /** Stops the motor and terminates the active sequence. */
    void stop() {
        stopMotor();
        pending_revolutions_.store(0U, std::memory_order_relaxed);
        running_ = false;
    }

    /**
     * Processes all revolution events captured since the previous call.
     *
     * Call this immediately after the diagnostic task wakes from its ISR task
     * notification. Motor direction/speed changes happen here in normal task
     * context and therefore remain safe for Arduino/LEDC APIs.
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

        // Any excess event means the task could not react before an additional
        // complete revolution occurred. Do not attribute it to the next step,
        // because it was captured while the previous PWM command was active.
        const uint32_t excess_events = pending - remaining;
        completeCurrentStep(excess_events);
    }

    // Refreshes the reciprocal-period velocity estimate.
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
     * PCNT ISR callback.
     *
     * Keep this path minimal: capture the event and wake the diagnostic task.
     * No motor, Serial, estimator, iterator, or optional operation is allowed
     * here.
     */
    static void IRAM_ATTR handleRevolutionEvent(void* context) noexcept {
        auto* runner = static_cast<WheelRotationSequenceRunner*>(context);

        runner->pending_revolutions_.fetch_add(1U, std::memory_order_relaxed);

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

        // Issue the next hardware command before preparing diagnostic output.
        // This minimizes software reaction time at the revolution boundary.
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

    BLDC2430Encoder& revolution_encoder_;
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