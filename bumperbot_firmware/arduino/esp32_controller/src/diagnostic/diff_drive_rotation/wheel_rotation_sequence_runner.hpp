
#ifndef WHEEL_ROTATION_SEQUENCE_RUNNER_HPP
#define WHEEL_ROTATION_SEQUENCE_RUNNER_HPP

#include <array>
#include <cstdint>
#include <optional>

#include "diff_drive/bl2418_encoder.hpp"
#include "diff_drive/bl2418_motor.hpp"
#include "diff_drive/wheel_velocity_estimator.hpp"

struct WheelRotationStep {
    int pwm_speed;
    size_t revolutions;
};

template <std::size_t StepsNum>
using WheelRotationSequence = std::array<WheelRotationStep, StepsNum>;

template <std::size_t StepsNum>
class WheelRotationSequenceRunner {
 public:
    struct MeasuredPwmVelocity {
        int pwm_speed;
        float velocity;
    };

    struct MeasuredStaticFriction {
        MeasuredPwmVelocity forward;
        MeasuredPwmVelocity reverse;
    };

    WheelRotationSequenceRunner(BL2418Encoder& full_revolution_encoder,
                                BL2418Encoder& velocity_encoder,
                                BL2418Motor& motor,
                                uint32_t control_period_ms,
                                float ticks_per_rev,
                                const WheelRotationSequence<StepsNum>& sequence)
        : motor_{motor},
          velocity_encoder_{velocity_encoder},
          velocity_estimator_{ticks_per_rev},
          sequence_{sequence} {
        static_assert(StepsNum > 0);
        auto callback = [](void* context) {
            static_cast<WheelRotationSequenceRunner*>(context)->wheelRevolutionCompleted();
        };
        motor.begin();
        velocity_encoder_.begin();
        full_revolution_encoder.begin(callback, this);
        velocity_estimator_.configure(control_period_ms);
    }

    void run() {
        current_step_it_ = sequence_.cbegin();
        finished_revolutions_ = 0;
        velocity_encoder_.reset();
        motor_.setPwmSpeed(current_step_it_->pwm_speed);
        finished_ = false;
    }

    void stop() {
        motor_.setPwmSpeed(0);
        finished_ = true;
    }

    float measureVelocity() {
        const auto edge_data = velocity_encoder_.getEdgeData();
        return velocity_estimator_.update(edge_data);
    }

    std::optional<MeasuredPwmVelocity> getLastStepVelocity() const { return last_step_velocity_; }

    void resetLastStepVelocity() { last_step_velocity_ = std::nullopt; }

 private:
    void wheelRevolutionCompleted() {
        if (finished_) {
            return;
        }
        if (++finished_revolutions_ < current_step_it_->revolutions) {
            return;
        }
        last_step_velocity_ = MeasuredPwmVelocity{current_step_it_->pwm_speed, measureVelocity()};
        if (++current_step_it_ != sequence_.cend()) {
            finished_revolutions_ = 0;
            motor_.setPwmSpeed(current_step_it_->pwm_speed);
        } else {
            stop();
        }
    }

    BL2418Motor& motor_;
    BL2418Encoder& velocity_encoder_;
    WheelVelocityEstimator velocity_estimator_;
    const WheelRotationSequence<StepsNum>& sequence_;

    WheelRotationSequence<StepsNum>::const_iterator current_step_it_{};
    size_t finished_revolutions_{0};
    std::optional<MeasuredPwmVelocity> last_step_velocity_{std::nullopt};
    volatile bool finished_{true};
};

#endif  // WHEEL_ROTATION_SEQUENCE_RUNNER_HPP