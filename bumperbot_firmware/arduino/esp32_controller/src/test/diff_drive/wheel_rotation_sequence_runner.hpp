
#ifndef WHEEL_ROTATION_SEQUENCE_RUNNER_HPP
#define WHEEL_ROTATION_SEQUENCE_RUNNER_HPP

#include <array>
#include <cstdint>

#include "bl2418_encoder.hpp"
#include "bl2418_motor.hpp"

struct WheelRotationStep {
    int pwm_speed;
    size_t revolutions;
};

template <std::size_t StepsNum>
using WheelRotationSequence = std::array<WheelRotationStep, StepsNum>;

template <std::size_t StepsNum>
class WheelRotationSequenceRunner {
 public:
    WheelRotationSequenceRunner(BL2418Encoder& encoder,
                                BL2418Motor& motor,
                                const WheelRotationSequence<StepsNum>& sequence)
        : motor_{motor}, sequence_{sequence} {
        static_assert(StepsNum > 0);
        auto callback = [](void* context) {
            static_cast<WheelRotationSequenceRunner*>(context)->wheelRevolutionCompleted();
        };
        EncoderPcntTestHelper::registerLowHighLimitEventCallback(encoder, callback, this);
    }

    void run() {
        current_step_it_ = sequence_.cbegin();
        finished_revolutions_ = 0;
        motor_.setPwmSpeed(current_step_it_->pwm_speed);
        finished_ = false;
    }

    void stop() {
        motor_.setPwmSpeed(0);
        finished_ = true;
    }

 private:
    void wheelRevolutionCompleted() {
        if (finished_) {
            return;
        }
        if (++finished_revolutions_ < current_step_it_->revolutions) {
            return;
        }
        if (++current_step_it_ != sequence_.cend()) {
            finished_revolutions_ = 0;
            motor_.setPwmSpeed(current_step_it_->pwm_speed);
        } else {
            stop();
        }
    }

    BL2418Motor& motor_;
    const WheelRotationSequence<StepsNum>& sequence_;

    WheelRotationSequence<StepsNum>::const_iterator current_step_it_{};
    size_t finished_revolutions_{0};
    volatile bool finished_{true};
};

#endif  // WHEEL_ROTATION_SEQUENCE_RUNNER_HPP