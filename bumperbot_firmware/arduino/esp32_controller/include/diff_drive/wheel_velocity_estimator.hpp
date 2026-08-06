#ifndef WHEEL_VELOCITY_ESTIMATOR_HPP
#define WHEEL_VELOCITY_ESTIMATOR_HPP

#include "bl2418_encoder.hpp"

class WheelVelocityEstimator {
 public:
    WheelVelocityEstimator(float ticks_per_rev);

    void configure(const uint32_t control_period_ms);

    void reset() {
        reset(0u, false);
    }

    void reset(uint32_t consumed_edge_time_us, bool discard_next_edge);

    /**
     * @brief Updates velocity estimate based on reciprocal period timing.
     */
    float update(const EncoderEdgeData& edge_data);

 private:
    const float rad_per_tick_us_;  // Radians per single encoder pulse edge

    float alpha_{0.0};
    float filtered_velocity_{0.0};
    uint32_t last_processed_edge_time_us_{0};
    bool discard_next_edge_{false};
};

#endif  // WHEEL_VELOCITY_ESTIMATOR_HPP