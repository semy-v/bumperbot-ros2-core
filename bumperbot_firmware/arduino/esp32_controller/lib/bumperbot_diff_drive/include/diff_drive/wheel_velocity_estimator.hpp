#ifndef WHEEL_VELOCITY_ESTIMATOR_HPP
#define WHEEL_VELOCITY_ESTIMATOR_HPP

#include "bldc2430_encoder.hpp"

/**
 * @brief Estimates wheel angular velocity from encoder edge timing.
 *
 * The estimator uses reciprocal-period measurement rather than counting
 * encoder ticks over the controller update interval.
 *
 * For each newly observed encoder edge:
 *
 *      omega = delta_theta / delta_t
 *
 * where:
 *
 *      delta_theta = 2*pi / ticks_per_rev
 *      delta_t     = edge period in seconds
 *
 * Because the encoder reports the edge period in microseconds, the
 * conversion factor is precomputed as:
 *
 *      velocity_scale =
 *          (2*pi / ticks_per_rev) * 1'000'000
 *
 * and the instantaneous wheel velocity becomes:
 *
 *      omega [rad/s] =
 *          velocity_scale / edge_period_us
 *
 * The direction reported by the encoder is then applied as +/-1.
 *
 * A first-order exponential moving average (EMA) smooths the raw
 * reciprocal-period estimate.
 *
 * If no encoder edge has been observed for a configured timeout, the
 * velocity is explicitly forced to zero.
 *
 * @note ticks_per_rev must represent the number of EncoderEdgeData updates
 *       generated during one complete output-wheel revolution. If both
 *       rising and falling FG edges are counted, both edges must therefore
 *       be included in ticks_per_rev.
 */
class WheelVelocityEstimator {
 public:
    /**
     * @param ticks_per_rev Number of encoder edges corresponding to one
     *        complete output-wheel revolution.
     */
    explicit WheelVelocityEstimator(float ticks_per_rev);

    /**
     * @brief Configures the velocity low-pass filter for the controller
     *        update period and resets estimator state.
     */
    void configure(uint32_t control_period_ms);

    /**
     * @brief Resets the estimator with no previously consumed encoder edge.
     */
    void reset() {
        reset(0u, false);
    }

    /**
     * @brief Resets estimator history.
     *
     * @param consumed_edge_time_us Encoder timestamp that should be treated
     *        as already consumed. This prevents an old edge from being
     *        processed again after reset.
     *
     * @param discard_next_edge If true, the first new edge after reset is
     *        consumed without calculating velocity from its delta. This is
     *        useful after starting/reversing a motor because the first edge
     *        period may include time during which the motor was stopped and
     *        therefore does not represent steady wheel velocity.
     */
    void reset(uint32_t consumed_edge_time_us,
               bool discard_next_edge);

    /**
     * @brief Updates and returns the wheel angular velocity estimate.
     *
     * A new velocity sample is calculated only when edge_time_us changes.
     * Calls made between encoder edges simply return the most recently
     * filtered velocity.
     *
     * @return Filtered wheel angular velocity in rad/s.
     */
    float update(const EncoderEdgeData& edge_data);

 private:
    /**
     * Conversion factor used by the reciprocal-period calculation:
     *
     *   (2*pi / ticks_per_rev) * 1'000'000
     *
     * so dividing this value by an edge period expressed in microseconds
     * directly produces rad/s.
     *
     * Despite the historical member name, this is not simply
     * "radians per tick"; the 1e6 factor converts microseconds to seconds.
     */
    const float rad_per_tick_us_;

    // EMA coefficient computed from the configured control-loop period.
    float alpha_{0.0F};

    // Last filtered wheel velocity in rad/s.
    float filtered_velocity_{0.0F};

    // Timestamp of the most recently consumed encoder edge. Used to detect
    // whether EncoderEdgeData contains a new measurement.
    uint32_t last_processed_edge_time_us_{0};

    // Causes the first edge following reset to establish fresh timing
    // history without contributing a potentially invalid velocity sample.
    bool discard_next_edge_{false};
};

#endif  // WHEEL_VELOCITY_ESTIMATOR_HPP