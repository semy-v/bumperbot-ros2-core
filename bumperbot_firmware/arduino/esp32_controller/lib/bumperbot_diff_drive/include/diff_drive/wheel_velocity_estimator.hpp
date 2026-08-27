#ifndef WHEEL_VELOCITY_ESTIMATOR_HPP
#define WHEEL_VELOCITY_ESTIMATOR_HPP

#include <cstdint>

#include "bldc2430_encoder.hpp"

/**
 * @brief Estimates wheel angular velocity from the period between consecutive
 *        FG falling edges.
 *
 * The BL2430 encoder path is configured to timestamp only one FG polarity
 * (falling edges). Therefore each EncoderEdgeData::delta_us measurement spans
 * one complete FG period rather than alternating high/low half-periods.
 * Measuring full periods removes sensitivity to FG duty-cycle asymmetry and
 * produces a cleaner reciprocal-period velocity estimate.
 *
 * The estimator uses reciprocal-period measurement rather than counting the
 * number of encoder events received during the controller update interval.
 * This is particularly useful at low speed, where a count-per-control-period
 * estimate becomes strongly quantized.
 *
 * For two consecutive falling edges:
 *
 *      delta_theta = 2*pi / ticks_per_rev          [rad]
 *      delta_t     = falling-edge period            [s]
 *      omega       = delta_theta / delta_t          [rad/s]
 *
 * Since EncoderEdgeData::delta_us is expressed in microseconds, the constructor
 * precomputes:
 *
 *      rad_per_tick_us =
 *          (2*pi / ticks_per_rev) * 1'000'000
 *
 * so each new velocity observation requires only:
 *
 *      omega = rad_per_tick_us / delta_us
 *
 * followed by the encoder-provided direction sign.
 *
 * A first-order exponential moving average (EMA) smooths the instantaneous
 * reciprocal-period estimate. Calls made between falling edges simply return
 * the previously filtered velocity. If no falling edge arrives for the
 * zero-velocity timeout, the estimate is explicitly forced to zero because a
 * stopped wheel cannot generate a final "zero-speed" encoder event.
 *
 * @note ticks_per_rev must be the empirically measured number of falling-edge
 *       events per complete output-wheel revolution. It must not contain the
 *       old rising+falling transition count used by the previous encoder
 *       implementation.
 */
class WheelVelocityEstimator {
 public:
    /**
     * @param ticks_per_rev Number of FG falling-edge events corresponding to
     *        one complete output-wheel revolution.
     */
    explicit WheelVelocityEstimator(float ticks_per_rev);

    /**
     * @brief Configures the velocity low-pass filter for the nominal controller
     *        update period and resets estimator state.
     *
     * The EMA coefficient is derived from the controller period. The filter is
     * actually updated only when a new falling-edge period is available, so at
     * very low wheel speeds its effective update rate is lower than the nominal
     * controller rate.
     */
    void configure(uint32_t control_period_ms);

    /**
     * @brief Resets the estimator with no previously consumed encoder edge.
     */
    void reset() { reset(0u, false); }

    /**
     * @brief Resets estimator history.
     *
     * @param consumed_edge_time_us Falling-edge timestamp that should be
     *        treated as already consumed. An EncoderEdgeData snapshot carrying
     *        the same timestamp will therefore not be processed again.
     *
     * @param discard_next_edge If true, the first new falling edge after reset
     *        is consumed without using its delta_us for a velocity update. This
     *        is useful after start, stop, or direction reversal because that
     *        first period may include stationary/braking time and therefore may
     *        not represent the current rotating-wheel velocity.
     */
    void reset(uint32_t consumed_edge_time_us, bool discard_next_edge);

    /**
     * @brief Updates and returns the filtered wheel angular velocity estimate.
     *
     * A new reciprocal-period observation is processed only when
     * edge_data.edge_time_us identifies a new falling edge. Otherwise the last
     * filtered estimate is returned unchanged.
     *
     * @return Filtered wheel angular velocity in rad/s.
     */
    float update(const EncoderEdgeData& edge_data);

 private:
    /**
     * Maximum physically plausible wheel velocity accepted by the estimator.
     *
     * 30 rad/s is intentionally above the expected operating/no-load speed of
     * both BL2430 and BL2418 wheel assemblies, leaving headroom for motor and
     * supply tolerances while still allowing implausibly short FG periods to be
     * rejected as glitches.
     */
    static constexpr float kMaximumValidWheelVelocityRadSec{30.0f};

    /**
     * Reciprocal-period conversion factor:
     *
     *      (2*pi / ticks_per_rev) * 1'000'000
     *
     * Dividing this value by the measured falling-edge period in microseconds
     * directly produces rad/s.
     */
    const float rad_per_tick_us_;

    /**
     * Minimum accepted period between consecutive FG falling edges.
     *
     * It is calculated once from ticks_per_rev and the maximum plausible wheel
     * velocity:
     *
     *      minimum_period_us = rad_per_tick_us /
     *                          kMaximumValidWheelVelocityRadSec
     *
     * This keeps the glitch-rejection threshold physically equivalent when a
     * motor/gearbox with a different number of FG events per revolution is
     * installed.
     */
    const uint32_t minimum_edge_period_us_;

    // EMA coefficient computed from the nominal controller update period.
    float alpha_{0.0f};

    // Last filtered wheel velocity in rad/s.
    float filtered_velocity_{0.0f};

    // Timestamp of the most recently consumed FG falling edge. Used to detect
    // whether EncoderEdgeData contains a new reciprocal-period measurement.
    uint32_t last_processed_edge_time_us_{0};

    // Causes the first falling edge after reset to establish fresh timing
    // history without contributing a potentially invalid velocity observation.
    bool discard_next_edge_{false};
};

#endif  // WHEEL_VELOCITY_ESTIMATOR_HPP