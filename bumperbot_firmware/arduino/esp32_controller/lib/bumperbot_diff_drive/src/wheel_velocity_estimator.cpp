#include <algorithm>
#include <cmath>
#include <numbers>

#include "diff_drive/wheel_velocity_estimator.hpp"

WheelVelocityEstimator::WheelVelocityEstimator(float ticks_per_rev)
    /*
     * Angular displacement represented by one encoder edge:
     *
     *      delta_theta = 2*pi / ticks_per_rev       [rad]
     *
     * edge_data.delta_us is expressed in microseconds, so multiply by
     * 1'000'000 us/s here. This allows update() to calculate:
     *
     *      omega = rad_per_tick_us_ / delta_us      [rad/s]
     *
     * without converting the edge period to seconds every iteration.
     */
    : rad_per_tick_us_{
          (2.0F * std::numbers::pi_v<float>) *
          1'000'000.0F /
          ticks_per_rev} {
}

void WheelVelocityEstimator::configure(
    const uint32_t control_period_ms) {

    /*
     * First-order low-pass filter cutoff.
     *
     * The reciprocal-period measurement can vary slightly because of
     * encoder edge jitter, motor commutation, gearbox effects and wheel
     * load changes. The EMA removes much of that high-frequency variation.
     */
    constexpr float kCutoffHz{10.0F};

    // Convert controller update period from milliseconds to seconds.
    const float dt =
        static_cast<float>(control_period_ms) /
        1000.0F;

    /*
     * Discrete-time coefficient corresponding approximately to a
     * first-order continuous low-pass filter:
     *
     *      alpha = 1 - exp(-2*pi*f_c*dt)
     *
     * The EMA update then becomes:
     *
     *      filtered += alpha * (raw - filtered)
     */
    alpha_ =
        1.0F -
        std::exp(
            -2.0F *
            std::numbers::pi_v<float> *
            kCutoffHz *
            dt);

    reset();
}

void WheelVelocityEstimator::reset(
    uint32_t consumed_edge_time_us,
    bool discard_next_edge) {

    // Remove all previous filter history.
    filtered_velocity_ = 0.0F;

    /*
     * Treat this encoder edge as already consumed. An update containing
     * the same timestamp will therefore not be processed again.
     */
    last_processed_edge_time_us_ =
        consumed_edge_time_us;

    /*
     * Optionally discard the first future period measurement.
     *
     * This is useful after a motor start or reversal because the first
     * encoder delta may span the stopped/braking interval and would
     * therefore produce an artificially low velocity.
     */
    discard_next_edge_ =
        discard_next_edge;
}

float WheelVelocityEstimator::update(
    const EncoderEdgeData& edge_data) {

    /*
     * If no encoder transition occurs for this long, consider the wheel
     * stopped.
     *
     * This is necessary for reciprocal-period estimation: when a wheel
     * stops there is no "zero-speed edge" from which zero velocity could
     * otherwise be calculated.
     *
     * NOTE: The corresponding minimum observable nonzero velocity depends
     * on ticks_per_rev, so the "~0.16 rad/s" value is not universal.
     */
    constexpr uint32_t kZeroVelocityTimeoutUs{
        150'000};  // 150 ms

    /*
     * Reject unrealistically short encoder periods.
     *
     * Such a sample can result from electrical noise/glitches or otherwise
     * exceed the physically expected maximum wheel speed.
     *
     * The equivalent maximum angular velocity depends on ticks_per_rev.
     */
    constexpr uint32_t kMinimumEdgePeriodUs{
        550};

    // Suppress negligible numerical/filter residue around zero.
    constexpr float kVelocityDeadbandRadSec{
        0.02F};

    /*
     * Reciprocal-period estimators have an important special case:
     * when the wheel stops, no further encoder edges are generated.
     *
     * edge_data.elapsed_us represents the time since the most recent
     * hardware edge. If it exceeds the timeout, explicitly force the
     * velocity estimate to zero.
     */
    if (edge_data.elapsed_us >
        kZeroVelocityTimeoutUs) {

        filtered_velocity_ = 0.0F;

        /*
         * Mark the last available edge as consumed. If update() is called
         * repeatedly while stopped, that stale edge will not subsequently
         * be interpreted as a new measurement.
         */
        last_processed_edge_time_us_ =
            edge_data.edge_time_us;

        return filtered_velocity_;
    }

    /*
     * EncoderEdgeData is a latest-value snapshot.
     *
     * The control loop normally executes more frequently than new encoder
     * edges arrive at low speed. An unchanged timestamp means there is no
     * new period measurement, so retain the previous filtered estimate.
     */
    if (edge_data.edge_time_us ==
        last_processed_edge_time_us_) {

        return filtered_velocity_;
    }

    // Consume this edge exactly once.
    last_processed_edge_time_us_ =
        edge_data.edge_time_us;

    /*
     * After selected resets (especially motor start/reversal), discard the
     * first new edge.
     *
     * Its delta_us may contain a long stopped/braking interval and therefore
     * does not represent the current rotating-wheel velocity.
     */
    if (discard_next_edge_) {
        discard_next_edge_ = false;
        return filtered_velocity_;
    }

    /*
     * Reject invalid edge periods.
     *
     * Too short:
     *   potentially a glitch or physically impossible speed.
     *
     * Too long:
     *   period is already in the region treated as zero/stalled motion and
     *   would create a misleading very-low velocity estimate.
     */
    if (edge_data.delta_us <=
            kMinimumEdgePeriodUs ||
        edge_data.delta_us >=
            kZeroVelocityTimeoutUs) {

        return filtered_velocity_;
    }

    /*
     * Reciprocal-period velocity estimate.
     *
     * For one encoder edge:
     *
     *      delta_theta = 2*pi / ticks_per_rev
     *
     * Therefore:
     *
     *      omega = delta_theta / delta_t
     *
     * Since rad_per_tick_us_ already contains the 1e6 conversion:
     *
     *      raw_velocity =
     *          rad_per_tick_us_ / delta_us
     *
     * The encoder-provided +/- direction converts the magnitude into a
     * signed wheel angular velocity.
     */
    const float raw_velocity =
        (rad_per_tick_us_ /
         static_cast<float>(edge_data.delta_us)) *
        static_cast<float>(edge_data.direction);

    /*
     * First-order exponential moving average:
     *
     *      y[k] = y[k-1] + alpha * (x[k] - y[k-1])
     *
     * This avoids abrupt changes caused by individual encoder-period
     * variations while preserving a lightweight implementation suitable
     * for the real-time wheel-control loop.
     */
    filtered_velocity_ +=
        alpha_ *
        (raw_velocity - filtered_velocity_);

    /*
     * Eliminate very small residual values caused by filtering or floating
     * point noise.
     */
    if (std::fabs(filtered_velocity_) <
        kVelocityDeadbandRadSec) {

        filtered_velocity_ = 0.0F;
    }

    return filtered_velocity_;
}