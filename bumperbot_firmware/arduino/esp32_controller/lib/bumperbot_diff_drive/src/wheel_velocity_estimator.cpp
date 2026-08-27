#include <algorithm>
#include <cmath>
#include <numbers>

#include "diff_drive/wheel_velocity_estimator.hpp"

WheelVelocityEstimator::WheelVelocityEstimator(float ticks_per_rev)
    /*
     * One EncoderEdgeData update now corresponds to the interval between two
     * consecutive FG falling edges, i.e. one complete FG period.
     *
     * Angular displacement represented by one falling-edge event:
     *
     *      delta_theta = 2*pi / ticks_per_rev       [rad]
     *
     * edge_data.delta_us is expressed in microseconds, so the 1'000'000 us/s
     * factor is folded into the conversion constant. update() can therefore
     * calculate the instantaneous velocity as:
     *
     *      omega = rad_per_tick_us_ / delta_us      [rad/s]
     *
     * without converting the measured period to seconds on every update.
     */
    : rad_per_tick_us_{(2.0f * std::numbers::pi_v<float>)*1'000'000.0f / ticks_per_rev},
      /*
       * Convert the maximum plausible wheel velocity into the shortest valid
       * falling-edge period for this particular ticks-per-revolution value:
       *
       *      T_min = delta_theta / omega_max
       *
       * Because rad_per_tick_us_ already includes the microsecond conversion:
       *
       *      T_min_us = rad_per_tick_us_ / omega_max
       *
       * Truncation is conservative here: it slightly lowers the rejection
       * threshold and therefore leaves a small amount of additional high-speed
       * headroom rather than rejecting a legitimate edge too early.
       */
      minimum_edge_period_us_{
          static_cast<uint32_t>(rad_per_tick_us_ / kMaximumValidWheelVelocityRadSec)} {}

void WheelVelocityEstimator::configure(const uint32_t control_period_ms) {
    /*
     * Nominal first-order low-pass cutoff for the reciprocal-period samples.
     *
     * Full-period falling-edge timing removes the former alternating
     * rising-to-falling / falling-to-rising duty-cycle ripple, but smaller
     * variations can still remain because of FG timing jitter, commutation,
     * gearbox ripple and changing wheel load.
     */
    constexpr float kCutoffHz{10.0f};

    // Convert the nominal controller update period from milliseconds to seconds.
    const float dt = static_cast<float>(control_period_ms) / 1000.0f;

    /*
     * EMA coefficient corresponding to a first-order continuous-time low-pass
     * evaluated at the nominal controller period:
     *
     *      alpha = 1 - exp(-2*pi*f_c*dt)
     *
     * and the sample update is:
     *
     *      filtered += alpha * (raw - filtered)
     *
     * Note that the EMA is actually changed only when update() observes a new
     * FG falling edge. At low wheel speed the edge rate can be lower than the
     * controller rate, so the effective filter update interval is then longer
     * than dt.
     */
    alpha_ = 1.0f - std::exp(-2.0f * std::numbers::pi_v<float> * kCutoffHz * dt);

    reset();
}

void WheelVelocityEstimator::reset(uint32_t consumed_edge_time_us, bool discard_next_edge) {
    // Remove previous low-pass-filter history.
    filtered_velocity_ = 0.0f;

    /*
     * Treat the supplied falling-edge timestamp as already consumed. A later
     * update() call containing the same latest-value snapshot will therefore
     * not process that old period again.
     */
    last_processed_edge_time_us_ = consumed_edge_time_us;

    /*
     * Optionally discard the first future falling-edge period.
     *
     * After motor start, stop or direction reversal, the first measured period
     * can span time during which the wheel was stationary or changing state.
     * Consuming that first edge without updating velocity makes the following
     * edge-to-edge interval the first clean full-period measurement.
     */
    discard_next_edge_ = discard_next_edge;
}

float WheelVelocityEstimator::update(const EncoderEdgeData& edge_data) {
    /*
     * Declare the wheel stopped when no new FG falling edge has arrived for
     * this interval.
     *
     * Reciprocal-period estimation has no natural zero-speed sample: once the
     * wheel stops, encoder events simply cease. The timeout therefore provides
     * an explicit transition to zero velocity.
     *
     * The corresponding minimum observable non-zero velocity depends on
     * ticks_per_rev because each falling edge represents 2*pi/ticks_per_rev
     * radians of wheel rotation.
     */
    constexpr uint32_t kZeroVelocityTimeoutUs{
        150'000};  // 150 ms without a falling edge => zero velocity

    // Suppress negligible residual filter values around zero.
    constexpr float kVelocityDeadbandRadSec{0.02F};

    /*
     * If the newest falling edge is too old, force zero velocity. This check is
     * performed before looking for a new timestamp because a stopped wheel will
     * repeatedly expose the same final EncoderEdgeData snapshot.
     */
    if (edge_data.elapsed_us > kZeroVelocityTimeoutUs) {
        filtered_velocity_ = 0.0f;

        /*
         * Mark the last available edge as consumed so repeated calls while the
         * wheel remains stopped cannot later reinterpret that stale edge as a
         * fresh period measurement.
         */
        last_processed_edge_time_us_ = edge_data.edge_time_us;

        return filtered_velocity_;
    }

    /*
     * EncoderEdgeData is a latest-value snapshot rather than a queue.
     *
     * At low speed the 100 Hz control loop may execute several times between
     * consecutive FG falling edges. An unchanged timestamp means there is no
     * new reciprocal-period observation, so preserve the current filtered
     * velocity.
     */
    if (edge_data.edge_time_us == last_processed_edge_time_us_) {
        return filtered_velocity_;
    }

    // Consume this falling edge exactly once.
    last_processed_edge_time_us_ = edge_data.edge_time_us;

    /*
     * After selected resets, consume the first new falling edge only to
     * establish fresh timing history. Its delta_us may include a stopped or
     * braking interval and therefore may not describe the current wheel speed.
     */
    if (discard_next_edge_) {
        discard_next_edge_ = false;
        return filtered_velocity_;
    }

    /*
     * Reject physically implausible full-period measurements.
     *
     * Too short:
     *   delta_us <= minimum_edge_period_us_
     *   implies |omega| >= kMaximumValidWheelVelocityRadSec and is treated as
     *   a likely electrical glitch/spurious FG transition.
     *
     * Too long:
     *   delta_us >= kZeroVelocityTimeoutUs lies in the same interval in which
     *   the estimator declares the wheel stopped, so using it as a very small
     *   non-zero velocity would be inconsistent with the timeout policy.
     */
    if (edge_data.delta_us <= minimum_edge_period_us_ ||
        edge_data.delta_us >= kZeroVelocityTimeoutUs) {
        return filtered_velocity_;
    }

    /*
     * Reciprocal full-period velocity estimate.
     *
     * Consecutive falling edges represent one encoder tick:
     *
     *      delta_theta = 2*pi / ticks_per_rev
     *
     * and therefore:
     *
     *      omega = delta_theta / delta_t
     *
     * rad_per_tick_us_ already contains the 1e6 conversion from microseconds
     * to seconds, so the runtime calculation is only one division followed by
     * application of the encoder-provided direction sign.
     */
    const float raw_velocity = (rad_per_tick_us_ / static_cast<float>(edge_data.delta_us)) *
                               static_cast<float>(edge_data.direction);

    /*
     * First-order exponential moving average:
     *
     *      y[k] = y[k-1] + alpha * (x[k] - y[k-1])
     *
     * Full-period timing has already removed the dominant duty-cycle-induced
     * half-period ripple; the EMA smooths the remaining edge-to-edge timing
     * variation while retaining a lightweight real-time implementation.
     */
    filtered_velocity_ += alpha_ * (raw_velocity - filtered_velocity_);

    // Eliminate insignificant numerical/filter residue around zero.
    if (std::fabs(filtered_velocity_) < kVelocityDeadbandRadSec) {
        filtered_velocity_ = 0.0f;
    }

    return filtered_velocity_;
}
