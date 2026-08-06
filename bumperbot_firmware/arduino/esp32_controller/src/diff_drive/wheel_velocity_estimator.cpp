#include <algorithm>
#include <cmath>
#include <numbers>

#include "diff_drive/wheel_velocity_estimator.hpp"

WheelVelocityEstimator::WheelVelocityEstimator(float ticks_per_rev)
    : rad_per_tick_us_{(2.0f * std::numbers::pi_v<float>)*1'000'000.0f / ticks_per_rev} {
}

void WheelVelocityEstimator::configure(const uint32_t control_period_ms) {
    constexpr float kCutoffHz = 10.0f;  // Filter cutoff frequency in Hz
    const float dt = control_period_ms / 1000.0f;  // Convert milliseconds to seconds

    // EMA smoothing factor: alpha = 1 - exp(-2 * pi * f_c * dt)
    alpha_ = 1.0f - std::exp(-2.0f * std::numbers::pi_v<float> * kCutoffHz * dt);

    reset();
}

void WheelVelocityEstimator::reset(uint32_t consumed_edge_time_us, bool discard_next_edge) {
    filtered_velocity_ = 0.0f;
    last_processed_edge_time_us_ = consumed_edge_time_us;
    discard_next_edge_ = discard_next_edge;
}

float WheelVelocityEstimator::update(const EncoderEdgeData& edge_data) {
    constexpr uint32_t kZeroVelocityTimeoutUs = 150'000;  // 150 ms timeout (~0.16 rad/s limit)
    constexpr uint32_t kMinimumEdgePeriodUs = 550;  // 1800 Hz (0.55ms) minimum period between edges
    constexpr float kVelocityDeadbandRadSec = 0.02;  // rad/s

    // Zero-Velocity Timeout Check
    if (edge_data.elapsed_us > kZeroVelocityTimeoutUs) {
        filtered_velocity_ = 0.0f;
        last_processed_edge_time_us_ = edge_data.edge_time_us;
        return filtered_velocity_;
    }

    if (edge_data.edge_time_us == last_processed_edge_time_us_) {
        // No new edge data to process
        return filtered_velocity_;
    }

    last_processed_edge_time_us_ = edge_data.edge_time_us;

    if (discard_next_edge_) {
        discard_next_edge_ = false;
        return filtered_velocity_;
    }

    if (edge_data.delta_us <= kMinimumEdgePeriodUs ||
        edge_data.delta_us >= kZeroVelocityTimeoutUs) {
        return filtered_velocity_;
    }

    const float raw_velocity =
        (rad_per_tick_us_ / static_cast<float>(edge_data.delta_us)) * edge_data.direction;

    // Apply Exponential Moving Average (EMA)
    filtered_velocity_ += alpha_ * (raw_velocity - filtered_velocity_);

    // Deadband Noise Suppression
    if (std::fabs(filtered_velocity_) < kVelocityDeadbandRadSec) {
        filtered_velocity_ = 0.0f;
    }

    return filtered_velocity_;
}