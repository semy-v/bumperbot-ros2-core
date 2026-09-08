#ifndef WHEEL_CONTROLLER_HPP
#define WHEEL_CONTROLLER_HPP

#include <cmath>
#include <cstdint>
#include <utility>
#include <variant>

#include <QuickPID.h>

#include "diff_drive/bldc2430_encoder.hpp"
#include "diff_drive/bldc2430_motor.hpp"
#include "diff_drive/wheel_velocity_estimator.hpp"
#include "protocol/system_data.hpp"

/**
 * @brief Closed-loop angular-velocity controller for one BLDC2430-driven wheel.
 *
 * WheelController combines three layers:
 *
 *   1. BLDC2430Encoder + WheelVelocityEstimator
 *      The encoder timestamps consecutive FG falling edges and the estimator
 *      converts the full FG period into filtered wheel angular velocity [rad/s].
 *      ticks_per_rev therefore means FG falling-edge events per output-wheel
 *      revolution, not the historical rising+falling transition count.
 *
 *   2. Directional steady-state feed-forward
 *
 *        forward:  PWM_ff = +(Ks_forward + Kv * |target_velocity|)
 *        reverse:  PWM_ff = -(Ks_reverse + Kv * |target_velocity|)
 *
 *      Ks is direction-specific because the calibrated BLDC2430 wheel assemblies
 *      exhibit repeatable forward/reverse friction asymmetry. Kv is shared
 *      between directions. There is deliberately no startup/breakaway PWM term.
 *
 *   3. Bounded PID feedback
 *      QuickPID corrects residual tracking error around the feed-forward command.
 *      The PID correction is limited both by max_feedback_pwm and by the
 *      remaining actuator range, so PWM_ff + PWM_pid always stays in [-255,255].
 *      The final command is additionally prevented from changing sign relative
 *      to the requested target; direction reversal is handled only by Brake.
 *
 *
 * The controller uses a non-polymorphic std::variant state machine.
 * State-local behavior and state-local data are owned by the corresponding
 * state type. WheelController contains only data and operations shared across
 * multiple states or required by the public API.
 *
 * Control states:
 *
 *   Inactive
 *      Controller disabled. Motor command is zero and PID/velocity state is reset.
 *
 *   Stopped
 *      Controller active with zero target. Motor command and reported controller
 *      velocity are forced to zero.
 *
 *   Normal
 *      Read the latest falling-edge period, update the reciprocal-period velocity
 *      estimate, run PID feedback, combine feed-forward + feedback, and command
 *      the motor.
 *
 *   Brake
 *      Used for stop requests and direction reversals. PID is disabled/reset and
 *      a small PWM command in the current motion direction is applied so the
 *      BLDC2430 driver continues producing FG pulses while wheel speed decays.
 *      Once braking completes, the controller either enters Stopped or starts
 *      the latest nonzero target.
 *
 * @important BLDC2430 FG is a single-channel speed signal. Encoder direction is
 *            inferred from the motor direction command, not independently
 *            measured from quadrature feedback. A physical direction reversal
 *            therefore must not be commanded until the wheel has actually slowed
 *            sufficiently in Brake.
 */
class WheelController {
    // -------------------------------------------------------------------------
    // State transition requests.
    // -------------------------------------------------------------------------
    enum class TransitionRequest {
        None,
        ToInactive,
        ToStopped,
        ToNormal,
        ToBrake,
    };

    // -------------------------------------------------------------------------
    // Typed state-machine events.
    // -------------------------------------------------------------------------
    struct ActivateEvent {};

    struct DeactivateEvent {};

    struct TargetVelocityEvent {
        bool stop;
        bool reversal;
    };

    // -------------------------------------------------------------------------
    // State base.
    //
    // The controller reference gives every concrete state direct access to the
    // controller's private data and methods without virtual dispatch.
    //
    // Deactivation is common to every active state. InactiveState explicitly
    // overrides it because deactivating an already inactive controller is a
    // no-op and must preserve the target velocity stored while inactive.
    // -------------------------------------------------------------------------
    struct StateBase {
        explicit StateBase(WheelController& controller) noexcept : controller(controller) {}

        WheelController& controller;

        TransitionRequest handle(const DeactivateEvent&) noexcept;
    };

    // -------------------------------------------------------------------------
    // Inactive
    // -------------------------------------------------------------------------
    struct InactiveState final : StateBase {
        using StateBase::StateBase;

        // Deactivation while already inactive is a no-op. In particular, do not
        // clear target_velocity_: the public API intentionally allows a target
        // to be stored while inactive and used on the next activation.
        TransitionRequest handle(const DeactivateEvent&) noexcept {
            return TransitionRequest::None;
        }

        TransitionRequest handle(const ActivateEvent&) const noexcept {
            return controller.target_velocity_ == 0.0f ? TransitionRequest::ToStopped
                                                       : TransitionRequest::ToNormal;
        }
    };

    // -------------------------------------------------------------------------
    // Stopped
    // -------------------------------------------------------------------------
    struct StoppedState final : StateBase {
        explicit StoppedState(WheelController& controller) noexcept : StateBase(controller) {
            onEnter();
        }

        // Preserve the common DeactivateEvent handler from StateBase while
        // adding StoppedState's TargetVelocityEvent overload.
        using StateBase::handle;

        TransitionRequest handle(const TargetVelocityEvent& event) const noexcept {
            return event.stop ? TransitionRequest::None : TransitionRequest::ToNormal;
        }

     private:
        inline void onEnter();
    };

    // -------------------------------------------------------------------------
    // Normal
    // -------------------------------------------------------------------------
    struct NormalState final : StateBase {
        explicit NormalState(WheelController& controller) noexcept : StateBase(controller) {
            onEnter();
        }

        // Preserve the common DeactivateEvent handler from StateBase while
        // adding NormalState's TargetVelocityEvent overload.
        using StateBase::handle;

        inline TransitionRequest update(uint32_t) noexcept;

        TransitionRequest handle(const TargetVelocityEvent& event) noexcept {
            if (event.stop || event.reversal) {
                return TransitionRequest::ToBrake;
            }

            // Same-direction target changes remain in Normal and therefore
            // preserve the existing PID history.
            return TransitionRequest::None;
        }

     private:
        void onEnter() noexcept { prepareMotionStart(); }
        inline void prepareMotionStart() noexcept;

        /**
         * @brief Combines feed-forward and PID correction into the actuator command.
         *
         * The result is saturated to [-255,255] and constrained to the target
         * direction so feedback cannot perform an uncoordinated direction reversal.
         */
        inline int calculateMotorSpeedPwm() const noexcept;
    };

    // -------------------------------------------------------------------------
    // Brake
    // -------------------------------------------------------------------------
    struct BrakeState final : StateBase {
        explicit BrakeState(WheelController& controller) noexcept
            : StateBase(controller), total_period_ms_(0U), brake_pwm_(calculateBrakePwm()) {
            onEnter();
        }

        // Brake has no special DeactivateEvent behavior, so inherit the common
        // active-state cleanup from StateBase.
        using StateBase::handle;

        inline TransitionRequest update(uint32_t dt_ms) noexcept;

     private:
        // These fields exist only while BrakeState is the active alternative.
        uint32_t total_period_ms_{};
        const int brake_pwm_{};

        void onEnter() noexcept { controller.resetPid(); }

        /**
         * @brief Returns the low-speed command used while decelerating.
         *
         * A small command in the currently reported motion direction asks the motor
         * to decelerate toward a speed below its sustainable range while keeping the
         * driver active enough to continue generating FG timing information.
         *
         * This is not a reverse-torque command: the sign intentionally follows the
         * current motion direction. Direction reversal occurs only after Brake is
         * declared complete.
         */
        int calculateBrakePwm() const noexcept {
            constexpr float kBrakeDeadBandPwm{5.0f};
            return static_cast<int>(std::copysign(kBrakeDeadBandPwm, controller.current_velocity_));
        }

        inline TransitionRequest finishBrake() noexcept;
    };

    using State = std::variant<InactiveState, StoppedState, NormalState, BrakeState>;

 public:
    /**
     * @param direction_pin Motor CW/CCW command pin. The same signal is used by
     *        BLDC2430Encoder/PCNT to assign the logical velocity direction.
     * @param speed_control_pin Active-low BLDC2430 PWM speed-control pin.
     * @param speed_state_pin BLDC2430 FG feedback pin.
     * @param ticks_per_rev Empirically measured FG falling-edge events per
     *        output-wheel revolution.
     * @param invert_logic Mirrors logical forward/reverse for the opposite side
     *        of the differential drive while preserving a common robot convention.
     */
    WheelController(uint8_t direction_pin,
                    uint8_t speed_control_pin,
                    uint8_t speed_state_pin,
                    float ticks_per_rev,
                    bool invert_logic = false);

    /**
     * @brief Applies velocity-estimator, feed-forward, and PID configuration.
     *
     * max_feedback_pwm limits only the PID correction. The feed-forward term is
     * allowed to use the full actuator range; updatePidOutputLimits() further
     * clips PID headroom against the remaining +/-255 PWM range.
     */
    void configure(uint32_t control_period_ms, const WheelConfig& wheel_config);

    /**
     * @brief Initializes motor and encoder hardware.
     *
     * The motor is initialized first so the shared direction signal has a
     * deterministic output level before the encoder enables PCNT direction
     * tracking and falling-edge interrupts.
     */
    void begin();

    /**
     * @brief Enables or disables closed-loop control.
     *
     * Disabling clears the target, stops the motor, and discards controller
     * history. Re-enabling chooses Stopped for a zero target or prepares a fresh
     * motion start for a nonzero target.
     */
    void setActive(bool active = true);

    /**
     * @brief Executes one controller update.
     *
     * @param dt_ms Elapsed scheduler period in milliseconds. QuickPID uses its
     *        configured sample period internally; dt_ms is currently used for
     *        the bounded Brake-state duration.
     */
    void update(uint32_t dt_ms);

    /**
     * @brief Sets the desired wheel angular velocity [rad/s].
     *
     * Small commands inside the target-zero epsilon are normalized to zero.
     * A sign change while running enters Brake instead of immediately changing
     * the motor direction pin.
     */
    void setTargetVelocity(float target);

    /**
     * @return Controller's current filtered velocity estimate [rad/s].
     *
     * In Inactive and Stopped this intentionally reports zero rather than
     * continuously observing externally forced/coasting wheel motion.
     */
    float getCurrentVelocity() const noexcept { return current_velocity_; }

 private:
    static constexpr float kMaxPwm{255.0f};
    static constexpr float kMinPwm{-255.0f};

    // -------------------------------------------------------------------------
    // Shared hardware and controller data.
    // -------------------------------------------------------------------------
    BLDC2430Motor motor_;
    BLDC2430Encoder encoder_;
    WheelVelocityEstimator velocity_estimator_;

    // QuickPID reads current_velocity_ and target_velocity_ by pointer and writes
    // only the feedback correction term. Feed-forward is combined separately.
    QuickPID pid_;

    // Signed wheel angular velocities [rad/s].
    float target_velocity_{};
    float current_velocity_{};

    // Maximum configured feedback authority and latest QuickPID correction [PWM].
    float max_pid_correction_pwm_{};
    float pid_correction_pwm_{};

    // Directional steady-state feed-forward model.
    float feedforward_ks_forward_{};
    float feedforward_ks_reverse_{};
    float feedforward_kv_{};
    float feedforward_pwm_{};

    // State machine. The current state is the active alternative of the variant.
    State state_;

    // Typed event dispatcher.
    //
    // State handlers only return a TransitionRequest. They never modify state_.
    // processTransition() is called only after std::visit() has returned, which
    // prevents destruction of the state object while its member function is on
    // the call stack.
    template <typename EventT>
    void dispatch(EventT&& event) {
        const auto transition = std::visit(
            [&event](auto& state) {
                if constexpr (requires { state.handle(std::forward<EventT>(event)); }) {
                    return state.handle(std::forward<EventT>(event));
                }

                return TransitionRequest::None;
            },
            state_);

        processTransition(transition);
    }

    // Dedicated visitor for the periodic controller update. NormalState and
    // BrakeState implement update(); InactiveState and StoppedState simply have
    // no update() operation, so the visitor returns None for those alternatives.
    void dispatchUpdate(uint32_t dt_ms) {
        const auto transition = std::visit(
            [dt_ms](auto& state) {
                if constexpr (requires { state.update(dt_ms); }) {
                    return state.update(dt_ms);
                }

                return TransitionRequest::None;
            },
            state_);

        processTransition(transition);
    }

    // This is the only function allowed to modify state_. It is called after
    // the current state's visitor invocation has completely returned.
    void processTransition(TransitionRequest transition);

    // PWM zero disables motor drive. For the BLDC2430 driver this also prevents
    // relying on FG feedback during the fully-off interval.
    void stopMotor() noexcept { motor_.setPwmSpeed(0); }

    // Recomputes PID correction limits around the current feed-forward PWM.
    void updatePidOutputLimits();

    // Clears PID integral/output history and the stored correction term.
    void resetPid();

    // Calculates directional feed-forward PWM from the latest target velocity.
    float calculateFeedForwardPwm() const;
};

#endif  // WHEEL_CONTROLLER_HPP
