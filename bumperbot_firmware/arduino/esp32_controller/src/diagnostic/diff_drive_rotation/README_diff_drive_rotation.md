# BLDC2430 Differential-Drive Rotation Diagnostic

This diagnostic validates the BLDC2430 differential-drive wheel rotation path
from motor command through FG pulse counting.

Its main purpose is to physically verify that the configured number of
**FG falling-edge pulses per output-wheel revolution** is correct for each wheel.

A simple physical check is to place a visible reference mark on each wheel before
the test. After the complete sequence finishes, both wheel marks should return
very close to their initial angular orientation.

The diagnostic also verifies:

- forward and reverse pulse counting;
- left/right direction inversion;
- full-revolution PCNT events;
- motor-command transitions at revolution boundaries;
- velocity estimation from FG periods;
- repeatability across low-, medium-, and high-PWM operation.

> The wheel marks are expected to return to the same **angular orientation**.
> The robot chassis is not expected to return to the same location on the floor.

---

# 1. Source structure

```text
src/diagnostic/diff_drive_rotation/
├── main.cpp
├── wheel_rotation_sequence_runner.hpp
└── README.md
```

The diagnostic reuses the production drivetrain components:

```text
diff_drive/bldc2430_motor.hpp
diff_drive/bldc2430_encoder.hpp
diff_drive/bldc2430_pulse_counter.hpp
diff_drive/wheel_velocity_estimator.hpp
diff_drive/diff_drive_constants.hpp
```

Each wheel uses two independent FG-processing paths:

```text
FG signal
   |
   +--> BLDC2430PulseCounter
   |       limits = +/- pulses_per_revolution
   |       purpose: full wheel revolution events
   |
   +--> BLDC2430Encoder
           limits = +/-1
           purpose: FG period / velocity estimation
```

This separation is intentional. Custom PCNT limits must not change the meaning of
`BLDC2430Encoder::delta_us`, which represents the interval between consecutive
FG falling edges.

---

# 2. Pulses-per-revolution verification

For one wheel define:

- $P_{cfg}$ — configured falling-edge count per output-wheel revolution;
- $P_{true}$ — actual falling-edge count per physical wheel revolution.

The full-revolution pulse counter is configured with:

```text
high limit = +P_cfg
low limit  = -P_cfg
```

For the current BLDC2430 setup, the intended constants are:

```cpp
kLeftMotorPulsePerRevolution
kRightMotorPulsePerRevolution
```

and currently correspond to approximately:

```text
306 falling edges / wheel revolution
```

Do not use the historical rising+falling transition count.

If the configuration is correct:

```math
P_{cfg}=P_{true}
```

then one PCNT limit event corresponds to exactly one physical wheel revolution.

For a step requesting $N$ revolutions:

```math
R_{physical}
=
N\frac{P_{cfg}}{P_{true}}
```

Therefore:

```math
P_{cfg}=P_{true}
\quad\Longrightarrow\quad
R_{physical}=N
```

### Example

With:

```text
P_true = 306
P_cfg  = 306
N      = 4
```

the wheel performs:

```math
R_{physical}
=
4\frac{306}{306}
=
4
```

physical revolutions.

If `P_cfg = 305`:

```math
R_{physical}
=
4\frac{305}{306}
\approx
3.9869
```

If `P_cfg = 307`:

```math
R_{physical}
=
4\frac{307}{306}
\approx
4.0131
```

A small PPR error therefore produces a repeatable angular phase error.

---

# 3. Physical wheel-mark verification

Before starting:

1. Place a visible high-contrast mark on each drive wheel.
2. Align each mark with a repeatable chassis reference.
3. Do not manually rotate the wheels after alignment.
4. Run the complete diagnostic.
5. After both motors stop, compare the final mark orientation with the initial
   orientation.

Expected result:

```text
left final mark  ~= left initial mark
right final mark ~= right initial mark
```

The comparison is angular. The robot may have moved forward or backward on the
floor.

---

# 4. Why the current sequence returns the mark to the same angle

The current sequence is:

```cpp
constexpr std::array kWheelRotationSequence{
    WheelRotationStep{50, 3},
    WheelRotationStep{100, 4},
    WheelRotationStep{255, 15},
    WheelRotationStep{50, 3},

    WheelRotationStep{-50, 3},
    WheelRotationStep{-100, 4},
    WheelRotationStep{-255, 15},
    WheelRotationStep{-50, 3},

    WheelRotationStep{50, 3},
};
```

Each element is:

```text
{ PWM command, requested revolutions }
```

The signed net rotation is:

```math
R_{net}
=
3+4+15+3-3-4-15-3+3
=
3
```

The wheel therefore finishes three complete revolutions ahead of its starting
rotation count.

Three revolutions correspond to:

```math
\Delta\theta
=
3(2\pi)
=
6\pi
```

and:

```math
6\pi
\equiv
0
\pmod{2\pi}
```

So the wheel mark should return to the same angular orientation.

The final step uses only `+50 PWM`, which also reduces final coasting compared
with stopping directly from the `+255` segment.

---

# 5. Important limitation of the final-mark check

A matching final mark alone does **not uniquely prove** that the configured PPR
is correct.

Example:

```text
P_true = 306
P_cfg  = 612
```

Then the current net physical rotation would be:

```math
R_{net,physical}
=
3\frac{612}{306}
=
6
```

Six complete revolutions also return the wheel mark to the same angular phase.

This is why the recommended verification has two stages:

```text
1. Absolute one-revolution check.
2. Full-sequence accumulated phase check.
```

For the absolute check, temporarily use:

```cpp
constexpr std::array kWheelRotationSequence{
    WheelRotationStep{50, 1},
};
```

Then visually verify:

```text
requested revolution count = 1
physical wheel turns       = 1
```

A factor-of-two error would be immediately obvious.

The full sequence is then useful for detecting accumulated errors across
different speeds and direction transitions.

---

# 6. Full-revolution counter versus velocity encoder

The runner uses:

```cpp
WheelRotationSequenceRunner(
    BLDC2430PulseCounter& full_revolution_counter,
    BLDC2430Encoder& velocity_encoder,
    BLDC2430Motor& motor,
    uint32_t velocity_update_period_ms,
    float velocity_ticks_per_revolution,
    const WheelRotationSequence<StepsNum>& sequence);
```

## Full-revolution counter

`full_revolution_counter_` uses:

```text
+P_cfg / -P_cfg
```

limits.

Its path is:

```text
FG falling edges
      |
      v
PCNT counts +/-P_cfg
      |
      v
one full-revolution event
```

Sequence progression depends on this counter.

## Velocity encoder

`velocity_encoder_` uses normal `BLDC2430Encoder` semantics:

```text
PCNT limits = +1 / -1
```

so every falling edge updates the timing measurement.

The elapsed time between consecutive falling edges is converted by
`WheelVelocityEstimator` into wheel angular velocity.

Velocity is diagnostic information only. It does not determine when a revolution
has completed.

---

# 7. Direction handling

The motors are physically mirrored.

The left side uses:

```cpp
invert_logic = false
```

and the right side uses:

```cpp
invert_logic = true
```

for both motor and pulse-counting abstractions.

The logical convention is therefore:

```text
positive PWM -> robot-forward wheel direction
negative PWM -> robot-reverse wheel direction
```

The FG signal itself is single-channel, not quadrature. Direction is inferred
from the motor direction-command GPIO.

The diagnostic therefore validates consistency across:

```text
motor direction
      +
PCNT direction control
      +
left/right inversion
      +
velocity sign
```

---

# 8. Event-driven revolution handling

The full-revolution pulse counter generates an ISR event when the PCNT high or
low limit is reached.

The ISR performs only minimal work:

```text
PCNT limit event
      |
      v
increment pending revolution count
      |
      v
notify diagnostic task
```

The ISR does not:

- print to `Serial`;
- change motor PWM;
- allocate memory;
- perform velocity calculations.

The pending-event count is transferred from ISR to task context using a lock-free
atomic counter.

---

# 9. Task wake-up and step transition

The diagnostic task waits for either:

- a revolution ISR notification; or
- the periodic velocity-update timeout.

The current timeout is:

```cpp
kVelocityUpdatePeriodMs = 10;
```

After wake-up, the first operation is:

```cpp
processRunnerEventsImmediately();
```

Only afterward are velocity estimates and diagnostic logs updated.

This minimizes the delay:

```text
revolution boundary
      |
      v
ISR
      |
      v
task wake-up
      |
      v
next PWM command / final PWM zero
```

That ordering is important for accurate revolution-boundary behavior.

---

# 10. Step completion

Each sequence step defines:

```cpp
struct WheelRotationStep {
    int pwm_speed;
    std::size_t revolutions;
};
```

For each full-revolution event:

```text
completed_revolutions += 1
```

When:

```text
completed_revolutions == requested revolutions
```

the runner immediately:

```text
advances to next step
```

or, for the final step:

```text
commands PWM = 0
```

The left and right runners operate independently. If one wheel reaches its
revolution boundary first, it advances immediately without waiting for the other
wheel.

This is useful because each wheel's pulse-count path is validated independently.

---

# 11. Excess revolution events

If more full-revolution events arrive before the task can react than the current
step still requires, the runner reports:

```text
WARNING: N excess revolution event(s)
```

Excess events are not carried into the next step because they occurred while the
previous PWM command was still active.

A non-zero excess count indicates that the motor continued long enough for at
least one additional full-revolution event before the task changed the command.

For accurate mark-position verification, the preferred result is:

```text
0 excess revolution events
```

for every step.

---

# 12. Velocity reporting

Velocity is updated using:

```cpp
current_velocity_ =
    velocity_estimator_.update(
        velocity_encoder_.getEdgeData());
```

When a step completes, the runner reports approximately:

```text
Left  wheel | PWM:   50 | velocity:  2.xxx rad/s
Right wheel | PWM:   50 | velocity:  2.xxx rad/s
```

The measured velocity helps verify:

- both wheels rotate at every test PWM;
- velocity sign follows command direction;
- high-speed FG timing remains valid;
- the two wheel assemblies behave reasonably similarly.

Velocity does not control revolution completion.

---

# 13. Current sequence coverage

| Step | PWM | Revolutions | Main purpose |
|---:|---:|---:|---|
| 1 | +50 | 3 | Low-speed forward |
| 2 | +100 | 4 | Medium-speed forward |
| 3 | +255 | 15 | Maximum-speed forward |
| 4 | +50 | 3 | High-to-low forward transition |
| 5 | -50 | 3 | Forward-to-reverse transition |
| 6 | -100 | 4 | Medium-speed reverse |
| 7 | -255 | 15 | Maximum-speed reverse |
| 8 | -50 | 3 | High-to-low reverse transition |
| 9 | +50 | 3 | Reverse-to-forward transition and final stop |

The long `+255` and `-255` sections help expose errors that may not appear during
a single low-speed revolution.

---

# 14. Recommended test procedure

## Preparation

1. Place the assembled robot in a clear area.
2. Verify both drive wheels and the rear caster can move freely.
3. Verify motor power and common ground.
4. Verify the FG pull-ups and wiring.
5. Confirm the configured values are falling-edge counts.
6. Mark both drive wheels.
7. Align each mark with a repeatable reference.

## Absolute PPR check

Before relying on the full sequence, preferably perform:

```cpp
WheelRotationStep{50, 1}
```

and visually verify exactly one physical revolution.

## Full sequence

Run the standard diagnostic and observe:

- correct direction;
- successful completion of all steps;
- reasonable velocity output;
- no excess revolution warnings.

Wait until:

```text
Rotation sequence completed successfully; both motors are stopped.
```

Then verify both wheel marks have returned close to their initial angular
orientation.

---

# 15. Interpreting mark error

A small final offset does not automatically mean the PPR value is wrong.

Possible contributors include:

### PPR error

A repeatable angular offset can indicate:

```text
P_cfg != P_true
```

For small error:

```math
\Delta P
=
P_{cfg}-P_{true}
```

the approximate phase error after the current net sequence is:

```math
\Delta\theta
\approx
2\pi R_{net}
\frac{\Delta P}{P_{true}}
```

With:

```text
P_true = 306
R_net  = 3
```

a one-edge error gives approximately:

```math
\lvert\Delta\theta\rvert
\approx
2\pi(3)\frac{1}{306}
\approx
0.0616\;\mathrm{rad}
\approx
3.53^\circ
```

### Final coasting

The final PWM becomes zero only after the full-revolution event has reached task
context. The wheel can coast slightly afterward.

### ISR-to-task latency

There is finite latency through:

```text
PCNT -> ISR -> task wake-up -> PWM update
```

although the implementation minimizes it.

### Mechanical effects

Gearbox backlash, wheel deformation, floor slip, and drivetrain compliance can
also create a small visible offset.

### FG counting problems

Noise, poor pull-up behavior, missed edges, or false edges can produce cumulative
error.

---

# 16. Failure patterns

| Observation | Likely interpretation |
|---|---|
| Two physical turns for requested `1` | PPR may be approximately 2x too large, e.g. old dual-edge count used |
| Less than one turn for requested `1` | PPR too small or false edges counted |
| More than one turn for requested `1` | PPR too large or real FG edges missed |
| One wheel direction incorrect | Check `invert_logic`, direction wiring, and PCNT direction mapping |
| Repeatable wheel-specific final offset | Investigate each wheel's PPR independently |
| Final offset varies strongly between runs | More likely coast, slip, noise, or timing variation than a fixed PPR error |
| Excess revolution warnings | Task reaction occurred after additional revolution events |

The left and right constants can be calibrated independently:

```cpp
kLeftMotorPulsePerRevolution
kRightMotorPulsePerRevolution
```

They do not have to be forced to the same empirical value.

---

# 17. What constitutes a strong pass

A strong result has all of the following:

- a one-revolution test produces exactly one physical wheel turn;
- both wheels rotate in the correct logical direction;
- all sequence steps complete;
- no excess revolution events are reported;
- velocity readings remain plausible;
- behavior is repeatable;
- both final wheel marks return very close to their initial angular orientation.

The combination of:

```text
absolute one-revolution check
+
full-sequence phase-return check
```

is significantly stronger than either test alone.

---

# 18. End-to-end path being validated

```text
BLDC2430 motor command
        |
        v
physical wheel rotation
        |
        v
FG falling edges
        |
        v
ESP32 PCNT
        |
        v
+/- pulses-per-revolution limit
        |
        v
ISR revolution event
        |
        v
atomic pending-event count
        |
        v
FreeRTOS task notification
        |
        v
task-context PWM transition
        |
        v
final PWM zero
        |
        v
wheel reference mark
```

This diagnostic should be rerun after changes to:

- motor or gearbox;
- FG edge-counting policy;
- pulse-per-revolution constants;
- direction wiring;
- PCNT configuration;
- motor/encoder abstractions;
- ISR/task event handling.

---

# 19. Sequence summary

```text
initialize motor, velocity encoder, and revolution counter
        |
        v
reset counters / estimator
        |
        v
apply step PWM
        |
        v
count FG falling edges in hardware
        |
        v
PCNT reaches +/- pulses_per_revolution
        |
        v
ISR records full revolution and wakes task
        |
        v
requested revolutions complete?
        |
        +-- no --> continue current PWM
        |
        +-- yes --> apply next PWM
                         |
                         v
                     next step
                         |
                         v
                        ...
                         |
                         v
                 final revolution reached
                         |
                         v
                      PWM = 0
                         |
                         v
                 both runners finished
                         |
                         v
              inspect wheel reference marks
```

For the current sequence:

```math
R_{net}=3
```

which is an integer number of net revolutions, so a correctly counted wheel
should finish with the same visible angular mark orientation as it started.
