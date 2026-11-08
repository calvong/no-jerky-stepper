# Roadmap

## Done

| Stage | Content |
|---|---|
| 0 Scaffold | C++17 ESP-IDF component, Kconfig, host + QEMU test projects. The old precomputed LUT pipeline is removed (still in git history). |
| 1 Math core | `PolySegment`, min-jerk quintic, free-end quartic, exact peaks, durations from limits, `Trajectory`, `StepGenerator`, `StepEncoder` |
| 2 ESP32-S3 driver | `RmtStepChannel` (simple encoder, ISR pull), planner task, `nj::Stepper`, `single_axis` example |
| 3 Retarget & stops | commit-horizon splice with C2 continuity, smooth stop (quartic), immediate stop, best-effort limits from moving states |
| 4 Waypoints | `followPath` / `appendWaypoints`, `Stop` / `Heuristic` / `Optimal` (C4) via policies, limit-scaled durations, frozen near-arrival waypoints, `waypoints` example |
| 5 Groups | `nj::Group`, shared commit tick and durations, RMT sync start, keep-alive streaming, `multi_axis` example |
| Closed-loop feedback | generic `PositionSensor` / `IncrementalSensor` / `CallbackSensor`; offset track summed into the step generator; slip = commanded − measured with a latency-compensated history; `Settle`, `Continuous`, `ContinuousAndSettle`; faults (slip, jump, timeout, settle failed) with smooth stop; group members with any mix of sensors (fault stops the group); offset absorbed into the counter when idle; `feedback` example (SoftPot, PCNT encoder templates) |
| Cruise (point-to-point) | ramp + constant velocity + ramp for `moveTo` / `move` / `Group::moveTo`; `Distance`, `Percent` and `Auto` modes with `min_distance`; per-stepper / per-group setting plus per-call override |

## Next: hardware bring-up (do first)

QEMU has no RMT model, so the RMT layer has only been compiled, never run. On a real
ESP32-S3 with a logic analyser on STEP/DIR:

1. `single_axis`: the pulse count matches the targets; the step-rate envelope matches
   `python/plot_steps.py` for the same scenario; no gaps at the retarget.
2. Measure DIR-to-STEP setup at reversals (should be ≥ `dir_setup_us`) and confirm no
   old-direction pulse follows a DIR edge.
3. Check `stats()`: `underruns == 0` and `stretched_ticks == 0` in normal use. Stress it
   with 4 channels at 100 kHz plus Wi-Fi load.
4. `multi_axis`: rising edges of both channels align at the start (sync manager).
5. `feedback`: with a SoftPot and with a quadrature encoder, block the carriage briefly (or
   lower the motor current) to lose steps. The settle phase must bring `measuredPosition()`
   back within tolerance, `position()` must equal the target afterwards, and a hard stall must
   raise `SlipTooLarge` and stop smoothly. Tune `sensor_latency_ms` until a fast move without
   losses causes no corrections.

## Stage 6: useful features

- **TODO: cruise on waypoint paths.** Cruise is only implemented for point-to-point moves.
  Paths ignore the `Cruise` setting. For long path segments, split each one into ramp,
  cruise and ramp. This needs:
  - The via-point solver to treat the cruise ends as knots with fixed `v = ±v_c`, `a = 0`.
    The optimal C4 conditions then apply only between ramps and true waypoints.
  - A decision on the cruise speed when the neighbouring waypoints are not at rest
    (`Heuristic` / `Optimal` via velocities).
  - Bookkeeping for several trajectory segments per waypoint: tags and `pop_passed`, frozen
    arrivals (`kFreezeTime`), the `kPathCapacity + 2 <= kSegmentPool` budget, and
    `appendWaypoints` after a cruising move.
  - Shared timing for group paths (`group_follow_path`), so coordinated paths stay in sync.
- **Feedback: sensor drivers.** VL53L0X / VL53L1X (I2C, `FeedbackTask` context), SoftPot on
  ADC continuous/DMA mode with averaging, PCNT quadrature, magnetic linear/rotary encoders
  (AS5600, AS5047, ...), using the generic `nj/sensor.hpp` interface.
- **Feedback for groups: re-plan option.** Groups support feedback with per-member
  corrections (done). Optional later: a mode that re-plans the whole group on a large slip
  so the members slow down together instead of one member catching up.
- **Feedback: Kalman slip estimator.** A 1-D Kalman filter with the commanded trajectory as
  its process model (`SlipEstimator` interface), for slow, noisy rangefinders.
- **Rewritable step queue (low-latency corrections and retargets).** Today a plan change
  starts behind the whole software queue (about `NJ_LOOKAHEAD_MS`). Only what the encoder
  has copied into RMT memory is truly committed. Cut the queue just after that point and
  regenerate from there: latency drops to a few ms while the full margin stays for normal
  streaming. Needs:
  - A generator snapshot (scan tick, position, direction, last emit) per queued command,
    so the generator can be wound back to the cut point.
  - A safe handover with the encoder ISR: claim the cut index atomically so the ISR never
    reads a command that is being replaced, and fix up `pushed_ticks`.
  - Group members cut at one common tick.
  - Tests in the host simulation; `underruns` and `stretched_ticks` must stay 0 on hardware.
  DMA is not an alternative: a DMA buffer is committed and adds latency (and only one S3
  TX channel has DMA).
- **Feedback: homing helper** for incremental sensors (index pulse / limit switch → reference).
- **IMU (later, not for position).** Integrating an accelerometer twice drifts within
  seconds, and the commanded trajectory is a better motion model than an IMU. An IMU is
  useful for stall detection (commanded vs measured acceleration) and vibration/resonance
  monitoring. Fusion only matters for compliant loads or machines on a moving base.
- **Exact position feedback**: PCNT counting the STEP pin through the GPIO matrix, with DIR
  as the count direction. Fixes `position()` lead and immediate-stop accounting.
- **Enable management**: auto-enable before motion, disable after an idle timeout, with
  configurable delays.
- **Homing / limit switches**: GPIO ISR → immediate or smooth stop + `setPosition()`;
  homing sequence helper (fast seek, back off, slow seek).
- **Soft limits**: reject or clamp targets and waypoints outside `[min, max]`.
- **Velocity / jog mode**: `runAt(v, T)` using `min_jerk_free_end(s0, v1, T)` transitions
  and a constant-velocity segment.
- **Units**: steps-per-unit scaling (mm, deg) with microstepping in the config.
- **Events**: segment done, waypoint reached, idle, underrun. Sent from the planner task to
  a FreeRTOS queue or callback.
- **Diagnostics**: planner time per wake, queue fill histogram, a dry-run API that
  returns StepCmds without hardware (preview/plotting on the device).
- **Backlash compensation**: extra steps inserted on reversal.
- **More channels**: MCPWM+PCNT driver (as in FastAccelStepper) for more than 4 axes; ESP32 /
  C6 / P4 targets with their RMT layouts.

## Stage 7: other motion strategies

Each one is a `MotionStrategy` emitting `PolySegment`s, so the generator and driver stay
unchanged.

- **Trapezoid** (quadratic pieces): classic, fastest to compute.
- **Jerk-limited S-curve** (7-phase, cubic pieces): rest-to-rest first, then arbitrary start
  states (Ruckig-style time-optimal with kinematic limits).
- **Time-optimal min-jerk**: the shortest quintic within limits, already available as
  `moveTo(target)` with duration 0.
- **Time-synchronised multi-axis**: every axis finishes together with its own optimal profile.
- **Path strategies**: time-optimal path parameterisation along the waypoint spline.

## Open design questions

- Should the commit horizon shrink automatically for low-latency retargets at low speed
  (shorter look-ahead when the step rate is low)? The rewritable step queue (Stage 6) may
  make this unnecessary.
- Should `moveTo` with an explicit duration warn when the limits are exceeded (stats flag)?
- `int32` public positions: enough for most machines; switch to `int64` if needed.
