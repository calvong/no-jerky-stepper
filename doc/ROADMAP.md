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

## Stage 6: useful features

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
  (shorter look-ahead when the step rate is low)?
- Should `moveTo` with an explicit duration warn when the limits are exceeded (stats flag)?
- `int32` public positions: enough for most machines; switch to `int64` if needed.
