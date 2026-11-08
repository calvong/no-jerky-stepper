# Architecture

## Layers

| Layer | Files | Runs in | Depends on IDF |
|---|---|---|---|
| Public API | `include/nj/*.hpp`, `src/stepper.cpp`, `src/group.cpp` | caller task | yes |
| Planner task + RMT glue | `src/platform/esp32/*` | planner task, RMT ISR | yes |
| Axis / group planning | `src/core/axis.*`, `src/core/group_planner.*` | planner lock | no |
| Feedback (slip estimate, correction control) | `src/core/feedback.*`, `src/core/sensor.cpp`, `include/nj/sensor.hpp` | planner lock | no |
| Blocking sensor sampling | `PlannerTask::sample_blocking_sensors` | `nj_feedback` task | yes |
| Motion maths | `src/motion/*` (PolySegment, min-jerk, paths, strategy) | planner lock | no |
| Step generation | `src/stepgen/step_generator.*` | planner task | no |
| Symbol encoding | `src/stepgen/step_encoder.*` | RMT ISR | no |

Everything below the API/glue layers is plain C++17 and is unit-tested on the host and on
an emulated ESP32-S3.

## Data flow

```
 moveTo / followPath / stop / Group::*           (any task, under the planner mutex)
        │
        ▼
 Axis ─ Trajectory: queue of PolySegments on an absolute time axis (seconds)
        │  replan: truncate at the commit horizon, start from the analytic (p, v, a) there
        ▼
 StepGenerator::run(until = playback + look-ahead)          (planner task, ~every 5-10 ms)
        │  StepCmd {ticks, steps, dir}  — one to three per 1 ms slice, pauses when slow
        ▼
 StepStream: SPSC ring (128 cmds) + pushed/consumed tick counters + idle flag
        │
        ▼
 StepEncoder::encode()        (RMT ISR via rmt_new_simple_encoder, 24 symbols per refill)
        │  rmt_symbol_word_t: {low, pulse} per step, low-only fillers for long gaps
        ▼
 RMT TX channel ─► STEP pin            DIR pin ◄─ gpio_ll from the encoder
```

The planner task wakes on: a low-water notification from the encoder (lead < 10 ms), the
end of an RMT transaction, any API call, or a timeout of look-ahead/4.

## Trajectory representation

`PolySegment` is `{T, c0..c5}`: a polynomial of degree ≤ 5 in local time. Every strategy
emits these:

- min-jerk move: one quintic (`min_jerk()`, coefficients from the sympy solution);
- smooth stop / velocity change: one quartic (`min_jerk_free_end()`). For a free end
  position, the natural boundary condition x⁽⁵⁾(T) = 0 forces c5 = 0;
- future trapezoid / S-curve / time-optimal moves: quadratic or cubic pieces.

Planning uses double precision. It runs in task context, a few evaluations per millisecond,
so the S3's software double is cheap enough. The ISR only does integer work.

## Durations from limits

- Rest-to-rest: `T = max(1.875·D/v, √(5.7735·D/a), ∛(60·D/j))` (exact peak factors).
- From a moving state, or for paths: `search_min_scale()` scans durations upward in ×1.12
  steps, then bisects. The peaks of |v|, |a| and |j| over a segment are exact: the extremes of
  each are found at the roots of its derivative (`roots_in()`, recursive bracketing).
- If the start state already violates a limit (e.g. limits lowered while moving), that limit
  is widened to the start value. If the limits still cannot be met, the duration with the
  smallest excess is used.

## Cruise (long point-to-point moves)

One quintic reaches its peak velocity, 1.875 times its average, for only an instant. A long
v-bound move therefore takes `1.875·D/v_max`. With a `Cruise` setting, `moveTo` plans three
segments instead: ramp, constant velocity, ramp (`src/motion/cruise.*`).

- Each ramp is **half of a rest-to-rest min-jerk profile**, covering distance `d_r` in time
  `T_r`. It ends at `v_c = 1.875·d_r/T_r` with `a = 0`. The quintic through those boundary
  values is unique, so `min_jerk()` reproduces it exactly. From the limits,
  `T_r = rest_to_rest_duration(2·d_r)/2`. Its peaks are `v_c`, `2.887·d_r/T_r²` and
  `15·d_r/T_r³`, so `v_c ≤ v_max` holds automatically.
- With cruise share `f` (`d_c = f·L`, `d_r = (1−f)·L/2`), the cruise time is
  `T_c = 2f·T_r/(1.875(1−f))`, which does not depend on L. Group members with the same f and
  `T_r` therefore share their timing and move in a straight line. For an explicit total `T`:
  `T_r = T·0.9375(1−f)/(1.875(1−f)+f)`.
- Modes:
  - `Distance`: `d_c = value`.
  - `Percent`: `d_c = value% · L`.
  - `Auto`: the shortest ramp that reaches `v_max`,
    `d_r* = max(0.821·v²/a, √(2.276·v³/j))`, and the cruise covers the rest.

  A move cruises only when `L ≥ min_distance` and `0 < d_c < L`. Otherwise it is the usual
  single quintic.
- Groups: Distance, Percent and min_distance are measured on the member with the longest
  travel. Auto tries the share at which each member just reaches its own `v_max` and keeps
  the fastest option for the whole group. Members' own cruise settings are ignored.
- Moving start (a retarget): the ramp-out keeps the rest-based timing. The ramp-in from the
  current state uses the shortest duration within the limits (the `min_jerk_duration`
  search, best effort). p, v and a stay continuous at every joint; jerk jumps, as at a
  retarget.
- Waypoint paths do not cruise yet (see the roadmap).

## Step generation

Position steps happen where the planned position crosses `pos ± 0.5`.

- **Fast (≥ 2 steps per 1 ms slice):** the first and last crossing times are solved
  exactly (safeguarded Newton). The steps in between are spaced uniformly in integer ticks,
  with the remainder spread over the block so its total length is exact. The step rate
  therefore follows the true velocity with no quantisation jitter between slices. Tracking
  error stays below 0.75 step (0.5 is inherent to rounding).
- **Slow (one step per slice):** the exact crossing time is used. Pauses keep the queue
  fed, but the last `dir_lead` ticks before the scan point are always held back.
- **Reversal:** a reversed step is only emitted once the position has moved back by
  `0.5 + hysteresis` (default 0.5). This avoids forward/back dithering at turnaround points
  and keeps the gap long enough for the DIR drain (below).
- **Landing:** at the end the generator steps to exactly `round(final position)`.
- **Over-speed:** steps are never dropped. When the plan asks for more than
  `NJ_MAX_STEP_RATE_HZ`, steps are delayed (`rate_clamps` statistic).

Time is int64 ticks of the RMT clock, accumulated in absolute time, so there is no drift.

## The RMT stream and DIR changes

One RMT transaction lasts as long as the motion. The simple-encoder callback pulls
commands from the queue every time half of the channel memory (24 symbols) has been played.
It ends the transaction (`done = true`, after a short low tail) once the queue is empty and
the producer is idle. The planner restarts it when new work arrives.

The DIR pin is a GPIO written from the encoder, i.e. at *encode* time. Encoding runs at
most one RMT memory (K = 48 symbols) ahead of playback. Before toggling DIR, the encoder
therefore makes sure that K symbols have been written since the last pulse: it splits the
leading low time of the reversed step into the missing "drain" symbols (each at least
`NJ_FILLER_SYMBOL_US`). After the toggle it keeps at least `dir_setup_us` of low time before
the new pulse. The generator gives every reversed step at least
`K·filler + dir_setup + pulse` of low time (about 0.5 ms), so this normally costs nothing. If
not, the encoder stretches the gap and counts it in `stretched_ticks`.

The lead (pushed − consumed ticks) tells the planner where playback is:
`playback ≈ emitted − lead`, and it fills to `playback + NJ_LOOKAHEAD_MS`.

## Runtime retarget (C2 continuity)

The **commit horizon** is the generator's scan time: commands up to there have been
generated. Every plan change:

1. evaluates the trajectory at the commit horizon (exact p, v, a);
2. truncates the trajectory there;
3. plans from that state and appends.

The generator continues scanning from the same time, so position, velocity and acceleration
are continuous; jerk may jump, as min-jerk segments allow. Latency equals the look-ahead
(20 ms by default). API durations are measured from the call, and the glue subtracts the
lead.

### Latency budget

How long after a plan change (retarget, or a feedback reading) the motor reacts:

| Stage | Typical | Set by |
|---|---|---|
| Sensor read | µs (SoftPot on ADC, encoder on PCNT); 20–50 ms for ToF rangefinders | the sensor |
| Wait for the next planner service | 0–5 ms (wakes every look-ahead / 4) | `NJ_LOOKAHEAD_MS` |
| Software step queue (`StepStream` ring) | about `NJ_LOOKAHEAD_MS`, 20 ms | `NJ_LOOKAHEAD_MS` |
| RMT hardware memory (ping-pong, 48 symbols per block) | a few ms or less | `NJ_RMT_MEM_BLOCKS`, step rate |
| Feedback filter and correction move | `filter_tau_s` + `correction_time_s` (50 + 150 ms by default) | `FeedbackConfig` |

- **The software queue is most of the fixed delay**, not the RMT write. It is a deliberate
  margin: the planner is a FreeRTOS task, and the queue must cover scheduling jitter
  (Wi-Fi interrupts, other tasks, long planning calls) without an underrun. Plan changes
  never touch queued commands, so they start behind it.
- **DMA does not help here.** RMT TX DMA streams large buffers with fewer interrupts, but
  what is in a DMA buffer is committed, so it adds latency instead of removing it. On the
  ESP32-S3 only one TX channel supports DMA. It would only matter if the refill ISR load at
  very high step rates became a problem.
- **To react faster today:** lower `NJ_LOOKAHEAD_MS` (5–10 ms) and watch `stats().underruns`
  under the real load; for clean, fast sensors also lower `filter_tau_s` and
  `correction_time_s` (the defaults suit noisy rangefinders).
- **Planned: rewritable queue.** Only what the encoder has copied into RMT memory is truly
  committed. A plan change could cut the software queue just after that point and
  regenerate from there, giving a latency of a few ms while keeping the full margin for
  normal streaming (see the roadmap).

Paths keep the schedule of waypoints that are already planned when more are appended. A
waypoint arriving within 100 ms of the commit horizon is frozen, so a late append cannot
force a violent correction.

## Via-point optimality

For fixed segment times the total cost Σ∫j² depends on the free (v, a) at each
intermediate waypoint. Integration by parts gives ∂J/∂aₖ = 2·Δjerkₖ and
∂J/∂vₖ = −2·Δsnapₖ, so the optimum has continuous jerk and snap (a C4 spline). Each knot
contributes two linear equations coupling its neighbours. The result is a block-tridiagonal
system of 2×2 blocks, solved by the block Thomas algorithm in O(n). Waypoints with explicit
v/a become identity rows.

## Groups

- One commit tick for all members: the latest commit horizon among them. Members that have
  generated less simply truncate later.
- One duration per segment for all members, set by the slowest axis (auto durations).
- Synchronous start: `rmt_sync_reset()`, then `rmt_transmit()` on every member. While any
  member moves, idle members stream pauses (`keep_alive`), so every channel stays in its
  transaction and in lock-step.
- If some members finish their transaction before others (motion ended), the group waits
  until all have stopped before restarting them together.
- An immediate stop of one member halts the whole group. A feedback fault on one member
  smoothly stops the whole group.

## Closed-loop feedback

An optional position sensor (`nj::PositionSensor`, SimpleFOC-style `init` / `update` /
`getPosition`) closes the loop on a single stepper.

- **Offset track.** Corrections are a second trajectory on the `Axis` that the step generator
  adds to the plan (`TrajView`). It is spliced at the commit horizon like any retarget,
  using min-jerk segments that start from the offset's own (p, v, a), so corrections are
  C2-smooth. The plan itself, its waypoints, tags and arrival times are untouched.
- **Slip.** `slip = commanded(t_sample) − measured(t_sample)`: steps that were commanded
  but did not show up. The commanded position includes the offset, so the slip does not
  depend on the correction already applied. A delayed sample therefore never double-counts,
  and there is no loop gain to tune or oscillate. This is a disturbance observer (estimate
  the lost steps, add them back), not a PID: the step stream already sets the position, so
  there is no plant to stabilise, and the filter time constant alone sets the response.
- **Timing.** Each planner cycle records (when the commit horizon will be played, the
  commanded position there) in a 64-entry history. A sample at `t − sensor_latency` is
  compared with the interpolated history value. Corrections act after the look-ahead
  (`NJ_LOOKAHEAD_MS`) plus the sensor delay (see "Latency budget").
- **Estimator.** `SlipEstimator` interface. v1 is a first-order low-pass. A Kalman filter
  that uses the commanded trajectory as its process model is the planned upgrade.
- **Modes.** Continuous: the offset target follows the filtered slip outside a deadband.
  Settle: after the plan has ended and the stream has drained, wait, average N samples,
  correct the mean, and repeat up to `settle_max_attempts`. ContinuousAndSettle: continuous
  while the plan moves, then one settle phase.
- **Absorb.** Once idle (and not settling), the offset is folded into the step counter
  (`Axis::absorb_offset`, then the encoder counter is resynced). `position()` then equals the
  plan's end position again.
- **Faults.** Slip above `max_correction_steps` (stall or glitch), a slip jump above
  `max_jump_steps`, no sample for `sensor_timeout_ms` (checked only while corrections rely
  on samples), or a failed settle. Corrections then freeze and, with `stop_on_fault`, a
  smooth stop is issued. `setFeedbackMode()` or `calibrateSensor()` clears the fault.
- **Sampling.** `SampleContext::Planner` sensors (PCNT, cached ADC values) are read at the
  start of each planner service. `FeedbackTask` sensors (I2C, slow ADC) are read in the
  low-priority `nj_feedback` task, outside the planner lock, so a blocking read cannot delay
  step generation. Lock order: the feedback mutex first, then the planner mutex.
- **References.** Absolute sensors are referenced by `offset_steps`. Incremental sensors
  start unreferenced. `calibrateSensor()` (and, for incremental sensors, `setPosition()`)
  maps the next sample to the commanded position. `setPosition()` on an absolute sensor
  shifts the mapping along with the step frame.
- Groups. Members may have sensors in any mix. `service_group` runs each member's feedback
  before filling. Corrections stay per member: slip is measured against the member's own
  commanded position, so correcting it moves the real position back onto the coordinated
  line (only the correction transient deviates). While the group is fully at rest, the
  members' time bases are re-aligned (`group_align`) each service, so a settle correction
  on one member restarts all channels in step (idle members stream pauses) and a later
  group move still plans at one common tick. Absorbing waits until every channel of the
  group has stopped. A fault requesting a stop issues `group_stop()` on all members.

## Memory and CPU (defaults)

- Per axis: about 17 KB (40-segment trajectory and offset track, 32-waypoint path, solver
  scratch, feedback history) plus a 1 KB step queue.
- Planner task: 6 KB stack, priority 20, core 1.
- ISR rate: one refill per 24 symbols. A symbol is one step (≥ 10 µs at 100 kHz) or one
  filler (≥ 10 µs), so the worst case is about 4 kHz per channel.

## Known limitations

- `position()` counts pulses when they are *encoded*: up to 48 symbols ahead of the
  motor. A PCNT loop-back for exact position is on the roadmap. (A feedback sensor measures
  the real position: `measuredPosition()`.)
- An immediate stop cuts pulses abruptly, so a fast motor may lose steps.
- The RMT layer has not been verified on hardware yet (QEMU has no RMT model).
