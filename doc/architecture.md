# Architecture

## Layers

| Layer | Files | Runs in | Depends on IDF |
|---|---|---|---|
| Public API | `include/nj/*.hpp`, `src/stepper.cpp`, `src/group.cpp` | caller task | yes |
| Planner task + RMT glue | `src/platform/esp32/*` | planner task, RMT ISR | yes |
| Axis / group planning | `src/core/axis.*`, `src/core/group_planner.*` | planner lock | no |
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
- An immediate stop of one member halts the whole group.

## Memory and CPU (defaults)

- Per axis: about 14 KB (40-segment trajectory, 32-waypoint path, solver scratch) plus a
  1 KB step queue.
- Planner task: 6 KB stack, priority 20, core 1.
- ISR rate: one refill per 24 symbols. A symbol is one step (≥ 10 µs at 100 kHz) or one
  filler (≥ 10 µs), so the worst case is about 4 kHz per channel.

## Known limitations

- `position()` counts pulses when they are *encoded*: up to 48 symbols ahead of the
  motor. A PCNT loop-back for exact position is on the roadmap.
- An immediate stop cuts pulses abruptly, so a fast motor may lose steps.
- The RMT layer has not been verified on hardware yet (QEMU has no RMT model).
