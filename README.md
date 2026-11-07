# No Jerky Stepper

Smooth stepper motor motion control for the ESP32 family (ESP-IDF component, C++17).

The core is the **minimum-jerk trajectory**: the motion that minimises the integral of
jerk², a 5th-order polynomial between two states. Motion is planned as polynomial segments,
sampled into exactly timed step pulses, and streamed to the STEP pin by the RMT peripheral.
The RMT driver approach follows [FastAccelStepper](https://github.com/gin66/FastAccelStepper)'s
RMT v2 driver.

## Features

- **Point A → B in a given time** with a minimum-jerk profile, or as fast as velocity /
  acceleration / jerk limits allow.
- **Cruise for long moves**: point-to-point moves can be ramp → constant velocity → ramp,
  with min-jerk ramps. The cruise is a fixed distance, a percentage of the move, or Auto
  (reach `v_max`, cruise the rest), and only applies above a minimum move distance.
- **Waypoint paths**: three via-point policies (`Stop`, `Heuristic`, jerk-`Optimal` C4
  spline), timed or limit-scaled segments, streaming `appendWaypoints()`.
- **Runtime retargeting**: any move, path or stop can be issued while moving. The new plan
  starts from the exact planned position, velocity *and acceleration* about 20 ms ahead (the
  commit horizon), so there are no velocity or acceleration jumps.
- **Smooth stop** (min-jerk deceleration with a free end position) and immediate stop.
- **Multi-axis groups** (up to 4 on the S3): synchronous start through the RMT sync
  manager, shared timing, straight-line coordinated moves and coordinated paths.
- **Exact step timing**: integer ticks of a 10 MHz clock accumulated in absolute time (no
  drift), exact crossing times at low speed, and safe DIR changes inside the RMT stream.
- **Strategy interface**: segments are generic polynomials, so time-optimal / S-curve /
  trapezoid strategies can be added later without touching the step generator or driver.

## Quick start

```cpp
#include "nj/no_jerky_stepper.hpp"

extern "C" void app_main()
{
    nj::StepperConfig cfg;
    cfg.step_pin = GPIO_NUM_4;
    cfg.dir_pin = GPIO_NUM_5;
    cfg.enable_pin = GPIO_NUM_6;
    cfg.limits = {20000, 1e5, 5e6};  // steps/s, steps/s^2, steps/s^3

    nj::Stepper* m;
    ESP_ERROR_CHECK(nj::Stepper::create(cfg, &m));

    m->moveTo(3200, 1.5f);   // exactly 1.5 s, minimum jerk
    m->waitUntilIdle();
    m->moveTo(-8000);        // as fast as the limits allow
    vTaskDelay(pdMS_TO_TICKS(300));
    m->moveTo(2000);         // retarget while moving: v and a stay continuous
    m->waitUntilIdle();
}
```

More in [examples/](examples): `single_axis`, `waypoints`, `multi_axis`.

To use the component in your project, add it to `EXTRA_COMPONENT_DIRS` (as the examples
do) or put it in your project's `components/` folder, then add `REQUIRES no-jerky-stepper`.
Requirements: ESP-IDF ≥ 5.3 (`rmt_new_simple_encoder`), target ESP32-S3.

## API overview

| Call | What it does |
|---|---|
| `Stepper::create(cfg, &m)` | Claim an RMT TX channel; position starts at 0 |
| `moveTo(target, T=0)` / `move(delta, T=0)` | Min-jerk move ending at rest; `T = 0` uses the limits |
| `setCruise(cruise)`, `moveTo(target, T, cruise)` | Cruise phase for long moves (per stepper, or per call) |
| `followPath(wps, n, {policy})` | Replace the pending motion with a path |
| `appendWaypoints(wps, n)` | Extend the current path or move |
| `stop(Smooth \| Immediate, T=0)` | Decelerate to rest, or cut pulses now |
| `position()`, `commandedState()`, `isMoving()`, `waitUntilIdle()`, `stats()` | Queries |
| `Group::create(members, n, &g)` → `moveTo(targets)`, `followPath(...)`, `stop()` | Coordinated axes |

Durations are counted from the call. When a duration is 0, the shortest one within
`Limits` is used. Limits are not enforced on durations you give explicitly. If a
moving start state makes the limits unreachable, `moveTo` uses the duration with the
smallest excess.

### Cruise

```cpp
nj::Cruise c;
c.mode = nj::CruiseMode::Auto;  // or Distance / Percent
c.min_distance = 10000;         // shorter moves stay one min-jerk quintic
m->setCruise(c);                // also: cfg.cruise, or m->moveTo(target, T, c) for one move
```

Pick the mode by what should stay fixed as the move length changes:

| Mode | `value` | Use when |
|---|---|---|
| `Distance` | cruise length [steps] | you need a constant-speed stretch of known length (scan, dispense, cut) |
| `Percent` | share of the move, 0–100 | the profile shape should scale with the move |
| `Auto` | unused | you want the fastest long moves: ramp up to `v_max`, cruise the rest |

A move cruises only if it is at least `min_distance` long and the cruise is shorter than the
move. Otherwise it is the usual single quintic. In v-bound moves, cruising saves up to 47% of
the time (`1.875·D/v_max` becomes about `D/v_max`). With an explicit duration, the cruise
splits the time instead of changing it. `Group::setCruise` / `Group::moveTo(targets, T, c)`
do the same for coordinated straight-line moves. Waypoint paths do not cruise yet.

## How it works (short)

```
API call ──► Axis (task): trajectory of PolySegments, spliced at the commit horizon
                 │  StepGenerator: 1 ms slices → StepCmd{ticks, steps, dir}
                 ▼
        lock-free SPSC queue (~20 ms look-ahead)
                 │  StepEncoder (RMT ISR, simple-encoder callback)
                 ▼
        RMT symbols → STEP pin; DIR toggled after the RMT buffer drained
```

See [doc/architecture.md](doc/architecture.md) for the design and
[doc/ROADMAP.md](doc/ROADMAP.md) for what's next. The maths is derived in
[python/gen_reference.py](python/gen_reference.py) and
[doc/latex/background_theory.pdf](doc/latex/background_theory.pdf).

## Tests

The motion core (`src/motion`, `src/stepgen`, `src/core`) has no ESP-IDF dependency and
is covered by Unity tests in [test/unit](test/unit). They run in two places:

- **Emulated ESP32-S3 (QEMU)**: `idf.py -C test/target set-target esp32s3 build`, then
  `idf.py -C test/target qemu` (install QEMU once with
  `python $IDF_PATH/tools/idf_tools.py install qemu-xtensa`; on Windows the QEMU binary needs
  `libiconv-2.dll`, e.g. from `C:\Espressif\tools\git\mingw64\bin`, on `PATH`).
- **Host**: `cmake -S test/host -B build-host && cmake --build build-host && ctest --test-dir build-host`.
  `build-host/dump_steps <move|retarget|path|reverse> > steps.csv` and
  `python python/plot_steps.py steps.csv` plot the step rate against the planned velocity.

## Status

Stages 0–5 of the roadmap are implemented. All unit tests pass on the emulated ESP32-S3,
and all examples build without warnings. QEMU does not emulate the RMT peripheral, so the
RMT driver layer (`src/platform/esp32`) still has to be checked on real hardware with a
logic analyser (see the roadmap).
