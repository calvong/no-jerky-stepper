// nj::Stepper — one stepper motor driven by STEP/DIR through an RMT TX channel.
//
// All motion calls are non-blocking and may be made at any time, including while moving:
// the new plan is spliced in at the commit horizon (~NJ_LOOKAHEAD_MS ahead of the motor)
// starting from the exact planned position/velocity/acceleration there, so motion stays
// continuous in position, velocity and acceleration.
//
// Durations are measured from the call. 0 means "as fast as the limits allow".
// Positions are in steps.
#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "nj/types.hpp"

namespace nj {

struct StepperConfig {
    gpio_num_t step_pin = GPIO_NUM_NC;
    gpio_num_t dir_pin = GPIO_NUM_NC;
    gpio_num_t enable_pin = GPIO_NUM_NC;  ///< optional
    bool invert_dir = false;
    bool enable_active_low = true;
    float step_pulse_us = 3.0f;  ///< STEP high time
    float dir_setup_us = 5.0f;   ///< DIR stable before the next STEP edge
    Limits limits{};             ///< used when a duration is 0
    Cruise cruise{};             ///< cruise phase of moveTo/move (default off)
};

struct StepperImpl;
class Group;
class PositionSensor;

class Stepper {
public:
    /// Create a stepper (allocates one RMT TX channel; up to 4 on the ESP32-S3).
    /// The motor is enabled (if an enable pin is given) and starts at position 0.
    static esp_err_t create(const StepperConfig& config, Stepper** out);
    /// Stops the motor and frees the channel. Delete any Group containing it first.
    ~Stepper();
    Stepper(const Stepper&) = delete;
    Stepper& operator=(const Stepper&) = delete;

    /// Minimum-jerk move to `target`, ending at rest. Long moves get a constant-velocity
    /// phase according to the cruise setting (see setCruise()).
    esp_err_t moveTo(int32_t target, float duration_s = 0.0f);
    /// Same, with a cruise setting for this move only.
    esp_err_t moveTo(int32_t target, float duration_s, const Cruise& cruise);
    /// Relative move from the current target.
    esp_err_t move(int32_t delta, float duration_s = 0.0f);
    esp_err_t move(int32_t delta, float duration_s, const Cruise& cruise);

    /// Replace the pending motion by a path through `waypoints` (ends at rest).
    esp_err_t followPath(const Waypoint* waypoints, size_t n, const PathOptions& options = {});
    /// Append waypoints to the current path (or after the current move).
    esp_err_t appendWaypoints(const Waypoint* waypoints, size_t n);

    /// Smooth: min-jerk deceleration to rest (duration 0 = from limits).
    /// Immediate: stop pulses now. On a group member this halts the whole group.
    esp_err_t stop(StopMode mode = StopMode::Smooth, float duration_s = 0.0f);

    /// Redefine the current position (only while idle). With an absolute sensor the sensor
    /// mapping follows the new frame; an incremental sensor is re-referenced to `position`.
    esp_err_t setPosition(int32_t position);
    esp_err_t setLimits(const Limits& limits);
    /// Cruise setting used by moveTo/move (ESP_ERR_INVALID_ARG if out of range).
    esp_err_t setCruise(const Cruise& cruise);
    esp_err_t enable(bool on);

    // ---- closed-loop feedback (see nj/sensor.hpp) ----
    // Works on single steppers and on group members; a group may mix members with and without
    // sensors. A fault on a member stops the whole group (smooth stop with shared timing).
    /// Attach a position sensor (calls sensor->init()). The sensor must outlive the stepper or
    /// be detached first. Incremental sensors stay inactive until calibrateSensor() or
    /// setPosition() gives them a reference.
    esp_err_t attachSensor(PositionSensor* sensor, const FeedbackConfig& config);
    esp_err_t detachSensor();
    /// Change the feedback mode; also clears a fault.
    esp_err_t setFeedbackMode(FeedbackMode mode);
    /// Idle only: map the next sensor reading to the current position (sets the reference /
    /// offset_steps) and clear a fault. Waits up to `timeout` for that reading.
    esp_err_t calibrateSensor(TickType_t timeout = pdMS_TO_TICKS(500));
    FeedbackStatus feedbackStatus() const;
    /// Last measured position [steps].
    double measuredPosition() const;
    /// Wait until the motion and any settle phase are finished. ESP_FAIL if feedback ended in
    /// a fault.
    esp_err_t waitUntilSettled(TickType_t timeout = portMAX_DELAY) const;

    /// Signed number of pulses emitted (leads the motor by at most one RMT buffer). With
    /// feedback, corrections are folded into this counter once idle, so it follows the plan.
    int32_t position() const;
    /// Final position of the current plan.
    int32_t targetPosition() const;
    /// Planned state at the commit horizon.
    State commandedState() const;
    /// Also true during a feedback settle phase.
    bool isMoving() const;
    esp_err_t waitUntilIdle(TickType_t timeout = portMAX_DELAY) const;
    Stats stats() const;

private:
    friend class Group;
    explicit Stepper(StepperImpl* impl) : impl_(impl) {}
    StepperImpl* impl_;
};

}  // namespace nj
