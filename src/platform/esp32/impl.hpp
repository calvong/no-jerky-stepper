// Internal objects behind the public nj::Stepper / nj::Group handles.
#pragma once

#include "core/axis.hpp"
#include "core/feedback.hpp"
#include "driver/rmt_tx.h"
#include "nj/sensor.hpp"
#include "nj/stepper.hpp"
#include "platform/esp32/rmt_step_channel.hpp"

namespace nj {

struct GroupImpl;

struct StepperImpl {
    StepperConfig cfg;
    Axis axis;
    RmtStepChannel channel;
    GroupImpl* group = nullptr;
    // Closed-loop feedback (optional).
    PositionSensor* sensor = nullptr;
    Feedback fb;
    int64_t next_sample_us = 0;  ///< FeedbackTask sensors: next update() time
};

struct GroupImpl {
    StepperImpl* members[kMaxGroupAxes] = {};
    Axis* axes[kMaxGroupAxes] = {};
    int n = 0;
    Cruise cruise;  ///< cruise setting of Group::moveTo
    rmt_sync_manager_handle_t sync = nullptr;
};

esp_err_t to_esp_err(Status st);

}  // namespace nj
