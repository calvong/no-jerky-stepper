// nj::Group — coordinated multi-axis motion.
//
// Members share one time line: they start together through an RMT sync manager and are
// always planned at the same commit tick with the same segment durations. A rest-to-rest
// group move is a straight line in joint space. While any member moves, idle members stream
// pauses so all channels stay in lock-step.
//
// Members may have feedback sensors (Stepper::attachSensor), any mix of closed- and open-loop
// members. Corrections apply per member: they put that member's real position back on the
// planned line, so the other members need no change. A settle correction after a move streams
// all members again (idle ones send pauses). A feedback fault on any member stops the group.
#pragma once

#include <cstddef>
#include <cstdint>

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "nj/stepper.hpp"
#include "nj/types.hpp"

namespace nj {

struct GroupImpl;

class Group {
public:
    /// Members must be idle and not already in a group. Up to 4 members.
    static esp_err_t create(Stepper* const* members, size_t n, Group** out);
    ~Group();
    Group(const Group&) = delete;
    Group& operator=(const Group&) = delete;

    /// targets[i] belongs to member i. Long moves get a constant-velocity phase according to
    /// the group's cruise setting (members' own settings are not used). All members share one
    /// timing, so a rest-to-rest move stays a straight line.
    esp_err_t moveTo(const int32_t* targets, float duration_s = 0.0f);
    /// Same, with a cruise setting for this move only.
    esp_err_t moveTo(const int32_t* targets, float duration_s, const Cruise& cruise);
    /// Cruise setting of moveTo (default off). Distance / Percent / min_distance are measured
    /// on the member with the longest travel. ESP_ERR_INVALID_ARG if out of range.
    esp_err_t setCruise(const Cruise& cruise);
    esp_err_t followPath(const GroupWaypoint* waypoints, size_t n, const PathOptions& options = {});
    esp_err_t stop(StopMode mode = StopMode::Smooth, float duration_s = 0.0f);

    /// True while any member moves, streams, or (with a feedback sensor) is still settling.
    bool isMoving() const;
    esp_err_t waitUntilIdle(TickType_t timeout = portMAX_DELAY) const;
    /// waitUntilIdle(), then ESP_FAIL if any member's feedback is in Fault.
    esp_err_t waitUntilSettled(TickType_t timeout = portMAX_DELAY) const;
    size_t size() const;

private:
    explicit Group(GroupImpl* impl) : impl_(impl) {}
    GroupImpl* impl_;
};

}  // namespace nj
