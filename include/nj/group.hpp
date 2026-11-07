// nj::Group — coordinated multi-axis motion.
//
// Members share one time line: they start together through an RMT sync manager and are
// always planned at the same commit tick with the same segment durations. A rest-to-rest
// group move is a straight line in joint space. While any member moves, idle members stream
// pauses so all channels stay in lock-step.
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

    /// targets[i] belongs to member i.
    esp_err_t moveTo(const int32_t* targets, float duration_s = 0.0f);
    esp_err_t followPath(const GroupWaypoint* waypoints, size_t n, const PathOptions& options = {});
    esp_err_t stop(StopMode mode = StopMode::Smooth, float duration_s = 0.0f);

    bool isMoving() const;
    esp_err_t waitUntilIdle(TickType_t timeout = portMAX_DELAY) const;
    size_t size() const;

private:
    explicit Group(GroupImpl* impl) : impl_(impl) {}
    GroupImpl* impl_;
};

}  // namespace nj
