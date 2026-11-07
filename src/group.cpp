#include "nj/group.hpp"

#include <new>

#include "core/group_planner.hpp"
#include "esp_check.h"
#include "platform/esp32/planner_task.hpp"

namespace nj {

static const char* TAG = "nj_group";

esp_err_t Group::create(Stepper* const* members, size_t n, Group** out)
{
    ESP_RETURN_ON_FALSE(members && out && n >= 1 && n <= static_cast<size_t>(kMaxGroupAxes), ESP_ERR_INVALID_ARG,
                        TAG, "1..4 members required");
    PlannerTask& planner = PlannerTask::get();
    auto* g = new (std::nothrow) GroupImpl;
    ESP_RETURN_ON_FALSE(g, ESP_ERR_NO_MEM, TAG, "no memory");

    rmt_channel_handle_t chans[kMaxGroupAxes] = {};
    {
        PlannerLock lk;
        for (size_t i = 0; i < n; ++i) {
            StepperImpl* s = members[i]->impl_;
            if (s->group || planner.busy_locked(s)) {
                delete g;
                ESP_LOGE(TAG, "member %u is busy or already grouped", static_cast<unsigned>(i));
                return ESP_ERR_INVALID_STATE;
            }
            g->members[i] = s;
            g->axes[i] = &s->axis;
            chans[i] = s->channel.handle();
        }
        g->n = static_cast<int>(n);

        rmt_sync_manager_config_t sc = {};
        sc.tx_channel_array = chans;
        sc.array_size = n;
        const esp_err_t err = rmt_new_sync_manager(&sc, &g->sync);
        if (err != ESP_OK) {
            delete g;
            ESP_LOGE(TAG, "rmt_new_sync_manager: %s", esp_err_to_name(err));
            return err;
        }
        const esp_err_t add = planner.add_group(g);
        if (add != ESP_OK) {
            rmt_del_sync_manager(g->sync);
            delete g;
            return add;
        }
        for (int i = 0; i < g->n; ++i) {
            g->members[i]->group = g;
        }
        group_align(g->axes, g->n);
    }
    *out = new (std::nothrow) Group(g);
    if (!*out) {
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

Group::~Group()
{
    PlannerTask& planner = PlannerTask::get();
    {
        PlannerLock lk;
        for (int i = 0; i < impl_->n; ++i) {
            StepperImpl* s = impl_->members[i];
            s->channel.halt();
            s->axis.halt(s->channel.position(), s->channel.encoder().dir_positive());
            s->group = nullptr;
        }
        rmt_del_sync_manager(impl_->sync);
    }
    planner.remove_group(impl_);
    delete impl_;
}

namespace {

esp_err_t group_move_impl(GroupImpl* g, const int32_t* targets, float duration_s, const Cruise* cruise)
{
    ESP_RETURN_ON_FALSE(targets, ESP_ERR_INVALID_ARG, TAG, "targets is null");
    PlannerTask& planner = PlannerTask::get();
    Status st;
    {
        PlannerLock lk;
        planner.prepare_locked(g->members[0]);
        double t[kMaxGroupAxes];
        for (int i = 0; i < g->n; ++i) {
            t[i] = targets[i];
        }
        st = group_move_to(g->axes, g->n, t, planner.from_commit(g->members[0], duration_s),
                           cruise ? *cruise : g->cruise);
    }
    planner.wake();
    return to_esp_err(st);
}

}  // namespace

esp_err_t Group::moveTo(const int32_t* targets, float duration_s)
{
    return group_move_impl(impl_, targets, duration_s, nullptr);
}

esp_err_t Group::moveTo(const int32_t* targets, float duration_s, const Cruise& cruise)
{
    ESP_RETURN_ON_FALSE(cruise_valid(cruise), ESP_ERR_INVALID_ARG, TAG, "invalid cruise setting");
    return group_move_impl(impl_, targets, duration_s, &cruise);
}

esp_err_t Group::setCruise(const Cruise& cruise)
{
    ESP_RETURN_ON_FALSE(cruise_valid(cruise), ESP_ERR_INVALID_ARG, TAG, "invalid cruise setting");
    PlannerLock lk;
    impl_->cruise = cruise;
    return ESP_OK;
}

esp_err_t Group::followPath(const GroupWaypoint* waypoints, size_t n, const PathOptions& options)
{
    ESP_RETURN_ON_FALSE(waypoints && n > 0, ESP_ERR_INVALID_ARG, TAG, "empty path");
    ESP_RETURN_ON_FALSE(n <= static_cast<size_t>(cfg::kPathCapacity), ESP_ERR_NO_MEM, TAG, "path too long");
    PlannerTask& planner = PlannerTask::get();
    Status st;
    {
        PlannerLock lk;
        planner.prepare_locked(impl_->members[0]);
        GroupWaypoint* buf = new (std::nothrow) GroupWaypoint[n];
        if (!buf) {
            return ESP_ERR_NO_MEM;
        }
        for (size_t i = 0; i < n; ++i) {
            buf[i] = waypoints[i];
        }
        if (buf[0].duration > 0.0) {
            buf[0].duration = planner.from_commit(impl_->members[0], buf[0].duration);
        }
        st = group_follow_path(impl_->axes, impl_->n, buf, static_cast<int>(n), options);
        delete[] buf;
    }
    planner.wake();
    return to_esp_err(st);
}

esp_err_t Group::stop(StopMode mode, float duration_s)
{
    PlannerTask& planner = PlannerTask::get();
    if (mode == StopMode::Immediate) {
        esp_err_t err;
        {
            PlannerLock lk;
            err = planner.halt_locked(impl_->members[0]);
        }
        planner.wake();
        return err;
    }
    Status st;
    {
        PlannerLock lk;
        st = group_stop(impl_->axes, impl_->n, planner.from_commit(impl_->members[0], duration_s));
    }
    planner.wake();
    return to_esp_err(st);
}

bool Group::isMoving() const
{
    PlannerLock lk;
    for (int i = 0; i < impl_->n; ++i) {
        if (PlannerTask::get().busy_locked(impl_->members[i])) {
            return true;
        }
    }
    return false;
}

esp_err_t Group::waitUntilIdle(TickType_t timeout) const
{
    const TickType_t start = xTaskGetTickCount();
    while (isMoving()) {
        if (timeout != portMAX_DELAY && xTaskGetTickCount() - start >= timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
    return ESP_OK;
}

size_t Group::size() const
{
    return static_cast<size_t>(impl_->n);
}

}  // namespace nj
