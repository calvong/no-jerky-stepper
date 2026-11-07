#include "platform/esp32/planner_task.hpp"

#include <algorithm>

#include "core/group_planner.hpp"
#include "esp_log.h"

namespace nj {

static const char* TAG = "nj_planner";

esp_err_t to_esp_err(Status st)
{
    switch (st) {
    case Status::Ok:
        return ESP_OK;
    case Status::InvalidArg:
        return ESP_ERR_INVALID_ARG;
    case Status::NoMem:
        return ESP_ERR_NO_MEM;
    case Status::Infeasible:
        return ESP_ERR_INVALID_SIZE;
    case Status::Busy:
        return ESP_ERR_INVALID_STATE;
    }
    return ESP_FAIL;
}

PlannerTask& PlannerTask::get()
{
    static PlannerTask instance;
    return instance;
}

esp_err_t PlannerTask::ensure_started()
{
    if (task_) {
        return ESP_OK;
    }
    if (!mutex_) {
        mutex_ = xSemaphoreCreateRecursiveMutexStatic(&mutex_buf_);
    }
    const BaseType_t core = CONFIG_NJ_PLANNER_TASK_CORE < 0 ? tskNO_AFFINITY : CONFIG_NJ_PLANNER_TASK_CORE;
    const BaseType_t ok = xTaskCreatePinnedToCore(&PlannerTask::entry, "nj_planner", CONFIG_NJ_PLANNER_TASK_STACK,
                                                  this, CONFIG_NJ_PLANNER_TASK_PRIORITY, &task_, core);
    if (ok != pdPASS) {
        task_ = nullptr;
        ESP_LOGE(TAG, "failed to create planner task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void PlannerTask::lock()
{
    xSemaphoreTakeRecursive(mutex_, portMAX_DELAY);
}

void PlannerTask::unlock()
{
    xSemaphoreGiveRecursive(mutex_);
}

void PlannerTask::wake()
{
    if (task_) {
        xTaskNotifyGive(task_);
    }
}

esp_err_t PlannerTask::add(StepperImpl* s)
{
    PlannerLock lk;
    for (auto& slot : steppers_) {
        if (!slot) {
            slot = s;
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

void PlannerTask::remove(StepperImpl* s)
{
    PlannerLock lk;
    for (auto& slot : steppers_) {
        if (slot == s) {
            slot = nullptr;
        }
    }
}

esp_err_t PlannerTask::add_group(GroupImpl* g)
{
    PlannerLock lk;
    for (auto& slot : groups_) {
        if (!slot) {
            slot = g;
            return ESP_OK;
        }
    }
    return ESP_ERR_NO_MEM;
}

void PlannerTask::remove_group(GroupImpl* g)
{
    PlannerLock lk;
    for (auto& slot : groups_) {
        if (slot == g) {
            slot = nullptr;
        }
    }
}

bool PlannerTask::group_all_running(const GroupImpl* g)
{
    for (int i = 0; i < g->n; ++i) {
        if (!g->members[i]->channel.running()) {
            return false;
        }
    }
    return true;
}

void PlannerTask::prepare_locked(StepperImpl* s)
{
    GroupImpl* g = s->group;
    if (!g || group_all_running(g)) {
        return;
    }
    // Group not streaming: every member is idle, so align their time bases. Planning then
    // happens at one common commit tick and the members start together.
    bool idle = true;
    for (int i = 0; i < g->n; ++i) {
        idle = idle && !g->axes[i]->has_motion();
    }
    if (idle) {
        group_align(g->axes, g->n);
    }
}

double PlannerTask::from_commit(StepperImpl* s, double duration) const
{
    if (!(duration > 0.0) || !s->channel.running()) {
        return duration;
    }
    const StepGenerator& gen = s->axis.generator();
    const int64_t playback = gen.emitted_tick() - static_cast<int64_t>(s->channel.stream().lead_ticks());
    const double lead_s = gen.to_time(gen.scan_tick() - playback);
    return std::max(duration - lead_s, 0.5 * duration);
}

bool PlannerTask::busy_locked(StepperImpl* s) const
{
    return s->axis.has_motion() || s->channel.running() || !s->channel.stream().ring.empty();
}

esp_err_t PlannerTask::halt_locked(StepperImpl* s)
{
    GroupImpl* g = s->group;
    StepperImpl* const* list = g ? g->members : &s;
    const int n = g ? g->n : 1;
    esp_err_t err = ESP_OK;
    for (int i = 0; i < n; ++i) {
        StepperImpl* m = list[i];
        const esp_err_t e = m->channel.halt();
        m->axis.halt(m->channel.position(), m->channel.encoder().dir_positive());
        m->channel.stream().producer_idle.store(true, std::memory_order_release);
        if (e != ESP_OK) {
            err = e;
        }
    }
    if (g) {
        group_align(g->axes, g->n);
    }
    return err;
}

void PlannerTask::entry(void* arg)
{
    auto* self = static_cast<PlannerTask*>(arg);
    const TickType_t wait = std::max<TickType_t>(1, pdMS_TO_TICKS(CONFIG_NJ_LOOKAHEAD_MS / 4));
    for (;;) {
        ulTaskNotifyTake(pdTRUE, wait);
        self->service();
    }
}

void PlannerTask::service()
{
    PlannerLock lk;
    for (StepperImpl* s : steppers_) {
        if (s && !s->group) {
            service_single(s);
        }
    }
    for (GroupImpl* g : groups_) {
        if (g) {
            service_group(g);
        }
    }
}

void PlannerTask::fill(StepperImpl* s, bool keep_alive)
{
    StepStream& st = s->channel.stream();
    const StepGenerator& gen = s->axis.generator();
    const int64_t playback = gen.emitted_tick() - static_cast<int64_t>(st.lead_ticks());
    s->axis.generate(st, playback + cfg::kLookaheadTicks, keep_alive);
}

void PlannerTask::service_single(StepperImpl* s)
{
    StepStream& st = s->channel.stream();
    if (s->axis.has_motion()) {
        fill(s, false);
    }
    // Push first, then publish idleness: the encoder checks the queue before the flag.
    st.producer_idle.store(!s->axis.has_motion(), std::memory_order_release);
    if (!s->channel.running() && !st.ring.empty()) {
        s->channel.start();
    }
    st.notify_armed.store(true, std::memory_order_release);
}

void PlannerTask::service_group(GroupImpl* g)
{
    bool any_running = false;
    bool all_running = true;
    bool any_motion = false;
    bool any_queued = false;
    for (int i = 0; i < g->n; ++i) {
        StepperImpl* m = g->members[i];
        const bool r = m->channel.running();
        any_running = any_running || r;
        all_running = all_running && r;
        any_motion = any_motion || m->axis.has_motion();
        any_queued = any_queued || !m->channel.stream().ring.empty();
    }

    if (any_running && !all_running) {
        // Some members finished their transaction while others still stream: wait for all to
        // stop so the group can restart in sync. Happens only right after a motion ended.
        for (int i = 0; i < g->n; ++i) {
            StepStream& st = g->members[i]->channel.stream();
            st.producer_idle.store(true, std::memory_order_release);
            st.notify_armed.store(true, std::memory_order_release);
        }
        return;
    }

    if (any_motion) {
        for (int i = 0; i < g->n; ++i) {
            fill(g->members[i], true);  // members without motion stream pauses (keep-alive)
        }
    }
    bool motion_after = false;
    for (int i = 0; i < g->n; ++i) {
        motion_after = motion_after || g->members[i]->axis.has_motion();
    }
    for (int i = 0; i < g->n; ++i) {
        g->members[i]->channel.stream().producer_idle.store(!motion_after, std::memory_order_release);
    }

    if (!any_running && (any_motion || any_queued)) {
        rmt_sync_reset(g->sync);
        for (int i = 0; i < g->n; ++i) {
            g->members[i]->channel.start();
        }
    }
    for (int i = 0; i < g->n; ++i) {
        g->members[i]->channel.stream().notify_armed.store(true, std::memory_order_release);
    }
}

}  // namespace nj
