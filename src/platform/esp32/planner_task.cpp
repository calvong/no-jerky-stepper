#include "platform/esp32/planner_task.hpp"

#include <algorithm>

#include "core/group_planner.hpp"
#include "esp_log.h"
#include "esp_timer.h"

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
    if (!fb_mutex_) {
        fb_mutex_ = xSemaphoreCreateMutexStatic(&fb_mutex_buf_);
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

void PlannerTask::fb_lock()
{
    xSemaphoreTake(fb_mutex_, portMAX_DELAY);
}

void PlannerTask::fb_unlock()
{
    xSemaphoreGive(fb_mutex_);
}

esp_err_t PlannerTask::ensure_feedback_task()
{
    if (fb_task_) {
        return ESP_OK;
    }
    const BaseType_t core = CONFIG_NJ_FEEDBACK_TASK_CORE < 0 ? tskNO_AFFINITY : CONFIG_NJ_FEEDBACK_TASK_CORE;
    const BaseType_t ok = xTaskCreatePinnedToCore(&PlannerTask::feedback_entry, "nj_feedback",
                                                  CONFIG_NJ_FEEDBACK_TASK_STACK, this,
                                                  CONFIG_NJ_FEEDBACK_TASK_PRIORITY, &fb_task_, core);
    if (ok != pdPASS) {
        fb_task_ = nullptr;
        ESP_LOGE(TAG, "failed to create feedback task");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

void PlannerTask::feedback_entry(void* arg)
{
    auto* self = static_cast<PlannerTask*>(arg);
    for (;;) {
        self->sample_blocking_sensors();
    }
}

void PlannerTask::sample_blocking_sensors()
{
    int64_t wake_us = esp_timer_get_time() + 50000;
    bool any = false;
    fb_lock();  // keeps every StepperImpl and its sensor alive while update() runs
    for (int i = 0; i < cfg::kMaxSteppers; ++i) {
        StepperImpl* s;
        {
            PlannerLock lk;
            s = steppers_[i];
        }
        if (!s || !s->sensor || s->sensor->context() != SampleContext::FeedbackTask) {
            continue;
        }
        const int64_t now = esp_timer_get_time();
        const int64_t period = static_cast<int64_t>(std::max<uint32_t>(1, s->fb.config().sample_period_ms)) * 1000;
        if (now >= s->next_sample_us) {
            const bool ok = s->sensor->update();  // may block: no planner lock held
            const int64_t t = esp_timer_get_time();
            if (ok) {
                const double v = s->sensor->getPosition();
                PlannerLock lk;
                s->fb.add_sample(t, v);
                any = true;
            }
            s->next_sample_us = std::max(s->next_sample_us + period, now);
        }
        wake_us = std::min(wake_us, s->next_sample_us);
    }
    fb_unlock();
    if (any) {
        wake();
    }
    const int64_t wait_us = wake_us - esp_timer_get_time();
    const TickType_t ticks = wait_us > 0 ? static_cast<TickType_t>((wait_us + portTICK_PERIOD_MS * 1000 - 1) /
                                                                   (portTICK_PERIOD_MS * 1000))
                                         : 0;
    vTaskDelay(std::max<TickType_t>(1, ticks));
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
        m->fb.cancel_settle();  // an immediate stop must not be followed by a correction move
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

bool PlannerTask::service_feedback(StepperImpl* s)
{
    PositionSensor* sensor = s->sensor;
    int64_t now = esp_timer_get_time();
    if (sensor->context() == SampleContext::Planner && sensor->update()) {
        now = esp_timer_get_time();
        s->fb.add_sample(now, sensor->getPosition());
    }
    const StepStream& st = s->channel.stream();
    const StepGenerator& gen = s->axis.generator();
    const int64_t playback = gen.emitted_tick() - static_cast<int64_t>(st.lead_ticks());
    FeedbackIo io;
    io.now_us = now;
    io.scan_play_us = now + std::max<int64_t>(0, gen.scan_tick() - playback) / cfg::kTicksPerUs;
    io.stream_busy = s->channel.running() || !st.ring.empty();
    const FeedbackOutcome out = s->fb.service(s->axis, io);
    if (out.absorbed != 0) {
        // Idle and drained: the hardware pulse counter follows the corrected step counter.
        s->channel.encoder().set_position(static_cast<int32_t>(s->axis.generator().position()));
    }
    if (out.stop) {
        const FeedbackStatus fs = s->fb.status();
        ESP_LOGW(TAG, "feedback fault %d (slip %.1f steps): stopping%s", static_cast<int>(fs.fault), fs.slip,
                 s->group ? " the group" : "");
    }
    return out.stop;
}

void PlannerTask::service_single(StepperImpl* s)
{
    StepStream& st = s->channel.stream();
    if (s->sensor && service_feedback(s)) {
        if (s->axis.has_motion() && s->axis.stop(0.0) != Status::Ok) {
            s->axis.stop(0.25);  // no limits configured: fixed-duration smooth stop
        }
    }
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

    // Feedback on members (any mix of closed- and open-loop members).
    bool has_sensor = false;
    for (int i = 0; i < g->n; ++i) {
        has_sensor = has_sensor || g->members[i]->sensor;
    }
    if (has_sensor) {
        if (!any_running && !any_queued && !any_motion) {
            // Group at rest: keep the time bases aligned, so a settle correction on one member
            // starts the group in step and a later group move is planned at one common tick.
            group_align(g->axes, g->n);
        }
        bool stop = false;
        for (int i = 0; i < g->n; ++i) {
            StepperImpl* m = g->members[i];
            if (m->sensor && service_feedback(m)) {
                stop = true;
            }
        }
        if (stop) {
            // A fault on one member stops all of them together, so the path stays coordinated.
            bool moving = false;
            for (int i = 0; i < g->n; ++i) {
                moving = moving || g->axes[i]->has_motion();
            }
            if (moving) {
                group_stop(g->axes, g->n, 0.0);
            }
        }
        any_motion = false;
        for (int i = 0; i < g->n; ++i) {
            any_motion = any_motion || g->axes[i]->has_motion();
        }
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
