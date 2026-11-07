// PlannerTask: the one FreeRTOS task that feeds every channel.
//
// Wakes on (a) a low-water notification from a channel's encoder ISR, (b) the end of an RMT
// transaction, (c) any API call, or (d) a timeout of a quarter of the look-ahead. Each wake it
// fills every active channel up to `playback + look-ahead`, starts stopped channels that have
// work, and starts groups synchronously through their RMT sync manager.
// A single mutex serialises the planner and all API calls; the ISR never takes it.
#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "platform/esp32/impl.hpp"

namespace nj {

class PlannerTask {
public:
    static PlannerTask& get();

    esp_err_t ensure_started();
    TaskHandle_t handle() const { return task_; }

    void lock();
    void unlock();
    void wake();

    esp_err_t add(StepperImpl* s);
    void remove(StepperImpl* s);
    esp_err_t add_group(GroupImpl* g);
    void remove_group(GroupImpl* g);

    /// Called under the lock before planning on `s`: aligns an idle group's time base.
    void prepare_locked(StepperImpl* s);
    /// Duration measured from now, converted to "from the commit horizon".
    double from_commit(StepperImpl* s, double duration) const;
    /// True if the stepper still moves or streams.
    bool busy_locked(StepperImpl* s) const;
    /// Immediate stop of `s` (of its whole group if it has one).
    esp_err_t halt_locked(StepperImpl* s);

private:
    static void entry(void* arg);
    void service();
    void service_single(StepperImpl* s);
    void service_group(GroupImpl* g);
    void fill(StepperImpl* s, bool keep_alive);
    static bool group_all_running(const GroupImpl* g);

    SemaphoreHandle_t mutex_ = nullptr;
    StaticSemaphore_t mutex_buf_;
    TaskHandle_t task_ = nullptr;
    StepperImpl* steppers_[cfg::kMaxSteppers] = {};
    GroupImpl* groups_[cfg::kMaxSteppers] = {};
};

class PlannerLock {
public:
    PlannerLock() { PlannerTask::get().lock(); }
    ~PlannerLock() { PlannerTask::get().unlock(); }
    PlannerLock(const PlannerLock&) = delete;
    PlannerLock& operator=(const PlannerLock&) = delete;
};

}  // namespace nj
