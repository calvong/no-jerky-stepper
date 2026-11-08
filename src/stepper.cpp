#include "nj/stepper.hpp"

#include <cmath>
#include <new>

#include "core/group_planner.hpp"
#include "esp_check.h"
#include "platform/esp32/planner_task.hpp"

namespace nj {

static const char* TAG = "nj_stepper";

namespace {

uint32_t us_to_ticks(float us, uint32_t lo, uint32_t hi)
{
    const double t = std::round(static_cast<double>(us) * cfg::kTicksPerUs);
    if (!(t >= lo)) {
        return lo;
    }
    return t > hi ? hi : static_cast<uint32_t>(t);
}

}  // namespace

esp_err_t Stepper::create(const StepperConfig& config, Stepper** out)
{
    ESP_RETURN_ON_FALSE(out, ESP_ERR_INVALID_ARG, TAG, "out is null");
    ESP_RETURN_ON_FALSE(cruise_valid(config.cruise), ESP_ERR_INVALID_ARG, TAG, "invalid cruise setting");
    PlannerTask& planner = PlannerTask::get();
    ESP_RETURN_ON_ERROR(planner.ensure_started(), TAG, "planner task");

    auto* impl = new (std::nothrow) StepperImpl;
    ESP_RETURN_ON_FALSE(impl, ESP_ERR_NO_MEM, TAG, "no memory");
    impl->cfg = config;

    const uint32_t pulse = us_to_ticks(config.step_pulse_us, 1, 0x7FFF);
    const uint32_t setup = us_to_ticks(config.dir_setup_us, 1, 0x7FFF);

    AxisConfig ac;
    ac.gen = make_gen_config(pulse, setup);
    ac.limits = config.limits;
    ac.cruise = config.cruise;
    impl->axis.configure(ac);

    ChannelConfig cc;
    cc.step_pin = config.step_pin;
    cc.dir_pin = config.dir_pin;
    cc.invert_dir = config.invert_dir;
    cc.pulse_ticks = pulse;
    cc.dir_setup_ticks = setup;
    esp_err_t err = impl->channel.init(cc, planner.handle());
    if (err == ESP_OK && config.enable_pin != GPIO_NUM_NC) {
        gpio_config_t io = {};
        io.pin_bit_mask = 1ULL << config.enable_pin;
        io.mode = GPIO_MODE_OUTPUT;
        err = gpio_config(&io);
        if (err == ESP_OK) {
            err = gpio_set_level(config.enable_pin, config.enable_active_low ? 0 : 1);
        }
    }
    if (err == ESP_OK) {
        err = planner.add(impl);
    }
    if (err != ESP_OK) {
        delete impl;
        return err;
    }
    *out = new (std::nothrow) Stepper(impl);
    if (!*out) {
        planner.remove(impl);
        delete impl;
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

Stepper::~Stepper()
{
    PlannerTask& planner = PlannerTask::get();
    planner.fb_lock();  // the feedback task may be inside this stepper's sensor update()
    planner.lock();
    impl_->channel.halt();
    impl_->sensor = nullptr;
    planner.unlock();
    planner.remove(impl_);
    planner.fb_unlock();
    delete impl_;
}

namespace {

esp_err_t move_to_impl(StepperImpl* impl, int32_t target, float duration_s, const Cruise* cruise)
{
    PlannerTask& planner = PlannerTask::get();
    Status st;
    {
        PlannerLock lk;
        planner.prepare_locked(impl);
        st = impl->axis.move_to(target, planner.from_commit(impl, duration_s), -1, cruise);
    }
    planner.wake();
    return to_esp_err(st);
}

int32_t relative_target(StepperImpl* impl, int32_t delta)
{
    PlannerLock lk;
    return static_cast<int32_t>(std::llround(impl->axis.target())) + delta;
}

}  // namespace

esp_err_t Stepper::moveTo(int32_t target, float duration_s)
{
    return move_to_impl(impl_, target, duration_s, nullptr);
}

esp_err_t Stepper::moveTo(int32_t target, float duration_s, const Cruise& cruise)
{
    ESP_RETURN_ON_FALSE(cruise_valid(cruise), ESP_ERR_INVALID_ARG, TAG, "invalid cruise setting");
    return move_to_impl(impl_, target, duration_s, &cruise);
}

esp_err_t Stepper::move(int32_t delta, float duration_s)
{
    return moveTo(relative_target(impl_, delta), duration_s);
}

esp_err_t Stepper::move(int32_t delta, float duration_s, const Cruise& cruise)
{
    return moveTo(relative_target(impl_, delta), duration_s, cruise);
}

esp_err_t Stepper::followPath(const Waypoint* waypoints, size_t n, const PathOptions& options)
{
    ESP_RETURN_ON_FALSE(waypoints && n > 0, ESP_ERR_INVALID_ARG, TAG, "empty path");
    ESP_RETURN_ON_FALSE(n <= static_cast<size_t>(cfg::kPathCapacity), ESP_ERR_NO_MEM, TAG, "path too long");
    PlannerTask& planner = PlannerTask::get();
    Status st;
    {
        PlannerLock lk;
        planner.prepare_locked(impl_);
        // Durations are from now; the first segment starts at the commit horizon.
        Waypoint first = waypoints[0];
        if (first.duration > 0.0) {
            first.duration = planner.from_commit(impl_, first.duration);
        }
        if (n == 1) {
            st = impl_->axis.follow_path(&first, 1, options);
        } else {
            Waypoint* buf = new (std::nothrow) Waypoint[n];
            if (!buf) {
                return ESP_ERR_NO_MEM;
            }
            buf[0] = first;
            for (size_t i = 1; i < n; ++i) {
                buf[i] = waypoints[i];
            }
            st = impl_->axis.follow_path(buf, static_cast<int>(n), options);
            delete[] buf;
        }
    }
    planner.wake();
    return to_esp_err(st);
}

esp_err_t Stepper::appendWaypoints(const Waypoint* waypoints, size_t n)
{
    ESP_RETURN_ON_FALSE(waypoints && n > 0, ESP_ERR_INVALID_ARG, TAG, "empty path");
    PlannerTask& planner = PlannerTask::get();
    Status st;
    {
        PlannerLock lk;
        planner.prepare_locked(impl_);
        st = impl_->axis.append_waypoints(waypoints, static_cast<int>(n));
    }
    planner.wake();
    return to_esp_err(st);
}

esp_err_t Stepper::stop(StopMode mode, float duration_s)
{
    PlannerTask& planner = PlannerTask::get();
    esp_err_t err = ESP_OK;
    {
        PlannerLock lk;
        if (mode == StopMode::Immediate) {
            err = planner.halt_locked(impl_);
        } else {
            err = to_esp_err(impl_->axis.stop(planner.from_commit(impl_, duration_s)));
        }
    }
    planner.wake();
    return err;
}

esp_err_t Stepper::setPosition(int32_t position)
{
    PlannerLock lk;
    if (PlannerTask::get().busy_locked(impl_) || impl_->fb.settling()) {
        return ESP_ERR_INVALID_STATE;
    }
    const int64_t old = impl_->axis.generator().position();
    const Status st = impl_->axis.set_position(position);
    if (st == Status::Ok) {
        impl_->channel.encoder().set_position(position);
        if (impl_->sensor) {
            if (impl_->sensor->kind() == SensorKind::Incremental) {
                impl_->fb.request_reference();
            } else {
                impl_->fb.reframe(static_cast<double>(position - old));
            }
        }
    }
    return to_esp_err(st);
}

namespace {

bool feedback_config_valid(const FeedbackConfig& c)
{
    const auto nonneg = [](double x) { return std::isfinite(x) && x >= 0.0; };
    return std::isfinite(c.steps_per_unit) && c.steps_per_unit != 0.0 && std::isfinite(c.offset_steps) &&
           nonneg(c.sensor_latency_ms) && nonneg(c.filter_tau_s) && nonneg(c.deadband_steps) &&
           nonneg(c.correction_time_s) && nonneg(c.max_correction_steps) && nonneg(c.max_jump_steps) &&
           nonneg(c.settle_tolerance_steps) && c.mode <= FeedbackMode::ContinuousAndSettle;
}

}  // namespace

esp_err_t Stepper::attachSensor(PositionSensor* sensor, const FeedbackConfig& config)
{
    ESP_RETURN_ON_FALSE(sensor, ESP_ERR_INVALID_ARG, TAG, "sensor is null");
    ESP_RETURN_ON_FALSE(feedback_config_valid(config), ESP_ERR_INVALID_ARG, TAG, "invalid feedback config");
    PlannerTask& planner = PlannerTask::get();
    ESP_RETURN_ON_FALSE(sensor->init(), ESP_FAIL, TAG, "sensor init failed");
    if (sensor->context() == SampleContext::FeedbackTask) {
        ESP_RETURN_ON_ERROR(planner.ensure_feedback_task(), TAG, "feedback task");
    }
    planner.fb_lock();
    {
        PlannerLock lk;
        impl_->sensor = sensor;
        impl_->fb.configure(config, sensor->kind());
        impl_->next_sample_us = 0;
    }
    planner.fb_unlock();
    planner.wake();
    return ESP_OK;
}

esp_err_t Stepper::detachSensor()
{
    PlannerTask& planner = PlannerTask::get();
    planner.fb_lock();
    {
        PlannerLock lk;
        impl_->sensor = nullptr;
        impl_->fb.set_mode(FeedbackMode::Off);
    }
    planner.fb_unlock();
    return ESP_OK;
}

esp_err_t Stepper::setFeedbackMode(FeedbackMode mode)
{
    ESP_RETURN_ON_FALSE(mode <= FeedbackMode::ContinuousAndSettle, ESP_ERR_INVALID_ARG, TAG, "invalid mode");
    PlannerLock lk;
    ESP_RETURN_ON_FALSE(impl_->sensor, ESP_ERR_INVALID_STATE, TAG, "no sensor attached");
    impl_->fb.set_mode(mode);
    return ESP_OK;
}

esp_err_t Stepper::calibrateSensor(TickType_t timeout)
{
    PlannerTask& planner = PlannerTask::get();
    {
        PlannerLock lk;
        ESP_RETURN_ON_FALSE(impl_->sensor, ESP_ERR_INVALID_STATE, TAG, "no sensor attached");
        ESP_RETURN_ON_FALSE(!planner.busy_locked(impl_) && !impl_->fb.settling(), ESP_ERR_INVALID_STATE, TAG,
                            "stepper is moving");
        impl_->fb.request_reference();
    }
    planner.wake();
    const TickType_t start = xTaskGetTickCount();
    for (;;) {
        {
            PlannerLock lk;
            if (!impl_->fb.reference_pending()) {
                return ESP_OK;
            }
        }
        if (xTaskGetTickCount() - start >= timeout) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
}

FeedbackStatus Stepper::feedbackStatus() const
{
    PlannerLock lk;
    return impl_->fb.status();
}

double Stepper::measuredPosition() const
{
    PlannerLock lk;
    return impl_->fb.status().measured;
}

esp_err_t Stepper::waitUntilSettled(TickType_t timeout) const
{
    const esp_err_t err = waitUntilIdle(timeout);
    if (err != ESP_OK) {
        return err;
    }
    PlannerLock lk;
    return impl_->fb.status().state == FeedbackState::Fault ? ESP_FAIL : ESP_OK;
}

esp_err_t Stepper::setLimits(const Limits& limits)
{
    PlannerLock lk;
    impl_->axis.set_limits(limits);
    return ESP_OK;
}

esp_err_t Stepper::setCruise(const Cruise& cruise)
{
    ESP_RETURN_ON_FALSE(cruise_valid(cruise), ESP_ERR_INVALID_ARG, TAG, "invalid cruise setting");
    PlannerLock lk;
    impl_->axis.set_cruise(cruise);
    return ESP_OK;
}

esp_err_t Stepper::enable(bool on)
{
    const gpio_num_t pin = impl_->cfg.enable_pin;
    if (pin == GPIO_NUM_NC) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    const bool level = on != impl_->cfg.enable_active_low;
    return gpio_set_level(pin, level ? 1 : 0);
}

int32_t Stepper::position() const
{
    return impl_->channel.position();
}

int32_t Stepper::targetPosition() const
{
    PlannerLock lk;
    return static_cast<int32_t>(std::llround(impl_->axis.target()));
}

State Stepper::commandedState() const
{
    PlannerLock lk;
    return impl_->axis.commanded();
}

bool Stepper::isMoving() const
{
    PlannerLock lk;
    return PlannerTask::get().busy_locked(impl_) || (impl_->sensor && impl_->fb.settling());
}

esp_err_t Stepper::waitUntilIdle(TickType_t timeout) const
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

Stats Stepper::stats() const
{
    PlannerLock lk;
    Stats s;
    const StepStream& st = impl_->channel.stream();
    s.underruns = st.underruns.load(std::memory_order_relaxed);
    s.stretched_ticks = st.stretched_ticks.load(std::memory_order_relaxed);
    s.rate_clamps = impl_->axis.generator().rate_clamps();
    s.dir_stretches = impl_->axis.generator().dir_stretches();
    return s;
}

}  // namespace nj
