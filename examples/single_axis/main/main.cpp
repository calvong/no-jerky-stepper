// Single axis demo: fixed-duration move, limit-based move, live retarget, smooth stop.
//
// Wiring (change to match your board): STEP = GPIO4, DIR = GPIO5, EN = GPIO6 (active low),
// e.g. an A4988/DRV8825/TMC2209 in STEP/DIR mode. Positions are in (micro)steps.
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nj/no_jerky_stepper.hpp"

static const char* TAG = "single_axis";

static void report(nj::Stepper* m, const char* what)
{
    const nj::Stats s = m->stats();
    ESP_LOGI(TAG, "%-28s pos=%ld  underruns=%lu stretched=%lu clamps=%lu", what, static_cast<long>(m->position()),
             static_cast<unsigned long>(s.underruns), static_cast<unsigned long>(s.stretched_ticks),
             static_cast<unsigned long>(s.rate_clamps));
}

extern "C" void app_main(void)
{
    nj::StepperConfig cfg;
    cfg.step_pin = GPIO_NUM_4;
    cfg.dir_pin = GPIO_NUM_5;
    cfg.enable_pin = GPIO_NUM_6;
    cfg.limits = nj::Limits{20000.0, 100000.0, 5e6};  // steps/s, steps/s^2, steps/s^3

    nj::Stepper* m = nullptr;
    ESP_ERROR_CHECK(nj::Stepper::create(cfg, &m));

    // 1. Point A -> B in a given time: minimum-jerk profile, exactly 1.5 s.
    ESP_ERROR_CHECK(m->moveTo(3200, 1.5f));
    m->waitUntilIdle();
    report(m, "moveTo(3200, 1.5 s)");

    // 2. As fast as the limits allow.
    ESP_ERROR_CHECK(m->moveTo(0));
    m->waitUntilIdle();
    report(m, "moveTo(0, auto)");

    // 3. Change the target while moving: velocity and acceleration stay continuous.
    ESP_ERROR_CHECK(m->moveTo(16000, 3.0f));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(m->moveTo(-4000));  // reverses smoothly
    m->waitUntilIdle();
    report(m, "retarget 16000 -> -4000");

    // 4. Smooth stop from full speed.
    ESP_ERROR_CHECK(m->moveTo(100000));
    vTaskDelay(pdMS_TO_TICKS(1500));
    const nj::State st = m->commandedState();
    ESP_LOGI(TAG, "before stop: v=%.0f steps/s a=%.0f steps/s^2", st.v, st.a);
    ESP_ERROR_CHECK(m->stop(nj::StopMode::Smooth));
    m->waitUntilIdle();
    report(m, "smooth stop");

    ESP_ERROR_CHECK(m->moveTo(0));
    m->waitUntilIdle();
    report(m, "back home");
}
