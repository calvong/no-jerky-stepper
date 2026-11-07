// Waypoint demo: the three via-point policies, then appending waypoints while moving.
//
// Wiring: STEP = GPIO4, DIR = GPIO5, EN = GPIO6 (active low).
#include <cmath>
#include <limits>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nj/no_jerky_stepper.hpp"

static const char* TAG = "waypoints";

static nj::Waypoint wp(double pos, double duration = 0.0)
{
    nj::Waypoint w;
    w.pos = pos;
    w.duration = duration;
    return w;
}

extern "C" void app_main(void)
{
    nj::StepperConfig cfg;
    cfg.step_pin = GPIO_NUM_4;
    cfg.dir_pin = GPIO_NUM_5;
    cfg.enable_pin = GPIO_NUM_6;
    cfg.limits = nj::Limits{15000.0, 80000.0, 4e6};

    nj::Stepper* m = nullptr;
    ESP_ERROR_CHECK(nj::Stepper::create(cfg, &m));

    // The same path with each via-point policy. Durations are auto (from limits).
    const nj::Waypoint path[] = {wp(2000), wp(6000), wp(4000), wp(9000), wp(0)};
    const nj::ViaPolicy policies[] = {nj::ViaPolicy::Stop, nj::ViaPolicy::Heuristic, nj::ViaPolicy::Optimal};
    const char* names[] = {"Stop", "Heuristic", "Optimal"};
    for (int i = 0; i < 3; ++i) {
        nj::PathOptions o;
        o.policy = policies[i];
        const TickType_t t0 = xTaskGetTickCount();
        ESP_ERROR_CHECK(m->followPath(path, sizeof(path) / sizeof(path[0]), o));
        m->waitUntilIdle();
        ESP_LOGI(TAG, "%-9s path done in %lu ms, pos=%ld", names[i],
                 static_cast<unsigned long>(pdTICKS_TO_MS(xTaskGetTickCount() - t0)),
                 static_cast<long>(m->position()));
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    // Timed waypoints: arrive at each one after the given segment duration.
    const nj::Waypoint timed[] = {wp(3000, 0.8), wp(3000, 0.5) /* dwell */, wp(-3000, 1.2), wp(0, 0.6)};
    ESP_ERROR_CHECK(m->followPath(timed, 4));
    m->waitUntilIdle();
    ESP_LOGI(TAG, "timed path done, pos=%ld", static_cast<long>(m->position()));

    // Streaming: keep appending waypoints while moving (e.g. from a host over UART).
    // The path is re-optimised from the commit horizon with continuous v and a.
    ESP_ERROR_CHECK(m->moveTo(5000));
    for (int k = 0; k < 8; ++k) {
        vTaskDelay(pdMS_TO_TICKS(300));
        const nj::Waypoint next = wp((k % 2) ? 5000.0 * (k + 1) / 8 : -5000.0 * (k + 1) / 8);
        ESP_ERROR_CHECK(m->appendWaypoints(&next, 1));
    }
    const nj::Waypoint home = wp(0);
    ESP_ERROR_CHECK(m->appendWaypoints(&home, 1));
    m->waitUntilIdle();
    ESP_LOGI(TAG, "streamed path done, pos=%ld", static_cast<long>(m->position()));
}
