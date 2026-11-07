// Multi-axis demo: an X/Y gantry moving in coordinated straight lines and along a path.
//
// Wiring: X: STEP = GPIO4, DIR = GPIO5. Y: STEP = GPIO7, DIR = GPIO15. Shared EN = GPIO6.
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nj/no_jerky_stepper.hpp"

static const char* TAG = "multi_axis";

extern "C" void app_main(void)
{
    nj::StepperConfig cx;
    cx.step_pin = GPIO_NUM_4;
    cx.dir_pin = GPIO_NUM_5;
    cx.enable_pin = GPIO_NUM_6;
    cx.limits = nj::Limits{20000.0, 100000.0, 5e6};
    nj::StepperConfig cy = cx;
    cy.step_pin = GPIO_NUM_7;
    cy.dir_pin = GPIO_NUM_15;
    cy.limits = nj::Limits{10000.0, 50000.0, 2e6};  // a heavier axis

    nj::Stepper* x = nullptr;
    nj::Stepper* y = nullptr;
    ESP_ERROR_CHECK(nj::Stepper::create(cx, &x));
    ESP_ERROR_CHECK(nj::Stepper::create(cy, &y));
    nj::Stepper* members[] = {x, y};
    nj::Group* g = nullptr;
    ESP_ERROR_CHECK(nj::Group::create(members, 2, &g));

    // Straight lines: both axes share one duration (the slowest axis decides).
    const int32_t square[][2] = {{8000, 0}, {8000, 8000}, {0, 8000}, {0, 0}, {8000, 8000}, {0, 0}};
    for (const auto& p : square) {
        ESP_ERROR_CHECK(g->moveTo(p));
        g->waitUntilIdle();
        ESP_LOGI(TAG, "at (%ld, %ld)", static_cast<long>(x->position()), static_cast<long>(y->position()));
    }

    // A smooth jerk-optimal curve through waypoints.
    nj::GroupWaypoint path[5];
    const double pts[5][2] = {{4000, 1000}, {8000, 4000}, {6000, 8000}, {2000, 6000}, {0, 0}};
    for (int k = 0; k < 5; ++k) {
        path[k].pos[0] = pts[k][0];
        path[k].pos[1] = pts[k][1];
    }
    ESP_ERROR_CHECK(g->followPath(path, 5));
    g->waitUntilIdle();
    ESP_LOGI(TAG, "curve done at (%ld, %ld)", static_cast<long>(x->position()), static_cast<long>(y->position()));

    // Retarget the whole group mid-move.
    const int32_t far[] = {12000, 12000};
    const int32_t back[] = {2000, -2000};
    ESP_ERROR_CHECK(g->moveTo(far, 3.0f));
    vTaskDelay(pdMS_TO_TICKS(1200));
    ESP_ERROR_CHECK(g->moveTo(back));
    g->waitUntilIdle();
    ESP_LOGI(TAG, "retargeted to (%ld, %ld)", static_cast<long>(x->position()), static_cast<long>(y->position()));
}
