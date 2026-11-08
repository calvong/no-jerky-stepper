// Closed-loop feedback demo: correct lost steps with an absolute or an incremental sensor.
//
// Two sensor templates, pick one with NJ_EXAMPLE_SENSOR below. They show how to plug a sensor
// into the generic interface (nj/sensor.hpp); they are starting points, not full drivers.
//
//  * SoftPot (absolute): a membrane potentiometer read by the ADC. A SoftPot output floats
//    when nobody presses it, so wire a pull-down (e.g. 100k to GND) and treat readings near
//    0 V as "no touch" (update() returns false). The ADC is fast, so it is read in the planner
//    task (SampleContext::Planner), oversampled to reduce the ~9-10 effective bits of noise.
//  * Quadrature encoder (incremental): counted by a PCNT unit in x4 mode. The PCNT
//    accumulates overflows (accum_count), so rawRange() is 0 (no wrap to undo). An incremental
//    sensor needs a reference: calibrateSensor() or setPosition().
//
// Wiring (change to match your board): STEP = GPIO4, DIR = GPIO5, EN = GPIO6,
// SoftPot wiper -> GPIO7 (ADC1 channel 6), encoder A/B -> GPIO15 / GPIO16.
#include <cmath>

#include "driver/pulse_cnt.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nj/no_jerky_stepper.hpp"

#define NJ_SENSOR_SOFTPOT 1
#define NJ_SENSOR_ENCODER 2
#define NJ_EXAMPLE_SENSOR NJ_SENSOR_SOFTPOT

static const char* TAG = "feedback";

// ---------------------------------------------------------------------------------------------
// SoftPot via ADC oneshot, wrapped in a CallbackSensor. Position unit: mm along the strip.
namespace softpot {

constexpr adc_channel_t kChannel = ADC_CHANNEL_6;  // GPIO7 on the ESP32-S3
constexpr double kLengthMm = 100.0;                // active length of the strip
constexpr int kNoTouchRaw = 60;                    // below this the strip is not pressed
constexpr int kOversample = 4;

adc_oneshot_unit_handle_t g_adc = nullptr;

bool read(void* /*ctx*/, double* out_mm)
{
    int sum = 0;
    for (int i = 0; i < kOversample; ++i) {
        int raw = 0;
        if (adc_oneshot_read(g_adc, kChannel, &raw) != ESP_OK) {
            return false;
        }
        sum += raw;
    }
    const int raw = sum / kOversample;
    if (raw < kNoTouchRaw) {
        return false;  // not pressed: no valid position
    }
    *out_mm = kLengthMm * static_cast<double>(raw) / 4095.0;
    return true;
}

void init()
{
    adc_oneshot_unit_init_cfg_t ucfg = {};
    ucfg.unit_id = ADC_UNIT_1;
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&ucfg, &g_adc));
    adc_oneshot_chan_cfg_t ccfg = {};
    ccfg.atten = ADC_ATTEN_DB_12;
    ccfg.bitwidth = ADC_BITWIDTH_DEFAULT;
    ESP_ERROR_CHECK(adc_oneshot_config_channel(g_adc, kChannel, &ccfg));
}

}  // namespace softpot

// ---------------------------------------------------------------------------------------------
// Quadrature encoder on a PCNT unit, as an IncrementalSensor. Position unit: encoder counts.
class PcntEncoder : public nj::IncrementalSensor {
public:
    PcntEncoder(gpio_num_t a, gpio_num_t b) : a_(a), b_(b) {}

    bool init() override
    {
        pcnt_unit_config_t uc = {};
        uc.low_limit = -kLimit;
        uc.high_limit = kLimit;
        uc.flags.accum_count = 1;  // overflows are accumulated into the reported count
        if (pcnt_new_unit(&uc, &unit_) != ESP_OK) {
            return false;
        }
        pcnt_glitch_filter_config_t fc = {};
        fc.max_glitch_ns = 1000;
        pcnt_unit_set_glitch_filter(unit_, &fc);

        pcnt_chan_config_t ca = {};
        ca.edge_gpio_num = a_;
        ca.level_gpio_num = b_;
        pcnt_chan_config_t cb = {};
        cb.edge_gpio_num = b_;
        cb.level_gpio_num = a_;
        pcnt_channel_handle_t cha = nullptr;
        pcnt_channel_handle_t chb = nullptr;
        if (pcnt_new_channel(unit_, &ca, &cha) != ESP_OK || pcnt_new_channel(unit_, &cb, &chb) != ESP_OK) {
            return false;
        }
        // x4 decoding
        pcnt_channel_set_edge_action(cha, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE);
        pcnt_channel_set_level_action(cha, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        pcnt_channel_set_edge_action(chb, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE);
        pcnt_channel_set_level_action(chb, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE);
        // accum_count needs watch points at the limits.
        pcnt_unit_add_watch_point(unit_, kLimit);
        pcnt_unit_add_watch_point(unit_, -kLimit);
        return pcnt_unit_enable(unit_) == ESP_OK && pcnt_unit_clear_count(unit_) == ESP_OK &&
               pcnt_unit_start(unit_) == ESP_OK;
    }

protected:
    bool readRaw(int32_t* raw) override
    {
        int c = 0;
        if (pcnt_unit_get_count(unit_, &c) != ESP_OK) {
            return false;
        }
        *raw = c;
        return true;
    }
    int32_t rawRange() const override { return 0; }  // already accumulated by the driver

private:
    static constexpr int kLimit = 30000;
    gpio_num_t a_;
    gpio_num_t b_;
    pcnt_unit_handle_t unit_ = nullptr;
};

// ---------------------------------------------------------------------------------------------

static void report(nj::Stepper* m, const char* what)
{
    const nj::FeedbackStatus f = m->feedbackStatus();
    ESP_LOGI(TAG, "%-24s pos=%ld measured=%.1f slip=%.1f offset=%.1f state=%d fault=%d attempts=%u samples=%lu",
             what, static_cast<long>(m->position()), f.measured, f.slip, f.offset, static_cast<int>(f.state),
             static_cast<int>(f.fault), static_cast<unsigned>(f.settle_attempts),
             static_cast<unsigned long>(f.samples));
}

extern "C" void app_main(void)
{
    nj::StepperConfig cfg;
    cfg.step_pin = GPIO_NUM_4;
    cfg.dir_pin = GPIO_NUM_5;
    cfg.enable_pin = GPIO_NUM_6;
    cfg.limits = nj::Limits{8000.0, 40000.0, 2e6};

    nj::Stepper* m = nullptr;
    ESP_ERROR_CHECK(nj::Stepper::create(cfg, &m));

    nj::FeedbackConfig fc;
    fc.mode = nj::FeedbackMode::ContinuousAndSettle;
    fc.max_correction_steps = 400;  // more than this is treated as a stall
    fc.settle_tolerance_steps = 2;

#if NJ_EXAMPLE_SENSOR == NJ_SENSOR_SOFTPOT
    // Axis: 80 steps/mm (e.g. GT2 belt, 20T pulley, 16x microstepping).
    softpot::init();
    static nj::CallbackSensor sensor(&softpot::read, nullptr, nj::SensorKind::Absolute, nj::SampleContext::Planner);
    fc.steps_per_unit = 80.0;     // steps per mm
    fc.filter_tau_s = 0.03;       // ADC noise: a bit more filtering
    fc.deadband_steps = 4.0;      // ~0.05 mm
    fc.sensor_timeout_ms = 300;   // pressed at all times while moving, or it faults
    fc.settle_samples = 16;
#else
    // 3200 steps/rev motor, 1000 PPR encoder = 4000 counts/rev in x4.
    static PcntEncoder sensor(GPIO_NUM_15, GPIO_NUM_16);
    fc.steps_per_unit = 3200.0 / 4000.0;
    fc.filter_tau_s = 0.005;
    fc.deadband_steps = 1.0;
    fc.max_jump_steps = 50.0;     // a missed count wrap or a glitch shows up as a jump
#endif
    ESP_ERROR_CHECK(m->attachSensor(&sensor, fc));

    // Reference the sensor at the current position (required for an incremental sensor; for
    // the SoftPot it maps the strip to the current step count).
    if (m->calibrateSensor() != ESP_OK) {
        ESP_LOGW(TAG, "no sensor reading yet (SoftPot not pressed?)");
    }
    report(m, "calibrated");

    const int32_t targets[] = {4000, 1000, 6000, 0};
    for (int32_t t : targets) {
        ESP_ERROR_CHECK(m->moveTo(t));
        if (m->waitUntilSettled() != ESP_OK) {
            report(m, "FAULT");
            ESP_ERROR_CHECK(m->setFeedbackMode(fc.mode));  // clear and continue
            continue;
        }
        report(m, "settled");
    }
}
