// Generic position sensors for closed-loop feedback (SimpleFOC-style interface).
//
// A driver implements update() (take one reading) and getPosition() (the last reading, in the
// sensor's own units). The library calls update() either in the planner task (fast,
// non-blocking sensors) or in a separate low-priority task (sensors that block on I2C, a slow
// ADC, ...), see context(). The mapping to steps is part of FeedbackConfig, so a driver never
// needs to know about steps.
//
// Platform-independent: drivers for a concrete chip live outside this header.
#pragma once

#include <cstdint>

#include "nj/types.hpp"

namespace nj {

class PositionSensor {
public:
    virtual ~PositionSensor() = default;

    /// Called once by Stepper::attachSensor(). Return false on failure.
    virtual bool init() { return true; }
    /// Take one reading. Return false if there is no valid reading (e.g. a SoftPot not touched,
    /// out of range, bus error); getPosition() is then not used.
    virtual bool update() = 0;
    /// Last valid reading [sensor units].
    virtual double getPosition() const = 0;

    virtual SensorKind kind() const { return SensorKind::Absolute; }
    virtual SampleContext context() const { return SampleContext::FeedbackTask; }
};

/// Base for counters that wrap around (16-bit PCNT, 12/14-bit magnetic incremental, ...).
/// The driver implements readRaw() and rawRange(); update() unwraps the raw value into a
/// 64-bit accumulated count, like SimpleFOC's full-rotation tracking. update() must be called
/// often enough that the counter moves less than half a wrap between two calls.
class IncrementalSensor : public PositionSensor {
public:
    bool update() final;
    double getPosition() const final { return static_cast<double>(count_); }
    SensorKind kind() const override { return SensorKind::Incremental; }
    SampleContext context() const override { return SampleContext::Planner; }

    /// Re-reference: the accumulated count becomes `counts`.
    void zero(int64_t counts = 0) { count_ = counts; }
    int64_t count() const { return count_; }

protected:
    /// Current raw counter value. Return false on a read error.
    virtual bool readRaw(int32_t* raw) = 0;
    /// Counts per wrap: the raw value lives in [0, range) or [-range/2, range/2).
    /// 0 = never wraps (the raw value is already a full count).
    virtual int32_t rawRange() const = 0;

private:
    int64_t count_ = 0;
    int32_t last_raw_ = 0;
    bool have_last_ = false;
};

/// Sensor backed by a plain function, for quick integration or tests:
///   bool read(void* ctx, double* out)  -> fill *out, return false if no valid reading.
class CallbackSensor : public PositionSensor {
public:
    using ReadFn = bool (*)(void* ctx, double* out);

    CallbackSensor(ReadFn fn, void* ctx, SensorKind kind = SensorKind::Absolute,
                   SampleContext context = SampleContext::FeedbackTask)
        : fn_(fn), ctx_(ctx), kind_(kind), context_(context)
    {
    }

    bool init() override { return fn_ != nullptr; }
    bool update() override;
    double getPosition() const override { return value_; }
    SensorKind kind() const override { return kind_; }
    SampleContext context() const override { return context_; }

private:
    ReadFn fn_;
    void* ctx_;
    SensorKind kind_;
    SampleContext context_;
    double value_ = 0.0;
};

}  // namespace nj
