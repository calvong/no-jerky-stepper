#include "nj/sensor.hpp"

namespace nj {

bool IncrementalSensor::update()
{
    int32_t raw = 0;
    if (!readRaw(&raw)) {
        return false;
    }
    const int32_t range = rawRange();
    if (!have_last_ || range <= 0) {
        if (range <= 0) {
            count_ = raw;
        }
        last_raw_ = raw;
        have_last_ = true;
        return true;
    }
    // Shortest signed distance on the wrapped counter.
    int64_t d = static_cast<int64_t>(raw) - last_raw_;
    const int64_t r = range;
    d %= r;
    if (d >= (r + 1) / 2) {
        d -= r;
    } else if (d < -(r / 2)) {
        d += r;
    }
    count_ += d;
    last_raw_ = raw;
    return true;
}

bool CallbackSensor::update()
{
    double v = 0.0;
    if (!fn_ || !fn_(ctx_, &v)) {
        return false;
    }
    value_ = v;
    return true;
}

}  // namespace nj
