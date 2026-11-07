// Lock-free single-producer / single-consumer ring buffer.
// Producer: planner task. Consumer: RMT encoder callback (ISR). Safe across cores.
#pragma once

#include <atomic>
#include <cstdint>

namespace nj {

template <class T, uint32_t N>
class SpscRing {
    static_assert(N >= 2 && (N & (N - 1)) == 0, "capacity must be a power of two");

public:
    static constexpr uint32_t kCapacity = N;

    bool push(const T& v)
    {
        const uint32_t h = head_.load(std::memory_order_relaxed);
        const uint32_t t = tail_.load(std::memory_order_acquire);
        if (h - t >= N) {
            return false;
        }
        buf_[h & (N - 1)] = v;
        head_.store(h + 1, std::memory_order_release);
        return true;
    }

    bool pop(T& v)
    {
        const uint32_t t = tail_.load(std::memory_order_relaxed);
        const uint32_t h = head_.load(std::memory_order_acquire);
        if (h == t) {
            return false;
        }
        v = buf_[t & (N - 1)];
        tail_.store(t + 1, std::memory_order_release);
        return true;
    }

    uint32_t size() const
    {
        return head_.load(std::memory_order_acquire) - tail_.load(std::memory_order_acquire);
    }
    uint32_t free_slots() const { return N - size(); }
    bool empty() const { return size() == 0; }

    /// Discard all content. Only call while the consumer is stopped.
    void clear() { tail_.store(head_.load(std::memory_order_acquire), std::memory_order_release); }

private:
    T buf_[N];
    std::atomic<uint32_t> head_{0};
    std::atomic<uint32_t> tail_{0};
};

}  // namespace nj
