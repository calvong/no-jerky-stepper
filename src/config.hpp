// Compile-time configuration. On ESP-IDF the values come from Kconfig (sdkconfig.h);
// on the host the defaults below are used (tests may override them with -D).
#pragma once

#include <cstdint>

#if defined(ESP_PLATFORM)
#include "sdkconfig.h"
#include "soc/soc_caps.h"
#endif

#ifndef CONFIG_NJ_RMT_RESOLUTION_HZ
#define CONFIG_NJ_RMT_RESOLUTION_HZ 10000000
#endif
#ifndef CONFIG_NJ_RMT_MEM_BLOCKS
#define CONFIG_NJ_RMT_MEM_BLOCKS 1
#endif
#ifndef CONFIG_NJ_SLICE_US
#define CONFIG_NJ_SLICE_US 1000
#endif
#ifndef CONFIG_NJ_LOOKAHEAD_MS
#define CONFIG_NJ_LOOKAHEAD_MS 20
#endif
#ifndef CONFIG_NJ_STEP_QUEUE_LEN_LOG2
#define CONFIG_NJ_STEP_QUEUE_LEN_LOG2 7
#endif
#ifndef CONFIG_NJ_SEGMENT_POOL
#define CONFIG_NJ_SEGMENT_POOL 40
#endif
#ifndef CONFIG_NJ_PATH_CAPACITY
#define CONFIG_NJ_PATH_CAPACITY 32
#endif
#ifndef CONFIG_NJ_MAX_STEP_RATE_HZ
#define CONFIG_NJ_MAX_STEP_RATE_HZ 100000
#endif
#ifndef CONFIG_NJ_FILLER_SYMBOL_US
#define CONFIG_NJ_FILLER_SYMBOL_US 10
#endif
#ifndef CONFIG_NJ_UNDERRUN_FILL_US
#define CONFIG_NJ_UNDERRUN_FILL_US 100
#endif
#ifndef CONFIG_NJ_REVERSAL_HYSTERESIS_PCT
#define CONFIG_NJ_REVERSAL_HYSTERESIS_PCT 50
#endif
#ifndef CONFIG_NJ_PLANNER_TASK_PRIORITY
#define CONFIG_NJ_PLANNER_TASK_PRIORITY 20
#endif
#ifndef CONFIG_NJ_PLANNER_TASK_CORE
#define CONFIG_NJ_PLANNER_TASK_CORE 1
#endif
#ifndef CONFIG_NJ_PLANNER_TASK_STACK
#define CONFIG_NJ_PLANNER_TASK_STACK 6144
#endif
#ifndef CONFIG_NJ_FEEDBACK_TASK_PRIORITY
#define CONFIG_NJ_FEEDBACK_TASK_PRIORITY 5
#endif
#ifndef CONFIG_NJ_FEEDBACK_TASK_CORE
#define CONFIG_NJ_FEEDBACK_TASK_CORE -1
#endif
#ifndef CONFIG_NJ_FEEDBACK_TASK_STACK
#define CONFIG_NJ_FEEDBACK_TASK_STACK 4096
#endif

#ifndef SOC_RMT_MEM_WORDS_PER_CHANNEL
#define SOC_RMT_MEM_WORDS_PER_CHANNEL 48
#endif

namespace nj::cfg {

constexpr double kTickHz = CONFIG_NJ_RMT_RESOLUTION_HZ;
constexpr uint32_t kTicksPerUs = CONFIG_NJ_RMT_RESOLUTION_HZ / 1000000;

/// Symbols held by one channel's RMT memory: the maximum distance between the
/// encoder (writer) and playback (reader).
constexpr uint32_t kRmtMemSymbols = SOC_RMT_MEM_WORDS_PER_CHANNEL * CONFIG_NJ_RMT_MEM_BLOCKS;

constexpr uint32_t kSliceTicks =
    static_cast<uint32_t>(static_cast<uint64_t>(CONFIG_NJ_SLICE_US) * CONFIG_NJ_RMT_RESOLUTION_HZ / 1000000);
constexpr uint32_t kLookaheadTicks =
    static_cast<uint32_t>(static_cast<uint64_t>(CONFIG_NJ_LOOKAHEAD_MS) * CONFIG_NJ_RMT_RESOLUTION_HZ / 1000);

constexpr uint32_t kStepQueueLen = 1u << CONFIG_NJ_STEP_QUEUE_LEN_LOG2;
constexpr int kSegmentPool = CONFIG_NJ_SEGMENT_POOL;
constexpr int kPathCapacity = CONFIG_NJ_PATH_CAPACITY;
static_assert(kPathCapacity + 2 <= kSegmentPool, "NJ_PATH_CAPACITY must be smaller than NJ_SEGMENT_POOL");

constexpr uint32_t kMinStepPeriodTicks = CONFIG_NJ_RMT_RESOLUTION_HZ / CONFIG_NJ_MAX_STEP_RATE_HZ;
constexpr uint32_t kFillerSymbolTicks = CONFIG_NJ_FILLER_SYMBOL_US * kTicksPerUs;
constexpr uint32_t kUnderrunFillTicks = CONFIG_NJ_UNDERRUN_FILL_US * kTicksPerUs;
constexpr double kReversalHysteresis = CONFIG_NJ_REVERSAL_HYSTERESIS_PCT / 100.0;

constexpr int kMaxSteppers = 4;   ///< RMT TX channels on the ESP32-S3

}  // namespace nj::cfg
