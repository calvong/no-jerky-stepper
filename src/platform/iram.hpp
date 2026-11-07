#pragma once

#if defined(ESP_PLATFORM)
#include "esp_attr.h"
#define NJ_IRAM IRAM_ATTR
#else
#define NJ_IRAM
#endif
