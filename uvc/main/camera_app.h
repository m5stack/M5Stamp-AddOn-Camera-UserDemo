#pragma once

#include <stdint.h>

extern const bool UVC_ONLY_BEST_FORMAT_PER_RESOLUTION;
extern const bool UVC_ENABLE_MJPEG_SW_CONVERSION;
extern const uint8_t UVC_MJPEG_SW_QUALITY_DEFAULT;
extern const bool UVC_ENABLE_STARTUP_SELF_TEST;

void camera_task(void *arg);
