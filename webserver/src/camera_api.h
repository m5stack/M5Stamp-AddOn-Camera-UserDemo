#pragma once

#include <esp_camera.h>

bool camera_reconfigure(framesize_t frame_size);
framesize_t camera_max_frame_size();
void camera_frame_lock(void);
void camera_frame_unlock(void);
