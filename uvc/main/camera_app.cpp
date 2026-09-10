/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-FileCopyrightText: Copyright 2026
 */

#include "camera_app/camera_app_private.h"

/* Shared state for the camera app modules.
 *
 * These variables are defined in one translation unit so the DVP callbacks,
 * TinyUSB callbacks, and camera task all coordinate through the same state.
 */
frame_context_t s_frame_ctx;
tinyusb_uvc_context_t s_uvc_camera_ctx;
mjpeg_sw_encoder_t s_mjpeg_sw_encoder;
uvc_frame_profile_t s_uvc_frame_profiles[UVC_FRAME_PROFILE_COUNT];
uvc_profile_binding_t s_uvc_profile_bindings[UVC_FRAME_PROFILE_COUNT];
uint8_t *s_uvc_transfer_buffer = nullptr;
size_t s_uvc_transfer_buffer_size = 0;
usb_phy_handle_t s_usb_phy_handle = nullptr;
std::atomic<uint64_t> s_frame_count{0};
std::atomic<size_t> s_last_frame_size{0};
std::atomic<size_t> s_last_dma_window_size{0};
std::atomic<int> s_last_frame_slot{-1};
std::atomic<uint64_t> s_invalid_frame_count{0};
std::atomic<uint64_t> s_capture_buffer_miss_count{0};
std::atomic<bool> s_uvc_tx_busy{false};
std::atomic<bool> s_uvc_streaming_active{false};
std::atomic<bool> s_uvc_reconfig_in_progress{false};
std::atomic<bool> s_usb_device_mounted{false};
std::atomic<bool> s_usb_device_ready{false};
std::atomic<bool> s_usb_device_suspended{false};
std::atomic<size_t> s_active_capture_buffer_limit{0};
std::atomic<int> s_uvc_active_profile_index{-1};
std::atomic<int> s_uvc_pending_profile_index{-1};
std::atomic<int64_t> s_uvc_frame_interval_us{UVC_DEFAULT_FRAME_INTERVAL_US};
std::atomic<int64_t> s_uvc_last_frame_us{0};
std::atomic<uint64_t> s_uvc_send_attempt_count{0};
std::atomic<uint64_t> s_uvc_repeat_send_count{0};
std::atomic<uint64_t> s_uvc_xfer_started_count{0};
std::atomic<uint64_t> s_uvc_xfer_completed_count{0};
std::atomic<uint8_t> s_uvc_last_commit_format{0};
std::atomic<uint8_t> s_uvc_last_commit_frame{0};
std::atomic<uint32_t> s_uvc_last_commit_interval_us{0};
std::atomic<uint32_t> s_uvc_last_commit_payload{0};
std::atomic<bool> s_active_profile_is_mjpeg_sw{false};
std::atomic<bool> s_mjpeg_sw_capture_hold{false};
std::atomic<uint8_t> s_mjpeg_sw_quality{0};
