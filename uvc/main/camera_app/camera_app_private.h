#pragma once

/* Internal camera application API.
 *
 * The camera app is split into several .cpp files, but DVP callbacks,
 * TinyUSB callbacks, and the main camera task still coordinate through a
 * small set of shared runtime state. Keep that private contract here so
 * camera_app.h can remain the public entry point used by main.cpp.
 */

/* SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-FileCopyrightText: Copyright 2026
 */

#include <algorithm>
#include <atomic>
#include <cinttypes>
#include <climits>
#include <cstdlib>
#include <cstring>
#include <sys/time.h>
#include <vector>

#include "esp_log.h"
#include "esp_attr.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "esp_private/usb_phy.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"

#include "esp_cam_sensor.h"
#include "esp_cam_sensor_xclk.h"
#include "esp_cam_ctlr_dvp.h"
#include "esp_cam_ctlr_dvp_ext.h"
#include "esp_jpeg_enc.h"
#include "tusb.h"
#include "usb_descriptors.h"
extern "C" {
#include "esp_sccb_intf.h"
#include "esp_sccb_i2c.h"
}
#if CONFIG_CAMERA_OV3660
#include "ov3660.h"
#endif
#if CONFIG_CAMERA_GC0308
#include "gc0308.h"
#endif

#include "esp_idf_version.h"
#include "camera_app.h"
#include "pinmap.h"

inline constexpr const char *TAG = "cam_capture";

inline constexpr i2c_port_num_t OV3660_I2C_PORT = I2C_NUM_0;
inline constexpr int OV3660_I2C_SPEED_HZ = 100000;
inline constexpr uint32_t OV3660_XCLK_FREQ_HZ = 20'000'000;
inline constexpr TickType_t OV3660_POWER_DELAY_TICKS = pdMS_TO_TICKS(10);
inline constexpr TickType_t CAMERA_TASK_LOOP_DELAY_TICKS = pdMS_TO_TICKS(100);
inline constexpr TickType_t USB_DEVICE_REAL_DELAY_TICKS = 1;
inline constexpr int64_t USB_DEVICE_MAX_BUSY_US = 100000;
inline constexpr int64_t USB_CAMERA_IDLE_STOP_DELAY_US = 500000;
inline constexpr UBaseType_t USB_DEVICE_TASK_PRIORITY = 6;
#if CONFIG_FREERTOS_UNICORE
inline constexpr BaseType_t USB_DEVICE_TASK_CORE = 0;
inline constexpr bool MJPEG_SW_ENCODER_DUAL_TASK = false;
inline constexpr BaseType_t MJPEG_SW_ENCODER_HELPER_CORE = 0;
#else
inline constexpr BaseType_t USB_DEVICE_TASK_CORE = 1;
inline constexpr bool MJPEG_SW_ENCODER_DUAL_TASK = true;
inline constexpr BaseType_t MJPEG_SW_ENCODER_HELPER_CORE = 0;
#endif
inline constexpr UBaseType_t MJPEG_SW_ENCODER_HELPER_PRIORITY = USB_DEVICE_TASK_PRIORITY + 1;
inline constexpr size_t CAMERA_FRAME_SLOT_COUNT = 3;
inline constexpr size_t FRAME_LOG_HASH_BYTES = 64;
inline constexpr uint8_t TINYUSB_RHPORT = 0;
inline constexpr uint32_t UVC_YUY2_SAFE_BUDGET_BYTES_PER_SECOND = 850000U;
inline constexpr uint32_t UVC_MJPEG_SW_SAFE_PIXELS_PER_SECOND = 1536000U;
inline constexpr size_t UVC_MJPEG_CAPTURE_BUFFER_BYTES = 256U * 1024U;

static_assert(CAMERA_FRAME_SLOT_COUNT >= 3, "camera capture needs at least triple buffering");

/* UVC 描述符策略：
 * - format/frame 列表保持稳定顺序，便于 host/UI 完整测试；
 * - default format/frame 只通过描述符默认值表达，优先指向 MJPEG 最大分辨率。 */

enum class uvc_frame_conversion_t {
    none,
    uyvy_to_yuy2,
    rgb565_to_yuy2,
    rgb565_be_to_yuy2,
    rgb565_be_to_rgb565,
    grayscale_to_yuy2,
    raw_to_mjpeg,
};

/* One host-visible UVC mode plus the capture/transfer sizes needed to serve it. */
struct uvc_frame_profile_t {
    uvc_descriptor_payload_t payload_format = UVC_DESCRIPTOR_PAYLOAD_YUY2;
    uint8_t format_index = 0;
    uint8_t frame_index = 0;
    uint16_t width = 0;
    uint16_t height = 0;
    uint8_t sensor_fps = 0;
    uint8_t usb_frame_rate = 0;
    int64_t frame_interval_us = 0;
    size_t frame_buffer_size = 0;
    size_t capture_buffer_size = 0;
};

constexpr size_t UVC_FRAME_PROFILE_COUNT = UVC_DESCRIPTOR_MAX_FRAME_COUNT;
constexpr int64_t UVC_DEFAULT_FRAME_INTERVAL_US = 1000000LL;

enum class detected_sensor_model_t {
    none,
    ov3660,
    gc0308,
};

struct detected_sensor_t {
    detected_sensor_model_t model = detected_sensor_model_t::none;
    uint8_t sccb_addr = 0;
    const char *name = "unknown";
};

struct frame_slot_t {
    uint8_t *buffer = nullptr;
    size_t buffer_size = 0;
    std::atomic<size_t> frame_size{0};
};

/* Triple-buffer ownership:
 * - DMA writes dma_inflight_slot;
 * - latest_slot holds the newest complete frame;
 * - TinyUSB temporarily owns uvc_locked_slot during frame transfer.
 */
struct frame_context_t {
    frame_slot_t slots[CAMERA_FRAME_SLOT_COUNT];
    std::atomic<int> latest_slot{-1};
    std::atomic<int> uvc_locked_slot{-1};
    std::atomic<int> dma_inflight_slot{-1};
};

/* Updated when the active UVC profile changes; read by usb_device_task. */
struct tinyusb_uvc_context_t {
    esp_cam_sensor_format_t sensor_fmt{};
    uvc_frame_conversion_t conversion = uvc_frame_conversion_t::none;
    TaskHandle_t task_handle = nullptr;
};

/* Maps a host format/frame selection back to the sensor format and conversion. */
struct uvc_profile_binding_t {
    const uvc_frame_profile_t *profile = nullptr;
    const esp_cam_sensor_format_t *sensor_fmt = nullptr;
    uvc_frame_conversion_t conversion = uvc_frame_conversion_t::none;
    bool available = false;
};

struct uvc_profile_candidate_t {
    const esp_cam_sensor_format_t *sensor_fmt = nullptr;
    uvc_descriptor_payload_t payload_format = UVC_DESCRIPTOR_PAYLOAD_YUY2;
    uvc_frame_conversion_t conversion = uvc_frame_conversion_t::none;
};

/* Cached software JPEG encoder for RAW/YUV/RGB modes advertised as MJPEG(SW). */
struct mjpeg_sw_encoder_t {
    jpeg_enc_handle_t handle = nullptr;
    jpeg_pixel_format_t src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    jpeg_subsampling_t subsampling = JPEG_SUBSAMPLE_420;
    uint16_t width = 0;
    uint16_t height = 0;
    uint8_t quality = 0;
    bool task_enable = false;
    size_t input_size = 0;
};

enum class uvc_copy_result_t {
    ok,
    invalid_arg,
    no_frame,
    slot_locked,
    invalid_frame_size,
    invalid_frame_content,
};

/* Shared runtime state. Definitions live in camera_app.cpp. */
extern frame_context_t s_frame_ctx;
extern tinyusb_uvc_context_t s_uvc_camera_ctx;
extern mjpeg_sw_encoder_t s_mjpeg_sw_encoder;
extern uvc_frame_profile_t s_uvc_frame_profiles[UVC_FRAME_PROFILE_COUNT];
extern uvc_profile_binding_t s_uvc_profile_bindings[UVC_FRAME_PROFILE_COUNT];
extern uint8_t *s_uvc_transfer_buffer;
extern size_t s_uvc_transfer_buffer_size;
extern usb_phy_handle_t s_usb_phy_handle;
extern std::atomic<uint64_t> s_frame_count;
extern std::atomic<size_t> s_last_frame_size;
extern std::atomic<size_t> s_last_dma_window_size;
extern std::atomic<int> s_last_frame_slot;
extern std::atomic<uint64_t> s_invalid_frame_count;
extern std::atomic<uint64_t> s_capture_buffer_miss_count;
extern std::atomic<bool> s_uvc_tx_busy;
extern std::atomic<bool> s_uvc_streaming_active;
extern std::atomic<bool> s_uvc_reconfig_in_progress;
extern std::atomic<bool> s_usb_device_mounted;
extern std::atomic<bool> s_usb_device_ready;
extern std::atomic<bool> s_usb_device_suspended;
extern std::atomic<size_t> s_active_capture_buffer_limit;
extern std::atomic<int> s_uvc_active_profile_index;
extern std::atomic<int> s_uvc_pending_profile_index;
extern std::atomic<int64_t> s_uvc_frame_interval_us;
extern std::atomic<int64_t> s_uvc_last_frame_us;
extern std::atomic<uint64_t> s_uvc_send_attempt_count;
extern std::atomic<uint64_t> s_uvc_repeat_send_count;
extern std::atomic<uint64_t> s_uvc_xfer_started_count;
extern std::atomic<uint64_t> s_uvc_xfer_completed_count;
extern std::atomic<uint8_t> s_uvc_last_commit_format;
extern std::atomic<uint8_t> s_uvc_last_commit_frame;
extern std::atomic<uint32_t> s_uvc_last_commit_interval_us;
extern std::atomic<uint32_t> s_uvc_last_commit_payload;
extern std::atomic<bool> s_active_profile_is_mjpeg_sw;
extern std::atomic<bool> s_mjpeg_sw_capture_hold;
extern std::atomic<uint8_t> s_mjpeg_sw_quality;

inline bool usb_device_bus_ready(void)
{
    return s_usb_device_mounted.load(std::memory_order_acquire)
        && s_usb_device_ready.load(std::memory_order_acquire)
        && !s_usb_device_suspended.load(std::memory_order_acquire);
}

void usb_device_task(void *arg);
esp_err_t prime_camera_for_probe(void);
esp_err_t create_camera_sccb_handle(i2c_master_bus_handle_t i2c_bus, uint8_t device_address, esp_sccb_io_handle_t *sccb_io);
esp_err_t start_camera_capture(esp_cam_sensor_device_t *sensor, esp_cam_ctlr_handle_t cam_ctlr);
bool probe_ov3660_sensor(i2c_master_bus_handle_t i2c_bus, uint8_t device_address);
bool probe_gc0308_sensor(i2c_master_bus_handle_t i2c_bus, uint8_t device_address);
detected_sensor_t detect_camera_sensor(i2c_master_bus_handle_t i2c_bus);
esp_cam_sensor_device_t *create_sensor_device(detected_sensor_model_t model, esp_cam_sensor_config_t *cam_cfg);
void log_sensor_para_capabilities(esp_cam_sensor_device_t *sensor);
void uvc_ctrl_cache_init(esp_cam_sensor_device_t *sensor);
void uvc_ctrl_apply_descriptor_masks(void);
void uvc_ctrl_cache_clear(void);
size_t estimate_uvc_frame_buffer_size(const esp_cam_sensor_format_t &fmt,
                                             uvc_descriptor_payload_t payload_format,
                                             uvc_frame_conversion_t conversion);
size_t uvc_payload_bytes_per_pixel(uvc_descriptor_payload_t payload_format);
uvc_descriptor_payload_t sensor_format_payload(const esp_cam_sensor_format_t &format);
uvc_frame_conversion_t sensor_format_conversion_to_yuy2(const esp_cam_sensor_format_t &format);
uvc_frame_conversion_t sensor_format_conversion_for_payload(const esp_cam_sensor_format_t &format,
                                                                   uvc_descriptor_payload_t payload_format);
bool sensor_format_can_encode_to_mjpeg_sw(const esp_cam_sensor_format_t &format);
int sensor_format_uvc_priority(const esp_cam_sensor_format_t &format);
bool sensor_format_can_stream_as_uvc(const esp_cam_sensor_format_t &format);
const esp_cam_sensor_format_t *find_first_safe_uvc_sensor_format(const esp_cam_sensor_format_array_t &formats,
                                                                        detected_sensor_model_t sensor_model);
uint8_t choose_uvc_usb_frame_rate(uvc_descriptor_payload_t payload_format,
                                         size_t frame_buffer_size,
                                         uint8_t sensor_fps,
                                         uvc_frame_conversion_t conversion,
                                         uint16_t width,
                                         uint16_t height);
size_t populate_uvc_profile_bindings(const esp_cam_sensor_format_array_t &formats,
                                            detected_sensor_model_t sensor_model,
                                            uvc_descriptor_config_t *descriptor_config);
const char *uvc_payload_format_name(uvc_descriptor_payload_t payload_format);
const char *uvc_frame_conversion_name(uvc_frame_conversion_t conversion);
const char *uvc_copy_result_name(uvc_copy_result_t result);
int find_uvc_profile_binding_index(uint8_t format_index, uint8_t frame_index);
size_t get_max_uvc_capture_buffer_size(void);
void reset_frame_context(void);
void close_mjpeg_sw_encoder(void);
bool is_jpeg_frame_content_valid(const uint8_t *buffer, size_t len);
uint8_t clamp_mjpeg_sw_quality(int32_t value);
esp_err_t set_mjpeg_sw_quality(int32_t value);
void convert_uyvy_to_yuy2(uint8_t *dst, const uint8_t *src, size_t len);
bool convert_frame_for_uvc(uvc_frame_conversion_t conversion,
                                  const esp_cam_sensor_format_t &sensor_fmt,
                                  uint8_t *dst,
                                  size_t dst_len,
                                  const uint8_t *src,
                                  size_t src_len,
                                  size_t *written_len);
bool prepare_latest_frame_for_uvc_xfer(const tinyusb_uvc_context_t &ctx,
                                              uint8_t **frame_buffer,
                                              size_t *frame_len,
                                              bool *slot_locked_for_xfer,
                                              uvc_copy_result_t *copy_result = nullptr);
void stop_camera_pipeline(esp_cam_sensor_device_t *sensor, esp_cam_ctlr_handle_t *cam_ctlr);
esp_err_t create_camera_controller(const esp_cam_sensor_format_t &sensor_fmt,
                                          const esp_cam_ctlr_dvp_pin_config_t &dvp_pins,
                                          esp_cam_ctlr_handle_t *cam_ctlr);
esp_err_t reconfigure_camera_pipeline(esp_cam_sensor_device_t *sensor,
                                             esp_cam_ctlr_handle_t *cam_ctlr,
                                             const esp_cam_ctlr_dvp_pin_config_t &dvp_pins,
                                             int profile_index,
                                             esp_cam_sensor_format_t *sensor_fmt);
void run_capture_self_test(esp_cam_sensor_device_t *sensor,
                                  esp_cam_ctlr_handle_t *cam_ctlr,
                                  const esp_cam_ctlr_dvp_pin_config_t &dvp_pins,
                                  esp_cam_sensor_format_t *sensor_fmt);
esp_err_t init_usb_phy(void);
void deinit_usb_phy(void);
esp_err_t start_tinyusb_uvc(const esp_cam_sensor_format_t &sensor_fmt, uvc_frame_conversion_t conversion);
void stop_tinyusb_uvc(void);

extern "C" void tud_mount_cb(void);
extern "C" void tud_umount_cb(void);
extern "C" void tud_suspend_cb(bool remote_wakeup_en);
extern "C" void tud_resume_cb(void);
extern "C" void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx);
extern "C" int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx,
                                     video_probe_and_commit_control_t const *parameters);


size_t estimate_frame_buffer_size(const esp_cam_sensor_format_t &fmt);
uint8_t mjpeg_sw_quality_default_value(void);
uint8_t mjpeg_sw_quality_value(void);
bool mjpeg_sw_encoder_config_for_sensor_format(const esp_cam_sensor_format_t &fmt,
                                               jpeg_pixel_format_t *src_type,
                                               jpeg_subsampling_t *subsampling,
                                               size_t *input_size);
size_t estimate_mjpeg_sw_frame_buffer_size(const esp_cam_sensor_format_t &fmt);
int uvc_profile_candidate_priority(const uvc_profile_candidate_t &candidate);
bool prefer_profile_candidate_for_default(const uvc_profile_candidate_t &lhs,
                                          const uvc_profile_candidate_t &rhs,
                                          bool prefer_largest_resolution);
uint8_t choose_preferred_frame_index(const std::vector<uvc_profile_candidate_t> &candidates,
                                     bool prefer_largest_resolution);
const uvc_profile_candidate_t *choose_default_resolution_candidate(
    const std::vector<uvc_profile_candidate_t> &jpeg_candidates,
    const std::vector<uvc_profile_candidate_t> &yuy2_candidates);
const char *uvc_profile_payload_name(const uvc_profile_binding_t &binding);
const char *jpeg_error_name(jpeg_error_t err);
bool ensure_mjpeg_sw_encoder(const esp_cam_sensor_format_t &sensor_fmt);
bool encode_raw_frame_to_mjpeg(const esp_cam_sensor_format_t &sensor_fmt,
                               uint8_t *dst,
                               size_t dst_len,
                               const uint8_t *src,
                               size_t src_len,
                               size_t *written_len);
bool frame_content_valid_for_uvc(const tinyusb_uvc_context_t &ctx, const uint8_t *buffer, size_t len);
int choose_capture_slot(frame_context_t *ctx);
bool any_uvc_profile_needs_transfer_buffer(void);
cam_ctlr_color_t map_cam_color(const esp_cam_sensor_output_format_t format);
bool on_get_new_trans(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
bool on_trans_finished(esp_cam_ctlr_handle_t handle, esp_cam_ctlr_trans_t *trans, void *user_data);
uint8_t clamp_to_u8(int value);
struct yuv_sample_t;
yuv_sample_t rgb_to_yuv(uint8_t red, uint8_t green, uint8_t blue);
yuv_sample_t rgb565_pixel_to_yuv(const uint8_t *pixel, bool big_endian);
void convert_rgb565_to_yuy2(uint8_t *dst, const uint8_t *src, size_t pixel_count, bool big_endian);
void convert_grayscale_to_yuy2(uint8_t *dst, const uint8_t *src, size_t pixel_count);
void convert_rgb565_be_to_rgb565(uint8_t *dst, const uint8_t *src, size_t pixel_count);

inline size_t align_up(size_t v, size_t a)
{
    return (v + (a - 1)) & ~(a - 1);
}
