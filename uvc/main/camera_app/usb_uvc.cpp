#include "camera_app_private.h"

/* TinyUSB UVC runtime.
 *
 * TinyUSB callbacks run in USB context, while expensive sensor reconfiguration
 * is performed by camera_task. The commit callback records the requested mode
 * and the USB task pauses transfers until the camera pipeline catches up.
 */

namespace {

void update_usb_device_state(bool mounted, bool ready, bool suspended)
{
    s_usb_device_mounted.store(mounted, std::memory_order_release);
    s_usb_device_ready.store(ready, std::memory_order_release);
    s_usb_device_suspended.store(suspended, std::memory_order_release);
}

void clear_uvc_transfer_state(void)
{
    s_uvc_tx_busy.store(false, std::memory_order_release);
    s_uvc_streaming_active.store(false, std::memory_order_release);
    s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
    s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_release);
}

}

esp_err_t init_usb_phy(void)
{
    if (s_usb_phy_handle != nullptr) {
        return ESP_OK;
    }

    usb_phy_config_t phy_conf = { .controller = USB_PHY_CTRL_OTG, .target = USB_PHY_TARGET_INT, .otg_mode = USB_OTG_MODE_DEVICE, .otg_speed = USB_PHY_SPEED_UNDEFINED, .ext_io_conf = NULL, .otg_io_conf = NULL };

    return usb_new_phy(&phy_conf, &s_usb_phy_handle);
}

void deinit_usb_phy(void)
{
    if (s_usb_phy_handle != nullptr) {
        usb_del_phy(s_usb_phy_handle);
        s_usb_phy_handle = nullptr;
    }
}

esp_err_t start_tinyusb_uvc(const esp_cam_sensor_format_t &sensor_fmt, uvc_frame_conversion_t conversion)
{
    tusb_rhport_init_t dev_init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_AUTO,
    };
    esp_err_t err = init_usb_phy();
    if (err != ESP_OK) {
        return err;
    }

    const int active_profile_index = s_uvc_active_profile_index.load(std::memory_order_acquire);
    int64_t active_interval_us = UVC_DEFAULT_FRAME_INTERVAL_US;
    if (active_profile_index >= 0 && active_profile_index < static_cast<int>(UVC_FRAME_PROFILE_COUNT)
        && s_uvc_profile_bindings[active_profile_index].profile != nullptr) {
        active_interval_us = s_uvc_profile_bindings[active_profile_index].profile->frame_interval_us;
    }

    s_uvc_camera_ctx.sensor_fmt = sensor_fmt;
    s_uvc_camera_ctx.conversion = conversion;
    s_uvc_tx_busy.store(false, std::memory_order_relaxed);
    s_uvc_streaming_active.store(false, std::memory_order_relaxed);
    s_uvc_frame_interval_us.store(active_interval_us, std::memory_order_relaxed);
    s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
    s_uvc_xfer_started_count.store(0, std::memory_order_relaxed);
    s_uvc_xfer_completed_count.store(0, std::memory_order_relaxed);
    s_uvc_last_commit_format.store(0, std::memory_order_relaxed);
    s_uvc_last_commit_frame.store(0, std::memory_order_relaxed);
    s_uvc_last_commit_interval_us.store(0, std::memory_order_relaxed);
    s_uvc_last_commit_payload.store(0, std::memory_order_relaxed);
    update_usb_device_state(false, false, false);

    if (!tusb_init(TINYUSB_RHPORT, &dev_init)) {
        deinit_usb_phy();
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "starting TinyUSB UVC task core=%d priority=%u real_delay_tick=%u busy_slice=%lld us",
             (int)USB_DEVICE_TASK_CORE,
             (unsigned)USB_DEVICE_TASK_PRIORITY,
             (unsigned)USB_DEVICE_REAL_DELAY_TICKS,
             (long long)USB_DEVICE_MAX_BUSY_US);
    BaseType_t ok = xTaskCreatePinnedToCore(usb_device_task, "usb_uvc", 6 * 1024,
                                            &s_uvc_camera_ctx, USB_DEVICE_TASK_PRIORITY,
                                            &s_uvc_camera_ctx.task_handle, USB_DEVICE_TASK_CORE);
    if (ok != pdPASS) {
        tusb_deinit(TINYUSB_RHPORT);
        deinit_usb_phy();
        s_uvc_camera_ctx.task_handle = nullptr;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

void stop_tinyusb_uvc(void)
{
    if (s_uvc_camera_ctx.task_handle != nullptr) {
        vTaskDelete(s_uvc_camera_ctx.task_handle);
        s_uvc_camera_ctx.task_handle = nullptr;
    }
    if (tusb_inited()) {
        tusb_deinit(TINYUSB_RHPORT);
    }
    deinit_usb_phy();
    clear_uvc_transfer_state();
    s_uvc_reconfig_in_progress.store(false, std::memory_order_relaxed);
    s_uvc_active_profile_index.store(-1, std::memory_order_relaxed);
    s_uvc_pending_profile_index.store(-1, std::memory_order_relaxed);
    s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
    s_uvc_send_attempt_count.store(0, std::memory_order_relaxed);
    s_uvc_repeat_send_count.store(0, std::memory_order_relaxed);
    s_uvc_xfer_started_count.store(0, std::memory_order_relaxed);
    s_uvc_xfer_completed_count.store(0, std::memory_order_relaxed);
    s_uvc_last_commit_format.store(0, std::memory_order_relaxed);
    s_uvc_last_commit_frame.store(0, std::memory_order_relaxed);
    s_uvc_last_commit_interval_us.store(0, std::memory_order_relaxed);
    s_uvc_last_commit_payload.store(0, std::memory_order_relaxed);
    s_active_profile_is_mjpeg_sw.store(false, std::memory_order_relaxed);
    s_mjpeg_sw_capture_hold.store(false, std::memory_order_relaxed);
    update_usb_device_state(false, false, false);
}

void usb_device_task(void *arg)
{
    /* Service TinyUSB and push frames at the interval selected by the active
     * UVC profile. The task can repeat the latest frame when capture is slower
     * than the negotiated USB frame interval.
     */
    auto *ctx = reinterpret_cast<tinyusb_uvc_context_t *>(arg);
    uint64_t last_frame_count = 0;
    uint64_t send_attempt_count = 0;
    uint64_t copy_fail_count = 0;
    uint64_t xfer_fail_count = 0;
    bool last_streaming = false;
    bool last_usb_mounted = false;
    bool last_usb_ready = false;
    bool last_usb_suspended = false;
    bool slot_locked_for_xfer = false;
    int64_t last_not_streaming_log_us = 0;
    int64_t last_streaming_wait_log_us = 0;
    int64_t last_send_fail_log_us = 0;
    int64_t last_real_delay_us = esp_timer_get_time();
    auto scheduler_pause = [&](bool active_streaming, int64_t now_us) {
        /* Avoid a busy loop while streaming, but still call tud_task_ext often
         * enough to keep isochronous transfers responsive.
         */
        if (!active_streaming || ((now_us - last_real_delay_us) >= USB_DEVICE_MAX_BUSY_US)) {
            last_real_delay_us = esp_timer_get_time();
            vTaskDelay(USB_DEVICE_REAL_DELAY_TICKS);
        } else {
            taskYIELD();
        }
    };

    while (true) {
        tud_task_ext(0, false);

        const int64_t now_us = esp_timer_get_time();
        const bool usb_mounted = tud_mounted();
        const bool usb_ready = tud_ready();
        const bool usb_suspended = tud_suspended();
        const bool usb_bus_ready = usb_mounted && usb_ready && !usb_suspended;
        update_usb_device_state(usb_mounted, usb_ready, usb_suspended);

        if (slot_locked_for_xfer && !s_uvc_tx_busy.load(std::memory_order_acquire)) {
            slot_locked_for_xfer = false;
        }

        const uint64_t frame_count = s_frame_count.load(std::memory_order_relaxed);
        if (usb_mounted != last_usb_mounted
            || usb_ready != last_usb_ready
            || usb_suspended != last_usb_suspended) {
            ESP_LOGW(TAG,
                     "USB device state changed: mounted=%d ready=%d suspended=%d usable=%d fc=%llu xfer=%llu/%llu",
                     (int)usb_mounted,
                     (int)usb_ready,
                     (int)usb_suspended,
                     (int)usb_bus_ready,
                     (unsigned long long)frame_count,
                     (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                     (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed));
            last_usb_mounted = usb_mounted;
            last_usb_ready = usb_ready;
            last_usb_suspended = usb_suspended;
        }

        if (!usb_bus_ready) {
            clear_uvc_transfer_state();
            slot_locked_for_xfer = false;
            if (last_streaming) {
                ESP_LOGW(TAG,
                         "UVC streaming state changed: 1 -> 0 mounted=%d ready=%d suspended=%d fc=%llu lfc=%llu commit=%u/%u intv=%u payload=%u xfer=%llu/%llu",
                         (int)usb_mounted,
                         (int)usb_ready,
                         (int)usb_suspended,
                         (unsigned long long)frame_count,
                         (unsigned long long)last_frame_count,
                         (unsigned)s_uvc_last_commit_format.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_frame.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_interval_us.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_payload.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed));
                last_streaming = false;
            }
            last_frame_count = frame_count;
            if ((now_us - last_not_streaming_log_us) >= 5000000) {
                ESP_LOGW(TAG,
                         "USB inactive: mounted=%d ready=%d suspended=%d fc=%llu commit=%u/%u intv=%u payload=%u xfer=%llu/%llu attempts=%llu",
                         (int)usb_mounted,
                         (int)usb_ready,
                         (int)usb_suspended,
                         (unsigned long long)frame_count,
                         (unsigned)s_uvc_last_commit_format.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_frame.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_interval_us.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_payload.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed),
                         (unsigned long long)send_attempt_count);
                last_not_streaming_log_us = now_us;
            }
            scheduler_pause(false, now_us);
            continue;
        }

        if (s_uvc_reconfig_in_progress.load(std::memory_order_acquire)) {
            clear_uvc_transfer_state();
            if (slot_locked_for_xfer) {
                slot_locked_for_xfer = false;
            }
            if ((now_us - last_streaming_wait_log_us) >= 1000000) {
                ESP_LOGW(TAG, "UVC reconfig in progress: active=%d pending=%d xfer=%llu/%llu",
                         s_uvc_active_profile_index.load(std::memory_order_relaxed),
                         s_uvc_pending_profile_index.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed));
                last_streaming_wait_log_us = now_us;
            }
            scheduler_pause(false, now_us);
            continue;
        }

        const bool streaming = tud_video_n_streaming(0, 0);
        s_uvc_streaming_active.store(streaming, std::memory_order_release);
        if (streaming != last_streaming) {
            ESP_LOGW(TAG,
                     "UVC streaming state changed: %d -> %d mounted=%d ready=%d suspended=%d fc=%llu lfc=%llu commit=%u/%u intv=%u payload=%u xfer=%llu/%llu",
                     (int)last_streaming,
                     (int)streaming,
                     (int)tud_mounted(),
                     (int)tud_ready(),
                     (int)tud_suspended(),
                     (unsigned long long)frame_count,
                     (unsigned long long)last_frame_count,
                     (unsigned)s_uvc_last_commit_format.load(std::memory_order_relaxed),
                     (unsigned)s_uvc_last_commit_frame.load(std::memory_order_relaxed),
                     (unsigned)s_uvc_last_commit_interval_us.load(std::memory_order_relaxed),
                     (unsigned)s_uvc_last_commit_payload.load(std::memory_order_relaxed),
                     (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                     (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed));
            if (streaming) {
                last_frame_count = 0;
                s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
                last_streaming_wait_log_us = 0;
            }
            last_streaming = streaming;
        }

        if (streaming) {
            const int64_t last_frame_us = s_uvc_last_frame_us.load(std::memory_order_relaxed);
            const bool cond_not_busy = !s_uvc_tx_busy.load(std::memory_order_acquire);
            const bool cond_new_frame = (frame_count != last_frame_count);
            const bool cond_interval  = (last_frame_us == 0
                || (now_us - last_frame_us) >= s_uvc_frame_interval_us.load(std::memory_order_relaxed));
            const bool cond_repeat_frame = (!cond_new_frame && last_frame_us != 0 && cond_interval);
            if (cond_not_busy && (cond_new_frame || cond_repeat_frame) && cond_interval) {
                uint8_t *frame_buffer = nullptr;
                size_t frame_len = 0;
                bool lock_for_xfer = false;
                uvc_copy_result_t copy_result = uvc_copy_result_t::ok;
                ++send_attempt_count;
                s_uvc_send_attempt_count.store(send_attempt_count, std::memory_order_relaxed);
                const bool copy_ok = prepare_latest_frame_for_uvc_xfer(*ctx, &frame_buffer, &frame_len, &lock_for_xfer, &copy_result);
                const bool xfer_ok = copy_ok && tud_video_n_frame_xfer(0, 0, frame_buffer, frame_len);
                if (xfer_ok) {
                    s_uvc_tx_busy.store(true, std::memory_order_release);
                    slot_locked_for_xfer = lock_for_xfer;
                    const uint64_t started = s_uvc_xfer_started_count.fetch_add(1, std::memory_order_relaxed) + 1;
                    s_uvc_last_frame_us.store(now_us, std::memory_order_relaxed);
                    last_frame_count = frame_count;
                    if (cond_repeat_frame) {
                        s_uvc_repeat_send_count.fetch_add(1, std::memory_order_relaxed);
                    }
                    ESP_LOGD(TAG,
                             "uvc_send #%llu len=%u fc=%llu repeat=%d",
                             (unsigned long long)started,
                             (unsigned)frame_len,
                             (unsigned long long)frame_count,
                             (int)cond_repeat_frame);
                } else {
                    if (!copy_ok) {
                        ++copy_fail_count;
                    } else {
                        ++xfer_fail_count;
                        if (lock_for_xfer) {
                            s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_release);
                        }
                    }
                    if ((now_us - last_send_fail_log_us) >= 1000000 || copy_fail_count <= 3) {
                        ESP_LOGW(TAG,
                                 "uvc_send fail: copy=%d reason=%s xfer=%d frame_len=%u busy=%d streaming=%d fc=%llu lfc=%llu latest_slot=%d last_size=%u attempts=%llu copy_fail=%llu xfer_fail=%llu xfer=%llu/%llu invalid=%llu buf_miss=%llu",
                                 (int)copy_ok,
                                 uvc_copy_result_name(copy_result),
                                 (int)xfer_ok,
                                 (unsigned)frame_len,
                                 (int)s_uvc_tx_busy.load(std::memory_order_relaxed),
                                 (int)tud_video_n_streaming(0, 0),
                                 (unsigned long long)frame_count,
                                 (unsigned long long)last_frame_count,
                                 s_frame_ctx.latest_slot.load(std::memory_order_relaxed),
                                 (unsigned)s_last_frame_size.load(std::memory_order_relaxed),
                                 (unsigned long long)send_attempt_count,
                                 (unsigned long long)copy_fail_count,
                                 (unsigned long long)xfer_fail_count,
                                 (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                                 (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed),
                                 (unsigned long long)s_invalid_frame_count.load(std::memory_order_relaxed),
                                 (unsigned long long)s_capture_buffer_miss_count.load(std::memory_order_relaxed));
                        last_send_fail_log_us = now_us;
                    }
                    s_uvc_last_frame_us.store(now_us, std::memory_order_relaxed);
                }
            } else {
                if ((now_us - last_streaming_wait_log_us) >= 5000000) {
                    ESP_LOGD(TAG,
                             "uvc wait: busy=%d new=%d repeat=%d intv=%d mnt=%d rdy=%d fc=%llu lfc=%llu xfer=%llu/%llu attempts=%llu",
                             (int)!cond_not_busy,
                             (int)cond_new_frame,
                             (int)cond_repeat_frame,
                             (int)cond_interval,
                             (int)tud_mounted(),
                             (int)tud_ready(),
                             (unsigned long long)frame_count,
                             (unsigned long long)last_frame_count,
                             (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                             (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed),
                             (unsigned long long)send_attempt_count);
                    last_streaming_wait_log_us = now_us;
                }
            }
        } else {
            clear_uvc_transfer_state();
            if (slot_locked_for_xfer) {
                slot_locked_for_xfer = false;
            }
            last_frame_count = frame_count;
            if ((now_us - last_not_streaming_log_us) >= 2000000) {
                ESP_LOGW(TAG,
                         "tud_video_n_streaming=false mounted=%d ready=%d suspended=%d fc=%llu commit=%u/%u intv=%u payload=%u xfer=%llu/%llu attempts=%llu",
                         (int)tud_mounted(),
                         (int)tud_ready(),
                         (int)tud_suspended(),
                         (unsigned long long)frame_count,
                         (unsigned)s_uvc_last_commit_format.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_frame.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_interval_us.load(std::memory_order_relaxed),
                         (unsigned)s_uvc_last_commit_payload.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed),
                         (unsigned long long)send_attempt_count);
                last_not_streaming_log_us = now_us;
            }
        }

        scheduler_pause(streaming, now_us);
    }
}

extern "C" void tud_mount_cb(void)
{
    update_usb_device_state(tud_mounted(), tud_ready(), tud_suspended());
    ESP_LOGI(TAG, "TinyUSB device mounted");
}

extern "C" void tud_umount_cb(void)
{
    clear_uvc_transfer_state();
    update_usb_device_state(false, false, false);
    ESP_LOGI(TAG, "TinyUSB device unmounted xfer=%llu/%llu",
             (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
             (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed));
}

extern "C" void tud_suspend_cb(bool remote_wakeup_en)
{
    clear_uvc_transfer_state();
    update_usb_device_state(tud_mounted(), false, true);
    ESP_LOGW(TAG, "TinyUSB device suspended remote_wakeup=%d xfer=%llu/%llu",
             (int)remote_wakeup_en,
             (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
             (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed));
}

extern "C" void tud_resume_cb(void)
{
    update_usb_device_state(tud_mounted(), tud_ready(), false);
    ESP_LOGI(TAG, "TinyUSB device resumed xfer=%llu/%llu",
             (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
             (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed));
}

extern "C" void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx)
{
    const uint64_t completed = s_uvc_xfer_completed_count.fetch_add(1, std::memory_order_relaxed) + 1;
    s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_release);
    s_uvc_tx_busy.store(false, std::memory_order_release);
    ESP_LOGD(TAG,
             "uvc_xfer done #%llu: ctl=%u stm=%u started=%llu streaming=%d",
             (unsigned long long)completed,
             (unsigned)ctl_idx,
             (unsigned)stm_idx,
             (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
             (int)s_uvc_streaming_active.load(std::memory_order_relaxed));
}

extern "C" int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx,
                                     video_probe_and_commit_control_t const *parameters)
{
    (void) ctl_idx;
    (void) stm_idx;
    if (parameters == nullptr) {
        return VIDEO_ERROR_INVALID_REQUEST;
    }

    if (parameters->bFormatIndex == 0 || parameters->bFrameIndex == 0) {
        ESP_LOGW(TAG, "UVC commit rejected: unresolved fmt=%u frame=%u",
                 parameters->bFormatIndex,
                 parameters->bFrameIndex);
        return VIDEO_ERROR_INVALID_REQUEST;
    }

    const int profile_index = find_uvc_profile_binding_index(parameters->bFormatIndex, parameters->bFrameIndex);
    if (profile_index < 0 || !s_uvc_profile_bindings[profile_index].available) {
        ESP_LOGW(TAG, "UVC commit rejected: fmt=%u frame=%u unsupported",
                 parameters->bFormatIndex,
                 parameters->bFrameIndex);
        return VIDEO_ERROR_INVALID_REQUEST;
    }

    const uvc_profile_binding_t &binding = s_uvc_profile_bindings[profile_index];

    int64_t interval_us = static_cast<int64_t>(parameters->dwFrameInterval) / 10;
    if (interval_us < binding.profile->frame_interval_us) {
        interval_us = binding.profile->frame_interval_us;
    }
    s_uvc_frame_interval_us.store(interval_us, std::memory_order_relaxed);
    s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
    s_uvc_tx_busy.store(false, std::memory_order_release);
    s_uvc_last_commit_format.store(parameters->bFormatIndex, std::memory_order_relaxed);
    s_uvc_last_commit_frame.store(parameters->bFrameIndex, std::memory_order_relaxed);
    s_uvc_last_commit_interval_us.store(static_cast<uint32_t>(interval_us), std::memory_order_relaxed);
    s_uvc_last_commit_payload.store(parameters->dwMaxPayloadTransferSize, std::memory_order_relaxed);

    const int active_profile_index = s_uvc_active_profile_index.load(std::memory_order_acquire);
    const bool switch_pending = profile_index != active_profile_index;
    if (switch_pending) {
        /* Do not rebuild the DVP pipeline inside the USB control callback.
         * camera_task consumes this pending profile and performs the switch.
         */
        s_uvc_reconfig_in_progress.store(true, std::memory_order_release);
        s_uvc_pending_profile_index.store(profile_index, std::memory_order_release);
    }

    ESP_LOGI(TAG, "UVC commit: host fmt=%u frame=%u -> mode[%d] %ux%u interval=%lu us payload=%lu%s",
             parameters->bFormatIndex,
             parameters->bFrameIndex,
             profile_index,
             binding.profile->width,
             binding.profile->height,
             static_cast<unsigned long>(interval_us),
             static_cast<unsigned long>(parameters->dwMaxPayloadTransferSize),
             switch_pending ? " (switch pending)" : " (same active mode)");
    /* Host 打开默认 mode 时也会提交 same active mode；这里仅做信息日志。
     * 如果用户在 host app 中切换分辨率但仍反复看到同一个 fmt/frame，
     * 通常说明 host 没有推送新的格式/帧索引。 */
    if (!switch_pending && active_profile_index >= 0) {
        ESP_LOGI(TAG,
                 "UVC commit: same active mode fmt=%u frame=%u %ux%u; "
                 "no DVP reconfigure needed",
                 parameters->bFormatIndex,
                 parameters->bFrameIndex,
                 binding.profile->width,
                 binding.profile->height);
    }
    return VIDEO_ERROR_NONE;
}

/* 传感器原生可调参数探测与打印。
 * 遍历全部已知 CID，调用 query_para_desc 查询描述符，成功则说明该参数被驱动支持；
 * 失败则跳过，不对设备寄存器做任何写操作。 */
