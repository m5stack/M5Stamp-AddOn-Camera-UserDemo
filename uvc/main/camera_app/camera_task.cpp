#include "camera_app_private.h"

/* Top-level camera application task.
 *
 * This task owns sensor lifetime, DVP controller lifetime, frame buffer
 * allocation, and host-requested UVC mode switches. USB callbacks only request
 * changes; this task performs them in a controlled FreeRTOS context.
 */

void camera_task(void *arg)
{
    (void)arg;
    esp_err_t err = ESP_OK;
    uint64_t last_count = 0;
    uint64_t last_xfer_started = 0;
    uint64_t last_xfer_completed = 0;
    uint64_t last_repeat_count = 0;
    int64_t last_report_us = 0;
    int64_t last_stall_report_us = 0;
    int64_t no_frame_since_us = 0;
    uint32_t stall_recover_count = 0;
    constexpr int64_t CAMERA_STALL_LOG_INTERVAL_US = 2000000;
    constexpr int64_t CAMERA_STALL_RECOVER_US = 3000000;

    i2c_master_bus_handle_t i2c_bus = nullptr;
    esp_sccb_io_handle_t sccb_io = nullptr;
    esp_cam_sensor_device_t *sensor = nullptr;
    esp_cam_ctlr_handle_t cam_ctlr = nullptr;
    esp_cam_sensor_xclk_handle_t xclk_handle = nullptr;
    bool uvc_started = false;
    bool uvc_supported = false;
    bool capture_running = false;
    detected_sensor_t detected_sensor{};
    int active_uvc_profile_index = -1;
    int64_t usb_inactive_since_us = 0;

    i2c_master_bus_config_t i2c_bus_config{};
    esp_cam_sensor_config_t cam_cfg{};
    esp_cam_sensor_format_array_t sensor_formats{};
    esp_cam_sensor_format_t sensor_fmt{};
    size_t active_frame_size = 0;
    size_t buffer_alloc_size = 0;
    uint8_t *frame_buffers[CAMERA_FRAME_SLOT_COUNT] = {};
    esp_cam_sensor_xclk_config_t xclk_cfg{};
    esp_cam_ctlr_dvp_pin_config_t dvp_pins{};
    uvc_descriptor_config_t descriptor_config{};
    size_t uvc_profile_count = 0;
    const esp_cam_sensor_format_t *initial_sensor_format = nullptr;

    ESP_LOGI(TAG, "Camera demo start");
    ESP_LOGI(TAG, "Will init I2C + SCCB + DVP, auto-detect OV3660/GC0308, and optionally start TinyUSB UVC");
    ESP_LOGI(TAG, "UVC policy: only_best_per_resolution=%d mjpeg_sw=%d mjpeg_sw_quality=%u startup_self_test=%d",
             (int)UVC_ONLY_BEST_FORMAT_PER_RESOLUTION,
             (int)UVC_ENABLE_MJPEG_SW_CONVERSION,
             (unsigned)mjpeg_sw_quality_value(),
             (int)UVC_ENABLE_STARTUP_SELF_TEST);

    i2c_bus_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_bus_config.i2c_port = OV3660_I2C_PORT;
    i2c_bus_config.scl_io_num = OV3660_I2C_SCL_PIN;
    i2c_bus_config.sda_io_num = OV3660_I2C_SDA_PIN;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = 1;

    err = i2c_new_master_bus(&i2c_bus_config, &i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "create I2C bus failed: %s", esp_err_to_name(err));
        goto cleanup_bus;
    }

    if (OV3660_XCLK_PIN < 0) {
        ESP_LOGE(TAG, "XCLK pin is not set in pinmap");
        err = ESP_ERR_INVALID_ARG;
        goto cleanup_bus;
    }

    err = esp_cam_sensor_xclk_allocate(ESP_CAM_SENSOR_XCLK_LEDC, &xclk_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "allocate XCLK failed: %s", esp_err_to_name(err));
        goto cleanup_bus;
    }

    xclk_cfg.ledc_cfg.timer = LEDC_TIMER_0;
    xclk_cfg.ledc_cfg.clk_cfg = LEDC_AUTO_CLK;
    xclk_cfg.ledc_cfg.channel = LEDC_CHANNEL_0;
    xclk_cfg.ledc_cfg.xclk_freq_hz = OV3660_XCLK_FREQ_HZ;
    xclk_cfg.ledc_cfg.xclk_pin = OV3660_XCLK_PIN;
    err = esp_cam_sensor_xclk_start(xclk_handle, &xclk_cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start XCLK failed: %s", esp_err_to_name(err));
        goto cleanup_xclk;
    }

    err = prime_camera_for_probe();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "prime camera for probe failed: %s", esp_err_to_name(err));
        goto cleanup_xclk;
    }

    cam_cfg.sccb_handle = sccb_io;
    cam_cfg.reset_pin = OV3660_RESET_PIN;
    cam_cfg.pwdn_pin = OV3660_PWDN_PIN;
    cam_cfg.xclk_pin = OV3660_XCLK_PIN;
    cam_cfg.xclk_freq_hz = OV3660_XCLK_FREQ_HZ;
    cam_cfg.sensor_port = ESP_CAM_SENSOR_DVP;

    detected_sensor = detect_camera_sensor(i2c_bus);
    if (detected_sensor.model == detected_sensor_model_t::none) {
        ESP_LOGE(TAG, "no supported DVP sensor detected on SCCB");
        err = ESP_ERR_NOT_FOUND;
        goto cleanup_sccb;
    }

    err = create_camera_sccb_handle(i2c_bus, detected_sensor.sccb_addr, &sccb_io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "create SCCB IO failed for %s at 0x%02X: %s",
                 detected_sensor.name, detected_sensor.sccb_addr, esp_err_to_name(err));
        goto cleanup_xclk;
    }
    ESP_LOGI(TAG, "Detected %s on SCCB address 0x%02X", detected_sensor.name, detected_sensor.sccb_addr);
    cam_cfg.sccb_handle = sccb_io;

    sensor = create_sensor_device(detected_sensor.model, &cam_cfg);
    if (!sensor) {
        ESP_LOGE(TAG, "%s detect failed", detected_sensor.name);
        err = ESP_FAIL;
        goto cleanup_sccb;
    }

    err = esp_cam_sensor_query_format(sensor, &sensor_formats);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "%s supports %u format(s):",
                 sensor->name ? sensor->name : detected_sensor.name,
                 (unsigned)sensor_formats.count);
        for (size_t fi = 0; fi < sensor_formats.count; ++fi) {
            const esp_cam_sensor_format_t &f = sensor_formats.format_array[fi];
            ESP_LOGI(TAG, "  [%u] %-50s  %4ux%-4u  fps=%u",
                     (unsigned)fi, f.name ? f.name : "?",
                     f.width, f.height, f.fps);
        }
    } else {
        ESP_LOGE(TAG, "query format failed: %s", esp_err_to_name(err));
        goto cleanup_sensor;
    }

    log_sensor_para_capabilities(sensor);

    uvc_ctrl_cache_init(sensor);
    uvc_ctrl_apply_descriptor_masks();

    /* Build descriptors from the actual sensor formats. This lets the same
     * firmware advertise OV3660 or GC0308 capabilities without static tables.
     */
    uvc_profile_count = populate_uvc_profile_bindings(sensor_formats, detected_sensor.model, &descriptor_config);
    uvc_supported = (uvc_profile_count > 0) && uvc_descriptors_set_config(&descriptor_config);
    /* 默认从 descriptor 配置的 default_format/frame 对应的 profile 开始捕获 */
    if (uvc_supported) {
        const int def_prof = find_uvc_profile_binding_index(
            descriptor_config.default_format_index, descriptor_config.default_frame_index);
        active_uvc_profile_index = (def_prof >= 0) ? def_prof : 0;
    } else {
        active_uvc_profile_index = -1;
    }

    if (uvc_profile_count > 0 && !uvc_supported) {
        ESP_LOGE(TAG, "build runtime UVC descriptors failed");
    }

    if (uvc_supported) {
        const uvc_profile_binding_t &active_binding = s_uvc_profile_bindings[active_uvc_profile_index];
        initial_sensor_format = active_binding.sensor_fmt;
    } else {
        initial_sensor_format = find_first_safe_uvc_sensor_format(sensor_formats, detected_sensor.model);
    }
    if (initial_sensor_format == nullptr) {
        ESP_LOGE(TAG, "%s has no DVP format available for capture",
                 sensor->name ? sensor->name : detected_sensor.name);
        err = ESP_ERR_NOT_SUPPORTED;
        goto cleanup_sensor;
    }
    err = esp_cam_sensor_set_format(sensor, initial_sensor_format);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set sensor format failed: %s", esp_err_to_name(err));
        goto cleanup_sensor;
    }

    err = esp_cam_sensor_get_format(sensor, &sensor_fmt);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "get format failed: %s", esp_err_to_name(err));
        goto cleanup_sensor;
    }

    ESP_LOGI(TAG, "sensor=%s format=%s %ux%u fps=%u xclk=%d",
             sensor->name ? sensor->name : "unknown",
             sensor_fmt.name ? sensor_fmt.name : "unknown",
             sensor_fmt.width, sensor_fmt.height, sensor_fmt.fps, sensor_fmt.xclk);

    if (!uvc_supported) {
        ESP_LOGW(TAG, "%s has no sensor format that can be exposed as runtime UVC descriptors. TinyUSB UVC will stay disabled.",
                 sensor->name ? sensor->name : detected_sensor.name);
    } else {
        ESP_LOGI(TAG, "UVC descriptor summary: formats=%u frames=%u default=%u/%u max_uvc_frame=%u bytes max_capture=%u bytes",
                 descriptor_config.format_count,
                 descriptor_config.frame_count,
                 descriptor_config.default_format_index,
                 descriptor_config.default_frame_index,
             static_cast<unsigned>(uvc_descriptors_get_max_frame_buffer_size()),
             static_cast<unsigned>(get_max_uvc_capture_buffer_size()));
        for (size_t i = 0; i < uvc_profile_count; ++i) {
            const uvc_profile_binding_t &binding = s_uvc_profile_bindings[i];
            if (!binding.available) {
                ESP_LOGW(TAG, "UVC frame index %u unavailable on %s",
                         static_cast<unsigned>(i + 1),
                         sensor->name ? sensor->name : detected_sensor.name);
                continue;
            }

            ESP_LOGI(TAG, "UVC mode[%u] host fmt=%u frame=%u ready: %s %s %ux%u sensor_fps=%u, USB output throttled to %u fps conversion=%s",
                     static_cast<unsigned>(i),
                     binding.profile->format_index,
                     binding.profile->frame_index,
                     uvc_profile_payload_name(binding),
                     binding.sensor_fmt->name ? binding.sensor_fmt->name : "unknown",
                     binding.sensor_fmt->width,
                     binding.sensor_fmt->height,
                     binding.sensor_fmt->fps,
                     binding.profile->usb_frame_rate,
                     uvc_frame_conversion_name(binding.conversion));
        }

        const uvc_profile_binding_t &active_binding = s_uvc_profile_bindings[active_uvc_profile_index];
        s_uvc_active_profile_index.store(active_uvc_profile_index, std::memory_order_relaxed);
        s_uvc_pending_profile_index.store(-1, std::memory_order_relaxed);
        s_uvc_reconfig_in_progress.store(false, std::memory_order_relaxed);
        s_uvc_frame_interval_us.store(active_binding.profile->frame_interval_us, std::memory_order_relaxed);
        s_uvc_camera_ctx.sensor_fmt = sensor_fmt;
        s_uvc_camera_ctx.conversion = active_binding.conversion;

        ESP_LOGI(TAG, "UVC default source selected: %s %s %ux%u sensor_fps=%u, USB output throttled to %u fps conversion=%s",
             uvc_profile_payload_name(active_binding),
                 sensor_fmt.name ? sensor_fmt.name : "unknown",
                 sensor_fmt.width,
                 sensor_fmt.height,
                 sensor_fmt.fps,
                 active_binding.profile->usb_frame_rate,
                 uvc_frame_conversion_name(active_binding.conversion));
    }

    active_frame_size = estimate_frame_buffer_size(sensor_fmt);
    if (active_frame_size == 0) {
        ESP_LOGE(TAG, "invalid frame size");
        err = ESP_ERR_INVALID_SIZE;
        goto cleanup_sensor;
    }

    buffer_alloc_size = uvc_supported ? get_max_uvc_capture_buffer_size() : active_frame_size;
    s_active_capture_buffer_limit.store(active_frame_size, std::memory_order_release);

    /* Capture slots are sized for the largest advertised capture requirement.
     * The active profile may use a smaller DMA window through
     * s_active_capture_buffer_limit.
     */
    ESP_LOGI(TAG, "frame buffer want=%u bytes, active capture=%u bytes, free internal=%u, free spiram=%u",
             static_cast<unsigned>(buffer_alloc_size),
             static_cast<unsigned>(active_frame_size),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)),
             static_cast<unsigned>(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));

    for (size_t i = 0; i < CAMERA_FRAME_SLOT_COUNT; ++i) {
        frame_buffers[i] = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, buffer_alloc_size,
                                            MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM));
        if (!frame_buffers[i]) {
            frame_buffers[i] = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, buffer_alloc_size,
                                                  MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
        }
        if (!frame_buffers[i]) {
            ESP_LOGE(TAG, "allocate frame buffer[%u] failed, size=%u", static_cast<unsigned>(i), static_cast<unsigned>(buffer_alloc_size));
            err = ESP_ERR_NO_MEM;
            goto cleanup_sensor;
        }
        s_frame_ctx.slots[i].buffer = frame_buffers[i];
        s_frame_ctx.slots[i].buffer_size = buffer_alloc_size;
        s_frame_ctx.slots[i].frame_size.store(0, std::memory_order_relaxed);
    }
    reset_frame_context();

    if (uvc_supported && any_uvc_profile_needs_transfer_buffer()) {
        /* Conversion paths cannot hand DMA memory directly to USB. They use one
         * shared transfer buffer for byte-swapped, YUY2-converted, or MJPEG(SW)
         * output.
         */
        const size_t uvc_transfer_alloc_size = align_up(uvc_descriptors_get_max_frame_buffer_size(), 64);
        s_uvc_transfer_buffer = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, uvc_transfer_alloc_size,
                                            MALLOC_CAP_8BIT | MALLOC_CAP_SPIRAM));
        if (!s_uvc_transfer_buffer) {
            s_uvc_transfer_buffer = static_cast<uint8_t *>(heap_caps_aligned_alloc(64, uvc_transfer_alloc_size,
                                                MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL));
        }
        if (!s_uvc_transfer_buffer) {
            ESP_LOGE(TAG, "allocate UVC transfer buffer failed, size=%u", static_cast<unsigned>(uvc_transfer_alloc_size));
            err = ESP_ERR_NO_MEM;
            goto cleanup_sensor;
        }
        s_uvc_transfer_buffer_size = uvc_transfer_alloc_size;
    } else {
        s_uvc_transfer_buffer = nullptr;
        s_uvc_transfer_buffer_size = 0;
    }

    dvp_pins.data_width = CAM_CTLR_DATA_WIDTH_8;
    dvp_pins.data_io[0] = OV3660_D0_PIN;
    dvp_pins.data_io[1] = OV3660_D1_PIN;
    dvp_pins.data_io[2] = OV3660_D2_PIN;
    dvp_pins.data_io[3] = OV3660_D3_PIN;
    dvp_pins.data_io[4] = OV3660_D4_PIN;
    dvp_pins.data_io[5] = OV3660_D5_PIN;
    dvp_pins.data_io[6] = OV3660_D6_PIN;
    dvp_pins.data_io[7] = OV3660_D7_PIN;
    dvp_pins.vsync_io = OV3660_VSYNC_PIN;
    dvp_pins.de_io = OV3660_DE_PIN;
    dvp_pins.pclk_io = OV3660_PCLK_PIN;
    dvp_pins.xclk_io = GPIO_NUM_NC;

    err = create_camera_controller(sensor_fmt, dvp_pins, &cam_ctlr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "create DVP controller failed: %s", esp_err_to_name(err));
        goto cleanup_xclk;
    }

    err = start_camera_capture(sensor, cam_ctlr);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "start sensor capture failed: %s", esp_err_to_name(err));
        goto cleanup_stream;
    }
    capture_running = true;

    if (uvc_supported) {
        /* 启动期是否先做全量 UVC mode 自检，由 UVC_ENABLE_STARTUP_SELF_TEST 控制。
         * 无论是否自检，都统一 reconfigure 到 descriptor 默认 mode，确保运行态一致。 */
        if (UVC_ENABLE_STARTUP_SELF_TEST) {
            ESP_LOGI(TAG, "startup self-test enabled; probing all UVC modes before TinyUSB starts");
            run_capture_self_test(sensor, &cam_ctlr, dvp_pins, &sensor_fmt);
        } else {
            ESP_LOGI(TAG, "startup self-test disabled; skip UVC mode probe before TinyUSB starts");
        }

        err = reconfigure_camera_pipeline(sensor, &cam_ctlr, dvp_pins, active_uvc_profile_index, &sensor_fmt);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "configure default UVC profile failed: %s", esp_err_to_name(err));
            goto cleanup_stream;
        }

        err = start_tinyusb_uvc(sensor_fmt, s_uvc_camera_ctx.conversion);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "init TinyUSB UVC device failed: %s", esp_err_to_name(err));
            goto cleanup_stream;
        }
        uvc_started = true;
        ESP_LOGI(TAG, "stream started, TinyUSB UVC advertising default mode[%d] host fmt=%u frame=%u %s %ux%u@%u",
                 active_uvc_profile_index,
                 s_uvc_profile_bindings[active_uvc_profile_index].profile->format_index,
                 s_uvc_profile_bindings[active_uvc_profile_index].profile->frame_index,
                 uvc_profile_payload_name(s_uvc_profile_bindings[active_uvc_profile_index]),
                 s_uvc_profile_bindings[active_uvc_profile_index].profile->width,
                 s_uvc_profile_bindings[active_uvc_profile_index].profile->height,
                 s_uvc_profile_bindings[active_uvc_profile_index].profile->usb_frame_rate);
    } else {
        ESP_LOGI(TAG, "stream started without UVC; capture remains active for local validation");
    }
    last_report_us = esp_timer_get_time();
    last_stall_report_us = last_report_us;
    last_count = s_frame_count.load(std::memory_order_relaxed);
    last_xfer_started = s_uvc_xfer_started_count.load(std::memory_order_relaxed);
    last_xfer_completed = s_uvc_xfer_completed_count.load(std::memory_order_relaxed);
    last_repeat_count = s_uvc_repeat_send_count.load(std::memory_order_relaxed);
    while (true) {
        vTaskDelay(CAMERA_TASK_LOOP_DELAY_TICKS);
        const int64_t now_us = esp_timer_get_time();

        if (uvc_started) {
            const bool usb_ready = usb_device_bus_ready();
            if (!usb_ready) {
                if (usb_inactive_since_us == 0) {
                    usb_inactive_since_us = now_us;
                }
                if (capture_running && (now_us - usb_inactive_since_us) >= USB_CAMERA_IDLE_STOP_DELAY_US) {
                    ESP_LOGW(TAG,
                             "USB inactive for %lld ms; pausing camera pipeline mounted=%d ready=%d suspended=%d streaming=%d pending=%d xfer=%llu/%llu",
                             (long long)((now_us - usb_inactive_since_us) / 1000),
                             (int)s_usb_device_mounted.load(std::memory_order_acquire),
                             (int)s_usb_device_ready.load(std::memory_order_acquire),
                             (int)s_usb_device_suspended.load(std::memory_order_acquire),
                             (int)s_uvc_streaming_active.load(std::memory_order_acquire),
                             s_uvc_pending_profile_index.load(std::memory_order_relaxed),
                             (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                             (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed));
                    s_uvc_tx_busy.store(false, std::memory_order_release);
                    s_uvc_streaming_active.store(false, std::memory_order_release);
                    s_uvc_reconfig_in_progress.store(true, std::memory_order_release);
                    s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
                    s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_release);
                    s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);
                    stop_camera_pipeline(sensor, &cam_ctlr);
                    reset_frame_context();
                    close_mjpeg_sw_encoder();
                    s_active_profile_is_mjpeg_sw.store(false, std::memory_order_release);
                    capture_running = false;
                }
                last_report_us = now_us;
                last_stall_report_us = now_us;
                no_frame_since_us = 0;
                last_count = s_frame_count.load(std::memory_order_relaxed);
                last_xfer_started = s_uvc_xfer_started_count.load(std::memory_order_relaxed);
                last_xfer_completed = s_uvc_xfer_completed_count.load(std::memory_order_relaxed);
                last_repeat_count = s_uvc_repeat_send_count.load(std::memory_order_relaxed);
                continue;
            }

            usb_inactive_since_us = 0;
            if (!capture_running) {
                int resume_profile_index = active_uvc_profile_index;
                const int pending_profile_index = s_uvc_pending_profile_index.exchange(-1, std::memory_order_acq_rel);
                if (pending_profile_index >= 0) {
                    resume_profile_index = pending_profile_index;
                }
                if (resume_profile_index < 0
                    || resume_profile_index >= static_cast<int>(UVC_FRAME_PROFILE_COUNT)
                    || !s_uvc_profile_bindings[resume_profile_index].available) {
                    ESP_LOGE(TAG, "cannot resume camera pipeline: invalid active UVC profile %d",
                             resume_profile_index);
                    err = ESP_ERR_INVALID_STATE;
                    goto cleanup_stream;
                }

                ESP_LOGI(TAG,
                         "USB active again; restarting camera pipeline mode[%d] mounted=%d ready=%d suspended=%d",
                         resume_profile_index,
                         (int)s_usb_device_mounted.load(std::memory_order_acquire),
                         (int)s_usb_device_ready.load(std::memory_order_acquire),
                         (int)s_usb_device_suspended.load(std::memory_order_acquire));
                err = reconfigure_camera_pipeline(sensor,
                                                 &cam_ctlr,
                                                 dvp_pins,
                                                 resume_profile_index,
                                                 &sensor_fmt);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "resume camera pipeline failed: %s", esp_err_to_name(err));
                    goto cleanup_stream;
                }

                active_uvc_profile_index = resume_profile_index;
                capture_running = true;
                ESP_LOGI(TAG, "camera pipeline resumed: format %u frame %u %s %s %ux%u sensor_fps=%u conversion=%s",
                         s_uvc_profile_bindings[active_uvc_profile_index].profile->format_index,
                         s_uvc_profile_bindings[active_uvc_profile_index].profile->frame_index,
                         uvc_profile_payload_name(s_uvc_profile_bindings[active_uvc_profile_index]),
                         sensor_fmt.name ? sensor_fmt.name : "unknown",
                         sensor_fmt.width,
                         sensor_fmt.height,
                         sensor_fmt.fps,
                         uvc_frame_conversion_name(s_uvc_profile_bindings[active_uvc_profile_index].conversion));
                last_report_us = esp_timer_get_time();
                last_stall_report_us = last_report_us;
                no_frame_since_us = 0;
                last_count = s_frame_count.load(std::memory_order_relaxed);
                last_xfer_started = s_uvc_xfer_started_count.load(std::memory_order_relaxed);
                last_xfer_completed = s_uvc_xfer_completed_count.load(std::memory_order_relaxed);
                last_repeat_count = s_uvc_repeat_send_count.load(std::memory_order_relaxed);
                continue;
            }

            /* TinyUSB commit callbacks only publish a pending profile index.
             * Reconfiguration is serialized here with capture stop/start.
             */
            const int pending_profile_index = s_uvc_pending_profile_index.exchange(-1, std::memory_order_acq_rel);
            if (pending_profile_index >= 0 && pending_profile_index != active_uvc_profile_index) {
                err = reconfigure_camera_pipeline(sensor,
                                                 &cam_ctlr,
                                                 dvp_pins,
                                                 pending_profile_index,
                                                 &sensor_fmt);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "switch UVC format %u frame %u failed: %s",
                             s_uvc_profile_bindings[pending_profile_index].profile->format_index,
                             s_uvc_profile_bindings[pending_profile_index].profile->frame_index,
                             esp_err_to_name(err));
                    goto cleanup_stream;
                }

                active_uvc_profile_index = pending_profile_index;
                ESP_LOGI(TAG, "UVC active source switched: format %u frame %u %s %s %ux%u sensor_fps=%u, USB output throttled to %u fps conversion=%s",
                         s_uvc_profile_bindings[active_uvc_profile_index].profile->format_index,
                         s_uvc_profile_bindings[active_uvc_profile_index].profile->frame_index,
                         uvc_profile_payload_name(s_uvc_profile_bindings[active_uvc_profile_index]),
                         sensor_fmt.name ? sensor_fmt.name : "unknown",
                         sensor_fmt.width,
                         sensor_fmt.height,
                         sensor_fmt.fps,
                         s_uvc_profile_bindings[active_uvc_profile_index].profile->usb_frame_rate,
                         uvc_frame_conversion_name(s_uvc_profile_bindings[active_uvc_profile_index].conversion));
                last_report_us = esp_timer_get_time();
                last_stall_report_us = last_report_us;
                no_frame_since_us = 0;
                last_count = s_frame_count.load(std::memory_order_relaxed);
                last_xfer_started = s_uvc_xfer_started_count.load(std::memory_order_relaxed);
                last_xfer_completed = s_uvc_xfer_completed_count.load(std::memory_order_relaxed);
                last_repeat_count = s_uvc_repeat_send_count.load(std::memory_order_relaxed);
                continue;
            }
        }

        const uint64_t cur_count = s_frame_count.load(std::memory_order_relaxed);
        if (cur_count == last_count) {
            if (no_frame_since_us == 0) {
                no_frame_since_us = now_us;
            }

            const bool recoverable_stall = uvc_started
                && capture_running
                && usb_device_bus_ready()
                && !s_uvc_reconfig_in_progress.load(std::memory_order_acquire)
                && active_uvc_profile_index >= 0
                && active_uvc_profile_index < static_cast<int>(UVC_FRAME_PROFILE_COUNT)
                && s_uvc_profile_bindings[active_uvc_profile_index].available;
            if (recoverable_stall && (now_us - no_frame_since_us) >= CAMERA_STALL_RECOVER_US) {
                ++stall_recover_count;
                ESP_LOGW(TAG,
                         "cam watchdog recovery #%lu: no frame for %lld ms; resetting pipeline mode[%d] mounted=%d ready=%d suspended=%d streaming=%d xfer=%llu/%llu attempts=%llu",
                         (unsigned long)stall_recover_count,
                         (long long)((now_us - no_frame_since_us) / 1000),
                         active_uvc_profile_index,
                         (int)s_usb_device_mounted.load(std::memory_order_acquire),
                         (int)s_usb_device_ready.load(std::memory_order_acquire),
                         (int)s_usb_device_suspended.load(std::memory_order_acquire),
                         (int)s_uvc_streaming_active.load(std::memory_order_acquire),
                         (unsigned long long)s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_xfer_completed_count.load(std::memory_order_relaxed),
                         (unsigned long long)s_uvc_send_attempt_count.load(std::memory_order_relaxed));

                s_uvc_tx_busy.store(false, std::memory_order_release);
                s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
                s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_release);
                s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);
                s_uvc_reconfig_in_progress.store(true, std::memory_order_release);
                stop_camera_pipeline(sensor, &cam_ctlr);
                capture_running = false;

                if (detected_sensor.model == detected_sensor_model_t::ov3660) {
                    const esp_err_t reset_err = esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_SW_RESET, nullptr);
                    if (reset_err != ESP_OK) {
                        ESP_LOGW(TAG, "OV3660 soft reset before recovery failed: %s", esp_err_to_name(reset_err));
                    }
                }

                err = reconfigure_camera_pipeline(sensor,
                                                 &cam_ctlr,
                                                 dvp_pins,
                                                 active_uvc_profile_index,
                                                 &sensor_fmt);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "camera watchdog recovery failed: %s", esp_err_to_name(err));
                    goto cleanup_stream;
                }

                capture_running = true;
                last_report_us = esp_timer_get_time();
                last_stall_report_us = last_report_us;
                no_frame_since_us = 0;
                last_count = s_frame_count.load(std::memory_order_relaxed);
                last_xfer_started = s_uvc_xfer_started_count.load(std::memory_order_relaxed);
                last_xfer_completed = s_uvc_xfer_completed_count.load(std::memory_order_relaxed);
                last_repeat_count = s_uvc_repeat_send_count.load(std::memory_order_relaxed);
                continue;
            }

            if ((now_us - last_stall_report_us) >= CAMERA_STALL_LOG_INTERVAL_US) {
                const size_t last_size = s_last_frame_size.load(std::memory_order_relaxed);
                const size_t dma_window = s_last_dma_window_size.load(std::memory_order_relaxed);
                const double actual_kb = static_cast<double>(last_size) / 1024.0;
                const double window_kb = static_cast<double>(dma_window) / 1024.0;
                const double used_pct = dma_window > 0 ? (static_cast<double>(last_size) * 100.0 / static_cast<double>(dma_window)) : 0.0;
                ESP_LOGW(TAG,
                         "cam stall total=%" PRIu64 " cap=%.1f/%.1fKB(%.1f%%) slot=%d uvc=%d busy=%d xfer=%" PRIu64 "/%" PRIu64 " attempts=%" PRIu64,
                         cur_count,
                         actual_kb,
                         window_kb,
                         used_pct,
                         s_last_frame_slot.load(std::memory_order_relaxed),
                         (int)s_uvc_streaming_active.load(std::memory_order_acquire),
                         (int)s_uvc_tx_busy.load(std::memory_order_acquire),
                         s_uvc_xfer_started_count.load(std::memory_order_relaxed),
                         s_uvc_xfer_completed_count.load(std::memory_order_relaxed),
                         s_uvc_send_attempt_count.load(std::memory_order_relaxed));
                last_stall_report_us = now_us;
            }
            continue;
        }

        const int64_t elapsed_us = now_us - last_report_us;
        const uint64_t delta = cur_count - last_count;
        const double seconds = elapsed_us > 0 ? (static_cast<double>(elapsed_us) / 1000000.0) : 0.0;
        const double fps = seconds > 0.0 ? (static_cast<double>(delta) / seconds) : 0.0;
        const size_t last_size = s_last_frame_size.load(std::memory_order_relaxed);
        const size_t dma_window = s_last_dma_window_size.load(std::memory_order_relaxed);
        const double actual_kb = static_cast<double>(last_size) / 1024.0;
        const double window_kb = static_cast<double>(dma_window) / 1024.0;
        const double used_pct = dma_window > 0 ? (static_cast<double>(last_size) * 100.0 / static_cast<double>(dma_window)) : 0.0;
        const uint64_t xfer_started = s_uvc_xfer_started_count.load(std::memory_order_relaxed);
        const uint64_t xfer_completed = s_uvc_xfer_completed_count.load(std::memory_order_relaxed);
        const uint64_t repeat_count = s_uvc_repeat_send_count.load(std::memory_order_relaxed);
        const uint64_t xfer_started_delta = xfer_started - last_xfer_started;
        const uint64_t xfer_completed_delta = xfer_completed - last_xfer_completed;
        const uint64_t repeat_delta = repeat_count - last_repeat_count;

        ESP_LOGI(TAG,
                 "cam frame total=%" PRIu64 " +%" PRIu64 " fps=%.2f cap=%.1f/%.1fKB(%.1f%%) slot=%d "
                 "uvc=%d busy=%d xfer=%" PRIu64 "/%" PRIu64 "(+%" PRIu64 "/+%" PRIu64 ") repeat=%" PRIu64 " attempts=%" PRIu64 " "
                 "commit=%u/%u intv=%u payload=%u active=%d pending=%d",
                 cur_count,
                 delta,
                 fps,
                 actual_kb,
                 window_kb,
                 used_pct,
                 s_last_frame_slot.load(std::memory_order_relaxed),
                 (int)s_uvc_streaming_active.load(std::memory_order_acquire),
                 (int)s_uvc_tx_busy.load(std::memory_order_acquire),
                 xfer_started,
                 xfer_completed,
                 xfer_started_delta,
                 xfer_completed_delta,
                 repeat_delta,
                 s_uvc_send_attempt_count.load(std::memory_order_relaxed),
                 (unsigned)s_uvc_last_commit_format.load(std::memory_order_relaxed),
                 (unsigned)s_uvc_last_commit_frame.load(std::memory_order_relaxed),
                 (unsigned)s_uvc_last_commit_interval_us.load(std::memory_order_relaxed),
                 (unsigned)s_uvc_last_commit_payload.load(std::memory_order_relaxed),
                 s_uvc_active_profile_index.load(std::memory_order_relaxed),
                 s_uvc_pending_profile_index.load(std::memory_order_relaxed));

        last_count = cur_count;
        last_xfer_started = xfer_started;
        last_xfer_completed = xfer_completed;
        last_repeat_count = repeat_count;
        last_report_us = now_us;
        last_stall_report_us = now_us;
        no_frame_since_us = 0;
    }

cleanup_stream:
    if (uvc_started) {
        stop_tinyusb_uvc();
        uvc_started = false;
    }
    stop_camera_pipeline(sensor, &cam_ctlr);
    capture_running = false;
    close_mjpeg_sw_encoder();
cleanup_xclk:
    if (xclk_handle != nullptr) {
        esp_cam_sensor_xclk_stop(xclk_handle);
        esp_cam_sensor_xclk_free(xclk_handle);
        xclk_handle = nullptr;
    }
    if (s_uvc_transfer_buffer != nullptr) {
        heap_caps_free(s_uvc_transfer_buffer);
        s_uvc_transfer_buffer = nullptr;
    }
    s_uvc_transfer_buffer_size = 0;
    for (size_t i = 0; i < CAMERA_FRAME_SLOT_COUNT; ++i) {
        if (frame_buffers[i] != nullptr) {
            heap_caps_free(frame_buffers[i]);
            frame_buffers[i] = nullptr;
        }
        s_frame_ctx.slots[i].buffer = nullptr;
        s_frame_ctx.slots[i].buffer_size = 0;
        s_frame_ctx.slots[i].frame_size.store(0, std::memory_order_relaxed);
    }
    uvc_ctrl_cache_clear();
    s_frame_ctx.latest_slot.store(-1, std::memory_order_relaxed);
    s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_relaxed);
cleanup_sensor:
    if (sensor != nullptr) {
        esp_cam_sensor_del_dev(sensor);
        sensor = nullptr;
    }
cleanup_sccb:
    if (sccb_io != nullptr) {
        esp_sccb_del_i2c_io(sccb_io);
        sccb_io = nullptr;
    }
cleanup_bus:
    if (i2c_bus != nullptr) {
        i2c_del_master_bus(i2c_bus);
        i2c_bus = nullptr;
    }

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "aborted, last err=%s", esp_err_to_name(err));
    }
    vTaskDelete(nullptr);
}
