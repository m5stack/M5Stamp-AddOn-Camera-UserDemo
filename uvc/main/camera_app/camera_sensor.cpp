#include "camera_app_private.h"

/* Sensor bring-up helpers.
 *
 * The board can be populated with different DVP sensors. These helpers keep
 * SCCB probing, device construction, and sensor capability logging separate
 * from the main camera task.
 */

esp_err_t prime_camera_for_probe(void)
{
    gpio_config_t conf{};

    if (OV3660_PWDN_PIN >= 0) {
        conf.pin_bit_mask = 1ULL << OV3660_PWDN_PIN;
        conf.mode = GPIO_MODE_OUTPUT;
        ESP_RETURN_ON_ERROR(gpio_config(&conf), TAG, "config PWDN failed");
        /* Shared wake-up sequence used by the supported DVP sensors on this board:
         * drive PWDN high briefly, then release it low before SCCB probing. */
        ESP_RETURN_ON_ERROR(gpio_set_level(OV3660_PWDN_PIN, 1), TAG, "set PWDN high (power-down) failed");
        vTaskDelay(OV3660_POWER_DELAY_TICKS);
        ESP_RETURN_ON_ERROR(gpio_set_level(OV3660_PWDN_PIN, 0), TAG, "set PWDN low (active) failed");
        vTaskDelay(OV3660_POWER_DELAY_TICKS);
    }

    if (OV3660_RESET_PIN >= 0) {
        conf = {};
        conf.pin_bit_mask = 1ULL << OV3660_RESET_PIN;
        conf.mode = GPIO_MODE_OUTPUT;
        ESP_RETURN_ON_ERROR(gpio_config(&conf), TAG, "config RESET failed");
        ESP_RETURN_ON_ERROR(gpio_set_level(OV3660_RESET_PIN, 0), TAG, "set RESET low failed");
        vTaskDelay(OV3660_POWER_DELAY_TICKS);
        ESP_RETURN_ON_ERROR(gpio_set_level(OV3660_RESET_PIN, 1), TAG, "set RESET high failed");
        vTaskDelay(OV3660_POWER_DELAY_TICKS);
    }

    return ESP_OK;
}

esp_err_t start_camera_capture(esp_cam_sensor_device_t *sensor, esp_cam_ctlr_handle_t cam_ctlr)
{
    ESP_RETURN_ON_FALSE(sensor != nullptr, ESP_ERR_INVALID_ARG, TAG, "sensor is null");
    ESP_RETURN_ON_FALSE(cam_ctlr != nullptr, ESP_ERR_INVALID_ARG, TAG, "cam_ctlr is null");

    int stream = 1;
    esp_err_t err = esp_cam_ctlr_start(cam_ctlr);
    if (err != ESP_OK) {
        return err;
    }

    err = esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream);
    if (err != ESP_OK) {
        esp_cam_ctlr_stop(cam_ctlr);
        return err;
    }
    return ESP_OK;
}

esp_err_t create_camera_sccb_handle(i2c_master_bus_handle_t i2c_bus, uint8_t device_address, esp_sccb_io_handle_t *sccb_io)
{
    sccb_i2c_config_t sccb_i2c_config{};

    ESP_RETURN_ON_FALSE(sccb_io != nullptr, ESP_ERR_INVALID_ARG, TAG, "sccb_io is null");

    sccb_i2c_config.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    sccb_i2c_config.device_address = device_address;
    sccb_i2c_config.scl_speed_hz = OV3660_I2C_SPEED_HZ;

    return sccb_new_i2c_io(i2c_bus, &sccb_i2c_config, sccb_io);
}

bool probe_ov3660_sensor(i2c_master_bus_handle_t i2c_bus, uint8_t device_address)
{
#if CONFIG_CAMERA_OV3660
    esp_sccb_io_handle_t sccb_io = nullptr;
    uint8_t pid_h = 0;
    uint8_t pid_l = 0;
    const uint16_t expected_pid = 0x3660;
    if (create_camera_sccb_handle(i2c_bus, device_address, &sccb_io) != ESP_OK) {
        return false;
    }

    esp_err_t err = esp_sccb_transmit_receive_reg_a16v8(sccb_io, 0x300A, &pid_h);
    err |= esp_sccb_transmit_receive_reg_a16v8(sccb_io, 0x300B, &pid_l);
    esp_sccb_del_i2c_io(sccb_io);
    return err == ESP_OK && static_cast<uint16_t>((pid_h << 8) | pid_l) == expected_pid;
#else
    (void)i2c_bus;
    (void)device_address;
    return false;
#endif
}

bool probe_gc0308_sensor(i2c_master_bus_handle_t i2c_bus, uint8_t device_address)
{
#if CONFIG_CAMERA_GC0308
    esp_sccb_io_handle_t sccb_io = nullptr;
    uint8_t pid = 0;
    if (create_camera_sccb_handle(i2c_bus, device_address, &sccb_io) != ESP_OK) {
        return false;
    }

    esp_err_t err = esp_sccb_transmit_receive_reg_a8v8(sccb_io, 0x00, &pid);
    esp_sccb_del_i2c_io(sccb_io);
    return err == ESP_OK && pid == GC0308_PID;
#else
    (void)i2c_bus;
    (void)device_address;
    return false;
#endif
}

detected_sensor_t detect_camera_sensor(i2c_master_bus_handle_t i2c_bus)
{
    /* Probe explicit sensor IDs instead of relying on detect() side effects.
     * This lets us choose the correct SCCB address before creating the driver.
     */
#if CONFIG_CAMERA_OV3660
    if (probe_ov3660_sensor(i2c_bus, OV3660_SCCB_ADDR)) {
        return {detected_sensor_model_t::ov3660, OV3660_SCCB_ADDR, "OV3660"};
    }
#endif
#if CONFIG_CAMERA_GC0308
    if (probe_gc0308_sensor(i2c_bus, GC0308_SCCB_ADDR)) {
        return {detected_sensor_model_t::gc0308, GC0308_SCCB_ADDR, "GC0308"};
    }
#endif
    return {};
}

esp_cam_sensor_device_t *create_sensor_device(detected_sensor_model_t model, esp_cam_sensor_config_t *cam_cfg)
{
    switch (model) {
#if CONFIG_CAMERA_OV3660
    case detected_sensor_model_t::ov3660:
        return ov3660_detect(cam_cfg);
#endif
#if CONFIG_CAMERA_GC0308
    case detected_sensor_model_t::gc0308:
        return gc0308_detect(cam_cfg);
#endif
    case detected_sensor_model_t::none:
    default:
        return nullptr;
    }
}

void log_sensor_para_capabilities(esp_cam_sensor_device_t *sensor)
{
    if (!sensor || !sensor->ops || !sensor->ops->query_para_desc) {
        ESP_LOGW(TAG, "sensor para query not available");
        return;
    }

    struct cid_entry_t {
        uint32_t id;
        const char *name;
    };

    /* 已知 CID 列表：仅包含 esp_cam_sensor_types.h 中定义的参数 ID */
    const cid_entry_t known_cids[] = {
        { ESP_CAM_SENSOR_BRIGHTNESS,    "BRIGHTNESS"     },
        { ESP_CAM_SENSOR_CONTRAST,      "CONTRAST"       },
        { ESP_CAM_SENSOR_SATURATION,    "SATURATION"     },
        { ESP_CAM_SENSOR_HUE,           "HUE"            },
        { ESP_CAM_SENSOR_HMIRROR,       "HMIRROR"        },
        { ESP_CAM_SENSOR_VFLIP,         "VFLIP"          },
        { ESP_CAM_SENSOR_SHARPNESS,     "SHARPNESS"      },
        { ESP_CAM_SENSOR_JPEG_QUALITY,  "JPEG_QUALITY"   },
        { ESP_CAM_SENSOR_SPECIAL_EFFECT,"SPECIAL_EFFECT" },
        { ESP_CAM_SENSOR_SCENE,         "SCENE"          },
        { ESP_CAM_SENSOR_AWB,           "AWB"            },
        { ESP_CAM_SENSOR_WB,            "WB"             },
        { ESP_CAM_SENSOR_AGC,           "AGC"            },
        { ESP_CAM_SENSOR_GAIN,          "GAIN"           },
        { ESP_CAM_SENSOR_DGAIN,         "DGAIN"          },
        { ESP_CAM_SENSOR_ANGAIN,        "ANGAIN"         },
        { ESP_CAM_SENSOR_AE_LEVEL,      "AE_LEVEL"       },
        { ESP_CAM_SENSOR_3A_LOCK,       "3A_LOCK"        },
    };

    ESP_LOGI(TAG, "--- sensor %s native para capabilities ---",
             sensor->name ? sensor->name : "unknown");

    unsigned supported = 0;
    for (size_t i = 0; i < sizeof(known_cids) / sizeof(known_cids[0]); i++) {
        esp_cam_sensor_param_desc_t desc = {};
        desc.id = known_cids[i].id;
        esp_err_t ret = sensor->ops->query_para_desc(sensor, &desc);
        if (ret != ESP_OK) {
            continue;
        }
        ++supported;
        if (desc.type == ESP_CAM_SENSOR_PARAM_TYPE_NUMBER) {
            ESP_LOGI(TAG, "  [OK] %-16s  type=NUMBER  range=[%d, %d] step=%d default=%d",
                     known_cids[i].name,
                     (int)desc.number.minimum,
                     (int)desc.number.maximum,
                     (int)desc.number.step,
                     (int)desc.default_value);
        } else {
            ESP_LOGI(TAG, "  [OK] %-16s  type=%u  default=%d",
                     known_cids[i].name,
                     (unsigned)desc.type,
                     (int)desc.default_value);
        }
    }

    if (supported == 0) {
        ESP_LOGW(TAG, "  (none of the probed CIDs are supported by this driver)");
    } else {
        ESP_LOGI(TAG, "  %u/%u CIDs supported",
                 supported, (unsigned)(sizeof(known_cids) / sizeof(known_cids[0])));
    }
    ESP_LOGI(TAG, "-----------------------------------------");
}
