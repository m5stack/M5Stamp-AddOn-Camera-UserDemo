/*
  M5Stamp-S3Bat CameraWebServer
  Porting of espressif/arduino-esp32 example to M5Stamp-S3Bat
  See also https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer

  GC0308 and OV3660 are detected at runtime.

  Author GOB https://twitter.com/gob_52_gob (*^○^*)
*/
#include <esp_camera.h>
#include <esp_log.h>
#include <WiFi.h>
#include <gob_GC0308.hpp>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "camera_api.h"

#define CAMERA_WIFI_SSID     "YOUR_WIFI_SSID"
#define CAMERA_WIFI_PASSWORD "YOUR_WIFI_PASSWORD"

// Stamp Add-On Camera bus from M5Stamp-AddOn-Camera-UserDemo/main/pinmap.h.
#define CAM_PIN_PWDN  39
#define CAM_PIN_RESET 41
#define CAM_PIN_XCLK  46
#define CAM_PIN_SIOD  48
#define CAM_PIN_SIOC  47
#define CAM_PIN_D7    40
#define CAM_PIN_D6    38
#define CAM_PIN_D5    12
#define CAM_PIN_D4    14
#define CAM_PIN_D3    15
#define CAM_PIN_D2    16
#define CAM_PIN_D1    18
#define CAM_PIN_D0    21
#define CAM_PIN_VSYNC 42
#define CAM_PIN_HREF  17
#define CAM_PIN_PCLK  13

extern void startCameraServer();  // app_httpd.cpp

static constexpr uint32_t kWifiConnectTimeoutMs = 20000;

static camera_config_t camera_config = {
    .pin_pwdn     = CAM_PIN_PWDN,
    .pin_reset    = CAM_PIN_RESET,
    .pin_xclk     = CAM_PIN_XCLK,
    .pin_sscb_sda = CAM_PIN_SIOD,
    .pin_sscb_scl = CAM_PIN_SIOC,
    .pin_d7       = CAM_PIN_D7,
    .pin_d6       = CAM_PIN_D6,
    .pin_d5       = CAM_PIN_D5,
    .pin_d4       = CAM_PIN_D4,
    .pin_d3       = CAM_PIN_D3,
    .pin_d2       = CAM_PIN_D2,
    .pin_d1       = CAM_PIN_D1,
    .pin_d0       = CAM_PIN_D0,
    .pin_vsync    = CAM_PIN_VSYNC,
    .pin_href     = CAM_PIN_HREF,
    .pin_pclk     = CAM_PIN_PCLK,
    .xclk_freq_hz = 20000000,
    .ledc_timer   = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    .pixel_format = PIXFORMAT_RGB565,
    // RGB565 capture must use the same size as the sensor output.
    .frame_size    = FRAMESIZE_QVGA,
    .jpeg_quality  = 0,
    .fb_count      = 2,
    .fb_location   = CAMERA_FB_IN_PSRAM,
    .grab_mode     = CAMERA_GRAB_WHEN_EMPTY,
    .sccb_i2c_port = -1,
};

static SemaphoreHandle_t s_camera_mutex = nullptr;

void camera_frame_lock(void)
{
    if (s_camera_mutex) {
        xSemaphoreTake(s_camera_mutex, portMAX_DELAY);
    }
}

void camera_frame_unlock(void)
{
    if (s_camera_mutex) {
        xSemaphoreGive(s_camera_mutex);
    }
}

static void flush_camera_frames(void)
{
    for (int i = 0; i < 4; ++i) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb) {
            break;
        }
        esp_camera_fb_return(fb);
    }
}

static void tune_ov3660(sensor_t *sensor)
{
    sensor->set_hmirror(sensor, 1);
    sensor->set_vflip(sensor, 1);
    sensor->set_brightness(sensor, 1);
    sensor->set_saturation(sensor, -2);
}

static int ov3660_jpeg_quality(framesize_t frame_size)
{
    if (frame_size >= FRAMESIZE_HD) {
        return 18;
    }
    if (frame_size >= FRAMESIZE_SVGA) {
        return 16;
    }
    if (frame_size >= FRAMESIZE_VGA) {
        return 14;
    }
    return 12;
}

static bool restart_gc0308(framesize_t frame_size)
{
    camera_config.frame_size = frame_size;
    const esp_err_t err      = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("camera", "GC0308 init framesize=%d failed: %d", (int)frame_size, err);
        return false;
    }
    if (!goblib::camera::GC0308::complementDriver()) {
        esp_camera_deinit();
        return false;
    }
    return true;
}

bool camera_reconfigure(framesize_t frame_size)
{
    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor || (sensor->id.PID != GC0308_PID && sensor->id.PID != OV3660_PID)) {
        return false;
    }
    const uint16_t sensor_pid        = sensor->id.PID;
    const framesize_t max_frame_size = camera_max_frame_size();
    if (frame_size < FRAMESIZE_96X96 || frame_size > max_frame_size) {
        return false;
    }

    camera_frame_lock();
    sensor = esp_camera_sensor_get();
    if (!sensor || sensor->id.PID != sensor_pid) {
        camera_frame_unlock();
        return false;
    }

    if (sensor_pid == GC0308_PID) {
        const framesize_t previous_size = camera_config.frame_size;
        if (frame_size == previous_size) {
            camera_frame_unlock();
            return true;
        }
        // Hold the frame mutex until all capture buffers and driver metadata
        // have been recreated for the new RGB565 dimensions.
        if (esp_camera_deinit() != ESP_OK) {
            camera_frame_unlock();
            return false;
        }
        if (!restart_gc0308(frame_size)) {
            if (!restart_gc0308(previous_size)) {
                ESP_LOGE("camera", "GC0308 recovery failed");
                // Capture consumers must not resume with an absent driver.
                abort();
            }
            camera_frame_unlock();
            return false;
        }
    } else if (!sensor->set_framesize || sensor->set_framesize(sensor, frame_size) != 0) {
        ESP_LOGE("camera", "Sensor 0x%04x set framesize=%d failed", sensor_pid, (int)frame_size);
        camera_frame_unlock();
        return false;
    }
    if (sensor_pid == OV3660_PID) {
        sensor->set_quality(sensor, ov3660_jpeg_quality(frame_size));
    }
    camera_config.frame_size = frame_size;
    delay(50);
    flush_camera_frames();
    ESP_LOGI("camera", "Sensor 0x%04x reconfigured: framesize=%d format=%d", sensor_pid, (int)frame_size,
             (int)camera_config.pixel_format);
    camera_frame_unlock();
    return true;
}

framesize_t camera_max_frame_size()
{
    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        return FRAMESIZE_INVALID;
    }
    camera_sensor_info_t *info = esp_camera_sensor_get_info(&sensor->id);
    if (info) {
        return info->max_size;
    }
    return FRAMESIZE_VGA;
}

static bool init_camera(void)
{
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("setup", "Failed to init camera: %d", err);
        return false;
    }

    sensor_t *sensor = esp_camera_sensor_get();
    if (!sensor) {
        ESP_LOGE("setup", "Camera sensor not found after init");
        esp_camera_deinit();
        return false;
    }

    if (sensor->id.PID == GC0308_PID) {
        if (!goblib::camera::GC0308::complementDriver()) {
            ESP_LOGE("setup", "Failed to complement GC0308");
            esp_camera_deinit();
            return false;
        }
        ESP_LOGI("setup", "GC0308 detected (PID 0x%04x), RGB565 QVGA", sensor->id.PID);
        return true;
    }

    if (sensor->id.PID != OV3660_PID) {
        ESP_LOGE("setup", "Unsupported camera sensor PID 0x%04x", sensor->id.PID);
        esp_camera_deinit();
        return false;
    }

    ESP_LOGI("setup", "OV3660 detected (PID 0x%04x), switching to JPEG QVGA", sensor->id.PID);
    esp_camera_deinit();
    camera_config.pixel_format = PIXFORMAT_JPEG;
    camera_config.frame_size   = FRAMESIZE_QXGA;
    camera_config.jpeg_quality = ov3660_jpeg_quality(FRAMESIZE_QXGA);
    camera_config.grab_mode    = CAMERA_GRAB_LATEST;

    err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("setup", "OV3660 JPEG re-init failed: %d", err);
        return false;
    }

    sensor = esp_camera_sensor_get();
    if (!sensor || sensor->id.PID != OV3660_PID) {
        ESP_LOGE("setup", "OV3660 missing after JPEG re-init");
        esp_camera_deinit();
        return false;
    }

    tune_ov3660(sensor);
    if (sensor->set_framesize(sensor, FRAMESIZE_QVGA) != 0) {
        ESP_LOGE("setup", "Failed to set OV3660 default QVGA");
        esp_camera_deinit();
        return false;
    }
    sensor->set_quality(sensor, ov3660_jpeg_quality(FRAMESIZE_QVGA));
    camera_config.frame_size = FRAMESIZE_QVGA;
    delay(150);
    flush_camera_frames();
    ESP_LOGI("setup", "OV3660 ready, JPEG QVGA (QXGA buffers)");
    return true;
}

static bool connect_wifi(void)
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false);
    WiFi.setAutoReconnect(true);
    WiFi.begin(CAMERA_WIFI_SSID, CAMERA_WIFI_PASSWORD);

    ESP_LOGI("setup", "Connecting to Wi-Fi SSID \"%s\"", CAMERA_WIFI_SSID);
    const uint32_t start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < kWifiConnectTimeoutMs) {
        delay(250);
    }
    if (WiFi.status() != WL_CONNECTED) {
        ESP_LOGE("setup", "Wi-Fi connection failed, status=%d", (int)WiFi.status());
        return false;
    }

    ESP_LOGI("setup", "Wi-Fi connected, IP address: %s", WiFi.localIP().toString().c_str());
    return true;
}

void setup()
{
    s_camera_mutex = xSemaphoreCreateMutex();
    if (!s_camera_mutex) {
        ESP_LOGE("setup", "Failed to create camera mutex");
        abort();
    }

    if (!init_camera()) {
        delay(1000 * 10);
        abort();
    }

    if (!connect_wifi()) {
        delay(1000);
        abort();
    }

    // Server
    startCameraServer();
    ESP_LOGI("setup", "Open http://%s/", WiFi.localIP().toString().c_str());
}

void loop()
{
    camera_frame_lock();
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        esp_camera_fb_return(fb);
    }
    camera_frame_unlock();
    delay(30);
}
