#include "esp_log.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "camera_app.h"

static const char *TAG = "cam_main";

constexpr UBaseType_t CAMERA_TASK_PRIORITY = 5;
constexpr BaseType_t CAMERA_TASK_CORE = 0;
constexpr uint32_t CAMERA_TASK_STACK_SIZE = 8 * 1024;

/* UVC 模式广告策略。
 * UVC_ONLY_BEST_FORMAT_PER_RESOLUTION=true：每个分辨率只保留一个最佳格式，
 * 避免 host 在同分辨率的 YUY2/UYVY/RGBP/GRAY8 之间自动选到非预期路径。 */
extern const bool UVC_ONLY_BEST_FORMAT_PER_RESOLUTION = true;

/* MJPEG(SW) 转换开关。
 * true：对非 JPEG 的 sensor 原始格式额外暴露 MJPEG(SW) profile；
 * false：只暴露当前原生/已有转换路径。 */
extern const bool UVC_ENABLE_MJPEG_SW_CONVERSION = true;
/* 默认质量取 80，优先保证 Full-Speed USB 下的帧大小和编码耗时可控。
 * 可通过 UVC XU.MJPEG_SW_QUALITY 在运行时调高画质。 */
extern const uint8_t UVC_MJPEG_SW_QUALITY_DEFAULT = 80;

/* 启动期自检预处理开关。
 * true：TinyUSB UVC 启动前，逐个探测所有已广告的 UVC mode；
 * false：跳过启动前全量自检，直接进入默认 UVC mode，缩短启动时间。
 * 建议：调试多格式兼容性时设为 true，日常/量产启动速度优先时设为 false。 */
extern const bool UVC_ENABLE_STARTUP_SELF_TEST = false;

static const char *reset_reason_name(esp_reset_reason_t reason)
{
    switch (reason) {
    case ESP_RST_POWERON:   return "POWERON";
    case ESP_RST_EXT:       return "EXT";
    case ESP_RST_SW:        return "SW";
    case ESP_RST_PANIC:     return "PANIC";
    case ESP_RST_INT_WDT:   return "INT_WDT";
    case ESP_RST_TASK_WDT:  return "TASK_WDT";
    case ESP_RST_WDT:       return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default:                return "UNKNOWN";
    }
}

extern "C" void app_main(void)
{
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    ESP_LOGI(TAG, "reset reason: %s(%d)", reset_reason_name(reset_reason), (int)reset_reason);
    ESP_LOGI(TAG, "starting camera task core=%d priority=%u", (int)CAMERA_TASK_CORE, (unsigned)CAMERA_TASK_PRIORITY);
    BaseType_t ok = xTaskCreatePinnedToCore(camera_task, "cam_task", CAMERA_TASK_STACK_SIZE,
                                            nullptr, CAMERA_TASK_PRIORITY, nullptr, CAMERA_TASK_CORE);
    if (ok != pdPASS) {
        ESP_LOGE(TAG, "create cam_task failed");
    }
}
