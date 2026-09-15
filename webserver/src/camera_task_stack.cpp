#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// The Arduino camera driver is precompiled, so changing its sdkconfig macro
// in application build flags does not change the task's stack allocation.
extern "C" BaseType_t __real_xTaskCreatePinnedToCore(
    TaskFunction_t task, const char *name, uint32_t stack_depth,
    void *parameters, UBaseType_t priority, TaskHandle_t *handle,
    BaseType_t core_id);

extern "C" BaseType_t __wrap_xTaskCreatePinnedToCore(
    TaskFunction_t task, const char *name, uint32_t stack_depth,
    void *parameters, UBaseType_t priority, TaskHandle_t *handle,
    BaseType_t core_id)
{
    // ESP-IDF specifies stack depth in bytes. Preserve larger allocations.
    constexpr uint32_t kCameraTaskStackBytes = 4096;
    if (name && std::strcmp(name, "cam_task") == 0 &&
        stack_depth < kCameraTaskStackBytes) {
        stack_depth = kCameraTaskStackBytes;
    }
    return __real_xTaskCreatePinnedToCore(
        task, name, stack_depth, parameters, priority, handle, core_id);
}
