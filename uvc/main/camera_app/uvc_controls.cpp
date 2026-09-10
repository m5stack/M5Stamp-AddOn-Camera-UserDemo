#include "camera_app_private.h"

/* UVC Processing Unit / Extension Unit control bridge.
 *
 * Host control requests use UVC selector IDs, while esp_cam_sensor exposes
 * parameter CIDs. This file keeps the mapping table, cached ranges, and the
 * SET_CUR/GET_* request handling in one place.
 */

namespace {

constexpr size_t UVC_CTRL_SCRATCH_BYTES = 8;

struct uvc_ctrl_entry_t {
    uint8_t entity_id;
    uint8_t selector;
    uint8_t value_size;     /* 2 (PU int16) 或 4 (XU int32) */
    uint32_t cid;
    const char *name;
    bool mjpeg_sw_quality;
};

struct uvc_ctrl_state_t {
    bool supported = false; /* sensor 驱动 query_para_desc 成功 */
    int32_t v_min = 0;
    int32_t v_max = 0;
    int32_t v_step = 1;
    int32_t v_def = 0;
    int32_t v_cur = 0;
};

/* 与 usb_descriptors.h 中的 selector 宏一一对应 */
constexpr uvc_ctrl_entry_t kUvcCtrlMap[] = {
    { UVC_ENTITY_ID_PROCESSING_UNIT, UVC_PU_CTRL_CONTRAST,       2, ESP_CAM_SENSOR_CONTRAST,       "PU.CONTRAST",         false },
    { UVC_ENTITY_ID_PROCESSING_UNIT, UVC_PU_CTRL_SATURATION,     2, ESP_CAM_SENSOR_SATURATION,     "PU.SATURATION",       false },
    { UVC_ENTITY_ID_EXTENSION_UNIT,  UVC_XU_CTRL_HMIRROR,        4, ESP_CAM_SENSOR_HMIRROR,        "XU.HMIRROR",          false },
    { UVC_ENTITY_ID_EXTENSION_UNIT,  UVC_XU_CTRL_VFLIP,          4, ESP_CAM_SENSOR_VFLIP,          "XU.VFLIP",            false },
    { UVC_ENTITY_ID_EXTENSION_UNIT,  UVC_XU_CTRL_SPECIAL_EFFECT, 4, ESP_CAM_SENSOR_SPECIAL_EFFECT, "XU.EFFECT",           false },
    { UVC_ENTITY_ID_EXTENSION_UNIT,  UVC_XU_CTRL_AE_LEVEL,       4, ESP_CAM_SENSOR_AE_LEVEL,       "XU.AE_LEVEL",         false },
    { UVC_ENTITY_ID_EXTENSION_UNIT,  UVC_XU_CTRL_MJPEG_HW_QUALITY,4, ESP_CAM_SENSOR_JPEG_QUALITY,  "XU.MJPEG_HW_QUALITY", false },
    { UVC_ENTITY_ID_EXTENSION_UNIT,  UVC_XU_CTRL_WB_MODE,        4, ESP_CAM_SENSOR_WB,             "XU.WB",               false },
    { UVC_ENTITY_ID_EXTENSION_UNIT,  UVC_XU_CTRL_MJPEG_SW_QUALITY,4, 0,                            "XU.MJPEG_SW_QUALITY",  true  },
};
constexpr size_t kUvcCtrlCount = sizeof(kUvcCtrlMap) / sizeof(kUvcCtrlMap[0]);

esp_cam_sensor_device_t *s_active_sensor = nullptr;
uvc_ctrl_state_t s_ctrl_state[kUvcCtrlCount];
/* SETUP 阶段填好后挂在控制 EP 上的临时缓冲；DATA 阶段由它接收 host 数据。
 * 控制 EP 单线程处理，可静态共享一份。 */
uint8_t s_ctrl_scratch[UVC_CTRL_SCRATCH_BYTES];
/* 记录最近一次 SET_CUR 等待 DATA 阶段时的目标条目索引 */
int s_pending_set_idx = -1;

int find_ctrl_index(uint8_t entity_id, uint8_t selector)
{
    for (size_t i = 0; i < kUvcCtrlCount; ++i) {
        if (kUvcCtrlMap[i].entity_id == entity_id && kUvcCtrlMap[i].selector == selector) {
            return (int)i;
        }
    }
    return -1;
}

void encode_value(uint8_t value_size, int32_t value, uint8_t *out)
{
    /* UVC sends little-endian signed values. PU controls use 16-bit payloads;
     * extension controls in this demo use 32-bit payloads.
     */
    if (value_size == 2) {
        int16_t v16 = (int16_t)value;
        out[0] = (uint8_t)(v16 & 0xFF);
        out[1] = (uint8_t)((v16 >> 8) & 0xFF);
    } else {
        out[0] = (uint8_t)(value & 0xFF);
        out[1] = (uint8_t)((value >> 8) & 0xFF);
        out[2] = (uint8_t)((value >> 16) & 0xFF);
        out[3] = (uint8_t)((value >> 24) & 0xFF);
    }
}

int32_t decode_value(uint8_t value_size, const uint8_t *in)
{
    if (value_size == 2) {
        uint16_t u = (uint16_t)in[0] | ((uint16_t)in[1] << 8);
        return (int32_t)(int16_t)u;  /* 符号扩展 */
    }
    uint32_t u = (uint32_t)in[0] | ((uint32_t)in[1] << 8)
               | ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
    return (int32_t)u;
}

int32_t clamp_to_range(const uvc_ctrl_state_t &st, int32_t v)
{
    if (v < st.v_min) return st.v_min;
    if (v > st.v_max) return st.v_max;
    return v;
}

void init_mjpeg_sw_quality_ctrl_state(uvc_ctrl_state_t &st)
{
    st.supported = true;
    st.v_min = 1;
    st.v_max = 100;
    st.v_step = 1;
    st.v_def = mjpeg_sw_quality_default_value();
    st.v_cur = mjpeg_sw_quality_value();
}

}  /* namespace */

void uvc_ctrl_apply_descriptor_masks(void)
{
    uint16_t pu_bm_controls = 0;
    uint8_t xu_bm_controls = 0;
    unsigned advertised = 0;

    for (size_t i = 0; i < kUvcCtrlCount; ++i) {
        if (!s_ctrl_state[i].supported) {
            continue;
        }

        const uint8_t selector = kUvcCtrlMap[i].selector;
        if (selector == 0) {
            continue;
        }

        if (kUvcCtrlMap[i].entity_id == UVC_ENTITY_ID_PROCESSING_UNIT && selector <= 16) {
            pu_bm_controls |= (uint16_t)(1u << (selector - 1));
            ++advertised;
        } else if (kUvcCtrlMap[i].entity_id == UVC_ENTITY_ID_EXTENSION_UNIT && selector <= 8) {
            xu_bm_controls |= (uint8_t)(1u << (selector - 1));
            ++advertised;
        }
    }

    uvc_descriptors_set_control_masks(pu_bm_controls, xu_bm_controls);
    ESP_LOGI(TAG,
             "uvc_desc_ctrl: advertising %u/%u controls via descriptors (PU=0x%04X XU=0x%02X)",
             advertised,
             (unsigned)kUvcCtrlCount,
             (unsigned)pu_bm_controls,
             (unsigned)xu_bm_controls);
}

void uvc_ctrl_cache_init(esp_cam_sensor_device_t *sensor)
{
    /* Cache query_para_desc results once after sensor creation. Control
     * requests can then answer GET_MIN/MAX/RES/DEF without touching SCCB.
     */
    s_active_sensor = sensor;
    s_mjpeg_sw_quality.store(mjpeg_sw_quality_default_value(), std::memory_order_relaxed);

    if (!sensor || !sensor->ops || !sensor->ops->query_para_desc) {
        for (size_t i = 0; i < kUvcCtrlCount; ++i) {
            s_ctrl_state[i] = {};
            if (kUvcCtrlMap[i].mjpeg_sw_quality) {
                init_mjpeg_sw_quality_ctrl_state(s_ctrl_state[i]);
            }
        }
        return;
    }

    unsigned ok_count = 0;
    for (size_t i = 0; i < kUvcCtrlCount; ++i) {
        s_ctrl_state[i] = {};
        if (kUvcCtrlMap[i].mjpeg_sw_quality) {
            init_mjpeg_sw_quality_ctrl_state(s_ctrl_state[i]);
            ++ok_count;
            ESP_LOGI(TAG, "uvc_ctrl: %-18s cid=software range=[%d,%d] step=%d def=%d cur=%d",
                     kUvcCtrlMap[i].name,
                     (int)s_ctrl_state[i].v_min,
                     (int)s_ctrl_state[i].v_max,
                     (int)s_ctrl_state[i].v_step,
                     (int)s_ctrl_state[i].v_def,
                     (int)s_ctrl_state[i].v_cur);
            continue;
        }
        esp_cam_sensor_param_desc_t desc = {};
        desc.id = kUvcCtrlMap[i].cid;
        if (sensor->ops->query_para_desc(sensor, &desc) != ESP_OK) {
            ESP_LOGW(TAG, "uvc_ctrl: %s not supported by sensor", kUvcCtrlMap[i].name);
            continue;
        }
        s_ctrl_state[i].supported = true;
        if (desc.type == ESP_CAM_SENSOR_PARAM_TYPE_NUMBER) {
            s_ctrl_state[i].v_min = desc.number.minimum;
            s_ctrl_state[i].v_max = desc.number.maximum;
            s_ctrl_state[i].v_step = desc.number.step ? (int32_t)desc.number.step : 1;
        } else {
            /* 非 NUMBER 类型，按 0/1 处理 */
            s_ctrl_state[i].v_min = 0;
            s_ctrl_state[i].v_max = 1;
            s_ctrl_state[i].v_step = 1;
        }
        s_ctrl_state[i].v_def = desc.default_value;
        s_ctrl_state[i].v_cur = desc.default_value;
        ++ok_count;
        ESP_LOGI(TAG, "uvc_ctrl: %-18s cid=0x%08lx range=[%d,%d] step=%d def=%d",
                 kUvcCtrlMap[i].name, (unsigned long)kUvcCtrlMap[i].cid,
                 (int)s_ctrl_state[i].v_min, (int)s_ctrl_state[i].v_max,
                 (int)s_ctrl_state[i].v_step, (int)s_ctrl_state[i].v_def);
    }
    ESP_LOGI(TAG, "uvc_ctrl: %u/%u controls bound to sensor", ok_count, (unsigned)kUvcCtrlCount);
}

void uvc_ctrl_cache_clear(void)
{
    s_active_sensor = nullptr;
    for (size_t i = 0; i < kUvcCtrlCount; ++i) {
        s_ctrl_state[i] = {};
    }
    s_pending_set_idx = -1;
}

extern "C" int tud_video_vc_entity_req_cb(uint8_t rhport, uint8_t stage,
                                          tusb_control_request_t const *request,
                                          uint_fast8_t ctl_idx, uint_fast8_t entity_id)
{
    (void)ctl_idx;

    const uint8_t selector = (uint8_t)((request->wValue >> 8) & 0xFF);
    const uint8_t bRequest = request->bRequest;

    const int idx = find_ctrl_index((uint8_t)entity_id, selector);
    if (idx < 0) {
        return VIDEO_ERROR_INVALID_REQUEST;  /* host 收到 STALL 后即知不支持 */
    }
    const uvc_ctrl_entry_t &m = kUvcCtrlMap[idx];
    uvc_ctrl_state_t &st = s_ctrl_state[idx];
    if (!st.supported) {
        return VIDEO_ERROR_INVALID_REQUEST;
    }

    auto reply_byte = [&](uint8_t v) -> int {
        if (stage == CONTROL_STAGE_SETUP) {
            s_ctrl_scratch[0] = v;
            if (!tud_control_xfer(rhport, request, s_ctrl_scratch, 1)) {
                return VIDEO_ERROR_UNKNOWN;
            }
        }
        return VIDEO_ERROR_NONE;
    };

    auto reply_len = [&]() -> int {
        if (stage == CONTROL_STAGE_SETUP) {
            s_ctrl_scratch[0] = m.value_size;
            s_ctrl_scratch[1] = 0;
            if (!tud_control_xfer(rhport, request, s_ctrl_scratch, 2)) {
                return VIDEO_ERROR_UNKNOWN;
            }
        }
        return VIDEO_ERROR_NONE;
    };

    auto reply_int = [&](int32_t value) -> int {
        if (stage == CONTROL_STAGE_SETUP) {
            encode_value(m.value_size, value, s_ctrl_scratch);
            if (!tud_control_xfer(rhport, request, s_ctrl_scratch, m.value_size)) {
                return VIDEO_ERROR_UNKNOWN;
            }
        }
        return VIDEO_ERROR_NONE;
    };

    switch (bRequest) {
    case VIDEO_REQUEST_GET_INFO:
        /* bit0 = supports GET, bit1 = supports SET */
        return reply_byte(0x03);

    case VIDEO_REQUEST_GET_LEN:
        return reply_len();

    case VIDEO_REQUEST_GET_CUR:
        if (m.mjpeg_sw_quality) {
            st.v_cur = mjpeg_sw_quality_value();
        }
        return reply_int(st.v_cur);
    case VIDEO_REQUEST_GET_MIN:
        return reply_int(st.v_min);
    case VIDEO_REQUEST_GET_MAX:
        return reply_int(st.v_max);
    case VIDEO_REQUEST_GET_RES:
        return reply_int(st.v_step);
    case VIDEO_REQUEST_GET_DEF:
        if (m.mjpeg_sw_quality) {
            st.v_def = mjpeg_sw_quality_default_value();
        }
        return reply_int(st.v_def);

    case VIDEO_REQUEST_SET_CUR:
        if (stage == CONTROL_STAGE_SETUP) {
            if (request->wLength != m.value_size) {
                return VIDEO_ERROR_INVALID_REQUEST;
            }
            s_pending_set_idx = idx;
            if (!tud_control_xfer(rhport, request, s_ctrl_scratch, m.value_size)) {
                s_pending_set_idx = -1;
                return VIDEO_ERROR_UNKNOWN;
            }
            return VIDEO_ERROR_NONE;
        }
        if (stage == CONTROL_STAGE_DATA) {
            /* DATA 阶段：缓冲已被 host 写入，应用值到 sensor */
            if (s_pending_set_idx != idx) {
                s_pending_set_idx = -1;
                return VIDEO_ERROR_INVALID_REQUEST;
            }
            int32_t v_in = decode_value(m.value_size, s_ctrl_scratch);
            int32_t v = clamp_to_range(st, v_in);
            if (m.mjpeg_sw_quality) {
                esp_err_t r = set_mjpeg_sw_quality(v);
                st.v_cur = mjpeg_sw_quality_value();
                if (r == ESP_OK) {
                    ESP_LOGI(TAG, "uvc_ctrl: SET %s = %d (req=%d)",
                             m.name, (int)st.v_cur, (int)v_in);
                }
                s_pending_set_idx = -1;
                return VIDEO_ERROR_NONE;
            }

            if (s_active_sensor == nullptr) {
                s_pending_set_idx = -1;
                return VIDEO_ERROR_INVALID_REQUEST;
            }
            int32_t arg = v;  /* 驱动统一吃 4 字节 int */
            esp_err_t r = esp_cam_sensor_set_para_value(s_active_sensor, m.cid, &arg, sizeof(arg));
            if (r == ESP_OK) {
                st.v_cur = v;
                ESP_LOGI(TAG, "uvc_ctrl: SET %s = %d (req=%d)",
                         m.name, (int)v, (int)v_in);
            } else {
                ESP_LOGW(TAG, "uvc_ctrl: SET %s = %d failed: %s",
                         m.name, (int)v, esp_err_to_name(r));
            }
            s_pending_set_idx = -1;
            return VIDEO_ERROR_NONE;
        }
        return VIDEO_ERROR_NONE;  /* CONTROL_STAGE_ACK */

    default:
        return VIDEO_ERROR_INVALID_REQUEST;
    }
}
