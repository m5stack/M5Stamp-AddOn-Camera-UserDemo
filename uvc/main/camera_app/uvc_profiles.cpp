#include "camera_app_private.h"

/* Build the runtime UVC mode table from the sensor's native format list.
 *
 * The descriptor layer only knows host-facing payloads such as MJPEG/YUY2.
 * This module decides which sensor format backs each host mode, whether the
 * frame can be sent directly, and how much buffering each path needs.
 */

size_t estimate_frame_buffer_size(const esp_cam_sensor_format_t &fmt)
{
    if (fmt.width == 0 || fmt.height == 0) {
        return 0;
    }

    if (fmt.format == ESP_CAM_SENSOR_PIXFORMAT_JPEG) {
        /* 扩展 DVP 驱动使用内部 DMA 环形缓冲，应用缓冲只需要容纳实际 JPEG 码流。 */
        const size_t full_window = static_cast<size_t>(fmt.width) * static_cast<size_t>(fmt.height);
        return align_up(std::min(full_window, UVC_MJPEG_CAPTURE_BUFFER_BYTES), 64);
    }

    size_t bytes_per_pixel = 2;
    switch (fmt.format) {
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565:
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE:
        bytes_per_pixel = 2;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422:
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV:
        bytes_per_pixel = 2;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB888:
    case ESP_CAM_SENSOR_PIXFORMAT_BGR888:
        bytes_per_pixel = 3;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_RAW8:
    case ESP_CAM_SENSOR_PIXFORMAT_GRAYSCALE:
        bytes_per_pixel = 1;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_RAW10:
    case ESP_CAM_SENSOR_PIXFORMAT_RAW12:
        bytes_per_pixel = 2;
        break;
    default:
        break;
    }

    return align_up(static_cast<size_t>(fmt.width) * static_cast<size_t>(fmt.height) * bytes_per_pixel, 64);
}

uint8_t mjpeg_sw_quality_default_value(void)
{
    return clamp_mjpeg_sw_quality(UVC_MJPEG_SW_QUALITY_DEFAULT);
}

uint8_t mjpeg_sw_quality_value(void)
{
    const uint8_t runtime_quality = s_mjpeg_sw_quality.load(std::memory_order_relaxed);
    if (runtime_quality != 0) {
        return runtime_quality;
    }
    return mjpeg_sw_quality_default_value();
}

bool mjpeg_sw_encoder_config_for_sensor_format(const esp_cam_sensor_format_t &fmt,
                                                           jpeg_pixel_format_t *src_type,
                                                           jpeg_subsampling_t *subsampling,
                                                           size_t *input_size)
{
    /* This is the single compatibility gate for MJPEG(SW). If a raw sensor
     * format is added here, profile advertising and runtime encoding both pick
     * it up through the same rule.
     */
    if (fmt.width == 0 || fmt.height == 0) {
        return false;
    }

    size_t bytes_per_pixel = 0;
    jpeg_pixel_format_t mapped_src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    jpeg_subsampling_t mapped_subsampling = JPEG_SUBSAMPLE_420;

    switch (fmt.format) {
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV:
        mapped_src_type = JPEG_PIXEL_FORMAT_YCbYCr;
        bytes_per_pixel = 2;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_UYVY:
        mapped_src_type = JPEG_PIXEL_FORMAT_CbYCrY;
        bytes_per_pixel = 2;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565:
        mapped_src_type = JPEG_PIXEL_FORMAT_RGB565_LE;
        bytes_per_pixel = 2;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE:
        mapped_src_type = JPEG_PIXEL_FORMAT_RGB565_BE;
        bytes_per_pixel = 2;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_GRAYSCALE:
        mapped_src_type = JPEG_PIXEL_FORMAT_GRAY;
        mapped_subsampling = JPEG_SUBSAMPLE_GRAY;
        bytes_per_pixel = 1;
        break;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB888:
        mapped_src_type = JPEG_PIXEL_FORMAT_RGB888;
        bytes_per_pixel = 3;
        break;
    default:
        return false;
    }

    if (mapped_subsampling == JPEG_SUBSAMPLE_420
        && (((fmt.width & 1U) != 0) || ((fmt.height & 1U) != 0))) {
        return false;
    }

    if (src_type != nullptr) {
        *src_type = mapped_src_type;
    }
    if (subsampling != nullptr) {
        *subsampling = mapped_subsampling;
    }
    if (input_size != nullptr) {
        *input_size = static_cast<size_t>(fmt.width) * static_cast<size_t>(fmt.height) * bytes_per_pixel;
    }
    return true;
}

bool sensor_format_can_encode_to_mjpeg_sw(const esp_cam_sensor_format_t &format)
{
    if (format.format == ESP_CAM_SENSOR_PIXFORMAT_JPEG) {
        return false;
    }

    return mjpeg_sw_encoder_config_for_sensor_format(format, nullptr, nullptr, nullptr);
}

size_t estimate_mjpeg_sw_frame_buffer_size(const esp_cam_sensor_format_t &fmt)
{
    size_t input_size = 0;
    if (!mjpeg_sw_encoder_config_for_sensor_format(fmt, nullptr, nullptr, &input_size)) {
        return 0;
    }

    return align_up(input_size + 4096U, 64);
}

size_t estimate_uvc_frame_buffer_size(const esp_cam_sensor_format_t &fmt,
                                             uvc_descriptor_payload_t payload_format,
                                             uvc_frame_conversion_t conversion)
{
    if (fmt.width == 0 || fmt.height == 0) {
        return 0;
    }
    if (conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
        return estimate_mjpeg_sw_frame_buffer_size(fmt);
    }
    if (payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG) {
        return estimate_frame_buffer_size(fmt);
    }
    return align_up(static_cast<size_t>(fmt.width)
                    * static_cast<size_t>(fmt.height)
                    * uvc_payload_bytes_per_pixel(payload_format),
                    64);
}

size_t uvc_payload_bytes_per_pixel(uvc_descriptor_payload_t payload_format)
{
    switch (payload_format) {
    case UVC_DESCRIPTOR_PAYLOAD_GRAY8:
        return 1;
    case UVC_DESCRIPTOR_PAYLOAD_YUY2:
    case UVC_DESCRIPTOR_PAYLOAD_UYVY:
    case UVC_DESCRIPTOR_PAYLOAD_RGB565:
    case UVC_DESCRIPTOR_PAYLOAD_MJPEG:
    default:
        return 2;
    }
}

uvc_descriptor_payload_t sensor_format_payload(const esp_cam_sensor_format_t &format)
{
    switch (format.format) {
    case ESP_CAM_SENSOR_PIXFORMAT_JPEG:
        return UVC_DESCRIPTOR_PAYLOAD_MJPEG;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422:
        return UVC_DESCRIPTOR_PAYLOAD_UYVY;
    case ESP_CAM_SENSOR_PIXFORMAT_GRAYSCALE:
        return UVC_DESCRIPTOR_PAYLOAD_GRAY8;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565:
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE:
        return UVC_DESCRIPTOR_PAYLOAD_RGB565;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV:
    default:
        return UVC_DESCRIPTOR_PAYLOAD_YUY2;
    }
}

uvc_frame_conversion_t sensor_format_conversion_to_yuy2(const esp_cam_sensor_format_t &format)
{
    switch (format.format) {
    case ESP_CAM_SENSOR_PIXFORMAT_JPEG:
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV:
        return uvc_frame_conversion_t::none;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422:
        return uvc_frame_conversion_t::uyvy_to_yuy2;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565:
        return uvc_frame_conversion_t::rgb565_to_yuy2;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE:
        return uvc_frame_conversion_t::rgb565_be_to_yuy2;
    case ESP_CAM_SENSOR_PIXFORMAT_GRAYSCALE:
        return uvc_frame_conversion_t::grayscale_to_yuy2;
    default:
        return uvc_frame_conversion_t::none;
    }
}

uvc_frame_conversion_t sensor_format_conversion_for_payload(const esp_cam_sensor_format_t &format,
                                                                   uvc_descriptor_payload_t payload_format)
{
    if (payload_format == UVC_DESCRIPTOR_PAYLOAD_YUY2) {
        return sensor_format_conversion_to_yuy2(format);
    }
    if (payload_format == UVC_DESCRIPTOR_PAYLOAD_UYVY
        && format.format == ESP_CAM_SENSOR_PIXFORMAT_YUV422) {
        return uvc_frame_conversion_t::none;
    }
    if (payload_format == UVC_DESCRIPTOR_PAYLOAD_GRAY8
        && format.format == ESP_CAM_SENSOR_PIXFORMAT_GRAYSCALE) {
        return uvc_frame_conversion_t::none;
    }
    if (payload_format == UVC_DESCRIPTOR_PAYLOAD_RGB565
        && format.format == ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE) {
        return uvc_frame_conversion_t::rgb565_be_to_rgb565;
    }
    return uvc_frame_conversion_t::none;
}

int sensor_format_uvc_priority(const esp_cam_sensor_format_t &format)
{
    switch (format.format) {
    case ESP_CAM_SENSOR_PIXFORMAT_JPEG:
        return 0;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV:
        return 1;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422:
        return 2;
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565:
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE:
        return 3;
    case ESP_CAM_SENSOR_PIXFORMAT_GRAYSCALE:
        return 4;
    default:
        return 100;
    }
}

int uvc_profile_candidate_priority(const uvc_profile_candidate_t &candidate)
{
    if (candidate.sensor_fmt == nullptr) {
        return 100;
    }

    if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG) {
        return candidate.conversion == uvc_frame_conversion_t::none ? 0 : (1 + sensor_format_uvc_priority(*candidate.sensor_fmt));
    }
    if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_YUY2) {
        return 20 + sensor_format_uvc_priority(*candidate.sensor_fmt);
    }
    if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_UYVY) {
        return 20 + sensor_format_uvc_priority(*candidate.sensor_fmt);
    }
    if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_RGB565) {
        return 30 + sensor_format_uvc_priority(*candidate.sensor_fmt);
    }
    if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_GRAY8) {
        return 40 + sensor_format_uvc_priority(*candidate.sensor_fmt);
    }

    return 90 + sensor_format_uvc_priority(*candidate.sensor_fmt);
}

bool prefer_profile_candidate_for_default(const uvc_profile_candidate_t &lhs,
                                                 const uvc_profile_candidate_t &rhs,
                                                 bool prefer_largest_resolution)
{
    const esp_cam_sensor_format_t *lhs_fmt = lhs.sensor_fmt;
    const esp_cam_sensor_format_t *rhs_fmt = rhs.sensor_fmt;
    if (lhs_fmt == nullptr || rhs_fmt == nullptr) {
        return rhs_fmt == nullptr;
    }

    const uint64_t lhs_area = static_cast<uint64_t>(lhs_fmt->width) * lhs_fmt->height;
    const uint64_t rhs_area = static_cast<uint64_t>(rhs_fmt->width) * rhs_fmt->height;
    if (lhs_area != rhs_area) {
        return prefer_largest_resolution ? (lhs_area > rhs_area) : (lhs_area < rhs_area);
    }

    const int lhs_priority = uvc_profile_candidate_priority(lhs);
    const int rhs_priority = uvc_profile_candidate_priority(rhs);
    if (lhs_priority != rhs_priority) {
        return lhs_priority < rhs_priority;
    }

    if (lhs_fmt->fps != rhs_fmt->fps) {
        return lhs_fmt->fps > rhs_fmt->fps;
    }

    const char *lhs_name = lhs_fmt->name ? lhs_fmt->name : "";
    const char *rhs_name = rhs_fmt->name ? rhs_fmt->name : "";
    return std::strcmp(lhs_name, rhs_name) < 0;
}

uint8_t choose_preferred_frame_index(const std::vector<uvc_profile_candidate_t> &candidates,
                                            bool prefer_largest_resolution)
{
    if (candidates.empty()) {
        return 1;
    }

    size_t best_index = 0;
    for (size_t i = 1; i < candidates.size(); ++i) {
        if (prefer_profile_candidate_for_default(candidates[i],
                                                 candidates[best_index],
                                                 prefer_largest_resolution)) {
            best_index = i;
        }
    }

    return static_cast<uint8_t>(best_index + 1);
}

const uvc_profile_candidate_t *choose_default_resolution_candidate(
    const std::vector<uvc_profile_candidate_t> &jpeg_candidates,
    const std::vector<uvc_profile_candidate_t> &yuy2_candidates)
{
    const uvc_profile_candidate_t *best = nullptr;

    /* 默认打开优先 JPEG 最大分辨率；只有没有 JPEG 时才回退到 YUY2。 */
    for (const uvc_profile_candidate_t &candidate : jpeg_candidates) {
        if (best == nullptr || prefer_profile_candidate_for_default(candidate, *best, true)) {
            best = &candidate;
        }
    }
    if (best != nullptr) {
        return best;
    }

    for (const uvc_profile_candidate_t &candidate : yuy2_candidates) {
        if (best == nullptr || prefer_profile_candidate_for_default(candidate, *best, true)) {
            best = &candidate;
        }
    }

    return best;
}

bool sensor_format_can_stream_as_uvc(const esp_cam_sensor_format_t &format)
{
    return sensor_format_uvc_priority(format) < 100;
}

const esp_cam_sensor_format_t *find_first_safe_uvc_sensor_format(const esp_cam_sensor_format_array_t &formats,
                                                                        detected_sensor_model_t sensor_model)
{
    (void)sensor_model;

    const esp_cam_sensor_format_t *best = nullptr;
    for (uint32_t i = 0; i < formats.count; ++i) {
        const esp_cam_sensor_format_t &candidate = formats.format_array[i];
        if (!sensor_format_can_stream_as_uvc(candidate)) {
            continue;
        }
        if (best == nullptr || sensor_format_uvc_priority(candidate) < sensor_format_uvc_priority(*best)) {
            best = &candidate;
        }
    }
    return best;
}

uint8_t choose_uvc_usb_frame_rate(uvc_descriptor_payload_t payload_format,
                                         size_t frame_buffer_size,
                                         uint8_t sensor_fps,
                                         uvc_frame_conversion_t conversion,
                                         uint16_t width,
                                         uint16_t height)
{
    uint32_t max_fps = sensor_fps > 0 ? sensor_fps : 1;

    if (conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
        const uint32_t pixels = static_cast<uint32_t>(width) * static_cast<uint32_t>(height);
        const uint32_t jpeg_limit_fps = pixels > 0
            ? (UVC_MJPEG_SW_SAFE_PIXELS_PER_SECOND / pixels)
            : 1U;
        if (jpeg_limit_fps == 0) {
            return 1;
        }
        if (jpeg_limit_fps < max_fps) {
            max_fps = jpeg_limit_fps;
        }
        return static_cast<uint8_t>(max_fps > 0 ? max_fps : 1U);
    }

    if (payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG) {
        return static_cast<uint8_t>(max_fps);
    }

    const uint32_t raw_limit_fps = frame_buffer_size > 0
        ? (UVC_YUY2_SAFE_BUDGET_BYTES_PER_SECOND / static_cast<uint32_t>(frame_buffer_size))
        : 1U;
    if (raw_limit_fps == 0) {
        return 1;
    }
    if (raw_limit_fps < max_fps) {
        max_fps = raw_limit_fps;
    }

    return static_cast<uint8_t>(max_fps > 0 ? max_fps : 1U);
}

size_t populate_uvc_profile_bindings(const esp_cam_sensor_format_array_t &formats,
                                            detected_sensor_model_t sensor_model,
                                            uvc_descriptor_config_t *descriptor_config)
{
    (void)sensor_model;

    /* Candidate selection is deliberately separate from descriptor ordering.
     * First choose valid modes, then group them by UVC payload type so each
     * payload owns one format descriptor with one or more frame entries.
     */
    std::vector<uvc_profile_candidate_t> selected_candidates;
    std::vector<uvc_profile_candidate_t> selected_mjpeg_sw_candidates_by_resolution;
    std::vector<uvc_profile_candidate_t> selected_mjpeg_hw_candidates;
    std::vector<uvc_profile_candidate_t> selected_mjpeg_sw_candidates;
    std::vector<uvc_profile_candidate_t> selected_yuy2_candidates;
    std::vector<uvc_profile_candidate_t> selected_uyvy_candidates;
    std::vector<uvc_profile_candidate_t> selected_gray_candidates;
    std::vector<uvc_profile_candidate_t> selected_rgb565_candidates;

    for (size_t i = 0; i < UVC_FRAME_PROFILE_COUNT; ++i) {
        s_uvc_frame_profiles[i] = {};
        s_uvc_profile_bindings[i] = {};
    }

    if (descriptor_config == nullptr) {
        return 0;
    }
    *descriptor_config = {};

    auto add_selected_candidate = [&](const uvc_profile_candidate_t &candidate) {
        if (candidate.sensor_fmt == nullptr) {
            return;
        }
        if (!UVC_ONLY_BEST_FORMAT_PER_RESOLUTION) {
            selected_candidates.push_back(candidate);
            return;
        }

        for (uvc_profile_candidate_t &selected : selected_candidates) {
            if (selected.sensor_fmt == nullptr) {
                continue;
            }
            if (selected.sensor_fmt->width == candidate.sensor_fmt->width
                && selected.sensor_fmt->height == candidate.sensor_fmt->height) {
                if (prefer_profile_candidate_for_default(candidate, selected, true)) {
                    selected = candidate;
                }
                return;
            }
        }
        selected_candidates.push_back(candidate);
    };

    auto has_mjpeg_hw_resolution = [&](const esp_cam_sensor_format_t &format) {
        for (uint32_t i = 0; i < formats.count; ++i) {
            const esp_cam_sensor_format_t &candidate = formats.format_array[i];
            if (candidate.format == ESP_CAM_SENSOR_PIXFORMAT_JPEG
                && candidate.width == format.width
                && candidate.height == format.height
                && sensor_format_can_stream_as_uvc(candidate)) {
                return true;
            }
        }
        return false;
    };

    auto add_mjpeg_sw_candidate = [&](const uvc_profile_candidate_t &candidate) {
        if (candidate.sensor_fmt == nullptr) {
            return;
        }
        if (UVC_ONLY_BEST_FORMAT_PER_RESOLUTION) {
            add_selected_candidate(candidate);
            return;
        }

        for (uvc_profile_candidate_t &selected : selected_mjpeg_sw_candidates_by_resolution) {
            if (selected.sensor_fmt == nullptr) {
                continue;
            }
            if (selected.sensor_fmt->width == candidate.sensor_fmt->width
                && selected.sensor_fmt->height == candidate.sensor_fmt->height) {
                if (prefer_profile_candidate_for_default(candidate, selected, true)) {
                    selected = candidate;
                }
                return;
            }
        }
        selected_mjpeg_sw_candidates_by_resolution.push_back(candidate);
    };

    /* UVC_ONLY_BEST_FORMAT_PER_RESOLUTION=true 时，每个分辨率只保留一个最佳路径：
     * 原生 MJPEG 优先，其次软件 MJPEG，再到 YUY2/UYVY/RGBP/GRAY8。
     * false 时保留所有原生 profile；软件 MJPEG 只按缺失 JPEG 的分辨率补一个最佳 raw 源。 */
    for (uint32_t format_index = 0; format_index < formats.count; ++format_index) {
        const esp_cam_sensor_format_t &sensor_format = formats.format_array[format_index];
        if (!sensor_format_can_stream_as_uvc(sensor_format)) {
            continue;
        }

        uvc_profile_candidate_t direct_candidate{};
        direct_candidate.sensor_fmt = &sensor_format;
        direct_candidate.payload_format = sensor_format_payload(sensor_format);
        direct_candidate.conversion = sensor_format_conversion_for_payload(sensor_format,
                                                                           direct_candidate.payload_format);
        add_selected_candidate(direct_candidate);

        if (UVC_ENABLE_MJPEG_SW_CONVERSION
            && !has_mjpeg_hw_resolution(sensor_format)
            && sensor_format_can_encode_to_mjpeg_sw(sensor_format)) {
            uvc_profile_candidate_t jpeg_candidate{};
            jpeg_candidate.sensor_fmt = &sensor_format;
            jpeg_candidate.payload_format = UVC_DESCRIPTOR_PAYLOAD_MJPEG;
            jpeg_candidate.conversion = uvc_frame_conversion_t::raw_to_mjpeg;
            add_mjpeg_sw_candidate(jpeg_candidate);
        }
    }
    if (!UVC_ONLY_BEST_FORMAT_PER_RESOLUTION) {
        selected_candidates.insert(selected_candidates.end(),
                                   selected_mjpeg_sw_candidates_by_resolution.begin(),
                                   selected_mjpeg_sw_candidates_by_resolution.end());
    }

    auto sort_candidates_by_resolution = [](std::vector<uvc_profile_candidate_t> &list) {
        std::sort(list.begin(), list.end(),
              [](const uvc_profile_candidate_t &lhs, const uvc_profile_candidate_t &rhs) {
                  const esp_cam_sensor_format_t *lhs_fmt = lhs.sensor_fmt;
                  const esp_cam_sensor_format_t *rhs_fmt = rhs.sensor_fmt;
                  if (lhs_fmt == nullptr || rhs_fmt == nullptr) {
                      return rhs_fmt != nullptr;
                  }

                  const uint64_t lhs_area = static_cast<uint64_t>(lhs_fmt->width) * lhs_fmt->height;
                  const uint64_t rhs_area = static_cast<uint64_t>(rhs_fmt->width) * rhs_fmt->height;
                  if (lhs_area != rhs_area) {
                      return lhs_area < rhs_area;
                  }
                  if (lhs_fmt->width != rhs_fmt->width) {
                      return lhs_fmt->width < rhs_fmt->width;
                  }
                  if (lhs_fmt->height != rhs_fmt->height) {
                      return lhs_fmt->height < rhs_fmt->height;
                  }
                  if (lhs_fmt->fps != rhs_fmt->fps) {
                      return lhs_fmt->fps > rhs_fmt->fps;
                  }

                  const int lhs_priority = uvc_profile_candidate_priority(lhs);
                  const int rhs_priority = uvc_profile_candidate_priority(rhs);
                  if (lhs_priority != rhs_priority) {
                      return lhs_priority < rhs_priority;
                  }

                  const char *lhs_name = lhs_fmt->name ? lhs_fmt->name : "";
                  const char *rhs_name = rhs_fmt->name ? rhs_fmt->name : "";
                  return std::strcmp(lhs_name, rhs_name) < 0;
              });
    };

    sort_candidates_by_resolution(selected_candidates);

    if (selected_candidates.size() > UVC_FRAME_PROFILE_COUNT) {
        ESP_LOGW(TAG, "too many UVC profiles (%u), truncating to descriptor capacity %u",
                 static_cast<unsigned>(selected_candidates.size()),
                 static_cast<unsigned>(UVC_FRAME_PROFILE_COUNT));
        selected_candidates.resize(UVC_FRAME_PROFILE_COUNT);
    }

    for (const uvc_profile_candidate_t &candidate : selected_candidates) {
        if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG) {
            if (candidate.conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
                selected_mjpeg_sw_candidates.push_back(candidate);
            } else {
                selected_mjpeg_hw_candidates.push_back(candidate);
            }
        } else if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_UYVY) {
            selected_uyvy_candidates.push_back(candidate);
        } else if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_GRAY8) {
            selected_gray_candidates.push_back(candidate);
        } else if (candidate.payload_format == UVC_DESCRIPTOR_PAYLOAD_RGB565) {
            selected_rgb565_candidates.push_back(candidate);
        } else {
            selected_yuy2_candidates.push_back(candidate);
        }
    }

    sort_candidates_by_resolution(selected_mjpeg_hw_candidates);
    sort_candidates_by_resolution(selected_mjpeg_sw_candidates);
    sort_candidates_by_resolution(selected_yuy2_candidates);
    sort_candidates_by_resolution(selected_uyvy_candidates);
    sort_candidates_by_resolution(selected_gray_candidates);
    sort_candidates_by_resolution(selected_rgb565_candidates);

    size_t selected_count = 0;
    const size_t yuy2_count = selected_yuy2_candidates.size();
    const size_t uyvy_count = selected_uyvy_candidates.size();
    const size_t gray_count = selected_gray_candidates.size();
    const size_t mjpeg_hw_count = selected_mjpeg_hw_candidates.size();
    const size_t mjpeg_sw_count = selected_mjpeg_sw_candidates.size();
    const size_t rgb565_count = selected_rgb565_candidates.size();
    const size_t total_profile_count = mjpeg_hw_count + mjpeg_sw_count + yuy2_count + uyvy_count + gray_count + rgb565_count;
    if (total_profile_count == 0 || total_profile_count > UVC_FRAME_PROFILE_COUNT) {
        return 0;
    }

    /* UVC format descriptor 按稳定规则输出，便于 host/UI 侧观察：
     *   MJPEG(HW) > MJPEG(SW) > YUY2 > UYVY > RGBP > GRAY8。
     * default_format_index/default_frame_index 单独指定，不改变列表排序。 */
    uint8_t next_format_index = 1;
    uint8_t mjpeg_hw_format_index = 0;
    uint8_t mjpeg_sw_format_index = 0;
    uint8_t yuy2_format_index = 0;
    uint8_t uyvy_format_index = 0;
    uint8_t gray_format_index = 0;
    uint8_t rgb565_format_index = 0;
    std::vector<uvc_profile_candidate_t> default_mjpeg_candidates = selected_mjpeg_hw_candidates;
    default_mjpeg_candidates.insert(default_mjpeg_candidates.end(),
                                   selected_mjpeg_sw_candidates.begin(),
                                   selected_mjpeg_sw_candidates.end());
    const uvc_profile_candidate_t *default_resolution_candidate = choose_default_resolution_candidate(default_mjpeg_candidates,
                                                                                                      selected_yuy2_candidates);
    const uvc_descriptor_payload_t default_payload = (default_resolution_candidate != nullptr)
        ? default_resolution_candidate->payload_format
        : (uyvy_count > 0)
            ? UVC_DESCRIPTOR_PAYLOAD_UYVY
            : (gray_count > 0)
                ? UVC_DESCRIPTOR_PAYLOAD_GRAY8
                : UVC_DESCRIPTOR_PAYLOAD_RGB565;
    const bool default_is_mjpeg_sw = default_resolution_candidate != nullptr
        && default_resolution_candidate->payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG
        && default_resolution_candidate->conversion == uvc_frame_conversion_t::raw_to_mjpeg;

    auto assign_format_index = [&](uint8_t *format_index, bool enabled) {
        if (enabled && format_index != nullptr) {
            *format_index = next_format_index++;
        }
    };

    assign_format_index(&mjpeg_hw_format_index, mjpeg_hw_count > 0);
    assign_format_index(&mjpeg_sw_format_index, mjpeg_sw_count > 0);
    assign_format_index(&yuy2_format_index, yuy2_count > 0);
    assign_format_index(&uyvy_format_index, uyvy_count > 0);
    assign_format_index(&rgb565_format_index, rgb565_count > 0);
    assign_format_index(&gray_format_index, gray_count > 0);

    descriptor_config->payload_format = default_payload;
    descriptor_config->format_index = UVC_DESCRIPTOR_FORMAT_INDEX;
    /* format_count = number of UVC format descriptors: MJPEG + YUY2 + UYVY + GRAY8 + RGBP.
     * Each payload type owns one format descriptor with multiple frame indexes.
     * This matches the UVC spec: format_index = codec type, frame_index = resolution. */
    descriptor_config->format_count = static_cast<uint8_t>((mjpeg_hw_count > 0 ? 1 : 0)
        + (mjpeg_sw_count > 0 ? 1 : 0)
        + (yuy2_count > 0 ? 1 : 0)
        + (uyvy_count > 0 ? 1 : 0)
        + (gray_count > 0 ? 1 : 0)
        + (rgb565_count > 0 ? 1 : 0));
    descriptor_config->frame_count  = static_cast<uint8_t>(total_profile_count);
    descriptor_config->default_format_index = (default_payload == UVC_DESCRIPTOR_PAYLOAD_MJPEG)
        ? (default_is_mjpeg_sw ? mjpeg_sw_format_index : mjpeg_hw_format_index)
        : (default_payload == UVC_DESCRIPTOR_PAYLOAD_YUY2)
            ? yuy2_format_index
            : (default_payload == UVC_DESCRIPTOR_PAYLOAD_UYVY)
                ? uyvy_format_index
                : (default_payload == UVC_DESCRIPTOR_PAYLOAD_GRAY8)
                    ? gray_format_index
                    : rgb565_format_index;
    descriptor_config->default_frame_index = 1;
    /* frame 顺序保持小到大；default_frame_index 单独指向默认 payload 的最大分辨率。 */
    if (default_payload == UVC_DESCRIPTOR_PAYLOAD_YUY2 && yuy2_count > 0) {
        descriptor_config->default_frame_index = choose_preferred_frame_index(selected_yuy2_candidates, true);
    } else if (default_payload == UVC_DESCRIPTOR_PAYLOAD_MJPEG && default_is_mjpeg_sw && mjpeg_sw_count > 0) {
        descriptor_config->default_frame_index = choose_preferred_frame_index(selected_mjpeg_sw_candidates, true);
    } else if (default_payload == UVC_DESCRIPTOR_PAYLOAD_MJPEG && mjpeg_hw_count > 0) {
        descriptor_config->default_frame_index = choose_preferred_frame_index(selected_mjpeg_hw_candidates, true);
    } else if (default_payload == UVC_DESCRIPTOR_PAYLOAD_UYVY && uyvy_count > 0) {
        descriptor_config->default_frame_index = choose_preferred_frame_index(selected_uyvy_candidates, true);
    } else if (default_payload == UVC_DESCRIPTOR_PAYLOAD_GRAY8 && gray_count > 0) {
        descriptor_config->default_frame_index = choose_preferred_frame_index(selected_gray_candidates, true);
    } else if (rgb565_count > 0) {
        descriptor_config->default_frame_index = choose_preferred_frame_index(selected_rgb565_candidates, true);
    }

    auto add_profile = [&](const uvc_profile_candidate_t &candidate,
                           uint8_t format_index,
                           uint8_t frame_index) {
        const esp_cam_sensor_format_t *sensor_format = candidate.sensor_fmt;
        if (sensor_format == nullptr) {
            return;
        }

        const uvc_descriptor_payload_t payload_format = candidate.payload_format;
        const size_t capture_buffer_size = estimate_frame_buffer_size(*sensor_format);
        const size_t frame_buffer_size = estimate_uvc_frame_buffer_size(*sensor_format,
                                                                        payload_format,
                                                                        candidate.conversion);
        const uint8_t usb_frame_rate = choose_uvc_usb_frame_rate(payload_format,
                                                                 frame_buffer_size,
                                                                 sensor_format->fps,
                                                                 candidate.conversion,
                                                                 sensor_format->width,
                                                                 sensor_format->height);
        const uint32_t frame_interval_100ns = static_cast<uint32_t>(10000000UL / usb_frame_rate);
        const size_t i = selected_count++;

        s_uvc_frame_profiles[i].payload_format = payload_format;
        s_uvc_frame_profiles[i].format_index = format_index;
        s_uvc_frame_profiles[i].frame_index = frame_index;
        s_uvc_frame_profiles[i].width = sensor_format->width;
        s_uvc_frame_profiles[i].height = sensor_format->height;
        s_uvc_frame_profiles[i].sensor_fps = sensor_format->fps;
        s_uvc_frame_profiles[i].usb_frame_rate = usb_frame_rate;
        s_uvc_frame_profiles[i].frame_interval_us = static_cast<int64_t>(frame_interval_100ns) / 10;
        s_uvc_frame_profiles[i].frame_buffer_size = frame_buffer_size;
        s_uvc_frame_profiles[i].capture_buffer_size = capture_buffer_size;

        s_uvc_profile_bindings[i].profile = &s_uvc_frame_profiles[i];
        s_uvc_profile_bindings[i].sensor_fmt = sensor_format;
        s_uvc_profile_bindings[i].conversion = candidate.conversion;
        s_uvc_profile_bindings[i].available = true;

        descriptor_config->frames[i].payload_format = s_uvc_frame_profiles[i].payload_format;
        descriptor_config->frames[i].format_index = s_uvc_frame_profiles[i].format_index;
        descriptor_config->frames[i].frame_index = s_uvc_frame_profiles[i].frame_index;
        descriptor_config->frames[i].width = s_uvc_frame_profiles[i].width;
        descriptor_config->frames[i].height = s_uvc_frame_profiles[i].height;
        descriptor_config->frames[i].frame_rate = s_uvc_frame_profiles[i].usb_frame_rate;
        descriptor_config->frames[i].frame_interval_100ns = frame_interval_100ns;
        descriptor_config->frames[i].frame_buffer_size = static_cast<uint32_t>(frame_buffer_size);
        descriptor_config->frames[i].mjpeg_sw = candidate.conversion == uvc_frame_conversion_t::raw_to_mjpeg;
    };

    for (size_t i = 0; i < selected_mjpeg_hw_candidates.size(); ++i) {
        add_profile(selected_mjpeg_hw_candidates[i],
                    mjpeg_hw_format_index,
                    static_cast<uint8_t>(i + 1));
    }
    for (size_t i = 0; i < selected_mjpeg_sw_candidates.size(); ++i) {
        add_profile(selected_mjpeg_sw_candidates[i],
                    mjpeg_sw_format_index,
                    static_cast<uint8_t>(i + 1));
    }
    for (size_t i = 0; i < selected_yuy2_candidates.size(); ++i) {
        add_profile(selected_yuy2_candidates[i],
                    yuy2_format_index,
                    static_cast<uint8_t>(i + 1));
    }
    for (size_t i = 0; i < selected_uyvy_candidates.size(); ++i) {
        add_profile(selected_uyvy_candidates[i],
                    uyvy_format_index,
                    static_cast<uint8_t>(i + 1));
    }
    for (size_t i = 0; i < selected_rgb565_candidates.size(); ++i) {
        add_profile(selected_rgb565_candidates[i],
                    rgb565_format_index,
                    static_cast<uint8_t>(i + 1));
    }
    for (size_t i = 0; i < selected_gray_candidates.size(); ++i) {
        add_profile(selected_gray_candidates[i],
                    gray_format_index,
                    static_cast<uint8_t>(i + 1));
    }

    return selected_count;
}

const char *uvc_payload_format_name(uvc_descriptor_payload_t payload_format)
{
    switch (payload_format) {
    case UVC_DESCRIPTOR_PAYLOAD_MJPEG:
        return "MJPEG(HW)";
    case UVC_DESCRIPTOR_PAYLOAD_UYVY:
        return "UYVY";
    case UVC_DESCRIPTOR_PAYLOAD_GRAY8:
        return "GRAY8";
    case UVC_DESCRIPTOR_PAYLOAD_RGB565:
        return "RGBP";
    case UVC_DESCRIPTOR_PAYLOAD_YUY2:
    default:
        return "YUY2";
    }
}

const char *uvc_profile_payload_name(const uvc_profile_binding_t &binding)
{
    if (binding.conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
        return "MJPEG(SW)";
    }
    return uvc_payload_format_name(binding.profile ? binding.profile->payload_format : UVC_DESCRIPTOR_PAYLOAD_YUY2);
}

const char *uvc_frame_conversion_name(uvc_frame_conversion_t conversion)
{
    switch (conversion) {
    case uvc_frame_conversion_t::none:
        return "direct";
    case uvc_frame_conversion_t::uyvy_to_yuy2:
        return "UYVY->YUY2 copy";
    case uvc_frame_conversion_t::rgb565_to_yuy2:
        return "RGB565->YUY2 copy";
    case uvc_frame_conversion_t::rgb565_be_to_yuy2:
        return "RGB565_BE->YUY2 copy";
    case uvc_frame_conversion_t::rgb565_be_to_rgb565:
        return "RGB565_BE->RGBP byte-swap";
    case uvc_frame_conversion_t::grayscale_to_yuy2:
        return "GRAY8->YUY2 copy";
    case uvc_frame_conversion_t::raw_to_mjpeg:
        return "RAW->MJPEG encode";
    default:
        return "unknown conversion";
    }
}

const char *uvc_copy_result_name(uvc_copy_result_t result)
{
    switch (result) {
    case uvc_copy_result_t::ok:
        return "ok";
    case uvc_copy_result_t::invalid_arg:
        return "invalid_arg";
    case uvc_copy_result_t::no_frame:
        return "no_frame";
    case uvc_copy_result_t::slot_locked:
        return "slot_locked";
    case uvc_copy_result_t::invalid_frame_size:
        return "invalid_frame_size";
    case uvc_copy_result_t::invalid_frame_content:
        return "invalid_frame_content";
    default:
        return "unknown";
    }
}

int find_uvc_profile_binding_index(uint8_t format_index, uint8_t frame_index)
{
    for (size_t i = 0; i < UVC_FRAME_PROFILE_COUNT; ++i) {
        if (s_uvc_profile_bindings[i].profile != nullptr
            && s_uvc_profile_bindings[i].profile->format_index == format_index
            && s_uvc_profile_bindings[i].profile->frame_index == frame_index) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

size_t get_max_uvc_capture_buffer_size(void)
{
    size_t max_size = 0;
    for (size_t index = 0; index < UVC_FRAME_PROFILE_COUNT; ++index) {
        const uvc_frame_profile_t *profile = s_uvc_profile_bindings[index].profile;
        if (s_uvc_profile_bindings[index].available && profile != nullptr && profile->capture_buffer_size > max_size) {
            max_size = profile->capture_buffer_size;
        }
    }
    return max_size;
}
