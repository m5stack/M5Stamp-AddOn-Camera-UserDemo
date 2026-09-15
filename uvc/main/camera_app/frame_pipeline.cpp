#include "camera_app_private.h"

/* Capture pipeline and frame conversion.
 *
 * DVP capture, UVC transfer, and optional MJPEG software encoding run in
 * different callback/task contexts. The functions in this file protect the
 * shared frame slots and rebuild the sensor/DVP path when the host chooses a
 * different UVC mode.
 */

void reset_frame_context(void)
{
    s_frame_ctx.latest_slot.store(-1, std::memory_order_relaxed);
    s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_relaxed);
    s_frame_ctx.dma_inflight_slot.store(-1, std::memory_order_relaxed);
    for (size_t i = 0; i < CAMERA_FRAME_SLOT_COUNT; ++i) {
        s_frame_ctx.slots[i].frame_size.store(0, std::memory_order_relaxed);
    }
    s_last_frame_size.store(0, std::memory_order_relaxed);
    s_last_dma_window_size.store(0, std::memory_order_relaxed);
    s_last_frame_slot.store(-1, std::memory_order_relaxed);
    s_invalid_frame_count.store(0, std::memory_order_relaxed);
    s_capture_buffer_miss_count.store(0, std::memory_order_relaxed);
}

const char *jpeg_error_name(jpeg_error_t err)
{
    switch (err) {
    case JPEG_ERR_OK:
        return "JPEG_ERR_OK";
    case JPEG_ERR_FAIL:
        return "JPEG_ERR_FAIL";
    case JPEG_ERR_NO_MEM:
        return "JPEG_ERR_NO_MEM";
    case JPEG_ERR_NO_MORE_DATA:
        return "JPEG_ERR_NO_MORE_DATA";
    case JPEG_ERR_INVALID_PARAM:
        return "JPEG_ERR_INVALID_PARAM";
    case JPEG_ERR_BAD_DATA:
        return "JPEG_ERR_BAD_DATA";
    case JPEG_ERR_UNSUPPORT_FMT:
        return "JPEG_ERR_UNSUPPORT_FMT";
    case JPEG_ERR_UNSUPPORT_STD:
        return "JPEG_ERR_UNSUPPORT_STD";
    default:
        return "JPEG_ERR_UNKNOWN";
    }
}

uint8_t clamp_mjpeg_sw_quality(int32_t value)
{
    if (value < 1) {
        return 1;
    }
    if (value > 100) {
        return 100;
    }
    return static_cast<uint8_t>(value);
}

void close_mjpeg_sw_encoder(void)
{
    if (s_mjpeg_sw_encoder.handle != nullptr) {
        jpeg_enc_close(s_mjpeg_sw_encoder.handle);
        s_mjpeg_sw_encoder = {};
    }
}

esp_err_t set_mjpeg_sw_quality(int32_t value)
{
    const uint8_t quality = clamp_mjpeg_sw_quality(value);
    s_mjpeg_sw_quality.store(quality, std::memory_order_relaxed);

    if (s_mjpeg_sw_encoder.handle != nullptr) {
        const jpeg_error_t ret = jpeg_enc_set_quality(s_mjpeg_sw_encoder.handle, quality);
        if (ret != JPEG_ERR_OK) {
            ESP_LOGW(TAG, "MJPEG(SW) quality set failed: %s value=%u",
                     jpeg_error_name(ret),
                     static_cast<unsigned>(quality));
            return ESP_FAIL;
        }
        s_mjpeg_sw_encoder.quality = quality;
    }

    ESP_LOGI(TAG, "MJPEG(SW) quality set to %u", static_cast<unsigned>(quality));
    return ESP_OK;
}

bool ensure_mjpeg_sw_encoder(const esp_cam_sensor_format_t &sensor_fmt)
{
    /* Reuse the encoder while the negotiated source format and quality match.
     * Reopening it per frame would be too expensive for USB streaming.
     */
    jpeg_pixel_format_t src_type = JPEG_PIXEL_FORMAT_YCbYCr;
    jpeg_subsampling_t subsampling = JPEG_SUBSAMPLE_420;
    size_t input_size = 0;
    if (!mjpeg_sw_encoder_config_for_sensor_format(sensor_fmt, &src_type, &subsampling, &input_size)) {
        return false;
    }

    const uint8_t quality = mjpeg_sw_quality_value();
    if (s_mjpeg_sw_encoder.handle != nullptr
        && s_mjpeg_sw_encoder.width == sensor_fmt.width
        && s_mjpeg_sw_encoder.height == sensor_fmt.height
        && s_mjpeg_sw_encoder.src_type == src_type
        && s_mjpeg_sw_encoder.subsampling == subsampling
        && s_mjpeg_sw_encoder.quality == quality
        && s_mjpeg_sw_encoder.input_size == input_size) {
        return true;
    }

    close_mjpeg_sw_encoder();

    jpeg_enc_config_t jpeg_enc_cfg = DEFAULT_JPEG_ENC_CONFIG();
    jpeg_enc_cfg.width = sensor_fmt.width;
    jpeg_enc_cfg.height = sensor_fmt.height;
    jpeg_enc_cfg.src_type = src_type;
    jpeg_enc_cfg.subsampling = subsampling;
    jpeg_enc_cfg.quality = quality;
    jpeg_enc_cfg.rotate = JPEG_ROTATE_0D;
    jpeg_enc_cfg.task_enable = MJPEG_SW_ENCODER_DUAL_TASK;
    jpeg_enc_cfg.hfm_task_priority = static_cast<uint8_t>(MJPEG_SW_ENCODER_HELPER_PRIORITY);
    jpeg_enc_cfg.hfm_task_core = static_cast<uint8_t>(MJPEG_SW_ENCODER_HELPER_CORE);

    jpeg_enc_handle_t handle = nullptr;
    bool task_enabled = jpeg_enc_cfg.task_enable;
    jpeg_error_t ret = jpeg_enc_open(&jpeg_enc_cfg, &handle);
    if ((ret != JPEG_ERR_OK || handle == nullptr) && jpeg_enc_cfg.task_enable) {
        /* 双任务编码会额外消耗内存；资源不足时回退到单任务，避免直接丢失 UVC mode。 */
        ESP_LOGW(TAG, "MJPEG(SW) dual-task encoder open failed: %s; fallback to mono-task",
                 jpeg_error_name(ret));
        jpeg_enc_cfg.task_enable = false;
        task_enabled = false;
        handle = nullptr;
        ret = jpeg_enc_open(&jpeg_enc_cfg, &handle);
    }
    if (ret != JPEG_ERR_OK || handle == nullptr) {
        ESP_LOGE(TAG, "MJPEG(SW) encoder open failed: %s fmt=%s %ux%u quality=%u",
                 jpeg_error_name(ret),
                 sensor_fmt.name ? sensor_fmt.name : "unknown",
                 sensor_fmt.width,
                 sensor_fmt.height,
                 static_cast<unsigned>(quality));
        return false;
    }

    s_mjpeg_sw_encoder.handle = handle;
    s_mjpeg_sw_encoder.src_type = src_type;
    s_mjpeg_sw_encoder.subsampling = subsampling;
    s_mjpeg_sw_encoder.width = sensor_fmt.width;
    s_mjpeg_sw_encoder.height = sensor_fmt.height;
    s_mjpeg_sw_encoder.quality = quality;
    s_mjpeg_sw_encoder.task_enable = task_enabled;
    s_mjpeg_sw_encoder.input_size = input_size;

    ESP_LOGI(TAG, "MJPEG(SW) encoder ready: %s %ux%u input=%u out_max=%u quality=%u task=%d helper_core=%d helper_prio=%u",
             sensor_fmt.name ? sensor_fmt.name : "unknown",
             sensor_fmt.width,
             sensor_fmt.height,
             static_cast<unsigned>(input_size),
             static_cast<unsigned>(estimate_mjpeg_sw_frame_buffer_size(sensor_fmt)),
             static_cast<unsigned>(quality),
             (int)task_enabled,
             (int)MJPEG_SW_ENCODER_HELPER_CORE,
             (unsigned)MJPEG_SW_ENCODER_HELPER_PRIORITY);
    return true;
}

bool encode_raw_frame_to_mjpeg(const esp_cam_sensor_format_t &sensor_fmt,
                                      uint8_t *dst,
                                      size_t dst_len,
                                      const uint8_t *src,
                                      size_t src_len,
                                      size_t *written_len)
{
    size_t input_size = 0;
    if (dst == nullptr || src == nullptr || written_len == nullptr
        || !mjpeg_sw_encoder_config_for_sensor_format(sensor_fmt, nullptr, nullptr, &input_size)
        || src_len < input_size
        || input_size > static_cast<size_t>(INT_MAX)
        || dst_len > static_cast<size_t>(INT_MAX)
        || !ensure_mjpeg_sw_encoder(sensor_fmt)) {
        return false;
    }

    int out_size = 0;
    const jpeg_error_t ret = jpeg_enc_process(s_mjpeg_sw_encoder.handle,
                                              src,
                                              static_cast<int>(input_size),
                                              dst,
                                              static_cast<int>(dst_len),
                                              &out_size);
    if (ret != JPEG_ERR_OK || out_size <= 0) {
        ESP_LOGW(TAG, "MJPEG(SW) encode failed: %s in=%u out_max=%u out=%d",
                 jpeg_error_name(ret),
                 static_cast<unsigned>(input_size),
                 static_cast<unsigned>(dst_len),
                 out_size);
        return false;
    }

    const size_t encoded_size = static_cast<size_t>(out_size);
    if (!is_jpeg_frame_content_valid(dst, encoded_size)) {
        ESP_LOGW(TAG, "MJPEG(SW) encode invalid output len=%u", static_cast<unsigned>(encoded_size));
        return false;
    }

    *written_len = encoded_size;
    return true;
}

bool is_jpeg_frame_content_valid(const uint8_t *buffer, size_t len)
{
    if (buffer == nullptr || len < 4) {
        return false;
    }

    return buffer[0] == 0xff && buffer[1] == 0xd8
        && buffer[len - 2] == 0xff && buffer[len - 1] == 0xd9;
}

bool frame_content_valid_for_uvc(const tinyusb_uvc_context_t &ctx, const uint8_t *buffer, size_t len)
{
    if (ctx.sensor_fmt.format == ESP_CAM_SENSOR_PIXFORMAT_JPEG) {
        return is_jpeg_frame_content_valid(buffer, len);
    }

    return true;
}

int IRAM_ATTR choose_capture_slot(frame_context_t *ctx)
{
    const int locked_slot = ctx->uvc_locked_slot.load(std::memory_order_acquire);
    const int latest_slot = ctx->latest_slot.load(std::memory_order_acquire);
    const int dma_slot = ctx->dma_inflight_slot.load(std::memory_order_acquire);

    for (int i = 0; i < static_cast<int>(CAMERA_FRAME_SLOT_COUNT); ++i) {
        if (i != locked_slot && i != latest_slot && i != dma_slot) {
            return i;
        }
    }

    /* Do not recycle latest_slot just to keep capture running.
     * If USB is sending one slot, DMA owns another, and the third slot holds
     * the newest valid JPEG, overwriting latest_slot can leave UVC with
     * frame_count advanced but no frame to send (latest_slot = -1). It is
     * safer to drop this capture opportunity and keep the last valid frame
     * available for normal or repeated UVC transfers. */
    return -1;
}

bool any_uvc_profile_needs_transfer_buffer(void)
{
    for (size_t i = 0; i < UVC_FRAME_PROFILE_COUNT; ++i) {
        if (s_uvc_profile_bindings[i].available && s_uvc_profile_bindings[i].conversion != uvc_frame_conversion_t::none) {
            return true;
        }
    }

    return false;
}

cam_ctlr_color_t map_cam_color(const esp_cam_sensor_output_format_t format)
{
    switch (format) {
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565:
    case ESP_CAM_SENSOR_PIXFORMAT_RGB565_BE:
        return CAM_CTLR_COLOR_RGB565;
    case ESP_CAM_SENSOR_PIXFORMAT_GRAYSCALE:
        return CAM_CTLR_COLOR_GRAY8;
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422:
    case ESP_CAM_SENSOR_PIXFORMAT_YUV422_YUYV:
    default:
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
        return CAM_CTLR_COLOR_YUV422_YUYV;
#else
        return CAM_CTLR_COLOR_YUV422;
#endif
    }
}

bool IRAM_ATTR on_get_new_trans(esp_cam_ctlr_handle_t /*handle*/, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    /* Called by the DVP driver when it needs a DMA target buffer. Keep this
     * path short and avoid touching buffers currently owned by USB transfer.
     */
    frame_context_t *ctx = reinterpret_cast<frame_context_t *>(user_data);
    if (!ctx) {
        return false;
    }

    if (s_mjpeg_sw_capture_hold.load(std::memory_order_acquire)) {
        return false;
    }

    const int slot = choose_capture_slot(ctx);
    if (slot < 0) {
        s_capture_buffer_miss_count.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    frame_slot_t &frame_slot = ctx->slots[slot];
    if (!frame_slot.buffer || (frame_slot.buffer_size == 0)) {
        s_capture_buffer_miss_count.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    frame_slot.frame_size.store(0, std::memory_order_relaxed);
    ctx->dma_inflight_slot.store(slot, std::memory_order_release);
    trans->buffer = frame_slot.buffer;
    trans->buflen = frame_slot.buffer_size;
    const size_t active_limit = s_active_capture_buffer_limit.load(std::memory_order_relaxed);
    if (active_limit > 0 && active_limit < trans->buflen) {
        trans->buflen = active_limit;
    }
    return true;
}

bool IRAM_ATTR on_trans_finished(esp_cam_ctlr_handle_t /*handle*/, esp_cam_ctlr_trans_t *trans, void *user_data)
{
    /* Marks the just-filled DMA slot as the newest complete frame. USB will
     * lock this slot later if it sends the frame directly without conversion.
     */
    frame_context_t *ctx = reinterpret_cast<frame_context_t *>(user_data);
    if (!ctx || !trans || !trans->buffer) {
        return false;
    }

    int slot = -1;
    for (int i = 0; i < static_cast<int>(CAMERA_FRAME_SLOT_COUNT); ++i) {
        if (ctx->slots[i].buffer == trans->buffer) {
            slot = i;
            break;
        }
    }
    if (slot < 0) {
        return false;
    }

    if (ctx->dma_inflight_slot.load(std::memory_order_acquire) == slot) {
        ctx->dma_inflight_slot.store(-1, std::memory_order_release);
    }

    if (trans->received_size == 0 || trans->received_size > ctx->slots[slot].buffer_size) {
        ctx->slots[slot].frame_size.store(0, std::memory_order_release);
        s_invalid_frame_count.fetch_add(1, std::memory_order_relaxed);
        s_last_frame_size.store(0, std::memory_order_relaxed);
        s_last_dma_window_size.store(trans->buflen, std::memory_order_relaxed);
        s_last_frame_slot.store(slot, std::memory_order_relaxed);
        return false;
    }

    ctx->slots[slot].frame_size.store(trans->received_size, std::memory_order_release);
    ctx->latest_slot.store(slot, std::memory_order_release);
    s_frame_count.fetch_add(1, std::memory_order_relaxed);
    s_last_frame_size.store(trans->received_size, std::memory_order_relaxed);
    s_last_dma_window_size.store(trans->buflen, std::memory_order_relaxed);
    s_last_frame_slot.store(slot, std::memory_order_relaxed);
    if (s_active_profile_is_mjpeg_sw.load(std::memory_order_acquire)) {
        s_mjpeg_sw_capture_hold.store(true, std::memory_order_release);
    }
    return true;
}

void convert_uyvy_to_yuy2(uint8_t *dst, const uint8_t *src, size_t len)
{
    for (size_t i = 0; i + 3 < len; i += 4) {
        dst[i + 0] = src[i + 1];
        dst[i + 1] = src[i + 0];
        dst[i + 2] = src[i + 3];
        dst[i + 3] = src[i + 2];
    }
}

uint8_t clamp_to_u8(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 255) {
        return 255;
    }
    return static_cast<uint8_t>(value);
}

struct yuv_sample_t {
    uint8_t luma;
    uint8_t chroma_u;
    uint8_t chroma_v;
};

yuv_sample_t rgb_to_yuv(uint8_t red, uint8_t green, uint8_t blue)
{
    yuv_sample_t sample{};
    sample.luma = clamp_to_u8(((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
    sample.chroma_u = clamp_to_u8(((-38 * red - 74 * green + 112 * blue + 128) >> 8) + 128);
    sample.chroma_v = clamp_to_u8(((112 * red - 94 * green - 18 * blue + 128) >> 8) + 128);
    return sample;
}

yuv_sample_t rgb565_pixel_to_yuv(const uint8_t *pixel, bool big_endian)
{
    const uint16_t raw_pixel = big_endian
        ? static_cast<uint16_t>((static_cast<uint16_t>(pixel[0]) << 8) | pixel[1])
        : static_cast<uint16_t>((static_cast<uint16_t>(pixel[1]) << 8) | pixel[0]);
    const uint8_t red = static_cast<uint8_t>(((raw_pixel >> 11) & 0x1f) * 255 / 31);
    const uint8_t green = static_cast<uint8_t>(((raw_pixel >> 5) & 0x3f) * 255 / 63);
    const uint8_t blue = static_cast<uint8_t>((raw_pixel & 0x1f) * 255 / 31);
    return rgb_to_yuv(red, green, blue);
}

void convert_rgb565_to_yuy2(uint8_t *dst, const uint8_t *src, size_t pixel_count, bool big_endian)
{
    for (size_t pixel_index = 0, output_index = 0; pixel_index < pixel_count; pixel_index += 2, output_index += 4) {
        const yuv_sample_t first = rgb565_pixel_to_yuv(&src[pixel_index * 2], big_endian);
        const yuv_sample_t second = rgb565_pixel_to_yuv(&src[(pixel_index + 1) * 2], big_endian);
        dst[output_index + 0] = first.luma;
        dst[output_index + 1] = static_cast<uint8_t>((static_cast<uint16_t>(first.chroma_u) + second.chroma_u) / 2);
        dst[output_index + 2] = second.luma;
        dst[output_index + 3] = static_cast<uint8_t>((static_cast<uint16_t>(first.chroma_v) + second.chroma_v) / 2);
    }
}

void convert_grayscale_to_yuy2(uint8_t *dst, const uint8_t *src, size_t pixel_count)
{
    for (size_t pixel_index = 0, output_index = 0; pixel_index < pixel_count; pixel_index += 2, output_index += 4) {
        dst[output_index + 0] = src[pixel_index];
        dst[output_index + 1] = 128;
        dst[output_index + 2] = src[pixel_index + 1];
        dst[output_index + 3] = 128;
    }
}

void convert_rgb565_be_to_rgb565(uint8_t *dst, const uint8_t *src, size_t pixel_count)
{
    for (size_t i = 0; i < pixel_count; ++i) {
        dst[i * 2 + 0] = src[i * 2 + 1];
        dst[i * 2 + 1] = src[i * 2 + 0];
    }
}

bool convert_frame_for_uvc(uvc_frame_conversion_t conversion,
                                  const esp_cam_sensor_format_t &sensor_fmt,
                                  uint8_t *dst,
                                  size_t dst_len,
                                  const uint8_t *src,
                                  size_t src_len,
                                  size_t *written_len)
{
    if (dst == nullptr || src == nullptr || written_len == nullptr || sensor_fmt.width == 0 || sensor_fmt.height == 0) {
        return false;
    }

    if (conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
        return encode_raw_frame_to_mjpeg(sensor_fmt, dst, dst_len, src, src_len, written_len);
    }

    const size_t pixel_count = static_cast<size_t>(sensor_fmt.width) * sensor_fmt.height;
    if (pixel_count == 0 || (pixel_count % 2) != 0) {
        return false;
    }

    const size_t output_len = pixel_count * 2;
    if (dst_len < output_len) {
        return false;
    }

    switch (conversion) {
    case uvc_frame_conversion_t::uyvy_to_yuy2:
        if (src_len < output_len) {
            return false;
        }
        convert_uyvy_to_yuy2(dst, src, output_len);
        break;
    case uvc_frame_conversion_t::rgb565_to_yuy2:
    case uvc_frame_conversion_t::rgb565_be_to_yuy2:
        if (src_len < pixel_count * 2) {
            return false;
        }
        convert_rgb565_to_yuy2(dst, src, pixel_count, conversion == uvc_frame_conversion_t::rgb565_be_to_yuy2);
        break;
    case uvc_frame_conversion_t::grayscale_to_yuy2:
        if (src_len < pixel_count) {
            return false;
        }
        convert_grayscale_to_yuy2(dst, src, pixel_count);
        break;
    case uvc_frame_conversion_t::rgb565_be_to_rgb565:
        if (src_len < output_len) {
            return false;
        }
        convert_rgb565_be_to_rgb565(dst, src, pixel_count);
        break;
    case uvc_frame_conversion_t::none:
    default:
        return false;
    }

    *written_len = output_len;
    return true;
}

bool prepare_latest_frame_for_uvc_xfer(const tinyusb_uvc_context_t &ctx,
                                              uint8_t **frame_buffer,
                                              size_t *frame_len,
                                              bool *slot_locked_for_xfer,
                                              uvc_copy_result_t *copy_result)
{
    /* Produce the buffer TinyUSB should send next.
     *
     * Direct paths return a locked capture slot. Conversion paths copy/encode
     * into s_uvc_transfer_buffer and release the capture slot before returning.
     */
    auto finish = [&](uvc_copy_result_t result) {
        if (copy_result != nullptr) {
            *copy_result = result;
        }
        return result == uvc_copy_result_t::ok;
    };

    if (!frame_buffer || !frame_len || !slot_locked_for_xfer) {
        return finish(uvc_copy_result_t::invalid_arg);
    }

    *frame_buffer = nullptr;
    *frame_len = 0;
    *slot_locked_for_xfer = false;

    const int latest_slot = s_frame_ctx.latest_slot.load(std::memory_order_acquire);
    if (latest_slot < 0 || latest_slot >= static_cast<int>(CAMERA_FRAME_SLOT_COUNT)) {
        return finish(uvc_copy_result_t::no_frame);
    }

    int expected = -1;
    if (!s_frame_ctx.uvc_locked_slot.compare_exchange_strong(expected, latest_slot, std::memory_order_acq_rel)) {
        return finish(uvc_copy_result_t::slot_locked);
    }

    auto unlock = [&]() {
        s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_release);
    };

    if (s_frame_ctx.latest_slot.load(std::memory_order_acquire) != latest_slot
        || s_frame_ctx.dma_inflight_slot.load(std::memory_order_acquire) == latest_slot) {
        unlock();
        return finish(uvc_copy_result_t::no_frame);
    }

    const size_t len = s_frame_ctx.slots[latest_slot].frame_size.load(std::memory_order_acquire);
    if (len == 0 || len > s_frame_ctx.slots[latest_slot].buffer_size) {
        unlock();
        if (ctx.conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
            s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);
        }
        return finish(uvc_copy_result_t::invalid_frame_size);
    }

    uint8_t *slot_buffer = s_frame_ctx.slots[latest_slot].buffer;
    if (!frame_content_valid_for_uvc(ctx, slot_buffer, len)) {
        unlock();
        if (ctx.conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
            s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);
        }
        return finish(uvc_copy_result_t::invalid_frame_content);
    }

    if (ctx.conversion == uvc_frame_conversion_t::none) {
        *frame_buffer = slot_buffer;
        *frame_len = len;
        *slot_locked_for_xfer = true;
        return finish(uvc_copy_result_t::ok);
    }

    if (!s_uvc_transfer_buffer) {
        unlock();
        if (ctx.conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
            s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);
        }
        return finish(uvc_copy_result_t::invalid_frame_size);
    }

    if (ctx.conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
        size_t jpeg_input_size = 0;
        if (!mjpeg_sw_encoder_config_for_sensor_format(ctx.sensor_fmt, nullptr, nullptr, &jpeg_input_size)
            || jpeg_input_size == 0
            || len < jpeg_input_size) {
            unlock();
            s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);
            return finish(uvc_copy_result_t::invalid_frame_size);
        }

        size_t encoded_len = 0;
        if (!convert_frame_for_uvc(ctx.conversion,
                                   ctx.sensor_fmt,
                                   s_uvc_transfer_buffer,
                                   s_uvc_transfer_buffer_size,
                                   slot_buffer,
                                   jpeg_input_size,
                                   &encoded_len)) {
            unlock();
            s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);
            return finish(uvc_copy_result_t::invalid_frame_size);
        }
        unlock();
        s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);

        *frame_buffer = s_uvc_transfer_buffer;
        *frame_len = encoded_len;
        return finish(uvc_copy_result_t::ok);
    }

    size_t converted_len = 0;
    if (!convert_frame_for_uvc(ctx.conversion,
                               ctx.sensor_fmt,
                               s_uvc_transfer_buffer,
                               s_uvc_transfer_buffer_size,
                               slot_buffer,
                               len,
                               &converted_len)) {
        unlock();
        return finish(uvc_copy_result_t::invalid_frame_size);
    }
    unlock();
    *frame_buffer = s_uvc_transfer_buffer;
    *frame_len = converted_len;
    return finish(uvc_copy_result_t::ok);
}

void stop_camera_pipeline(esp_cam_sensor_device_t *sensor, esp_cam_ctlr_handle_t *cam_ctlr)
{
    if (sensor != nullptr) {
        int stream = 0;
        esp_cam_sensor_ioctl(sensor, ESP_CAM_SENSOR_IOC_S_STREAM, &stream);
    }

    if (cam_ctlr != nullptr && *cam_ctlr != nullptr) {
        esp_cam_ctlr_stop(*cam_ctlr);
        esp_cam_ctlr_disable(*cam_ctlr);
        esp_cam_ctlr_del(*cam_ctlr);
        *cam_ctlr = nullptr;
    }
}

esp_err_t create_camera_controller(const esp_cam_sensor_format_t &sensor_fmt,
                                          const esp_cam_ctlr_dvp_pin_config_t &dvp_pins,
                                          esp_cam_ctlr_handle_t *cam_ctlr)
{
    esp_cam_ctlr_dvp_config_t dvp_cfg{};
    esp_cam_ctlr_evt_cbs_t cam_cbs{};

    ESP_RETURN_ON_FALSE(cam_ctlr != nullptr, ESP_ERR_INVALID_ARG, TAG, "cam_ctlr is null");

    dvp_cfg.ctlr_id = 0;
    dvp_cfg.clk_src = CAM_CLK_SRC_DEFAULT;
    dvp_cfg.h_res = sensor_fmt.width;
    dvp_cfg.v_res = sensor_fmt.height;
    dvp_cfg.input_data_color_type = map_cam_color(sensor_fmt.format);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(6, 0, 0)
    dvp_cfg.output_data_color_type = map_cam_color(sensor_fmt.format);
#endif
    dvp_cfg.cam_data_width = CAM_CTLR_DATA_WIDTH_8;
    dvp_cfg.dma_burst_size = 0;
    dvp_cfg.pin = &dvp_pins;
    dvp_cfg.external_xtal = 1;
    dvp_cfg.xclk_freq = (sensor_fmt.xclk > 0) ? static_cast<uint32_t>(sensor_fmt.xclk) : OV3660_XCLK_FREQ_HZ;
    dvp_cfg.pic_format_jpeg = (sensor_fmt.format == ESP_CAM_SENSOR_PIXFORMAT_JPEG);
    dvp_cfg.byte_swap_en = 0;
    dvp_cfg.bk_buffer_dis = 0;
    dvp_cfg.pin_dont_init = 0;

    esp_err_t err = esp_cam_new_dvp_ctlr_ext(&dvp_cfg, cam_ctlr);
    if (err != ESP_OK) {
        return err;
    }

    cam_cbs.on_get_new_trans = on_get_new_trans;
    cam_cbs.on_trans_finished = on_trans_finished;
    err = esp_cam_ctlr_register_event_callbacks(*cam_ctlr, &cam_cbs, &s_frame_ctx);
    if (err != ESP_OK) {
        esp_cam_ctlr_del(*cam_ctlr);
        *cam_ctlr = nullptr;
        return err;
    }

    err = esp_cam_ctlr_enable(*cam_ctlr);
    if (err != ESP_OK) {
        esp_cam_ctlr_del(*cam_ctlr);
        *cam_ctlr = nullptr;
        return err;
    }

    return ESP_OK;
}

esp_err_t reconfigure_camera_pipeline(esp_cam_sensor_device_t *sensor,
                                             esp_cam_ctlr_handle_t *cam_ctlr,
                                             const esp_cam_ctlr_dvp_pin_config_t &dvp_pins,
                                             int profile_index,
                                             esp_cam_sensor_format_t *sensor_fmt)
{
    /* Host mode switches are handled in the camera task, not inside the USB
     * callback. This function stops capture, applies the new sensor format,
     * recreates the DVP controller, and restarts streaming as one operation.
     */
    ESP_RETURN_ON_FALSE(sensor != nullptr, ESP_ERR_INVALID_ARG, TAG, "sensor is null");
    ESP_RETURN_ON_FALSE(cam_ctlr != nullptr, ESP_ERR_INVALID_ARG, TAG, "cam_ctlr is null");
    ESP_RETURN_ON_FALSE(sensor_fmt != nullptr, ESP_ERR_INVALID_ARG, TAG, "sensor_fmt is null");
    ESP_RETURN_ON_FALSE(profile_index >= 0 && profile_index < static_cast<int>(UVC_FRAME_PROFILE_COUNT),
                        ESP_ERR_INVALID_ARG,
                        TAG,
                        "profile index out of range");
    ESP_RETURN_ON_FALSE(s_uvc_profile_bindings[profile_index].available,
                        ESP_ERR_NOT_SUPPORTED,
                        TAG,
                        "requested UVC profile unavailable");

    const uvc_profile_binding_t &binding = s_uvc_profile_bindings[profile_index];

    s_uvc_reconfig_in_progress.store(true, std::memory_order_release);
    s_uvc_tx_busy.store(false, std::memory_order_release);
    s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
    s_active_profile_is_mjpeg_sw.store(false, std::memory_order_release);
    s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);

    stop_camera_pipeline(sensor, cam_ctlr);
    reset_frame_context();

    esp_err_t err = esp_cam_sensor_set_format(sensor, binding.sensor_fmt);
    if (err != ESP_OK) {
        s_uvc_reconfig_in_progress.store(false, std::memory_order_release);
        return err;
    }

    err = esp_cam_sensor_get_format(sensor, sensor_fmt);
    if (err != ESP_OK) {
        s_uvc_reconfig_in_progress.store(false, std::memory_order_release);
        return err;
    }

    if (binding.conversion == uvc_frame_conversion_t::raw_to_mjpeg) {
        if (!ensure_mjpeg_sw_encoder(*sensor_fmt)) {
            s_uvc_reconfig_in_progress.store(false, std::memory_order_release);
            return ESP_ERR_NOT_SUPPORTED;
        }
    } else {
        close_mjpeg_sw_encoder();
    }

    const size_t capture_size = estimate_frame_buffer_size(*sensor_fmt);
    const size_t uvc_frame_size = estimate_uvc_frame_buffer_size(*sensor_fmt,
                                                                 binding.profile->payload_format,
                                                                 binding.conversion);
    size_t jpeg_input_size = 0;
    if (binding.conversion == uvc_frame_conversion_t::raw_to_mjpeg
        && !mjpeg_sw_encoder_config_for_sensor_format(*sensor_fmt, nullptr, nullptr, &jpeg_input_size)) {
        s_uvc_reconfig_in_progress.store(false, std::memory_order_release);
        return ESP_ERR_INVALID_SIZE;
    }
    if (capture_size == 0 || capture_size > get_max_uvc_capture_buffer_size()
        || (binding.conversion != uvc_frame_conversion_t::none && uvc_frame_size > s_uvc_transfer_buffer_size)
        || (binding.conversion == uvc_frame_conversion_t::raw_to_mjpeg && jpeg_input_size == 0)) {
        s_uvc_reconfig_in_progress.store(false, std::memory_order_release);
        return ESP_ERR_INVALID_SIZE;
    }
    s_active_capture_buffer_limit.store(capture_size, std::memory_order_release);

    err = create_camera_controller(*sensor_fmt, dvp_pins, cam_ctlr);
    if (err != ESP_OK) {
        s_uvc_reconfig_in_progress.store(false, std::memory_order_release);
        return err;
    }

    err = start_camera_capture(sensor, *cam_ctlr);
    if (err != ESP_OK) {
        stop_camera_pipeline(sensor, cam_ctlr);
        s_uvc_reconfig_in_progress.store(false, std::memory_order_release);
        return err;
    }

    s_uvc_camera_ctx.sensor_fmt = *sensor_fmt;
    s_uvc_camera_ctx.conversion = binding.conversion;
    s_active_profile_is_mjpeg_sw.store(binding.conversion == uvc_frame_conversion_t::raw_to_mjpeg,
                                            std::memory_order_release);
    s_mjpeg_sw_capture_hold.store(false, std::memory_order_release);
    s_uvc_frame_interval_us.store(binding.profile->frame_interval_us, std::memory_order_relaxed);
    s_uvc_last_frame_us.store(0, std::memory_order_relaxed);
    s_uvc_active_profile_index.store(profile_index, std::memory_order_release);
    s_uvc_pending_profile_index.store(-1, std::memory_order_release);
    s_uvc_repeat_send_count.store(0, std::memory_order_relaxed);
    s_uvc_reconfig_in_progress.store(false, std::memory_order_release);

    return ESP_OK;
}

/* 启动期对每个 UVC mode 都做一次抓帧自检：
 *   - 切换 sensor 格式 + 重建 DVP 控制器；
 *   - 给定窗口内最多重试 3 次，期间统计 frames/invalid/buf_miss；
 *   - 不论成功与否都继续，UVC 描述符依旧暴露所有候选项。
 * 该过程同时充当原先的 USB Serial/JTAG 宽限期，便于用户从串口看到日志。 */
void run_capture_self_test(esp_cam_sensor_device_t *sensor,
                                  esp_cam_ctlr_handle_t *cam_ctlr,
                                  const esp_cam_ctlr_dvp_pin_config_t &dvp_pins,
                                  esp_cam_sensor_format_t *sensor_fmt)
{
    constexpr uint32_t SELF_TEST_RETRIES = 3;
    constexpr TickType_t SELF_TEST_PER_RETRY_TICKS = pdMS_TO_TICKS(500);

    size_t mode_total = 0;
    for (size_t i = 0; i < UVC_FRAME_PROFILE_COUNT; ++i) {
        if (s_uvc_profile_bindings[i].available) {
            ++mode_total;
        }
    }

    ESP_LOGI(TAG, "self-test: probing %u UVC mode(s) with up to %u retries (each %u ms) before TinyUSB UVC starts",
             static_cast<unsigned>(mode_total),
             static_cast<unsigned>(SELF_TEST_RETRIES),
             static_cast<unsigned>(pdTICKS_TO_MS(SELF_TEST_PER_RETRY_TICKS)));

    size_t passed = 0;
    size_t failed = 0;

    for (size_t i = 0; i < UVC_FRAME_PROFILE_COUNT; ++i) {
        const uvc_profile_binding_t &binding = s_uvc_profile_bindings[i];
        if (!binding.available || binding.profile == nullptr || binding.sensor_fmt == nullptr) {
            continue;
        }

        ESP_LOGI(TAG,
                 "self-test mode[%u] -> %s %s %ux%u sensor_fps=%u conversion=%s capture_buf=%u",
                 static_cast<unsigned>(i),
                 uvc_profile_payload_name(binding),
                 binding.sensor_fmt->name ? binding.sensor_fmt->name : "?",
                 binding.profile->width,
                 binding.profile->height,
                 binding.profile->sensor_fps,
                 uvc_frame_conversion_name(binding.conversion),
                 static_cast<unsigned>(binding.profile->capture_buffer_size));

        bool got_frame = false;
        for (uint32_t attempt = 1; attempt <= SELF_TEST_RETRIES && !got_frame; ++attempt) {
            esp_err_t err = reconfigure_camera_pipeline(sensor, cam_ctlr, dvp_pins, static_cast<int>(i), sensor_fmt);
            if (err != ESP_OK) {
                ESP_LOGW(TAG,
                         "self-test mode[%u] attempt %u: reconfigure failed: %s",
                         static_cast<unsigned>(i),
                         static_cast<unsigned>(attempt),
                         esp_err_to_name(err));
                continue;
            }

            const uint64_t frames_before = s_frame_count.load(std::memory_order_relaxed);
            const uint64_t invalid_before = s_invalid_frame_count.load(std::memory_order_relaxed);
            const uint64_t miss_before = s_capture_buffer_miss_count.load(std::memory_order_relaxed);

            vTaskDelay(SELF_TEST_PER_RETRY_TICKS);

            const uint64_t frames_after = s_frame_count.load(std::memory_order_relaxed);
            const uint64_t invalid_after = s_invalid_frame_count.load(std::memory_order_relaxed);
            const uint64_t miss_after = s_capture_buffer_miss_count.load(std::memory_order_relaxed);
            const size_t last_size = s_last_frame_size.load(std::memory_order_relaxed);
            const int latest_slot = s_frame_ctx.latest_slot.load(std::memory_order_relaxed);
            const int dma_slot = s_frame_ctx.dma_inflight_slot.load(std::memory_order_relaxed);
            const uint64_t got = frames_after - frames_before;

            if (got > 0) {
                got_frame = true;
                /* 抓一帧出来读取头几个字节做指纹，便于判断是不是噪声/全 0 */
                tinyusb_uvc_context_t probe_ctx{};
                probe_ctx.sensor_fmt = *sensor_fmt;
                probe_ctx.conversion = binding.conversion;
                uint8_t *frame_buffer = nullptr;
                size_t frame_len = 0;
                bool slot_locked = false;
                uvc_copy_result_t copy_result = uvc_copy_result_t::ok;
                const bool snapshot_ok = prepare_latest_frame_for_uvc_xfer(probe_ctx, &frame_buffer, &frame_len, &slot_locked, &copy_result);

                uint32_t hash = 2166136261u;
                size_t hash_len = 0;
                uint8_t b0 = 0, b1 = 0, b2 = 0, b3 = 0, b4 = 0, b5 = 0, b6 = 0, b7 = 0;
                if (snapshot_ok && frame_buffer != nullptr) {
                    hash_len = (frame_len < FRAME_LOG_HASH_BYTES) ? frame_len : FRAME_LOG_HASH_BYTES;
                    for (size_t k = 0; k < hash_len; ++k) {
                        hash = (hash ^ frame_buffer[k]) * 16777619u;
                    }
                    if (frame_len > 0) { b0 = frame_buffer[0]; }
                    if (frame_len > 1) { b1 = frame_buffer[1]; }
                    if (frame_len > 2) { b2 = frame_buffer[2]; }
                    if (frame_len > 3) { b3 = frame_buffer[3]; }
                    if (frame_len > 4) { b4 = frame_buffer[4]; }
                    if (frame_len > 5) { b5 = frame_buffer[5]; }
                    if (frame_len > 6) { b6 = frame_buffer[6]; }
                    if (frame_len > 7) { b7 = frame_buffer[7]; }
                }
                if (slot_locked) {
                    s_frame_ctx.uvc_locked_slot.store(-1, std::memory_order_release);
                }
                ESP_LOGI(TAG,
                         "self-test mode[%u] attempt %u OK: frames=%llu last_size=%u snapshot=%s len=%u hash%u=0x%08" PRIX32
                         " first8=%02X %02X %02X %02X %02X %02X %02X %02X invalid=%llu buf_miss=%llu",
                         static_cast<unsigned>(i),
                         static_cast<unsigned>(attempt),
                         static_cast<unsigned long long>(got),
                         static_cast<unsigned>(last_size),
                         uvc_copy_result_name(copy_result),
                         static_cast<unsigned>(frame_len),
                         static_cast<unsigned>(hash_len),
                         hash,
                         b0, b1, b2, b3, b4, b5, b6, b7,
                         static_cast<unsigned long long>(invalid_after - invalid_before),
                         static_cast<unsigned long long>(miss_after - miss_before));
            } else {
                ESP_LOGW(TAG,
                         "self-test mode[%u] attempt %u FAIL: no frame in %u ms (invalid=%llu buf_miss=%llu latest_slot=%d dma_slot=%d last_size=%u)",
                         static_cast<unsigned>(i),
                         static_cast<unsigned>(attempt),
                         static_cast<unsigned>(pdTICKS_TO_MS(SELF_TEST_PER_RETRY_TICKS)),
                         static_cast<unsigned long long>(invalid_after - invalid_before),
                         static_cast<unsigned long long>(miss_after - miss_before),
                         latest_slot,
                         dma_slot,
                         static_cast<unsigned>(last_size));
            }
        }

        if (got_frame) {
            ++passed;
        } else {
            ++failed;
            ESP_LOGE(TAG,
                     "self-test mode[%u] FAILED after %u retries (UVC will still advertise this mode)",
                     static_cast<unsigned>(i),
                     static_cast<unsigned>(SELF_TEST_RETRIES));
        }
    }

    ESP_LOGI(TAG, "self-test summary: %u passed, %u failed (advertising all %u modes via UVC regardless)",
             static_cast<unsigned>(passed),
             static_cast<unsigned>(failed),
             static_cast<unsigned>(mode_total));
}
