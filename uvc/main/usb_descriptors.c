#include <string.h>

#include "tusb.h"
#include "usb_descriptors.h"

#define USB_VID   0xCafe
#define USB_PID   0x4032
#define USB_BCD   0x0200

#define UVC_CLOCK_FREQUENCY            27000000UL
#define UVC_ENTITY_CAP_INPUT_TERMINAL  UVC_ENTITY_ID_CAMERA_TERMINAL
#define UVC_ENTITY_CAP_OUTPUT_TERMINAL UVC_ENTITY_ID_OUTPUT_TERMINAL
#define UVC_CONFIGURATION_BUFFER_SIZE  1024

/* RGBP = RGB 16-bit 5:6:5 (little-endian), UVC uncompressed GUID based on FOURCC convention */
#define TUD_VIDEO_GUID_RGBP \
    0x52,0x47,0x42,0x50,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0xAA,0x00,0x38,0x9B,0x71

/* UYVY/Y800 use standard FOURCC-based UVC uncompressed GUID encoding. */
#define TUD_VIDEO_GUID_UYVY \
    0x55,0x59,0x56,0x59,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0xAA,0x00,0x38,0x9B,0x71
#define TUD_VIDEO_GUID_Y800 \
    0x59,0x38,0x30,0x30,0x00,0x00,0x10,0x00,0x80,0x00,0x00,0xAA,0x00,0x38,0x9B,0x71

typedef struct TU_ATTR_PACKED {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bDescriptorSubType;
    uint8_t bNumFormats;
    uint16_t wTotalLength;
    uint8_t bEndpointAddress;
    uint8_t bmInfo;
    uint8_t bTerminalLink;
    uint8_t bStillCaptureMethod;
    uint8_t bTriggerSupport;
    uint8_t bTriggerUsage;
    uint8_t bControlSize;
    uint8_t bmaControls[UVC_DESCRIPTOR_MAX_FORMAT_COUNT];
} runtime_vs_input_header_t;

#define UVC_VC_PU_DESC_LENGTH  0x0C
#define UVC_VC_XU_DESC_LENGTH  0x1A

#define UVC_PU_DEFAULT_BMCONTROLS \
    ((uint16_t)((1U << (UVC_PU_CTRL_CONTRAST - 1)) | (1U << (UVC_PU_CTRL_SATURATION - 1))))
#define UVC_XU_DEFAULT_BMCONTROLS  0x7FU

static uint16_t s_runtime_pu_bm_controls = UVC_PU_DEFAULT_BMCONTROLS;
static uint8_t s_runtime_xu_bm_controls = UVC_XU_DEFAULT_BMCONTROLS;

enum {
  STRID_LANGID = 0,
  STRID_MANUFACTURER,
  STRID_PRODUCT,
  STRID_SERIAL,
  STRID_UVC_CONTROL,
  STRID_UVC_STREAMING,
};

enum {
  ITF_NUM_VIDEO_CONTROL,
  ITF_NUM_VIDEO_STREAMING,
  ITF_NUM_TOTAL,
};

static const char *string_desc_arr[] = {
    (const char[]) {0x09, 0x04},
    "M5Stack",
    "Stamp Cam TinyUSB UVC",
    "CAM3660-UVC9",
    "UVC Control",
    "UVC Streaming",
};

static const tusb_desc_device_t desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = USB_BCD,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = USB_VID,
    .idProduct = USB_PID,
    .bcdDevice = 0x010A,
    .iManufacturer = STRID_MANUFACTURER,
    .iProduct = STRID_PRODUCT,
    .iSerialNumber = STRID_SERIAL,
    .bNumConfigurations = 0x01,
};

static const uvc_descriptor_config_t s_default_config = {
    .payload_format = UVC_DESCRIPTOR_PAYLOAD_MJPEG,
    .format_index = 1,
    .format_count = 2,
    .frame_count = 3,
    .default_format_index = 1,
    .default_frame_index = 1,
    .frames = {
        { UVC_DESCRIPTOR_PAYLOAD_MJPEG, 1, 1, 1280, 720, 12, 833333UL, 1280UL * 720UL },
        { UVC_DESCRIPTOR_PAYLOAD_YUY2, 2, 1, 640, 480, 1, 10000000UL, 640UL * 480UL * 2UL },
        { UVC_DESCRIPTOR_PAYLOAD_YUY2, 2, 2, 240, 240, 7, 1428571UL, 240UL * 240UL * 2UL },
    },
};

static uvc_descriptor_config_t s_active_config = {
    .payload_format = UVC_DESCRIPTOR_PAYLOAD_MJPEG,
    .format_index = 1,
    .format_count = 2,
    .frame_count = 3,
    .default_format_index = 1,
    .default_frame_index = 1,
    .frames = {
        { UVC_DESCRIPTOR_PAYLOAD_MJPEG, 1, 1, 1280, 720, 12, 833333UL, 1280UL * 720UL },
        { UVC_DESCRIPTOR_PAYLOAD_YUY2, 2, 1, 640, 480, 1, 10000000UL, 640UL * 480UL * 2UL },
        { UVC_DESCRIPTOR_PAYLOAD_YUY2, 2, 2, 240, 240, 7, 1428571UL, 240UL * 240UL * 2UL },
    },
};

static uint8_t s_desc_fs_configuration[UVC_CONFIGURATION_BUFFER_SIZE];
static uint16_t s_desc_fs_configuration_len = 0;
static uint16_t desc_str[32 + 1];

void uvc_descriptors_set_control_masks(uint16_t pu_bm_controls, uint8_t xu_bm_controls)
{
    s_runtime_pu_bm_controls = pu_bm_controls;
    s_runtime_xu_bm_controls = xu_bm_controls;
}

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &desc_device;
}

static bool append_descriptor(size_t *offset, const void *descriptor, size_t descriptor_len)
{
    if ((*offset + descriptor_len) > sizeof(s_desc_fs_configuration)) {
        return false;
    }

    memcpy(&s_desc_fs_configuration[*offset], descriptor, descriptor_len);
    *offset += descriptor_len;
    return true;
}

static uint32_t calc_frame_bits_per_second(const uvc_descriptor_frame_t *frame)
{
    uint32_t frame_rate = frame->frame_rate > 0 ? frame->frame_rate : 1;
    return frame->frame_buffer_size * 8UL * frame_rate;
}

static uint8_t count_frames_for_format(const uvc_descriptor_config_t *config, uint8_t format_index)
{
    uint8_t count = 0;
    for (uint8_t i = 0; i < config->frame_count; ++i) {
        if (config->frames[i].format_index == format_index) {
            ++count;
        }
    }
    return count;
}

static bool format_has_mjpeg_sw_frame(const uvc_descriptor_config_t *config, uint8_t format_index)
{
    for (uint8_t i = 0; i < config->frame_count; ++i) {
        if (config->frames[i].format_index == format_index
            && config->frames[i].payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG
            && config->frames[i].mjpeg_sw) {
            return true;
        }
    }
    return false;
}

static uvc_descriptor_payload_t payload_for_format(const uvc_descriptor_config_t *config, uint8_t format_index)
{
    for (uint8_t i = 0; i < config->frame_count; ++i) {
        if (config->frames[i].format_index == format_index) {
            return config->frames[i].payload_format;
        }
    }

    return UVC_DESCRIPTOR_PAYLOAD_YUY2;
}

static uint8_t default_frame_index_for_format(const uvc_descriptor_config_t *config, uint8_t format_index)
{
    if (format_index == config->default_format_index) {
        return config->default_frame_index;
    }

    for (uint8_t i = 0; i < config->frame_count; ++i) {
        if (config->frames[i].format_index == format_index) {
            return config->frames[i].frame_index;
        }
    }

    return 1;
}

static uint8_t payload_bits_per_pixel(uvc_descriptor_payload_t payload_format)
{
    switch (payload_format) {
    case UVC_DESCRIPTOR_PAYLOAD_GRAY8:
        return 8;
    case UVC_DESCRIPTOR_PAYLOAD_MJPEG:
        return 0;
    case UVC_DESCRIPTOR_PAYLOAD_YUY2:
    case UVC_DESCRIPTOR_PAYLOAD_UYVY:
    case UVC_DESCRIPTOR_PAYLOAD_RGB565:
    default:
        return 16;
    }
}

static bool append_format_descriptor(size_t *offset,
                                     const uvc_descriptor_config_t *config,
                                     uvc_descriptor_payload_t payload_format,
                                     uint8_t format_index)
{
    const uint8_t frame_count = count_frames_for_format(config, format_index);
    if (frame_count == 0) {
        return true;
    }

    if (payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG) {
        tusb_desc_video_format_mjpeg_t format_desc = {
            .bLength = sizeof(tusb_desc_video_format_mjpeg_t),
            .bDescriptorType = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VS_FORMAT_MJPEG,
            .bFormatIndex = format_index,
            .bNumFrameDescriptors = frame_count,
            .bmFlags = format_has_mjpeg_sw_frame(config, format_index)
                ? UVC_MJPEG_FORMAT_FLAG_SW_ENCODER
                : 0,
            .bDefaultFrameIndex = default_frame_index_for_format(config, format_index),
            .bAspectRatioX = 0,
            .bAspectRatioY = 0,
            .bmInterlaceFlags = 0,
            .bCopyProtect = 0,
        };
        return append_descriptor(offset, &format_desc, sizeof(format_desc));
    }

    static const uint8_t guid_yuy2[16]  = { TUD_VIDEO_GUID_YUY2 };
    static const uint8_t guid_uyvy[16]  = { TUD_VIDEO_GUID_UYVY };
    static const uint8_t guid_y800[16]  = { TUD_VIDEO_GUID_Y800 };
    static const uint8_t guid_rgbp[16]  = { TUD_VIDEO_GUID_RGBP };
    const uint8_t *guid = guid_yuy2;

    switch (payload_format) {
    case UVC_DESCRIPTOR_PAYLOAD_UYVY:
        guid = guid_uyvy;
        break;
    case UVC_DESCRIPTOR_PAYLOAD_GRAY8:
        guid = guid_y800;
        break;
    case UVC_DESCRIPTOR_PAYLOAD_RGB565:
        guid = guid_rgbp;
        break;
    case UVC_DESCRIPTOR_PAYLOAD_YUY2:
    default:
        guid = guid_yuy2;
        break;
    }

    tusb_desc_video_format_uncompressed_t format_desc = {
        .bLength = sizeof(tusb_desc_video_format_uncompressed_t),
        .bDescriptorType = TUSB_DESC_CS_INTERFACE,
        .bDescriptorSubType = VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED,
        .bFormatIndex = format_index,
        .bNumFrameDescriptors = frame_count,
        .guidFormat = { 0 },  /* filled via memcpy below */
        .bBitsPerPixel = payload_bits_per_pixel(payload_format),
        .bDefaultFrameIndex = default_frame_index_for_format(config, format_index),
        .bAspectRatioX = 0,
        .bAspectRatioY = 0,
        .bmInterlaceFlags = 0,
        .bCopyProtect = 0,
    };
    memcpy(format_desc.guidFormat, guid, sizeof(format_desc.guidFormat));
    return append_descriptor(offset, &format_desc, sizeof(format_desc));
}

static bool append_frame_descriptor(size_t *offset, const uvc_descriptor_frame_t *frame)
{
    const uint32_t bits_per_second = calc_frame_bits_per_second(frame);

    if (frame->payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG) {
        tusb_desc_video_frame_mjpeg_1int_t frame_desc = {
            .bLength = sizeof(tusb_desc_video_frame_mjpeg_1int_t),
            .bDescriptorType = TUSB_DESC_CS_INTERFACE,
            .bDescriptorSubType = VIDEO_CS_ITF_VS_FRAME_MJPEG,
            .bFrameIndex = frame->frame_index,
            .bmCapabilities = 0,
            .wWidth = frame->width,
            .wHeight = frame->height,
            .dwMinBitRate = bits_per_second,
            .dwMaxBitRate = bits_per_second,
            .dwMaxVideoFrameBufferSize = frame->frame_buffer_size,
            .dwDefaultFrameInterval = frame->frame_interval_100ns,
            .bFrameIntervalType = 1,
            .dwFrameInterval = {
                frame->frame_interval_100ns,
            },
        };
        return append_descriptor(offset, &frame_desc, sizeof(frame_desc));
    }

    tusb_desc_video_frame_uncompressed_1int_t frame_desc = {
        .bLength = sizeof(tusb_desc_video_frame_uncompressed_1int_t),
        .bDescriptorType = TUSB_DESC_CS_INTERFACE,
        .bDescriptorSubType = VIDEO_CS_ITF_VS_FRAME_UNCOMPRESSED,
        .bFrameIndex = frame->frame_index,
        .bmCapabilities = 0,
        .wWidth = frame->width,
        .wHeight = frame->height,
        .dwMinBitRate = bits_per_second,
        .dwMaxBitRate = bits_per_second,
        .dwMaxVideoFrameBufferSize = frame->frame_buffer_size,
        .dwDefaultFrameInterval = frame->frame_interval_100ns,
        .bFrameIntervalType = 1,
        .dwFrameInterval = {
            frame->frame_interval_100ns,
        },
    };
    return append_descriptor(offset, &frame_desc, sizeof(frame_desc));
}

static bool build_configuration_descriptor(const uvc_descriptor_config_t *config)
{
    size_t offset = 0;
    uint16_t vs_cs_total_length = 0;
    const uint8_t endpoint_count = CFG_TUD_VIDEO_STREAMING_BULK ? 1 : 0;
    uint16_t format_total_length = 0;
    uint16_t frame_total_length = 0;
    uint8_t vc_pu_desc[UVC_VC_PU_DESC_LENGTH] = {
        UVC_VC_PU_DESC_LENGTH,                /* bLength */
        0x24,                                 /* bDescriptorType (CS_INTERFACE) */
        0x05,                                 /* bDescriptorSubType (VC_PROCESSING_UNIT) */
        UVC_ENTITY_ID_PROCESSING_UNIT,        /* bUnitID */
        UVC_ENTITY_ID_CAMERA_TERMINAL,        /* bSourceID -> CT */
        0x00, 0x00,                           /* wMaxMultiplier */
        0x02,                                 /* bControlSize */
        (uint8_t)(s_runtime_pu_bm_controls & 0xFF),
        (uint8_t)((s_runtime_pu_bm_controls >> 8) & 0xFF),
        0x00,                                 /* iProcessing */
        0x00,                                 /* bmVideoStandards */
    };
    uint8_t vc_xu_desc[UVC_VC_XU_DESC_LENGTH] = {
        UVC_VC_XU_DESC_LENGTH,                /* bLength = 26 */
        0x24,                                 /* bDescriptorType (CS_INTERFACE) */
        0x06,                                 /* bDescriptorSubType (VC_EXTENSION_UNIT) */
        UVC_ENTITY_ID_EXTENSION_UNIT,         /* bUnitID */
        UVC_XU_GUID_M5STACK,                  /* guidExtensionCode (16 bytes) */
        UVC_XU_CTRL_COUNT,                    /* bNumControls */
        0x01,                                 /* bNrInPins */
        UVC_ENTITY_ID_PROCESSING_UNIT,        /* baSourceID[0] -> PU */
        0x01,                                 /* bControlSize */
        s_runtime_xu_bm_controls,
        0x00,                                 /* iExtension */
    };

    for (uint8_t i = 0; i < config->frame_count; ++i) {
        frame_total_length += (config->frames[i].payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG)
            ? sizeof(tusb_desc_video_frame_mjpeg_1int_t)
            : sizeof(tusb_desc_video_frame_uncompressed_1int_t);
    }
    for (uint8_t format_index = 1; format_index <= UVC_DESCRIPTOR_MAX_FORMAT_COUNT; ++format_index) {
        if (count_frames_for_format(config, format_index) == 0) {
            continue;
        }
        const uvc_descriptor_payload_t payload_format = payload_for_format(config, format_index);
        format_total_length += (payload_format == UVC_DESCRIPTOR_PAYLOAD_MJPEG)
            ? sizeof(tusb_desc_video_format_mjpeg_t)
            : sizeof(tusb_desc_video_format_uncompressed_t);
    }

    tusb_desc_configuration_t config_desc = {
        .bLength = sizeof(tusb_desc_configuration_t),
        .bDescriptorType = TUSB_DESC_CONFIGURATION,
        .wTotalLength = 0,
        .bNumInterfaces = ITF_NUM_TOTAL,
        .bConfigurationValue = 1,
        .iConfiguration = 0,
        .bmAttributes = TU_BIT(7),
        .bMaxPower = 100 / 2,
    };
    tusb_desc_interface_assoc_t iad_desc = {
        .bLength = sizeof(tusb_desc_interface_assoc_t),
        .bDescriptorType = TUSB_DESC_INTERFACE_ASSOCIATION,
        .bFirstInterface = ITF_NUM_VIDEO_CONTROL,
        .bInterfaceCount = 2,
        .bFunctionClass = TUSB_CLASS_VIDEO,
        .bFunctionSubClass = VIDEO_SUBCLASS_INTERFACE_COLLECTION,
        .bFunctionProtocol = VIDEO_ITF_PROTOCOL_UNDEFINED,
        .iFunction = 0,
    };
    tusb_desc_interface_t vc_interface = {
        .bLength = sizeof(tusb_desc_interface_t),
        .bDescriptorType = TUSB_DESC_INTERFACE,
        .bInterfaceNumber = ITF_NUM_VIDEO_CONTROL,
        .bAlternateSetting = 0,
        .bNumEndpoints = 0,
        .bInterfaceClass = TUSB_CLASS_VIDEO,
        .bInterfaceSubClass = VIDEO_SUBCLASS_CONTROL,
        .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
        .iInterface = STRID_UVC_CONTROL,
    };
    tusb_desc_video_control_header_1itf_t vc_header = {
        .bLength = sizeof(tusb_desc_video_control_header_1itf_t),
        .bDescriptorType = TUSB_DESC_CS_INTERFACE,
        .bDescriptorSubType = VIDEO_CS_ITF_VC_HEADER,
        .bcdUVC = VIDEO_BCD_1_50,
        .wTotalLength = sizeof(tusb_desc_video_control_header_1itf_t)
            + sizeof(tusb_desc_video_control_camera_terminal_t)
            + UVC_VC_PU_DESC_LENGTH
            + UVC_VC_XU_DESC_LENGTH
            + sizeof(tusb_desc_video_control_output_terminal_t),
        .dwClockFrequency = UVC_CLOCK_FREQUENCY,
        .bInCollection = 1,
        .baInterfaceNr = { ITF_NUM_VIDEO_STREAMING },
    };
    tusb_desc_video_control_camera_terminal_t camera_terminal = {
        .bLength = sizeof(tusb_desc_video_control_camera_terminal_t),
        .bDescriptorType = TUSB_DESC_CS_INTERFACE,
        .bDescriptorSubType = VIDEO_CS_ITF_VC_INPUT_TERMINAL,
        .bTerminalID = UVC_ENTITY_CAP_INPUT_TERMINAL,
        .wTerminalType = VIDEO_ITT_CAMERA,
        .bAssocTerminal = 0,
        .iTerminal = 0,
        .wObjectiveFocalLengthMin = 0,
        .wObjectiveFocalLengthMax = 0,
        .wOcularFocalLength = 0,
        .bControlSize = 3,
        .bmControls = { 0, 0, 0 },
    };
    tusb_desc_video_control_output_terminal_t output_terminal = {
        .bLength = sizeof(tusb_desc_video_control_output_terminal_t),
        .bDescriptorType = TUSB_DESC_CS_INTERFACE,
        .bDescriptorSubType = VIDEO_CS_ITF_VC_OUTPUT_TERMINAL,
        .bTerminalID = UVC_ENTITY_CAP_OUTPUT_TERMINAL,
        .wTerminalType = VIDEO_TT_STREAMING,
        .bAssocTerminal = 0,
        .bSourceID = UVC_ENTITY_ID_EXTENSION_UNIT,  /* 链路尾端连到 XU */
        .iTerminal = 0,
    };
    tusb_desc_interface_t vs_interface = {
        .bLength = sizeof(tusb_desc_interface_t),
        .bDescriptorType = TUSB_DESC_INTERFACE,
        .bInterfaceNumber = ITF_NUM_VIDEO_STREAMING,
        .bAlternateSetting = 0,
        .bNumEndpoints = endpoint_count,
        .bInterfaceClass = TUSB_CLASS_VIDEO,
        .bInterfaceSubClass = VIDEO_SUBCLASS_STREAMING,
        .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
        .iInterface = STRID_UVC_STREAMING,
    };
    tusb_desc_interface_t vs_interface_alt = {
        .bLength = sizeof(tusb_desc_interface_t),
        .bDescriptorType = TUSB_DESC_INTERFACE,
        .bInterfaceNumber = ITF_NUM_VIDEO_STREAMING,
        .bAlternateSetting = 1,
        .bNumEndpoints = 1,
        .bInterfaceClass = TUSB_CLASS_VIDEO,
        .bInterfaceSubClass = VIDEO_SUBCLASS_STREAMING,
        .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
        .iInterface = STRID_UVC_STREAMING,
    };
    /* VS input header = 13-byte fixed part + 1 bmaControls byte per format. */
    const uint8_t vs_hdr_append_size = (uint8_t)(TUD_VIDEO_DESC_CS_VS_IN_LEN + config->format_count);
    runtime_vs_input_header_t vs_header = {
        .bLength = vs_hdr_append_size,
        .bDescriptorType = TUSB_DESC_CS_INTERFACE,
        .bDescriptorSubType = VIDEO_CS_ITF_VS_INPUT_HEADER,
        .bNumFormats = config->format_count,
        .wTotalLength = 0,
        .bEndpointAddress = 0x81,
        .bmInfo = 0,
        .bTerminalLink = UVC_ENTITY_CAP_OUTPUT_TERMINAL,
        .bStillCaptureMethod = 0,
        .bTriggerSupport = 0,
        .bTriggerUsage = 0,
        .bControlSize = 1,
        .bmaControls = { 0 },
    };
    tusb_desc_video_streaming_color_matching_t color_desc = {
        .bLength = sizeof(tusb_desc_video_streaming_color_matching_t),
        .bDescriptorType = TUSB_DESC_CS_INTERFACE,
        .bDescriptorSubType = VIDEO_CS_ITF_VS_COLORFORMAT,
        .bColorPrimaries = VIDEO_COLOR_PRIMARIES_BT709,
        .bTransferCharacteristics = VIDEO_COLOR_XFER_CH_BT709,
        .bMatrixCoefficients = VIDEO_COLOR_COEF_SMPTE170M,
    };
    tusb_desc_endpoint_t endpoint_desc = {
        .bLength = sizeof(tusb_desc_endpoint_t),
        .bDescriptorType = TUSB_DESC_ENDPOINT,
        .bEndpointAddress = 0x81,
        .wMaxPacketSize = CFG_TUD_VIDEO_STREAMING_BULK ? 64 : CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE,
        .bInterval = 1,
    };

    endpoint_desc.bmAttributes.xfer = CFG_TUD_VIDEO_STREAMING_BULK ? TUSB_XFER_BULK : TUSB_XFER_ISOCHRONOUS;
    endpoint_desc.bmAttributes.sync = CFG_TUD_VIDEO_STREAMING_BULK ? 0 : 1;
    /* BULK 模式下 vs_interface_alt 不使用，避免编译器警告 */
    (void)vs_interface_alt;

    vs_cs_total_length = vs_hdr_append_size
        + format_total_length
        + frame_total_length
        + sizeof(tusb_desc_video_streaming_color_matching_t);
    vs_header.wTotalLength = vs_cs_total_length;

    /* VC 子描述符必须按 UVC 顺序：VC_HEADER -> CT -> PU -> XU -> OT，
     * 否则 host 解析时第一个 CS_INTERFACE 将不是 VC_HEADER，校验失败导致反复重枚举。 */
    if (!append_descriptor(&offset, &config_desc, sizeof(config_desc))
        || !append_descriptor(&offset, &iad_desc, sizeof(iad_desc))
        || !append_descriptor(&offset, &vc_interface, sizeof(vc_interface))
        || !append_descriptor(&offset, &vc_header, sizeof(vc_header))
        || !append_descriptor(&offset, &camera_terminal, sizeof(camera_terminal))
        || !append_descriptor(&offset, vc_pu_desc, sizeof(vc_pu_desc))
        || !append_descriptor(&offset, vc_xu_desc, sizeof(vc_xu_desc))
        || !append_descriptor(&offset, &output_terminal, sizeof(output_terminal))
        || !append_descriptor(&offset, &vs_interface, sizeof(vs_interface))
        || !append_descriptor(&offset, &vs_header, vs_hdr_append_size)) {
        return false;
    }

    for (uint8_t format_index = 1; format_index <= UVC_DESCRIPTOR_MAX_FORMAT_COUNT; ++format_index) {
        const uint8_t frame_count = count_frames_for_format(config, format_index);
        if (frame_count == 0) {
            continue;
        }
        const uvc_descriptor_payload_t payload_format = payload_for_format(config, format_index);
        if (!append_format_descriptor(&offset, config, payload_format, format_index)) {
            return false;
        }
        for (uint8_t i = 0; i < config->frame_count; ++i) {
            if (config->frames[i].format_index != format_index) {
                continue;
            }
            if (!append_frame_descriptor(&offset, &config->frames[i])) {
                return false;
            }
        }
    }

    if (!append_descriptor(&offset, &color_desc, sizeof(color_desc))) {
        return false;
    }

#if CFG_TUD_VIDEO_STREAMING_BULK
    if (!append_descriptor(&offset, &endpoint_desc, sizeof(endpoint_desc))) {
        return false;
    }
#else
    if (!append_descriptor(&offset, &vs_interface_alt, sizeof(vs_interface_alt))
        || !append_descriptor(&offset, &endpoint_desc, sizeof(endpoint_desc))) {
        return false;
    }
#endif

    ((tusb_desc_configuration_t *) s_desc_fs_configuration)->wTotalLength = (uint16_t) offset;
    s_desc_fs_configuration_len = (uint16_t) offset;
    return true;
}

bool uvc_descriptors_set_config(const uvc_descriptor_config_t *config)
{
    if (config == NULL
        || config->format_count == 0
        || config->format_count > UVC_DESCRIPTOR_MAX_FORMAT_COUNT
        || config->frame_count == 0
        || config->frame_count > UVC_DESCRIPTOR_MAX_FRAME_COUNT) {
        return false;
    }

    if (config->format_index == 0 || config->default_format_index == 0 || config->default_frame_index == 0) {
        return false;
    }

    for (uint8_t i = 0; i < config->frame_count; ++i) {
        const uvc_descriptor_frame_t *frame = &config->frames[i];
        if (frame->format_index == 0
            || frame->format_index > UVC_DESCRIPTOR_MAX_FORMAT_COUNT
            || frame->frame_index == 0
            || frame->width == 0
            || frame->height == 0
            || frame->frame_rate == 0
            || frame->frame_interval_100ns == 0
            || frame->frame_buffer_size == 0) {
            return false;
        }
    }

    s_active_config = *config;
    return build_configuration_descriptor(&s_active_config);
}

const uvc_descriptor_config_t *uvc_descriptors_get_config(void)
{
    return &s_active_config;
}

uint32_t uvc_descriptors_get_max_frame_buffer_size(void)
{
    uint32_t max_size = 0;
    for (uint8_t i = 0; i < s_active_config.frame_count; ++i) {
        if (s_active_config.frames[i].frame_buffer_size > max_size) {
            max_size = s_active_config.frames[i].frame_buffer_size;
        }
    }
    return max_size;
}

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index;
    if (s_desc_fs_configuration_len == 0) {
        uvc_descriptors_set_config(&s_default_config);
    }
    return s_desc_fs_configuration;
}

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;
    size_t chr_count;

    if (index == STRID_LANGID) {
        memcpy(&desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    } else {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0])) {
            return NULL;
        }

        const char *str = string_desc_arr[index];
        chr_count = strlen(str);
        if (chr_count > 32) {
            chr_count = 32;
        }

        for (size_t i = 0; i < chr_count; ++i) {
            desc_str[1 + i] = str[i];
        }
    }

    desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));
    return desc_str;
}
