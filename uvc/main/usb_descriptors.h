#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define UVC_DESCRIPTOR_MAX_FORMAT_COUNT 6
#define UVC_DESCRIPTOR_MAX_FRAME_COUNT  16
#define UVC_DESCRIPTOR_FORMAT_INDEX     1
#define UVC_MJPEG_FORMAT_FLAG_SW_ENCODER 0x80U

/* UVC entity ID 分配（与 usb_descriptors.c 内描述符保持一致） */
#define UVC_ENTITY_ID_CAMERA_TERMINAL  1   /* CT (Input Terminal) */
#define UVC_ENTITY_ID_OUTPUT_TERMINAL  2   /* OT */
#define UVC_ENTITY_ID_PROCESSING_UNIT  3   /* PU  */
#define UVC_ENTITY_ID_EXTENSION_UNIT   4   /* XU  */

/* PU 选择器（UVC 1.5 规范，bit 编号即 selector-1） */
#define UVC_PU_CTRL_CONTRAST     0x03  /* bmControls bit2，int16，range [-2,2] */
#define UVC_PU_CTRL_SATURATION   0x07  /* bmControls bit6，int16，range [-2,2] */

/* XU 选择器（自定义，bit 编号即 selector-1） */
#define UVC_XU_CTRL_HMIRROR        1   /* int32，[0,1]   */
#define UVC_XU_CTRL_VFLIP          2   /* int32，[0,1]   */
#define UVC_XU_CTRL_SPECIAL_EFFECT 3   /* int32，[0,6]   */
#define UVC_XU_CTRL_AE_LEVEL       4   /* int32，[-5,5]  */
#define UVC_XU_CTRL_MJPEG_HW_QUALITY 5 /* int32，[1,63] MJPEG(HW) sensor encoder */
#define UVC_XU_CTRL_WB_MODE        6   /* int32，[0,4]   */
#define UVC_XU_CTRL_MJPEG_SW_QUALITY 7  /* int32，[1,100] MJPEG(SW) encoder */

#define UVC_PU_CTRL_VALUE_SIZE   2     /* PU 控制值字节数 */
#define UVC_XU_CTRL_VALUE_SIZE   4     /* XU 控制值字节数 */
#define UVC_XU_CTRL_COUNT        7

/* M5Stack 自定义 XU GUID（保持稳定即可，host 据此识别 XU 实体）。
 * 16 字节，混合端序按 UVC 描述符常用方式给出 raw bytes。 */
#define UVC_XU_GUID_M5STACK \
    0xb1, 0xe6, 0x4f, 0x49, 0x82, 0x83, 0x46, 0x9a, \
    0x9d, 0x8a, 0x6b, 0x2e, 0xa1, 0x3c, 0x4f, 0x10

typedef enum {
	UVC_DESCRIPTOR_PAYLOAD_YUY2 = 0,
	UVC_DESCRIPTOR_PAYLOAD_UYVY = 1,
	UVC_DESCRIPTOR_PAYLOAD_GRAY8 = 2,
	UVC_DESCRIPTOR_PAYLOAD_RGB565 = 3,  /* Uncompressed RGBP (LE 5-6-5), FOURCC RGBP */
	UVC_DESCRIPTOR_PAYLOAD_MJPEG = 4,
} uvc_descriptor_payload_t;

typedef struct {
	uvc_descriptor_payload_t payload_format;
	uint8_t format_index;
	uint8_t frame_index;
	uint16_t width;
	uint16_t height;
	uint8_t frame_rate;
	uint32_t frame_interval_100ns;
	uint32_t frame_buffer_size;
	bool mjpeg_sw;
} uvc_descriptor_frame_t;

typedef struct {
	uvc_descriptor_payload_t payload_format;
	uint8_t format_index;
	uint8_t format_count;
	uint8_t frame_count;
	uint8_t default_format_index;
	uint8_t default_frame_index;
	uvc_descriptor_frame_t frames[UVC_DESCRIPTOR_MAX_FRAME_COUNT];
} uvc_descriptor_config_t;

void uvc_descriptors_set_control_masks(uint16_t pu_bm_controls, uint8_t xu_bm_controls);
bool uvc_descriptors_set_config(const uvc_descriptor_config_t *config);
const uvc_descriptor_config_t *uvc_descriptors_get_config(void);
uint32_t uvc_descriptors_get_max_frame_buffer_size(void);

#ifdef __cplusplus
}
#endif

#endif
