# M5Stamp-AddOn-Camera-UserDemo

| 目录           | 固件用途                     | 构建系统             |
| -------------- | ---------------------------- | -------------------- |
| `./uvc/`       | USB Video Class（UVC）摄像头 | ESP-IDF              |
| `./webserver/` | Wi-Fi 摄像头网页服务器       | PlatformIO + Arduino |

两个工程使用相同的摄像头接线，但底层摄像头和应用栈不同。每次只构建并烧录其中一个工程；它们目前不是合并在同一个固件中的两个运行模式。

## UVC 固件

`uvc/` 工程通过 DVP 采集图像，并使用 TinyUSB 将摄像头作为 UVC 设备输出给 Linux、macOS、Windows 或其他 USB Host。目前支持自动探测 OV3660 和 GC0308。

## WebServer 固件

`webserver/` 是独立的 CameraWebServer 工程。设备连接 Wi-Fi 后，可通过浏览器控制摄像头、获取静态图片、查看 MJPEG 视频流，并按配置启用人脸或 QR 功能。