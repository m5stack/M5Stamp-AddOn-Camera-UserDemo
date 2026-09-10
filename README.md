# M5Stamp-AddOn-Camera-UserDemo

| Directory      | Firmware                     | Build system         |
| -------------- | ---------------------------- | -------------------- |
| `./uvc/`       | USB Video Class (UVC) camera | ESP-IDF              |
| `./webserver/` | Wi-Fi camera web server      | PlatformIO + Arduino |

The two projects use the same camera wiring, but different camera and application stacks. Build and flash only one project at a time; they are not two runtime modes combined in a single firmware.

## UVC firmware

The `uvc/` project captures frames through DVP and exposes the camera as a UVC device to Linux, macOS, Windows, or another USB host through TinyUSB. It can automatically detect OV3660 and GC0308 sensors.

## WebServer firmware

`webserver/` is an independent CameraWebServer project. After connecting to Wi-Fi, it provides browser-based camera control, still image capture, MJPEG video streaming, and optional face or QR features.