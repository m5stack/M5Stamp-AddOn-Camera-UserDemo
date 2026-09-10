# M5Stamp-S3Bat-AddOn CameraWebServer

[English](README.md)

## 项目概述
这是将 [espressif/arduino-esp32](https://github.com/espressif/arduino-esp32) 的 [CameraWebServer](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer) 移植到 M5Stamp-S3Bat 的项目。

设备通过 Wi‑Fi 加入现有局域网，浏览器访问设备 IP 后可以控制 GC0308 和 OV3660、获取静态图像、查看视频流，并使用人脸检测/识别功能。

## 依赖与环境
* [esp32-camera](https://github.com/espressif/esp32-camera/tree/master)  
  PlatformIO 的 `espressif32 + arduino` 框架已内置，无需单独添加。
* [gob_GC0308](https://github.com/GOB52/gob_GC0308)
* 可连接的 2.4 GHz Wi‑Fi 环境

## 构建环境
| Env            | 说明                                                                             |
| -------------- | -------------------------------------------------------------------------------- |
| `release_face` | 摄像头控制、静态图像、视频流、人脸检测、人脸识别；同时保留 GC0308 二维码识别入口 |

## 使用方法
烧录并启动后，串口会打印设备在局域网中的 IP 地址。实际地址取决于路由器分配结果：
```
[setup] Wi-Fi connected, IP address: 192.168.8.171
```
在同一局域网的浏览器中访问 `http://<该 IP>/`。

### Wi‑Fi 设置
在 `src/main.cpp` 中修改：

```cpp
#define CAMERA_WIFI_SSID     "你的 Wi-Fi 名称"
#define CAMERA_WIFI_PASSWORD "你的 Wi-Fi 密码"
```

然后构建并烧录 `release_face`：

```sh
pio run -e release_face
pio run -e release_face -t upload
```
