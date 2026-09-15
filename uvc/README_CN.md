# Stamp S3 Add-On Camera UVC

这是一个运行在 ESP32-S3 上的 USB Video Class (UVC) 摄像头固件。固件通过 DVP 接口采集 Stamp Add-On Camera 的图像，并通过 TinyUSB 将设备暴露给 Linux、macOS、Windows 或其他 USB Host。

当前代码可自动探测 OV3660 和 GC0308。

## 从零开始构建

### 环境

- [ESP-IDF 6.1.0](https://github.com/espressif/esp-idf/tree/release/v6.1)

`components/esp_cam_sensor` 和 `components/tinyusb` 目前保留在仓库中，因为它们包含本项目的传感器和 UVC 定制修改。

安装并激活 ESP-IDF 6.1.0 后，确认 `idf.py` 可用：

```bash
source $IDF_PATH/export.sh
idf.py --version
```

### 配置和编译

```bash
idf.py set-target esp32s3
idf.py reconfigure
idf.py build
```

如果需要打开更多传感器模式，执行 `idf.py menuconfig`，在摄像头组件的 DVP format 菜单中启用对应选项，然后重新构建。不要直接复制另一块板卡的 `sdkconfig`。

## 烧录和串口日志

```bash
# Linux
idf.py -p /dev/ttyACMXXX flash monitor

# macOS 示例
idf.py -p /dev/cu.usbmodemXXXX flash monitor

# Windows 示例
idf.py -p COMXXX flash monitor
```

启动日志应包含传感器探测结果、SCCB 地址、原生格式列表和 UVC profile。若日志显示 `no supported camera sensor found`，先检查 PWDN/RESET、SCCB、XCLK、DVP 同步和数据位序，再检查 USB。

## 主机侧 UVC 验证

至少验证以下项目：设备枚举、默认 profile 出图、格式/分辨率切换、连续流、断开重连，以及 UVC 控制项。

### Linux

```bash
sudo apt install v4l-utils ffmpeg
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/videoX
ffplay -f v4l2 -i /dev/videoX
```

### macOS

使用系统的 QuickTime Player、Photo Booth 或其他 UVC 相机应用。在“新建影片录制”中选择对应摄像头。

### Windows

使用系统 Camera 应用或设备管理器确认 UVC 设备。切换分辨率/格式后检查是否持续出图；若主机只修改了 frame index，TinyUSB 定制代码会重新采用当前描述符的 frame size。
