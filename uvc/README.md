# Stamp S3 Add-On Camera UVC

[中文文档](README_CN.md)

This is a USB Video Class (UVC) camera firmware for ESP32-S3. It captures images from the Stamp Add-On Camera over a DVP interface and exposes the device through TinyUSB to Linux, macOS, Windows, or another USB host.

The current application can automatically detect OV3660 and GC0308 sensors.

## Build From Scratch

### Prerequisites

- [ESP-IDF 6.1.0](https://github.com/espressif/esp-idf/tree/release/v6.1)

`components/esp_cam_sensor` and `components/tinyusb` remain in the repository because they contain the sensor and UVC customizations used by this project.

Install and activate ESP-IDF 6.1.0, then confirm that `idf.py` is available:

```bash
source $IDF_PATH/export.sh
idf.py --version
```

### Configure and build

```bash
idf.py set-target esp32s3
idf.py reconfigure
idf.py build
```

To enable additional sensor modes, run `idf.py menuconfig`, enable the required DVP format options, and build again.

## Flashing and Serial Logs

```bash
# Linux
idf.py -p /dev/ttyACMXXX flash monitor

# macOS example
idf.py -p /dev/cu.usbmodemXXXX flash monitor

# Windows example
idf.py -p COMXXX flash monitor
```

The startup log should show the detected sensor, SCCB address, native format list, and generated UVC profiles. If it reports `no supported camera sensor found`, check PWDN/RESET, SCCB, XCLK, DVP synchronization, and data order before debugging USB.

## UVC Host

At minimum, verify device enumeration, the default profile producing video, format/resolution switching, continuous streaming, disconnect/reconnect, and UVC controls.

### Linux

```bash
sudo apt install v4l-utils ffmpeg
v4l2-ctl --list-devices
v4l2-ctl --list-formats-ext -d /dev/videoX
ffplay -f v4l2 -i /dev/videoX
```

### macOS

Use the system QuickTime Player, Photo Booth, or another UVC camera application. Select the corresponding camera in a new movie recording.

### Windows

Use the built-in Camera application or Device Manager to confirm the UVC device. After switching resolution or format, check that video continues. If the host changes only the frame index, the TinyUSB customization reuses the frame size from the selected descriptor.
