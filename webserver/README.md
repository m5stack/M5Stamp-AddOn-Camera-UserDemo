# M5Stamp-S3Bat CameraWebServer

[中文](README_CN.md)

## Overview
Port of the [CameraWebServer](https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer) example from [espressif/arduino-esp32](https://github.com/espressif/arduino-esp32) to M5Stamp-S3Bat.

The device joins an existing Wi‑Fi network in station mode. From a browser, you can control GC0308 and OV3660 cameras, capture still images, stream video, and use face detection/recognition.

## Required
* [esp32-camera](https://github.com/espressif/esp32-camera/tree/master)   
However, If you set platform = espressif32 and framework = arduino in platformio, esp32-camera is included in the package, so you do not need to specify esp32-camera in lib_deps.
* [gob_GC0308](https://github.com/GOB52/gob_GC0308)
* A reachable 2.4 GHz Wi‑Fi network

## Build type
| Env            | Description                                                                                                      |
| -------------- | ---------------------------------------------------------------------------------------------------------------- |
| `release_face` | Camera control, still images, streaming, face detection and recognition; the GC0308 QR endpoint is also included |

## How to use
After flashing and booting, the device prints its LAN IP address to the serial monitor. The address depends on the router:
```
[setup] Wi-Fi connected, IP address: 192.168.8.171
```
Open `http://<that IP>/` from a browser on the same network.

### WiFi setup
Edit `CAMERA_WIFI_SSID` and `CAMERA_WIFI_PASSWORD` in `src/main.cpp` before flashing:

```cpp
#define CAMERA_WIFI_SSID     "your Wi-Fi name"
#define CAMERA_WIFI_PASSWORD "your Wi-Fi password"
```

Build and upload the only available environment:

```sh
pio run -e release_face
pio run -e release_face -t upload
```

Do not commit real credentials to a public repository.

## Browser screen description
### Basic settings
* Resolution  
The default resolution is QVGA (320x240). The page reads the detected sensor from `/status` and lists resolutions from smallest to largest: `96x96`, `QQVGA`, `QCIF`, `HQVGA`, `240x240`, `QVGA`, `CIF`, `HVGA`, `VGA`, `SVGA`, `XGA`, `HD`, `SXGA`, `UXGA`, `QXGA`. GC0308 supports up to VGA (640x480), while OV3660 supports up to QXGA (2048x1536). Resolution changes reuse buffers allocated at startup and should not reboot the device.
* Contrast  
Change the contrast.
* Saturation  
Change the saturation.
* Special Effect  
Change the special effect.

| Menu      | Description       |
| --------- | ----------------- |
| NoEffect  | No effect         |
| Negative  | Negative effect   |
| Grayscale | Grayscale effect  |
| RedTint   | Red tint effect   |
| GreenTint | Green tint effect |
| BlueTint  | Blue tint effect  |
| Sepia     | Sepia effect      |

* WB Mode  
Change the white balance.

| Menu   | Description       |
| ------ | ----------------- |
| Auto   | Automatic         |
| Sunny  | Sunny             |
| Cloudy | Cloudy            |
| Office | Fluorescent light |
| Home   | Light bulb        |

* Gain  
Change the gain.
* H-Mirror  
Changes the horizontal image inversion.
* V-Flip  
Changes the vertical image inversion.
* Color Bar  
Color bar display ON/OFF.

* Face Detection  
Face detection ON/OFF. The page allows it only at QVGA (320x240) or lower.
* Face Recognition
Face recognition ON/OFF. The page allows it only at QVGA (320x240) or lower, and processing is computationally expensive.

* Get Still  
Obtains a still image.
* Start/Stop Stream  
Start/Stop receiving stream.
* Enroll Face  
Enroll the face on the camera.  
Once enrolled, the face will no longer be treated as an intruder (red frame) if Face Recognition is ON.
* Start/Stop Scan QR  
Start/Stop QR recognition. The current `/qr` implementation uses GC0308 RGB565 frames; OV3660 JPEG frames are not supported by this endpoint yet.

* Save  
Download incoming images.
* X  
Stop receiving stream.

### Advanced settings
* Register Get/Set  
The values can be set and retrieved for the camera registers.  
Refer to data sheets and other sources for register and function information.  
**Caution** <ins>Be careful not to set inappropriate values to inappropriate registers. </ins>

## Current implementation
* Fixed pin mapping for M5Stamp-S3Bat and the Stamp Add-On Camera.
* Detects GC0308 or OV3660 at startup.
* Keeps only the `release_face` build, with face detection and recognition enabled.
* Uses Wi‑Fi station mode and does not create a camera access-point hotspot.
* Reuses allocated DMA/frame buffers during runtime resolution changes to avoid rebooting.
