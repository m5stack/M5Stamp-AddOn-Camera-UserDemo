idf_component_register(SRCS "main.cpp"
                    "camera_app.cpp"
                    "camera_app/camera_sensor.cpp"
                    "camera_app/uvc_profiles.cpp"
                    "camera_app/frame_pipeline.cpp"
                    "camera_app/usb_uvc.cpp"
                    "camera_app/camera_task.cpp"
                    "camera_app/uvc_controls.cpp"
                    "usb_descriptors.c"
                    INCLUDE_DIRS "."
                    REQUIRES espressif__esp_cam_sensor espressif__esp_sccb_intf esp_timer espressif__tinyusb espressif__esp_new_jpeg
                    WHOLE_ARCHIVE)

idf_component_get_property(tusb_lib tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE ".")
