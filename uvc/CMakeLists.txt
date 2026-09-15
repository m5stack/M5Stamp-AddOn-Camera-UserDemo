# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.22)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(stamp_addon_camera_userdemo)

# Workaround: GCC ICE (segfault during IRA pass) on esp_lcd_panel_rgb.c
# when compiled at the default optimization level. Force -O1 for this file only.
set_source_files_properties(
    "$ENV{IDF_PATH}/components/esp_lcd/rgb/esp_lcd_panel_rgb.c"
    PROPERTIES COMPILE_FLAGS "-O1"
)
