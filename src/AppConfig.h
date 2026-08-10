#pragma once

#include "BuildConfig.h"

#define BW21CAM_VERSION "0.1.0-test"

// Direct access-point mode matches the old ESP32-CAM test workflow.
#define BW21CAM_USE_ACCESS_POINT 1
#define BW21CAM_AP_SSID "BW21-CAM-TEST"
#define BW21CAM_AP_PASSWORD "12345678"
#define BW21CAM_AP_CHANNEL "6"

// Used only when BW21CAM_USE_ACCESS_POINT is 0.
#define BW21CAM_STATION_SSID "YOUR_WIFI_SSID"
#define BW21CAM_STATION_PASSWORD "YOUR_WIFI_PASSWORD"

#define BW21CAM_CONTROL_PORT 80
#define BW21CAM_STREAM_PORT 81

#define BW21CAM_STREAM_WIDTH 1280
#define BW21CAM_STREAM_HEIGHT 720

// PlatformIO enables this only in the bw21-cam-wk2132 environment.
#define BW21CAM_FC_BAUD 115200
