#pragma once

#include "BuildConfig.h"

#define BW21CAM_VERSION "0.6.0-qr-human-face"

// Direct access-point mode matches the old ESP32-CAM test workflow.
#define BW21CAM_USE_ACCESS_POINT 1
#define BW21CAM_AP_SSID "BW21-CAM-TEST"
#define BW21CAM_AP_PASSWORD "12345678"
#define BW21CAM_AP_CHANNEL "36"
#define BW21CAM_AP_FALLBACK_CHANNEL "6"

// Used only when BW21CAM_USE_ACCESS_POINT is 0.
#define BW21CAM_STATION_SSID "YOUR_WIFI_SSID"
#define BW21CAM_STATION_PASSWORD "YOUR_WIFI_PASSWORD"

#define BW21CAM_CONTROL_PORT 80
#define BW21CAM_STREAM_PORT 81

#define BW21CAM_STREAM_WIDTH 1280
#define BW21CAM_STREAM_HEIGHT 720

// The ISP applies its generic wide-angle correction before QR and object
// processing. On-device QR adds moderate/strong software rectification retries.
#define BW21CAM_ENABLE_LENS_DISTORTION_CORRECTION 1

#define BW21CAM_QR_STALE_MS 5000
#define BW21CAM_OBJECT_STALE_MS 1500
#define BW21CAM_VISION_MAX_OBJECTS 8
#define BW21CAM_VISION_DEFAULT_ENABLED 1

// Set this one value to 1 to restore per-detection color classification in
// the web UI, JSON API, and serial diagnostics. It is off by default so QR,
// person, and face processing receive the available analysis time.
#define BW21CAM_ENABLE_COLOR_DETECTION 0

// PlatformIO enables this only in the bw21-cam-wk2132 environment.
#define BW21CAM_FC_BAUD 115200
