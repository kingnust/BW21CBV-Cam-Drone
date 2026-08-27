# BW21-CBV Camera Drone

Firmware for the Ai-Thinker BW21-CBV-Kit camera used on the indoor drone.

The default firmware creates a local access point, serves a browser-based MJPEG
viewer, and provides three simple frame-rate and JPEG-quality presets. It also
contains onboard vision that can be turned on and off at runtime. The optional
flight-controller link is compiled out by default so camera testing cannot
disturb the FC.

## PlatformIO workflow

The repository is a PlatformIO project. Its local AmebaPro2 platform wraps the
official Realtek Arduino core because RTL8735B is not currently available in
the public PlatformIO platform registry.

From the PlatformIO terminal:

```powershell
platformio run -e bw21-cam-test
platformio run -e bw21-cam-test -t upload
platformio device monitor -b 115200
```

The first `platformio run` automatically downloads the pinned Arduino CLI
1.5.1 and official Realtek AmebaPro2 4.1.0 core into
`%USERPROFILE%\.bw21-platformio`. Realtek's build executables require this
no-space working path. `platformio run -t setup` can prepare those tools without
compiling.

The application source is in `src/`. The selected target is AMB82-MINI with the
GC2053 sensor used by the BW21-CBV-Kit. Build configuration is kept in
`platformio.ini`.

## Test firmware

Default network:

- SSID: `BW21-CAM-TEST`
- Password: `12345678`
- Web page: use the IP printed at 115200 baud after boot
- MJPEG stream: `http://<board-ip>:81/stream`

The presets keep the resolution at 1280 x 720 so switching is reliable:

| Preset | Frame rate | JPEG quality |
| --- | ---: | ---: |
| Smooth | 30 FPS | 3/9 |
| Balanced | 20 FPS | 5/9 |
| Detail | 10 FPS | 8/9 |

Quality follows the Ameba API scale: 1 is lowest and 9 is highest. `Detail`
uses 8 because level 9 can create much larger images for little visible gain.

After uploading, open the serial monitor at 115200 baud, connect a phone or
laptop to the printed access point, and open the printed web address.

## Vision choices

Two independent vision paths are included so they can be evaluated before one
is selected for the drone.

### On-device BW21 vision

```powershell
platformio run
platformio run -t upload
```

The vision build starts in low-load `Camera` mode. The web-page `Camera` /
`Vision` control can start or stop QR scanning and object detection without a
reboot; disabling vision pauses the NN pipeline and stops both analysis camera
channels so the live stream receives the available processing time. When
enabled, a dedicated 640 x 480 camera channel feeds a fast QVGA QR search on
every analysis frame. Timed full-resolution retries preserve
small-code range. If a clean full-resolution pass fails, one QVGA software
rectification pass compensates for the fisheye lens; moderate and strong
profiles alternate so this fallback stays bounded. Detected rectified corners
are mapped back into the original 640 x 480 coordinates before localization.
Contrast/ZBar fallbacks remain available after that. This leaves the 1280 x 720
live stream cadence independent of QR latency. A separate 576 x 320, 10 FPS
channel feeds YOLOv4-tiny and SCRFD, but only `person` and `face` detections are
published; all other COCO classes are ignored. Color classification remains in
the firmware source but defaults off through
`BW21CAM_ENABLE_COLOR_DETECTION` in `src/AppConfig.h`; change that one value to
`1` to restore color processing and output. QR, person, and face vision starts
automatically at boot and can still be paused from the web page. Results,
fast/full/fisheye scan counters, and stage timings are available in the web
page, `/api/status`, and `/api/vision`.

The default `bw21-cam-vision-wk2132` environment includes the FC link. The
camera sends each decoded
QR payload plus normalized center, apparent size, image area, and orientation
through the acknowledged MSP2 v2 transport. A still-visible code is refreshed
at up to 4 Hz for localization filtering. Ordinary camera and WK2132 builds do
not initialize QR or neural-network code.

### Laptop vision

The `laptop_vision/` program reads the MJPEG stream directly. Every decoded
frame receives a full-field QR pass, with curved, contrast, and high-detail
fallbacks distributed across frames to prevent scan latency from accumulating.
Ultralytics YOLO runs in a separate latest-frame worker: slow inference replaces
old pending frames instead of replaying stale video. The laptop path also adds
per-object color, event logging, and optional calibrated OpenCV fisheye
correction. See `laptop_vision/README.md` for setup and calibration commands.

The default laptop model recognizes common objects. Project-specific packages,
dangerous items, landing markers, or other custom classes require a trained
model passed with `--model`.

## WK2132 flight-controller link

The WK2132 is an I2C-to-dual-UART bridge on the flight controller. The BW21 does
not connect to the WK2132 over I2C; it connects to the WK2132 camera UART:

| BW21-CBV-Kit | WK2132/FC camera connector |
| --- | --- |
| Pin 14, IOA2 / D21 / UART0_OUT | CAMERA_RX |
| Pin 15, IOA3 / D22 / UART0_IN | CAMERA_TX |
| Pin 17 or 33, GND | GND |

UART settings are 115200 baud, 8 data bits, no parity, and 1 stop bit. The
signals are 3.3 V logic. Always cross TX to RX and join grounds.

For a WK2132 link without onboard vision, select the camera-only environment:

```powershell
platformio run -e bw21-cam-wk2132
platformio run -e bw21-cam-wk2132 -t upload
```

The included link layer then:

- uses `Serial1`, which maps to BW21 D21/D22;
- queries the FC MSP API once per second as a link heartbeat;
- validates MSP v1 and MSP v2 checksums;
- exposes the reliable MSP2 QR observation and acknowledgement path.

The matching flight-controller environment is
`drone_proto_esp32s3_wk2132_experimental`, which is now the FC project's
default. It routes the WK2132 camera UART to
a dedicated MSP parser while the WK2132 itself remains on the shared FC I2C
bus. Use the FC CLI commands `wk2132`, `camstatus`, and `qrloc` to inspect the
bridge, QR observations, and experimental fused position. The full landmark
format and bench procedure are in the FC document
`docs/drone-proto-qr-localization.md`.

The camera-test and WK2132-only builds do not decode QR codes. Select one of the
vision environments when onboard QR/object processing is required.

Do not enable the FC link until the WK2132 build is running on the flight
controller and the crossed UART wiring has been checked.

## Power

Power the kit from a regulated 5 V supply rated above 1 A and use a common
ground with the flight controller. Do not power the kit from the FC 3.3 V rail.
