# BW21-CBV Camera Drone

Firmware for the Ai-Thinker BW21-CBV-Kit camera used on the indoor drone.

The first firmware is a camera and Wi-Fi test build. It creates a local access
point, serves a browser-based MJPEG viewer, and provides three simple frame-rate
and JPEG-quality presets. The optional flight-controller link is compiled out
by default so camera testing cannot disturb the FC.

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

After camera-only testing succeeds, select the dedicated PlatformIO environment:

```powershell
platformio run -e bw21-cam-wk2132
platformio run -e bw21-cam-wk2132 -t upload
```

The included link layer then:

- uses `Serial1`, which maps to BW21 D21/D22;
- queries the FC MSP API once per second as a link heartbeat;
- validates MSP v1 and MSP v2 checksums;
- exposes the existing reliable MSP2 QR-result publish/acknowledgement path for
  later QR scanning firmware.

The matching flight-controller environment is
`drone_proto_esp32s3_wk2132_experimental`. It routes the WK2132 camera UART to
a dedicated MSP parser while the WK2132 itself remains on the shared FC I2C
bus. Use the FC CLI commands `wk2132` and `camera_uart` to inspect both sides of
the link.

This first camera test build does not decode QR codes on the BW21. The QR API is
the prepared transport for that later firmware stage.

Do not enable the FC link until the WK2132 build is running on the flight
controller and the crossed UART wiring has been checked.

## Power

Power the kit from a regulated 5 V supply rated above 1 A and use a common
ground with the flight controller. Do not power the kit from the FC 3.3 V rail.
