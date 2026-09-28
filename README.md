# WiFi Receiver S3

ESP32-S3 N16R8 Wi-Fi audio receiver for **WiFi Audio Streaming / WFAS v2** Android.

## Hardware

- ESP32-S3 N16R8 — 16 MB Flash / 8 MB OPI PSRAM
- UDA1334A I2S DAC

### UDA1334A wiring

| ESP32-S3 | UDA1334A |
|---|---|
| GPIO 4 | BCLK |
| GPIO 5 | LRCK / WS |
| GPIO 21 | DIN |
| GND | GND |

## Audio

WFAS v2:
- 48 kHz
- 16-bit
- Stereo
- UDP port 9090
- Discovery multicast 239.255.0.1:9091

The firmware uses the official WFAS v2 C reference implementation as a PlatformIO dependency. WFAS v2 uses raw signed 16-bit little-endian PCM and a 10-byte protocol header.

## Setup

Edit the Wi-Fi credentials at the top of `src/main.cpp`:

```cpp
static constexpr char WIFI_SSID[] = "YOUR_WIFI_SSID";
static constexpr char WIFI_PASSWORD[] = "YOUR_WIFI_PASSWORD";
```

Build for **ESP32-S3 N16R8**.

After boot:
- receiver advertises itself to the Android app
- Android connects using WFAS v2
- audio is buffered in 1 MB PSRAM
- PCM is sent to the UDA1334A through I2S
- status page: `http://wifi-receiver-s3.local/`
- OTA page: `http://wifi-receiver-s3.local/` then upload the firmware binary

## Important

This first implementation intentionally uses WFAS v2 **unencrypted/off security mode**, matching the normal Android receiver discovery path. Encryption can be added later without changing the audio transport architecture.
