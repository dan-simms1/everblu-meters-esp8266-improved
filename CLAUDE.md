# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

Fork of [genestealer/everblu-meters-esp8266-improved](https://github.com/genestealer/everblu-meters-esp8266-improved) with SPI integration fixes for Arduino Nano ESP32 (ESP32-S3) + CC1101.

Reads water/gas usage from Itron EverBlu Cyble Enhanced RF meters via the RADIAN protocol on 433 MHz using ESP8266/ESP32 + CC1101 transceiver. Integrates with Home Assistant via MQTT AutoDiscovery or as a native ESPHome external component.

## Build Commands

### PlatformIO (standalone firmware)
```bash
pio run -e huzzah          # Build for Adafruit HUZZAH ESP8266 (default)
pio run -e esp32dev         # Build for ESP32 DevKit
pio run -e d1_mini_pro      # Build for WeMos D1 Mini Pro
pio run -e nodemcuv2        # Build for NodeMCU v2
pio run -t upload           # Upload via USB/Serial
```
OTA variants: append `-ota` to env name (e.g., `huzzah-ota`). Set device IP in `platformio.ini` first.

### ESPHome (external component)
No local build — ESPHome pulls from this repo's `ESPHOME-release/` directory. The component is referenced in ESPHome YAML:
```yaml
external_components:
  - source:
      type: git
      url: https://github.com/dan-simms1/everblu-meters-esp8266-improved
      ref: feature/esphome-spi-integration
      path: ESPHOME-release
    components: [ everblu_meter ]
```

## Architecture

The project has two deployment modes sharing common core logic:

### Standalone firmware (`src/`)
- `src/main.cpp` — entry point
- `src/core/` — CC1101 radio driver, logging, WiFi serial, utilities
- `src/services/` — meter reading, frequency management, scheduling, storage
- `src/adapters/` — abstraction interfaces (`config_provider.h`, `data_publisher.h`, `time_provider.h`) with platform-specific implementations
- `include/private.example.h` — copy to `private.h` with meter serial, WiFi, MQTT credentials

### ESPHome component (`ESPHOME-release/everblu_meter/`)
- `__init__.py` — ESPHome code generation (Python); defines YAML schema, registers SPI device
- `everblu_meter.cpp/.h` — ESPHome component wrapper; extends `SPIDevice` for native SPI integration
- Remaining `.cpp/.h` files are flattened copies of the core/services code adapted for ESPHome
- `DO_NOT_EDIT.md` — release copy; development happens in `ESPHOME/components/everblu_meter/`

### Development vs Release ESPHome paths
- `ESPHOME/components/everblu_meter/` — development copy (references `src/core/` headers)
- `ESPHOME-release/everblu_meter/` — release copy (self-contained, flattened)

### Key SPI Integration (this fork's changes)
The CC1101 radio uses ESPHome's native `spi:` component rather than raw Arduino SPI calls. In ESPHome mode, `cc1101.cpp` delegates SPI transfers through an `SPIDevice` pointer (`_spi_device->enable()`, `transfer_array()`, `disable()`). The `__init__.py` uses `spi.spi_device_schema()` and `spi.register_spi_device()` helpers.

## Important Notes

- Arduino framework only — ESP-IDF is not supported for the ESPHome component
- For Arduino Nano ESP32 (ESP32-S3), USB CDC build flags are required in ESPHome YAML:
  ```yaml
  platformio_options:
    build_unflags: [-DARDUINO_USB_CDC_ON_BOOT=0, -DARDUINO_USB_CDC_ON_BOOT=1]
    build_flags: [-DARDUINO_USB_CDC_ON_BOOT=1, -DARDUINO_USB_MODE=1]
  ```
- Run "Clean Build Files" in ESPHome after config changes
- The meter has a read counter that increments per query — excessive reads may cause issues with utility company wireless reads
