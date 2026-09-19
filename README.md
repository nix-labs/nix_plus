# NIX labs NIX+ Module Firmware

This repository contains the ESPHome firmware and custom components for the **NIX labs NIX+ Network Module**, an intelligent network bridge connecting NIX labs Nixie clocks (NIX4, NIX6, NIX4F, FL4, VF4) to Home Assistant, ESPHome, and modern IoT networks.

## Supported Clocks

The NIX+ module automatically identifies the attached base clock CPU (PIC32) and its digit configuration:
- **NIX4 / NIX4-B**: 4-digit Nixie Clock (primary product)
- **NIX6 / NIX6-B**: 6-digit Nixie Clock (primary product)
- **NIX4F**: 4-digit Nixie Clock with fast refresh (primary product)
- **FL4**: 4-digit Filament Clock (fully compatible)
- **VF4**: 4-digit VFD Clock (fully compatible)

## Repository Structure

- **[nix_plus.yaml](file:///Users/nathan/Documents/GitHub/nix_plus/nix_plus.yaml)**: Main entry point for standard users and Home Assistant ESPHome Dashboard adoption (pulls remote packages).
- **[nix_plus_factory.yaml](file:///Users/nathan/Documents/GitHub/nix_plus/nix_plus_factory.yaml)**: Entry point for factory releases and local development (uses local packages).
- **[packages/](file:///Users/nathan/Documents/GitHub/nix_plus/packages/)**: Modular configuration blocks:
  - `nix_plus_core.yaml`: Base board definitions, ESP-IDF settings, fallback AP hotspot, Web UI sorting groups, and UART bus (GPIO17 TX / GPIO16 RX @ 115200).
  - `nix_plus_update.yaml`: Managed OTA updates from `https://assets.nixlabs.com.au/nix_plus/`.
  - `nix_plus_time.yaml`: SNTP internet time synchronization, base clock RTCC synchronization, and IP geolocation auto-timezone detection.
  - `nix_plus_clock.yaml`: Universal `nix_plus` component, Nixie tube display power/brightness, RGB backlight colour selection and autonomous hardware cycle effects, sensors, and auto-brightness.
  - `nix_plus_inputs.yaml`: Timer controls (duration, start/pause/resume/stop) and display override controls (number display, screen switching).
- **[components/nix_plus/](file:///Users/nathan/Documents/GitHub/nix_plus/components/nix_plus/)**: Custom C++ component driving bidirectional serial communication with the base clock PIC32.
- **[components/improv_serial/](file:///Users/nathan/Documents/GitHub/nix_plus/components/improv_serial/)**: Improv Serial provisioning component over UART.

## Hardware & Pin Configuration

| Function | Pin | Notes |
| :--- | :--- | :--- |
| **UART TX** | GPIO17 | Connected to PIC32 RX @ 115200 baud |
| **UART RX** | GPIO16 | Connected to PIC32 TX @ 115200 baud |
| **Debug Logger** | UART0 (GPIO1/GPIO3) | Standard USB-Serial debug output |

*Note: Current hardware uses the ESP32 (`esp32dev`); future production hardware revisions will migrate directly to the ESP32-C3 (`esp32-c3-devkitm-1`) using the same ESP-IDF framework.*

## Documentation

For user guides, schematics, and troubleshooting:
- [NIX labs Documentation](https://docs.nixlabs.com.au)
- [NIX labs Assets](https://assets.nixlabs.com.au/nix_plus)

## License

This project is licensed under the **GNU General Public License v3.0 (GPLv3)**.

