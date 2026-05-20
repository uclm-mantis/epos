# EPOS ESP-IDF console plus independent UART example

This example runs the interactive `esp_console`/`linenoise` console on the transport selected by the example configuration and can use a separate UART for periodic bytes on GPIO 43 and GPIO 44.

The important part is that the telemetry UART is never attached to VFS or standard I/O. `stdin`, `stdout` and `stderr` remain owned by the configured interactive console transport, while UART1 is used only through `uart_write_bytes()` when its pins do not overlap with the console pins.

## Hardware

- Target with USB Serial/JTAG and GPIO 43/44 available, for example ESP32-S3.
- UART TX: GPIO 43.
- UART RX: GPIO 44.
- UART speed: 115200 8N1.
- Status RGB LED: WS2812-compatible LED on GPIO 48, toggled every 500 ms.
- CAN in this example: TX GPIO 14, RX GPIO 15, 1 Mbit/s.

Do not use GPIO 19 or GPIO 20 for CAN when USB Serial/JTAG is enabled on ESP32-S3. Those pins are used by native USB, and reconfiguring them for TWAI can disconnect the monitor.

Adjust `EXAMPLE_UART_TX_PIN`, `EXAMPLE_UART_RX_PIN` or the CAN pins in `main/main.c` if your board uses different pins.

## Build and flash

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

The monitor opens the configured interactive console. Use `help` or `about` to see the EPOS/CANopen commands.

When its pins do not overlap with the console, the second UART periodically prints messages like:

```text
EPOS UART heartbeat 1
EPOS UART heartbeat 2
```

## Console configuration

`sdkconfig.defaults` selects UART0 by default and disables the secondary console:

- `CONFIG_ESP_CONSOLE_UART_DEFAULT=y`
- `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y`
- `CONFIG_EPOS_EXAMPLE_CONSOLE_UART=y`

To switch the interactive console to USB Serial/JTAG, run `idf.py menuconfig` and select:

- `EPOS example configuration → Interactive console transport → USB Serial/JTAG`

The default UART0 mode is useful when the board should boot without waiting for a USB Serial/JTAG host. In this mode the example console uses the EPOS console UART settings, which default to UART0 on GPIO 43 and GPIO 44 at 115200 baud. Because those pins overlap with the example telemetry UART, the telemetry heartbeat is disabled in UART console mode.

This is the key configuration when using `linenoise`: line editing reads from standard input, so standard input must point to exactly the console transport you intend to use.
