# EPOS ESP-IDF console plus independent UART example

This example keeps the interactive `esp_console`/`linenoise` console on USB Serial/JTAG and uses a separate UART for periodic bytes on GPIO 43 and GPIO 44.

The important part is that the telemetry UART is never attached to VFS or standard I/O. `stdin`, `stdout` and `stderr` remain owned by the USB-JTAG console, while UART1 is used only through `uart_write_bytes()`.

## Hardware

- Target with USB Serial/JTAG and GPIO 43/44 available, for example ESP32-S3.
- UART TX: GPIO 43.
- UART RX: GPIO 44.
- UART speed: 115200 8N1.
- CAN in this example: TX GPIO 14, RX GPIO 15, 1 Mbit/s.

Do not use GPIO 19 or GPIO 20 for CAN when the console is on USB Serial/JTAG on ESP32-S3. Those pins are used by native USB, and reconfiguring them for TWAI can disconnect the monitor.

Adjust `EXAMPLE_UART_TX_PIN`, `EXAMPLE_UART_RX_PIN` or the CAN pins in `main/main.c` if your board uses different pins.

## Build and flash

```bash
idf.py set-target esp32s3
idf.py build flash monitor
```

The monitor opens the USB-JTAG console. Use `help` or `about` to see the EPOS/CANopen commands.

The second UART periodically prints messages like:

```text
EPOS UART heartbeat 1
EPOS UART heartbeat 2
```

## Console configuration

`sdkconfig.defaults` selects USB Serial/JTAG as the primary console and disables the secondary console:

- `CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG=y`
- `CONFIG_ESP_CONSOLE_SECONDARY_NONE=y`

This is the key configuration when using `linenoise`: line editing reads from standard input, so standard input must point to exactly the console transport you intend to use.
