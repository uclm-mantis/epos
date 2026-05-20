idf_component_register(
    SRCS "main.c"
    REQUIRES epos driver esp_driver_rmt esp_driver_usb_serial_jtag console esp_common nvs_flash
)
