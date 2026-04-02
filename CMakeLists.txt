idf_component_register(SRCS "src/moving_average.c" 
                            "src/motor.c" 
                            "src/canopen_server.c" 
                            "src/canopen.c" 
                            "src/canopen_console.c" 
                            "src/motor_console.c" 
                       REQUIRES esp_driver_gpio 
                                esp_driver_twai 
                                esp_driver_usb_serial_jtag 
                                lwip 
                                console 
                       INCLUDE_DIRS "src")