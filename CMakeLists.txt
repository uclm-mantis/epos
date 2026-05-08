idf_component_register(SRCS "src/moving_average.c" 
                            "src/motor.c" 
                            "src/canopen_server.c" 
                            "src/canopen.c" 
                            "src/canopen_console.c" 
                            "src/motor_console.c" 
                       REQUIRES driver
                                console
                                lwip
                       INCLUDE_DIRS "src")
