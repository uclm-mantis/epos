idf_component_register(SRCS "src/moving_average.c" "src/epos_motor.c" "src/epos.c" "src/epos_console.c" "src/epos_motor_console.c" "src/EposCAN.cpp"
                       REQUIRES driver console nvs_flash esp_timer
                       INCLUDE_DIRS "include")