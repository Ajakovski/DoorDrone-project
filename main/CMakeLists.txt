idf_component_register(SRCS "main.c" "imu.c"
                    INCLUDE_DIRS "."
                    REQUIRES mqtt esp_wifi nvs_flash json driver freertos)