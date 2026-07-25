idf_component_register(SRCS "slave.c" INCLUDE_DIRS "."
                       REQUIRES elotto_camera esp_driver_uart esp_timer)
