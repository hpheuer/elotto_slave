idf_component_register(SRCS "slave.c" INCLUDE_DIRS "."
                       REQUIRES elotto_camera elotto_gcp elotto_link elotto_ota
                                esp_timer esp_eth esp_netif esp_event
                                esp_http_server nvs_flash lwip
                                esp_app_format)
