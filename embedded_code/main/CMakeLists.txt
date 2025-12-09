idf_component_register(SRCS "main_code.c"
                       PRIV_REQUIRES esp_driver_pcnt esp_driver_gpio driver esp_timer
                       INCLUDE_DIRS ".")
