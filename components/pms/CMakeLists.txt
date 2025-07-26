idf_component_register(SRCS "pms.c"
                    REQUIRES esp_driver_uart esp_driver_gpio
                    INCLUDE_DIRS "include")
