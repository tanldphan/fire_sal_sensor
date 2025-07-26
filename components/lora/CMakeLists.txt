idf_component_register(SRCS "lora.c"
                    REQUIRES esp_driver_spi esp_driver_gpio
                    INCLUDE_DIRS "include")
