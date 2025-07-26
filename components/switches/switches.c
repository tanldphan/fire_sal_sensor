#include "include/switches.h"

void switches_init()
{
    gpio_config_t pms_bme_switch_config = 
        { 
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = (1ULL << PMS_SWITCH_PIN) | (1ULL << BME_SWITCH_PIN),
            .pull_down_en = 0,
            .pull_up_en = 0,
        };
    gpio_config (&pms_bme_switch_config);

    gpio_set_level(PMS_SWITCH_PIN, 1);
    gpio_set_level(BME_SWITCH_PIN, 1);
}
