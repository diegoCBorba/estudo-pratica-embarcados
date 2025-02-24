#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"  // Use the new ADC driver
#include "driver/gpio.h"

#define ADC_CHANNEL ADC_CHANNEL_6  // GPIO34
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ADC_ATTEN ADC_ATTEN_DB_12  // Use ADC_ATTEN_DB_12 instead of ADC_ATTEN_DB_11

#define LED_COUNT 10
const int led_pins[LED_COUNT] = {2, 4, 5, 18, 19, 21, 22, 23, 25, 26};

adc_oneshot_unit_handle_t adc1_handle;

void configure_adc() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    adc_oneshot_new_unit(&init_config, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_WIDTH,
        .atten = ADC_ATTEN,
    };
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL, &config);
}

void configure_leds() {
    for (int i = 0; i < LED_COUNT; i++) {
        esp_rom_gpio_pad_select_gpio(led_pins[i]);  // Use the new function
        gpio_set_direction(led_pins[i], GPIO_MODE_OUTPUT);
    }
}

void update_led_bar(int value) {
    int level = (value * LED_COUNT) / 4095;
    for (int i = 0; i < LED_COUNT; i++) {
        gpio_set_level(led_pins[i], i < level ? 1 : 0);
    }
}

void app_main() {
    configure_adc();
    configure_leds();

    while (1) {
        int adc_value = 0;
        adc_oneshot_read(adc1_handle, ADC_CHANNEL, &adc_value);
        printf("ADC Value: %d\n", adc_value);
        update_led_bar(adc_value);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
