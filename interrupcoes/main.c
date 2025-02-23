#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define LED_RED GPIO_NUM_2

#define BTN_RED GPIO_NUM_13

#define DELAY_TIME 200
 
volatile bool button_pressed = false;

static void gpio_isr_handler(void* arg) 
{
    button_pressed = true;
}

void button_config()
{
    gpio_install_isr_service(0);
    printf("configuring button\n");
    gpio_reset_pin(BTN_RED);
    gpio_set_direction(BTN_RED, GPIO_MODE_INPUT);
    gpio_pullup_en(BTN_RED);
    gpio_set_intr_type(BTN_RED, GPIO_INTR_POSEDGE);
    gpio_isr_handler_add(BTN_RED, gpio_isr_handler, NULL);
    printf("config complete\n");
}

void led_config()
{
    gpio_reset_pin(LED_RED);
    gpio_set_direction(LED_RED, GPIO_MODE_OUTPUT);
}

void app_main()
{   
    uint8_t led_value = 0;

    button_config();
    led_config();

    while (1) {
        if (button_pressed) {
            printf("*\n");
            button_pressed = false;
            led_value = !led_value;
            gpio_set_level(LED_RED, led_value);
        }
        vTaskDelay(DELAY_TIME / portTICK_PERIOD_MS);
    }
}
