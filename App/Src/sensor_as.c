#include "sensor_as.h"
#include "stm32l4xx_hal.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"

static volatile uint8_t  s_latched;
static volatile uint32_t s_last_ms;

void sensor_as_init(void)
{
    s_latched = 0;
    s_last_ms = 0;
    /* EXTI line + NVIC are configured by CubeMX (rising edge on ADS_GPIO_PIN). */
}

void sensor_as_on_exti(uint16_t gpio_pin)
{
    if (gpio_pin == ADS_GPIO_PIN) {
        s_latched = 1;
        s_last_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    }
}

uint8_t  sensor_as_triggered(void) { return s_latched; }
void     sensor_as_clear(void)     { s_latched = 0; }
uint32_t sensor_as_last_ms(void)   { return s_last_ms; }
