/**
 * sensor_as.h — External actuation sensor (ADS), Ch.3, EXTI rising edge.
 * The pulse can be as short as 0.2 ms, so it is latched in the ISR, never polled.
 */
#ifndef SENSOR_AS_H
#define SENSOR_AS_H

#include <stdint.h>

void    sensor_as_init(void);
/* Call from HAL_GPIO_EXTI_Callback when ADS_GPIO_PIN fires */
void    sensor_as_on_exti(uint16_t gpio_pin);
/* 1 if an actuation edge has been latched since the last clear */
uint8_t sensor_as_triggered(void);
void    sensor_as_clear(void);
uint32_t sensor_as_last_ms(void);

#endif /* SENSOR_AS_H */
