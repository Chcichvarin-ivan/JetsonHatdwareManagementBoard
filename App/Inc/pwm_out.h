/** pwm_out.h — TIM2 CH1/CH2 PWM, pulse width expressed directly in microseconds. */
#ifndef PWM_OUT_H
#define PWM_OUT_H

#include <stdint.h>
#include "stm32l4xx_hal.h"

void pwm_out_init(TIM_HandleTypeDef *htim);  /* starts both PWM channels at US_STANDBY */
void pwm_set_ch1_us(uint16_t us);            /* pressure channel */
void pwm_set_ch2_us(uint16_t us);            /* actuation channel */
void pwm_set_failsafe(void);                 /* both -> US_FAILSAFE */

#endif /* PWM_OUT_H */
