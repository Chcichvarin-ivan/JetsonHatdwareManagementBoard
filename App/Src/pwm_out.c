#include "pwm_out.h"
#include "app_config.h"
#include "app_status.h"

/* Tolerates a NULL timer handle: before TIM2 is configured in CubeMX the FSM
 * still runs and the intended pulse widths are still reflected in status; only
 * the hardware CCR writes are skipped. */
static TIM_HandleTypeDef *s_htim;
static uint16_t s_ch1_us;
static uint16_t s_ch2_us;

static inline uint16_t clamp_us(uint32_t us)
{
    if (us < 500u)  us = 500u;
    if (us > 2500u) us = 2500u;
    return (uint16_t)us;
}

void pwm_out_init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;
    s_ch1_us = US_STANDBY;
    s_ch2_us = US_STANDBY;
    if (s_htim) {
        __HAL_TIM_SET_COMPARE(s_htim, PWM_CH1_TIM_CHANNEL, s_ch1_us);
        __HAL_TIM_SET_COMPARE(s_htim, PWM_CH2_TIM_CHANNEL, s_ch2_us);
        HAL_TIM_PWM_Start(s_htim, PWM_CH1_TIM_CHANNEL);
        HAL_TIM_PWM_Start(s_htim, PWM_CH2_TIM_CHANNEL);
    }
    status_set_pwm(s_ch1_us, s_ch2_us);
}

void pwm_set_ch1_us(uint16_t us)
{
    s_ch1_us = clamp_us(us);
    if (s_htim) __HAL_TIM_SET_COMPARE(s_htim, PWM_CH1_TIM_CHANNEL, s_ch1_us);
    status_set_pwm(s_ch1_us, s_ch2_us);
}

void pwm_set_ch2_us(uint16_t us)
{
    s_ch2_us = clamp_us(us);
    if (s_htim) __HAL_TIM_SET_COMPARE(s_htim, PWM_CH2_TIM_CHANNEL, s_ch2_us);
    status_set_pwm(s_ch1_us, s_ch2_us);
}

void pwm_set_failsafe(void)
{
    pwm_set_ch1_us(US_FAILSAFE);
    pwm_set_ch2_us(US_FAILSAFE);
}
