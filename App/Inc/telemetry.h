/**
 * telemetry.h — Decode the actuation circuit's Ch.4 PWM telemetry (TIM2_CH4).
 *
 * Technique: capture rising edge -> record t_r, flip polarity to falling;
 * on falling -> width = (t_f - t_r) mod (ARR+1); classify into a tlm_state_t.
 * Runs entirely in the capture ISR; a small task watches for capture timeout.
 */
#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "proto.h"

void        telemetry_init(TIM_HandleTypeDef *htim);
/* Called from HAL_TIM_IC_CaptureCallback for the telemetry channel */
void        telemetry_on_capture(TIM_HandleTypeDef *htim);
/* Latest decoded state + measured pulse; updates TLM_TIMEOUT fault on timeout */
tlm_state_t telemetry_get_state(void);
uint16_t    telemetry_get_pulse_us(void);
uint8_t     telemetry_valid_pulse_seen(void);
/* Periodic check (called by telemetryTask) — flags timeout if stale */
void        telemetry_tick(uint32_t now_ms);

#endif /* TELEMETRY_H */
