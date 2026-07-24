#include "telemetry.h"
#include "app_config.h"
#include "app_status.h"

#include "FreeRTOS.h"
#include "task.h"

/* Tolerates a NULL timer handle: without TIM2 the capture is simply never
 * started, telemetry stays UNKNOWN and telemetry_tick() raises TLM_TIMEOUT. */
static TIM_HandleTypeDef *s_htim;
static volatile uint32_t  s_rise;
static volatile uint8_t   s_have_rise;
static volatile uint16_t  s_pulse_us;
static volatile uint32_t  s_last_capture_ms;
static volatile tlm_state_t s_state;

static volatile uint8_t s_valid_seen;   /* был ли РЕАЛЬНЫЙ валидный импульс с линии */

static tlm_state_t classify(uint16_t us)
{
    if (us >= TLM_WIN_CONN_LO   && us <= TLM_WIN_CONN_HI)   return TLM_CONN_VERIFY;
    if (us >= TLM_WIN_IDLE_LO   && us <= TLM_WIN_IDLE_HI)   return TLM_IDLE;
    if (us >= TLM_WIN_PUMPRX_LO && us <= TLM_WIN_PUMPRX_HI) return TLM_PUMP_RX;
    if (us >= TLM_WIN_READY_LO  && us <= TLM_WIN_READY_HI)  return TLM_READY;
    if (us >= TLM_WIN_ERROR_LO  && us <= TLM_WIN_ERROR_HI)  return TLM_ERROR;
    return TLM_UNKNOWN;
}

void telemetry_init(TIM_HandleTypeDef *htim)
{
    s_htim = htim;
    s_have_rise = 0;
    s_pulse_us = 0;
    s_state = TLM_UNKNOWN;
    s_last_capture_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (s_htim) {
        __HAL_TIM_SET_CAPTUREPOLARITY(s_htim, TLM_IC_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
        HAL_TIM_IC_Start_IT(s_htim, TLM_IC_TIM_CHANNEL);
    }
}

void telemetry_on_capture(TIM_HandleTypeDef *htim)
{
    if (htim->Channel != TLM_IC_ACTIVE_FLAG) return;

    uint32_t cap = HAL_TIM_ReadCapturedValue(htim, TLM_IC_TIM_CHANNEL);

    if (!s_have_rise) {
        s_rise = cap;
        s_have_rise = 1;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TLM_IC_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_FALLING);
    } else {
        uint32_t width = (cap >= s_rise) ? (cap - s_rise) : (cap + PWM_PERIOD_US - s_rise);
        if (width > 0 && width < PWM_PERIOD_US) {
            s_pulse_us = (uint16_t)width;
            s_state = classify(s_pulse_us);
            s_last_capture_ms = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
            if (s_state == TLM_IDLE || s_state == TLM_PUMP_RX ||
                s_state == TLM_READY || s_state == TLM_CONN_VERIFY) {
                s_valid_seen = 1u;
            }
        }
        s_have_rise = 0;
        __HAL_TIM_SET_CAPTUREPOLARITY(htim, TLM_IC_TIM_CHANNEL, TIM_INPUTCHANNELPOLARITY_RISING);
    }
}

tlm_state_t telemetry_get_state(void)    { return s_state; }
uint16_t    telemetry_get_pulse_us(void) { return s_pulse_us; }
uint8_t     telemetry_valid_pulse_seen(void) { return s_valid_seen; }

void telemetry_tick(uint32_t now_ms)
{
#if APP_ASSEMBLY_TEST
    /* Сборочный стол: исполнительной цепи может не быть, телеметрия молчит.
     * Не поднимаем FAULT_TLM_TIMEOUT по отсутствию импульсов. Если реальные
     * импульсы приходят — s_state отражает их (classify в ISR), иначе держим
     * IDLE, чтобы health-gate не блокировал взведение. */
    if ((now_ms - s_last_capture_ms) > TLM_TIMEOUT_MS) {
        s_state = TLM_IDLE;          /* «живо», не UNKNOWN и не ERROR */
    }
    fault_clear(FAULT_TLM_TIMEOUT);
#else
    uint32_t last = s_last_capture_ms;
    if ((now_ms - last) > TLM_TIMEOUT_MS) {
        s_state = TLM_UNKNOWN;
        fault_set(FAULT_TLM_TIMEOUT);
    } else {
        fault_clear(FAULT_TLM_TIMEOUT);
    }
#endif
    /* FAULT_POWER_ERROR is latched by the supervisor only after the ERROR
     * state is CONFIRMED (debounced) — see actuation.c. A single reading
     * during the circuit's own power-up must not latch a fault. */
    status_set_tlm((uint8_t)s_state, s_pulse_us);
}
