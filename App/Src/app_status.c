#include "app_status.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static app_status_t g;

/* ISR-safe: these functions run from both tasks and the I2C event ISR.
 * PRIMASK save/disable/restore is legal in every context; the guarded copies
 * are a few dozen bytes, so interrupts are masked for well under 1 us. */
#define ENTER()  uint32_t crit_pm_ = app_crit_enter()
#define EXIT()   app_crit_exit(crit_pm_)

void status_init(void)
{
    memset(&g, 0, sizeof(g));
    g.fsm_state    = FSM_BOOT;
    g.tlm_state    = TLM_UNKNOWN;
    g.last_result  = RES_NONE;
    g.config_valid = 0;
    /* No corridor yet -> activation blocked, flag it */
    g.fault_flags  = FAULT_CONFIG_MISSING;
}

void status_set_fsm(uint8_t state)
{
    ENTER(); g.fsm_state = state; EXIT();
}

void status_set_pwm(uint16_t ch1_us, uint16_t ch2_us)
{
    ENTER(); g.ch1_pulse_us = ch1_us; g.ch2_pulse_us = ch2_us; EXIT();
}

void status_set_tlm(uint8_t tlm_state, uint16_t pulse_us)
{
    ENTER(); g.tlm_state = tlm_state; g.tlm_pulse_us = pulse_us; EXIT();
}

void status_set_sensor(uint8_t s)
{
    ENTER(); g.sensor_state = s; EXIT();
}

void status_set_result(uint8_t seq, uint8_t result)
{
    ENTER(); g.seq_echo = seq; g.last_result = result; EXIT();
}

void status_set_arm_time_left(uint8_t x100ms)
{
    ENTER(); g.arm_time_left = x100ms; EXIT();
}

void status_set_permit(uint8_t permit, uint8_t hb_age, uint8_t hb_seq,
                       uint16_t alt_echo, uint16_t spd_echo)
{
    ENTER();
    g.permit_flags = permit;
    g.hb_age       = hb_age;
    g.hb_seq_echo  = hb_seq;
    g.alt_echo     = alt_echo;
    g.spd_echo     = spd_echo;
    EXIT();
}

void status_set_button(uint8_t code)
{
    ENTER(); g.btn_event = code; g.btn_seq++; EXIT();
}

void fault_set(uint16_t mask)   { ENTER(); g.fault_flags |= mask;  EXIT(); }
void fault_clear(uint16_t mask) { ENTER(); g.fault_flags &= ~mask; EXIT(); }

uint16_t fault_get(void)
{
    uint16_t v; ENTER(); v = g.fault_flags; EXIT(); return v;
}

void status_set_config(uint16_t alt_min, uint16_t alt_max, uint16_t spd_max)
{
    ENTER();
    g.alt_min = alt_min; g.alt_max = alt_max; g.spd_max = spd_max;
    g.config_valid = 1;
    g.fault_flags &= ~FAULT_CONFIG_MISSING;
    EXIT();
}

uint8_t status_get_config(uint16_t *alt_min, uint16_t *alt_max, uint16_t *spd_max)
{
    uint8_t v;
    ENTER();
    v = g.config_valid;
    if (alt_min) *alt_min = g.alt_min;
    if (alt_max) *alt_max = g.alt_max;
    if (spd_max) *spd_max = g.spd_max;
    EXIT();
    return v;
}

static inline void put16(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

void status_snapshot(uint8_t *out)
{
    ENTER();
    out[REG_PROTO_VERSION] = PROTO_VERSION;
    out[REG_FW_VERSION]    = FW_VERSION;
    out[REG_FSM_STATE]     = g.fsm_state;
    out[REG_TLM_STATE]     = g.tlm_state;
    put16(&out[REG_TLM_PULSE_US], g.tlm_pulse_us);
    out[REG_SENSOR_STATE]  = g.sensor_state;
    out[REG_LAST_RESULT]   = g.last_result;
    out[REG_SEQ_ECHO]      = g.seq_echo;
    out[REG_ARM_TIME_LEFT] = g.arm_time_left;
    put16(&out[REG_CH1_PULSE_US], g.ch1_pulse_us);
    put16(&out[REG_CH2_PULSE_US], g.ch2_pulse_us);
    put16(&out[REG_FAULT_FLAGS],  g.fault_flags);
    out[REG_PERMIT_FLAGS]  = g.permit_flags;
    out[REG_HB_AGE]        = g.hb_age;
    out[REG_HB_SEQ_ECHO]   = g.hb_seq_echo;
    put16(&out[REG_ALT_ECHO], g.alt_echo);
    put16(&out[REG_SPD_ECHO], g.spd_echo);
    out[REG_BTN_EVENT]     = g.btn_event;
    out[REG_BTN_SEQ]       = g.btn_seq;
    EXIT();
}
