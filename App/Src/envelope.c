#include "envelope.h"
#include "app_config.h"
#include "app_status.h"

#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t s_last_hb_ms;
static volatile uint8_t  s_hb_seq;
static volatile uint16_t s_alt;       /* 0.1 m  */
static volatile uint16_t s_spd;       /* 0.01 m/s */
static volatile uint8_t  s_hb_flags;
static volatile uint8_t  s_have_hb;

void envelope_init(void)
{
    s_last_hb_ms = 0;
    s_hb_seq = 0;
    s_alt = 0; s_spd = 0; s_hb_flags = 0;
    s_have_hb = 0;
}

void envelope_on_heartbeat(uint8_t hb_seq, uint16_t alt, uint16_t spd, uint8_t hb_flags)
{
    s_hb_seq    = hb_seq;
    s_alt       = alt;
    s_spd       = spd;
    s_hb_flags  = hb_flags;
    s_last_hb_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_have_hb   = 1;
}

uint32_t envelope_last_hb_ms(void) { return s_last_hb_ms; }

uint8_t envelope_hb_fresh(uint32_t now_ms)
{
    if (!s_have_hb) return 0;
    return ((now_ms - s_last_hb_ms) <= HB_FRESH_TIMEOUT_MS) ? 1 : 0;
}

uint8_t envelope_compute(uint32_t now_ms, uint8_t fsm_state, uint8_t tlm_state)
{
    uint8_t permit = 0;

    uint8_t fresh = envelope_hb_fresh(now_ms);
    if (fresh) permit |= PERMIT_HB_FRESH;
    else       fault_set(FAULT_HB_STALE);
    if (fresh) fault_clear(FAULT_HB_STALE);

    if (s_hb_flags & HB_FLAG_NAV_VALID) permit |= PERMIT_NAV_VALID;
    if (s_hb_flags & HB_FLAG_MISSION)   permit |= PERMIT_MISSION;

    uint16_t amin, amax, smax;
    if (status_get_config(&amin, &amax, &smax)) {
        if (s_alt >= amin && s_alt <= amax) permit |= PERMIT_ALT_OK;
        if (s_spd <= smax)                  permit |= PERMIT_SPD_OK;
    } /* else: corridor not configured -> ALT_OK/SPD_OK stay 0 (blocked) */

    if (tlm_state == TLM_READY)   permit |= PERMIT_TLM_READY;
    if (fsm_state == FSM_ARMED)   permit |= PERMIT_ARMED;

    if ((permit & PERMIT_ARM_MASK) == PERMIT_ARM_MASK && (permit & PERMIT_ARMED)) {
        permit |= PERMIT_ACTUATE_ALLOWED;
    }

    /* hb age clamped to x10 ms units */
    uint32_t age_ms = s_have_hb ? (now_ms - s_last_hb_ms) : 0xFFFFu;
    uint32_t age10  = age_ms / 10u;
    uint8_t  hb_age = (age10 > 255u) ? 255u : (uint8_t)age10;

    status_set_permit(permit, hb_age, s_hb_seq, s_alt, s_spd);
    return permit;
}
