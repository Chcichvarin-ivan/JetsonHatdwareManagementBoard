/**
 * envelope.h — Heartbeat ingest and flight-envelope permit computation.
 *
 * The heartbeat carries (HB_SEQ, ALT, SPD, HB_FLAGS). It both proves link
 * liveness and gates activation: PERMIT_FLAGS is recomputed from corridor
 * config + heartbeat freshness + telemetry readiness + FSM armed state.
 */
#ifndef ENVELOPE_H
#define ENVELOPE_H

#include <stdint.h>
#include "proto.h"

void    envelope_init(void);

/* Called by i2c_reg on a CRC-valid heartbeat frame (already parsed). */
void    envelope_on_heartbeat(uint8_t hb_seq, uint16_t alt, uint16_t spd, uint8_t hb_flags);

/* Recompute permit given current FSM state and telemetry; returns PERMIT_FLAGS.
 * Also pushes hb_age / echoes / permit into app_status and raises HB_STALE. */
uint8_t envelope_compute(uint32_t now_ms, uint8_t fsm_state, uint8_t tlm_state);

uint8_t envelope_hb_fresh(uint32_t now_ms);
uint32_t envelope_last_hb_ms(void);

#endif /* ENVELOPE_H */
