/**
 * app_status.h — Shared "shadow" status, updated by tasks and snapshotted by
 * the I2C service. Access is short and guarded by FreeRTOS critical sections.
 *
 * The flight-envelope corridor configuration also lives here.
 */
#ifndef APP_STATUS_H
#define APP_STATUS_H

#include <stdint.h>
#include "proto.h"

typedef struct {
    /* status (readable) */
    uint8_t  fsm_state;
    uint8_t  tlm_state;
    uint16_t tlm_pulse_us;
    uint8_t  sensor_state;
    uint8_t  last_result;
    uint8_t  seq_echo;
    uint8_t  arm_time_left;   /* x100 ms */
    uint16_t ch1_pulse_us;
    uint16_t ch2_pulse_us;
    uint16_t fault_flags;
    uint8_t  permit_flags;
    uint8_t  hb_age;          /* x10 ms (clamped 0..255) */
    uint8_t  hb_seq_echo;
    uint16_t alt_echo;        /* 0.1 m */
    uint16_t spd_echo;        /* 0.01 m/s */
    uint8_t  btn_event;
    uint8_t  btn_seq;

    /* corridor config (set via REG_CONFIG); valid only when config_valid != 0 */
    uint8_t  config_valid;
    uint16_t alt_min;         /* 0.1 m */
    uint16_t alt_max;         /* 0.1 m */
    uint16_t spd_max;         /* 0.01 m/s */
} app_status_t;

void     status_init(void);

/* Atomic helpers (critical-section guarded) */
void     status_set_fsm(uint8_t state);
void     status_set_pwm(uint16_t ch1_us, uint16_t ch2_us);
void     status_set_tlm(uint8_t tlm_state, uint16_t pulse_us);
void     status_set_sensor(uint8_t s);
void     status_set_result(uint8_t seq, uint8_t result);
void     status_set_arm_time_left(uint8_t x100ms);
void     status_set_permit(uint8_t permit, uint8_t hb_age, uint8_t hb_seq,
                           uint16_t alt_echo, uint16_t spd_echo);
void     status_set_button(uint8_t code);   /* increments btn_seq */

void     fault_set(uint16_t mask);
void     fault_clear(uint16_t mask);
uint16_t fault_get(void);

/* Corridor config */
void     status_set_config(uint16_t alt_min, uint16_t alt_max, uint16_t spd_max);
uint8_t  status_get_config(uint16_t *alt_min, uint16_t *alt_max, uint16_t *spd_max);

/* Build the contiguous status byte block [0 .. STATUS_BLOCK_SIZE) */
void     status_snapshot(uint8_t *out);

#endif /* APP_STATUS_H */
