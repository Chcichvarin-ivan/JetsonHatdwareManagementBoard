/**
 * proto.h — Wire protocol between Jetson (master) and the board (slave).
 *
 * Register model: master writes [REG, payload...]; reads start by writing the
 * register pointer, then repeated-start read with auto-increment.
 *
 * Read block (status)   : 0x00 .. 0x19
 * Command frame (write) : 0x20, 5 bytes  (OPCODE, SEQ, ARG_L, ARG_H, CRC8)
 * Heartbeat   (write)   : 0x30, 7 bytes  (HB_SEQ, ALT_L, ALT_H, SPD_L, SPD_H, HB_FLAGS, CRC8)
 * Config      (write)   : 0x40, 7 bytes  (ALT_MIN_L/H, ALT_MAX_L/H, SPD_MAX_L/H, CRC8)
 */
#ifndef PROTO_H
#define PROTO_H

#include <stdint.h>

/* ------------------------- Read (status) registers ------------------------ */
#define REG_PROTO_VERSION   0x00
#define REG_FW_VERSION      0x01
#define REG_FSM_STATE       0x02
#define REG_TLM_STATE       0x03
#define REG_TLM_PULSE_US    0x04  /* u16 LE */
#define REG_SENSOR_STATE    0x06
#define REG_LAST_RESULT     0x07
#define REG_SEQ_ECHO        0x08
#define REG_ARM_TIME_LEFT   0x09  /* deprecated (auto-arm model): always 0 */
#define REG_CH1_PULSE_US    0x0A  /* u16 LE */
#define REG_CH2_PULSE_US    0x0C  /* u16 LE */
#define REG_FAULT_FLAGS     0x0E  /* u16 LE */
#define REG_PERMIT_FLAGS    0x10
#define REG_HB_AGE          0x11  /* x10 ms */
#define REG_HB_SEQ_ECHO     0x12
#define REG_ALT_ECHO        0x14  /* u16 LE, 0.1 m */
#define REG_SPD_ECHO        0x16  /* u16 LE, 0.01 m/s */
#define REG_BTN_EVENT       0x18
#define REG_BTN_SEQ         0x19
#define STATUS_BLOCK_SIZE   0x1A  /* number of readable status bytes */

/* ------------------------- Write frame registers -------------------------- */
#define REG_CMD             0x20
#define REG_HEARTBEAT       0x30
#define REG_CONFIG          0x40

#define CMD_FRAME_LEN       5u
#define HB_FRAME_LEN        7u
#define CFG_FRAME_LEN       7u

/* ------------------------------- Opcodes ---------------------------------- */
#define OP_NOP              0x00
#define OP_SET_STANDBY      0x10
#define OP_PUMP             0x20
#define OP_ARM              0x30  /* deprecated: auto-arm on pump confirm; ACK only if already ARMED */
#define OP_ACTUATE          0x31  /* ARG must equal 0x1825 (ACTUATE_ARG_KEY) unless disabled */
#define OP_DISARM           0x40
#define OP_CLEAR_FAULT      0x50
#define OP_FORCE_FAILSAFE   0xF0

/* ----------------------------- Result codes ------------------------------- */
#define RES_NONE            0x00
#define RES_ACK             0x01
#define RES_NACK            0x02
#define RES_ERR             0x03

/* --------------------------- FSM state codes ------------------------------ */
typedef enum {
    FSM_BOOT       = 0,
    FSM_STANDBY    = 1,
    FSM_PUMPING    = 2,
    FSM_READY      = 3,
    FSM_ARMED      = 4,
    FSM_ACTUATING  = 5,
    FSM_FAILSAFE   = 6,
    FSM_FAULT      = 7
} fsm_state_t;

/* ------------------------ Telemetry (Ch.4) states ------------------------- */
typedef enum {
    TLM_UNKNOWN     = 0,
    TLM_CONN_VERIFY = 1,
    TLM_IDLE        = 2,
    TLM_PUMP_RX     = 3,
    TLM_READY       = 4,
    TLM_ERROR       = 5
} tlm_state_t;

/* ----------------------------- PERMIT flags ------------------------------- */
#define PERMIT_HB_FRESH        (1u << 0)
#define PERMIT_NAV_VALID       (1u << 1)
#define PERMIT_ALT_OK          (1u << 2)
#define PERMIT_SPD_OK          (1u << 3)
#define PERMIT_MISSION         (1u << 4)
#define PERMIT_ARMED           (1u << 5)
#define PERMIT_TLM_READY       (1u << 6)  /* pump stage passed (FSM latch) AND
                                              telemetry alive & non-error; the
                                              circuit's READY level itself is
                                              transient, not held */
#define PERMIT_ACTUATE_ALLOWED (1u << 7)
/* Base conditions that must hold to ARM and to remain ARMED */
#define PERMIT_ARM_MASK  (PERMIT_HB_FRESH | PERMIT_NAV_VALID | PERMIT_ALT_OK | \
                          PERMIT_SPD_OK | PERMIT_MISSION | PERMIT_TLM_READY)

/* --------------------------- FAULT flags (16) ----------------------------- */
#define FAULT_COMMS_TIMEOUT      (1u << 0)
#define FAULT_TLM_TIMEOUT        (1u << 1)
#define FAULT_POWER_ERROR        (1u << 2)
#define FAULT_CRC_ERROR          (1u << 3)
#define FAULT_SEQ_GAP            (1u << 4)
#define FAULT_ARM_TIMEOUT        (1u << 5)   /* reserved (no arm window anymore) */
#define FAULT_SENSOR_UNEXPECTED  (1u << 6)
#define FAULT_INTERNAL           (1u << 7)
#define FAULT_ENVELOPE_VIOLATION (1u << 8)   /* reserved (violation now only blocks ACTUATE) */
#define FAULT_HB_STALE           (1u << 9)
#define FAULT_HB_CRC_ERROR       (1u << 10)
#define FAULT_CONFIG_MISSING     (1u << 11)
#define FAULT_PUMP_NOACK         (1u << 12)  /* pump command never confirmed by telemetry */

/* --------------------------- Heartbeat flags ------------------------------ */
#define HB_FLAG_NAV_VALID  (1u << 0)
#define HB_FLAG_MISSION    (1u << 1)

/* --------------------------- Button gesture codes ------------------------- */
#define BTN_CODE_NONE      0x00
#define BTN_CODE_PRESSED   0x01
#define BTN_CODE_LONG      0x02
#define BTN_CODE_DOUBLE    0x03

/* Parsed command frame */
typedef struct {
    uint8_t opcode;
    uint8_t seq;
    uint16_t arg;     /* ARG_L | ARG_H<<8 */
} app_command_t;

#endif /* PROTO_H */
