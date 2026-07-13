#include "actuation.h"
#include "app.h"            /* App_FsmButtonQueue() */
#include "app_config.h"
#include "app_status.h"
#include "pwm_out.h"
#include "telemetry.h"
#include "sensor_as.h"
#include "envelope.h"
#include "comms_wdg.h"
#include "proto.h"
#include "btn_def.h"        /* project's button event types */

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

static QueueHandle_t s_cmd_q;
static QueueHandle_t s_btn_q;   /* fetched at runtime from the button engine */

static fsm_state_t s_state;
static uint32_t    s_enter_ms;
static uint32_t    s_arm_deadline;
static uint8_t     s_last_seq;
static uint8_t     s_seq_seen;

static void enter(fsm_state_t st, uint32_t now_ms)
{
    s_state = st;
    s_enter_ms = now_ms;
    status_set_fsm((uint8_t)st);
}

static void to_standby(uint32_t now_ms)
{
    pwm_set_ch1_us(US_STANDBY);
    pwm_set_ch2_us(US_STANDBY);
    enter(FSM_STANDBY, now_ms);
}
static void to_failsafe(uint32_t now_ms) { pwm_set_failsafe(); enter(FSM_FAILSAFE, now_ms); }
static void to_fault(uint32_t now_ms)    { pwm_set_failsafe(); enter(FSM_FAULT, now_ms); }

static uint8_t fault_source_active(uint8_t tlm_state)
{
    if (tlm_state == TLM_ERROR) return 1;
    if (fault_get() & (FAULT_COMMS_TIMEOUT | FAULT_HB_STALE | FAULT_CONFIG_MISSING)) return 1;
    return 0;
}

static void handle_command(const app_command_t *cmd, uint32_t now_ms,
                           uint8_t permit, uint8_t tlm_state)
{
    uint8_t result = RES_ACK;

    if (s_seq_seen && (uint8_t)(s_last_seq + 1u) != cmd->seq && cmd->opcode != OP_NOP) {
        fault_set(FAULT_SEQ_GAP);
    }
    s_last_seq = cmd->seq;
    s_seq_seen = 1;

    switch (cmd->opcode) {
    case OP_NOP:
        break;
    case OP_SET_STANDBY:
        to_standby(now_ms);
        break;
    case OP_PUMP:
        if (s_state == FSM_STANDBY) { pwm_set_ch1_us(US_PUMP); enter(FSM_PUMPING, now_ms); }
        else result = RES_NACK;
        break;
    case OP_ARM:
        if (s_state == FSM_READY && (permit & PERMIT_ARM_MASK) == PERMIT_ARM_MASK) {
            s_arm_deadline = now_ms + ARM_WINDOW_MS;
            pwm_set_ch2_us(US_STANDBY);
            enter(FSM_ARMED, now_ms);
        } else result = RES_NACK;
        break;
    case OP_ACTUATE:
        if (s_state == FSM_ARMED && (permit & PERMIT_ACTUATE_ALLOWED)) {
            pwm_set_ch2_us(US_ACTUATE);
            sensor_as_clear();
            enter(FSM_ACTUATING, now_ms);
        } else result = RES_NACK;
        break;
    case OP_DISARM:
        if (s_state == FSM_ARMED)      enter(FSM_READY, now_ms);
        else if (s_state != FSM_FAULT) to_standby(now_ms);
        break;
    case OP_CLEAR_FAULT:
        if (!fault_source_active(tlm_state)) {
            fault_clear(0xFFFFu & ~FAULT_CONFIG_MISSING);
            if (s_state == FSM_FAULT) to_standby(now_ms);
        } else result = RES_NACK;
        break;
    case OP_FORCE_FAILSAFE:
        to_failsafe(now_ms);
        break;
    default:
        result = RES_ERR;
        break;
    }
    status_set_result(cmd->seq, result);
}

/* Map a raw button gesture to a SAFE local action + reflect it in BTN_EVENT.
 * The button can NEVER pump, arm or actuate. */
static void handle_button(btn_evt_type_t type, uint32_t now_ms, uint8_t tlm_state)
{
#if APP_BTN_MODE
    /* Report-only: post the gesture; no local control actions. */
    (void)now_ms; (void)tlm_state;
    switch (type) {
    case BTN_EVT_PRESSED:      status_set_button(BTN_CODE_PRESSED); break;
    case BTN_EVT_DOUBLE_PRESS: status_set_button(BTN_CODE_DOUBLE);  break;
    case BTN_EVT_LONG_PRESS:   status_set_button(BTN_CODE_LONG);    break;
    default: break;
    }
#else
    switch (type) {
    case BTN_EVT_PRESSED:      /* acknowledge only */
        status_set_button(BTN_CODE_PRESSED);
        break;
    case BTN_EVT_DOUBLE_PRESS: /* clear fault if the source is gone */
        status_set_button(BTN_CODE_DOUBLE);
        if (!fault_source_active(tlm_state)) {
            fault_clear(0xFFFFu & ~FAULT_CONFIG_MISSING);
            if (s_state == FSM_FAULT) to_standby(now_ms);
        }
        break;
    case BTN_EVT_LONG_PRESS:   /* safe abort -> failsafe */
        status_set_button(BTN_CODE_LONG);
        to_failsafe(now_ms);
        break;
    default:
        break;
    }
#endif
}

void actuation_init(QueueHandle_t cmd_queue)
{
    s_cmd_q = cmd_queue;
    s_btn_q = NULL;
    s_state = FSM_BOOT;
    s_seq_seen = 0;
    s_last_seq = 0;
}

void actuation_task(void *arg)
{
    (void)arg;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    pwm_set_ch1_us(US_STANDBY);
    pwm_set_ch2_us(US_STANDBY);
    enter(FSM_BOOT, now);

    const TickType_t period = pdMS_TO_TICKS(ACTUATION_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        health_stamp(HEALTH_ACTUATION);

        if (s_btn_q == NULL) s_btn_q = App_FsmButtonQueue();  /* dedicated FSM queue */

        uint8_t tlm    = (uint8_t)telemetry_get_state();
        uint8_t permit = envelope_compute(now, (uint8_t)s_state, tlm);
        uint8_t link_ok = comms_ok(now);

        if (!link_ok) fault_set(FAULT_COMMS_TIMEOUT);
        else          fault_clear(FAULT_COMMS_TIMEOUT);

        if (sensor_as_triggered()) status_set_sensor(1);

        /* highest-priority safety transitions */
        if (tlm == TLM_ERROR && s_state != FSM_FAULT) {
            to_fault(now);
        } else if (!link_ok && s_state != FSM_FAILSAFE && s_state != FSM_FAULT
                   && s_state != FSM_ACTUATING) {
            to_failsafe(now);
        }

        /* one command + one button gesture per tick */
        app_command_t cmd;
        if (xQueueReceive(s_cmd_q, &cmd, 0) == pdPASS) {
            if (s_state != FSM_FAULT || cmd.opcode == OP_CLEAR_FAULT
                                     || cmd.opcode == OP_FORCE_FAILSAFE) {
                handle_command(&cmd, now, permit, tlm);
            } else {
                status_set_result(cmd.seq, RES_NACK);
            }
        }
        if (s_btn_q != NULL) {
            btn_event_msg_t ev;
            if (xQueueReceive(s_btn_q, &ev, 0) == pdPASS) {
                handle_button(ev.type, now, tlm);
            }
        }

        /* per-state behaviour */
        switch (s_state) {
        case FSM_BOOT:
            if (link_ok && tlm == TLM_IDLE) to_standby(now);
            break;
        case FSM_STANDBY:
            break;
        case FSM_PUMPING:
            if ((now - s_enter_ms) >= PUMP_MIN_HOLD_MS) {
                pwm_set_ch1_us(US_STANDBY);
                if (tlm == TLM_READY) enter(FSM_READY, now);
            }
            break;
        case FSM_READY:
            break;
        case FSM_ARMED: {
            uint32_t left = (s_arm_deadline > now) ? (s_arm_deadline - now) : 0;
            uint32_t left10 = left / 100u;
            status_set_arm_time_left(left10 > 255u ? 255u : (uint8_t)left10);
            if ((permit & PERMIT_ARM_MASK) != PERMIT_ARM_MASK) {
                fault_set(FAULT_ENVELOPE_VIOLATION);
                enter(FSM_READY, now);          /* auto-disarm */
            } else if (now >= s_arm_deadline) {
                fault_set(FAULT_ARM_TIMEOUT);
                enter(FSM_READY, now);
            }
            break;
        }
        case FSM_ACTUATING:
            if ((now - s_enter_ms) >= ACTUATE_MIN_HOLD_MS) {  /* never cut short */
                if (sensor_as_triggered()) status_set_sensor(1);
                if (!link_ok) to_failsafe(now);
                else          to_standby(now);
            }
            break;
        case FSM_FAILSAFE:
            status_set_arm_time_left(0);
            if (link_ok && envelope_hb_fresh(now)) to_standby(now);
            break;
        case FSM_FAULT:
            status_set_arm_time_left(0);
            break;
        default:
            to_failsafe(now);
            break;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
