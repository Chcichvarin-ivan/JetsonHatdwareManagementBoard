#include "app.h"
#include <stddef.h>
#include "app_config.h"
#include "app_status.h"
#include "proto.h"
#include "pwm_out.h"
#include "telemetry.h"
#include "sensor_as.h"
#include "envelope.h"
#include "comms_wdg.h"
#include "rgb_led.h"
#include "actuation.h"
#include "i2c_reg.h"
#include "btn_def.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/* Provided by the project's buttonTask (main.c). */
extern QueueHandle_t BTN_GetEventQueue(void);

/* ---- static RTOS allocations (consistent with the project's static use) ---- */
#define CMD_Q_LEN   8
static StaticQueue_t s_cmdq_buf;
static uint8_t       s_cmdq_store[CMD_Q_LEN * sizeof(app_command_t)];
static QueueHandle_t s_cmd_q;

/* Dedicated button queue for the actuation FSM (fan-out target). */
#define FSM_BTN_Q_LEN  8
static StaticQueue_t s_fsm_btnq_buf;
static uint8_t       s_fsm_btnq_store[FSM_BTN_Q_LEN * sizeof(btn_event_msg_t)];
static QueueHandle_t s_fsm_btn_q;

#define ACT_STACK   384
#define TLM_STACK   256
static StaticTask_t s_act_tcb;  static StackType_t s_act_stk[ACT_STACK];
static StaticTask_t s_tlm_tcb;  static StackType_t s_tlm_stk[TLM_STACK];
#if APP_ENABLE_ACT_TEST
#define HBSIM_STACK 192
static StaticTask_t s_hbsim_tcb; static StackType_t s_hbsim_stk[HBSIM_STACK];
static uint8_t  s_test_seq;
static uint8_t  s_hb_seq;
static volatile uint8_t  s_hb_auto;
static volatile uint16_t s_hb_alt, s_hb_spd;
static volatile uint8_t  s_hb_flags;
#endif

QueueHandle_t App_GetButtonQueue(void) { return BTN_GetEventQueue(); }
QueueHandle_t App_FsmButtonQueue(void) { return s_fsm_btn_q; }

void App_PostButtonEvent(const btn_event_msg_t *ev)
{
    if (s_fsm_btn_q && ev) (void)xQueueSend(s_fsm_btn_q, ev, 0);
}

static void telemetry_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(TELEMETRY_PERIOD_MS);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        health_stamp(HEALTH_TELEMETRY);
        telemetry_tick(now);
        vTaskDelayUntil(&last, period);
    }
}

#if APP_ENABLE_ACT_TEST
static uint8_t test_cmd(uint8_t opcode, uint16_t arg)
{
    app_command_t c = { .opcode = opcode, .seq = (uint8_t)(++s_test_seq), .arg = arg };
    return (s_cmd_q && xQueueSend(s_cmd_q, &c, 0) == pdPASS) ? 1u : 0u;
}
uint8_t App_TestStandby(void)    { return test_cmd(OP_SET_STANDBY, 0); }
uint8_t App_TestPump(void)       { return test_cmd(OP_PUMP, 0); }
uint8_t App_TestArm(void)        { return test_cmd(OP_ARM, 0); }
uint8_t App_TestFire(void)       { return test_cmd(OP_ACTUATE, 0); }
uint8_t App_TestDisarm(void)     { return test_cmd(OP_DISARM, 0); }
uint8_t App_TestClearFault(void) { return test_cmd(OP_CLEAR_FAULT, 0); }
uint8_t App_TestFailsafe(void)   { return test_cmd(OP_FORCE_FAILSAFE, 0); }

void App_TestHeartbeat(uint16_t alt_dm, uint16_t spd_cms, uint8_t flags)
{
    envelope_on_heartbeat((uint8_t)(++s_hb_seq), alt_dm, spd_cms, flags);
}
void App_TestHeartbeatAuto(uint8_t on, uint16_t alt_dm, uint16_t spd_cms, uint8_t flags)
{
    s_hb_alt = alt_dm; s_hb_spd = spd_cms; s_hb_flags = flags;
    s_hb_auto = on ? 1u : 0u;
}
void App_TestConfig(uint16_t alt_min_dm, uint16_t alt_max_dm, uint16_t spd_max_cms)
{
    status_set_config(alt_min_dm, alt_max_dm, spd_max_cms);
}
void App_TestGetStatus(app_test_status_t *out)
{
    if (!out) return;
    uint8_t s[STATUS_BLOCK_SIZE];
    status_snapshot(s);
    out->fsm         = s[REG_FSM_STATE];
    out->tlm_state   = s[REG_TLM_STATE];
    out->tlm_us      = (uint16_t)s[REG_TLM_PULSE_US] | ((uint16_t)s[REG_TLM_PULSE_US + 1] << 8);
    out->fault       = (uint16_t)s[REG_FAULT_FLAGS]  | ((uint16_t)s[REG_FAULT_FLAGS + 1] << 8);
    out->permit      = s[REG_PERMIT_FLAGS];
    out->hb_age10    = s[REG_HB_AGE];
    out->ch1_us      = (uint16_t)s[REG_CH1_PULSE_US] | ((uint16_t)s[REG_CH1_PULSE_US + 1] << 8);
    out->ch2_us      = (uint16_t)s[REG_CH2_PULSE_US] | ((uint16_t)s[REG_CH2_PULSE_US + 1] << 8);
    out->arm_left100 = s[REG_ARM_TIME_LEFT];
}

/* 10 Hz heartbeat simulator: keeps the envelope fresh during manual testing
 * (manual typing can't sustain the 300 ms freshness window otherwise). */
static void hbsim_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(100);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        if (s_hb_auto)
            envelope_on_heartbeat((uint8_t)(++s_hb_seq), s_hb_alt, s_hb_spd, s_hb_flags);
        vTaskDelayUntil(&last, period);
    }
}
#endif /* APP_ENABLE_ACT_TEST */

void App_Init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim_act)
{
    status_init();
    comms_wdg_init();
    envelope_init();
    sensor_as_init();

    pwm_out_init(htim_act);      /* NULL tolerated */
    telemetry_init(htim_act);    /* NULL tolerated */
    rgb_led_init();

    s_cmd_q = xQueueCreateStatic(CMD_Q_LEN, sizeof(app_command_t), s_cmdq_store, &s_cmdq_buf);
    s_fsm_btn_q = xQueueCreateStatic(FSM_BTN_Q_LEN, sizeof(btn_event_msg_t), s_fsm_btnq_store, &s_fsm_btnq_buf);

    actuation_init(s_cmd_q);
    i2c_reg_init(hi2c, s_cmd_q);

    xTaskCreateStatic(actuation_task, "act", ACT_STACK, NULL, tskIDLE_PRIORITY + 3, s_act_stk, &s_act_tcb);
    xTaskCreateStatic(telemetry_task, "tlm", TLM_STACK, NULL, tskIDLE_PRIORITY + 2, s_tlm_stk, &s_tlm_tcb);
#if APP_ENABLE_ACT_TEST
    xTaskCreateStatic(hbsim_task, "hbsim", HBSIM_STACK, NULL, tskIDLE_PRIORITY + 1, s_hbsim_stk, &s_hbsim_tcb);
#endif
}

void App_LedTick(void)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    health_stamp(HEALTH_RGBLED);

    uint8_t snap[STATUS_BLOCK_SIZE];
    status_snapshot(snap);
    uint8_t  fsm   = snap[REG_FSM_STATE];
    uint16_t fault = (uint16_t)snap[REG_FAULT_FLAGS] | ((uint16_t)snap[REG_FAULT_FLAGS + 1] << 8);
    uint8_t  fresh = envelope_hb_fresh(now);
    uint8_t  btn_code = snap[REG_BTN_EVENT];
    uint8_t  btn_seq  = snap[REG_BTN_SEQ];
    rgb_led_update(now, fsm, fault, fresh, btn_code, btn_seq);
}

void App_NoteButtonAlive(void) { health_stamp(HEALTH_BUTTON); }

void App_ButtonDebug(uint8_t *pressed, uint8_t *last_code, uint8_t *seq)
{
    if (pressed) {
        /* active-low: pressed when the pin reads low */
        *pressed = (HAL_GPIO_ReadPin(BTN_GPIO_PORT, BTN_GPIO_PIN) == GPIO_PIN_RESET) ? 1u : 0u;
    }
    uint8_t snap[STATUS_BLOCK_SIZE];
    status_snapshot(snap);
    if (last_code) *last_code = snap[REG_BTN_EVENT];
    if (seq)       *seq       = snap[REG_BTN_SEQ];
}

uint8_t App_WwdgShouldRefresh(void)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return health_all_ok(now);
}

/* ============================ HAL callback routers ========================= */
/* These replace the ones that lived in the removed i2c_app.c. */

void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t dir, uint16_t code)
{
    (void)code;
    i2c_reg_on_addr(hi2c, dir);
}
void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)  { i2c_reg_on_rx_cplt(hi2c); }
void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)  { i2c_reg_on_tx_cplt(hi2c); }
void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)  { i2c_reg_on_listen_cplt(hi2c); }
void HAL_I2C_ErrorCallback      (I2C_HandleTypeDef *hi2c)  { i2c_reg_on_error(hi2c); }

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)   { telemetry_on_capture(htim); }

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)             { sensor_as_on_exti(GPIO_Pin); }
