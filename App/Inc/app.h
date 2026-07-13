/**
 * app.h — Integration entry points for the actuation-circuit firmware.
 *
 * Call App_Init() once from main() (USER CODE BEGIN RTOS_EVENTS). It replaces
 * the old I2C_SlaveApp_Init(): sets up the register-model I2C slave, the
 * supervisory FSM, telemetry, sensor, envelope interlock and RGB status.
 *
 * The existing buttonTask (main.c) is reused unchanged except for calling
 * App_NoteButtonAlive() each loop; the old ledTask body is replaced by a call
 * to App_LedTick().
 */
#ifndef APP_H
#define APP_H

#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "btn_def.h"   /* btn_event_msg_t */

/* htim_act: TIM2 handle for actuation (Ch.1/Ch.2 PWM + Ch.4 capture), or NULL
 * until TIM2 is configured in CubeMX — everything else still runs. */
void     App_Init(I2C_HandleTypeDef *hi2c, TIM_HandleTypeDef *htim_act);

/* Call from the (repurposed) ledTask loop; drives the RGB status indicator. */
void     App_LedTick(void);

/* Call from the existing buttonTask loop so the watchdog sees it alive. */
void     App_NoteButtonAlive(void);

/* Return 1 only if every required task is alive; call from the WWDG EWI. */
uint8_t  App_WwdgShouldRefresh(void);

/* Button event queue created by the project's buttonTask (BTN_GetEventQueue).
 * This queue is now consumed ONLY by the debug console's `btn session`. */
QueueHandle_t App_GetButtonQueue(void);

/* Fan-out: the gesture engine (main.c) calls App_PostButtonEvent() for EVERY
 * gesture so the actuation FSM always gets its own copy, independent of the
 * console. The FSM reads App_FsmButtonQueue(). This keeps the button's safety
 * actions (failsafe on long-press) from being stolen by the console session. */
void          App_PostButtonEvent(const btn_event_msg_t *ev);
QueueHandle_t App_FsmButtonQueue(void);

/* Debug/console helper: live button state.
 *   pressed   -> 1 if the button is currently pressed (active-low), else 0
 *   last_code -> last reported gesture: 1=press, 2=long, 3=double (0=none)
 *   seq       -> gesture counter (increments on each new gesture)
 * Any argument may be NULL. Safe to call from the debug task. */
void App_ButtonDebug(uint8_t *pressed, uint8_t *last_code, uint8_t *seq);

/* ---- Actuation / fire-extinguishing bench-test API ------------------------
 * These drive the REAL supervisory FSM through the REAL interlocks (arming,
 * flight-envelope permit, minimum-hold). They do NOT bypass safety: ACTUATE
 * only fires when the FSM is armed AND PERMIT_ACTUATE_ALLOWED is set. Intended
 * for bench bring-up (see the `ext` console command). Units: alt in 0.1 m,
 * spd in 0.01 m/s, flags default 0x03 (NAV_VALID | MISSION_PERMIT). */
typedef struct {
    uint8_t  fsm;          /* fsm_state_t */
    uint8_t  tlm_state;    /* tlm_state_t */
    uint16_t tlm_us;       /* last Ch.4 pulse, us */
    uint16_t fault;        /* FAULT_FLAGS (16-bit) */
    uint8_t  permit;       /* PERMIT_FLAGS */
    uint8_t  hb_age10;     /* heartbeat age, x10 ms */
    uint16_t ch1_us;       /* Ch.1 pulse being driven, us */
    uint16_t ch2_us;       /* Ch.2 pulse being driven, us */
    uint8_t  arm_left100;  /* arm window remaining, x100 ms */
} app_test_status_t;

uint8_t App_TestStandby(void);      /* SET_STANDBY */
uint8_t App_TestPump(void);         /* PUMP        */
uint8_t App_TestArm(void);          /* ARM         */
uint8_t App_TestFire(void);         /* ACTUATE     */
uint8_t App_TestDisarm(void);       /* DISARM      */
uint8_t App_TestClearFault(void);   /* CLEAR_FAULT */
uint8_t App_TestFailsafe(void);     /* FORCE_FAILSAFE */
void    App_TestHeartbeat(uint16_t alt_dm, uint16_t spd_cms, uint8_t flags);
void    App_TestHeartbeatAuto(uint8_t on, uint16_t alt_dm, uint16_t spd_cms, uint8_t flags);
void    App_TestConfig(uint16_t alt_min_dm, uint16_t alt_max_dm, uint16_t spd_max_cms);
void    App_TestGetStatus(app_test_status_t *out);

#endif /* APP_H */
