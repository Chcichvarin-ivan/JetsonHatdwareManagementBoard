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
static uint8_t     s_last_seq;
static uint8_t     s_seq_seen;
static uint8_t     s_tlm_err_active;     /* ERROR debounce tracking */
static uint32_t    s_tlm_err_since;
static uint32_t    s_fault_clear_since;  /* pre-host self-heal timer */

static void enter(fsm_state_t st, uint32_t now_ms)
{
    s_state = st;
    s_enter_ms = now_ms;
    status_set_fsm((uint8_t)st);
}

static void to_failsafe(uint32_t now_ms) { pwm_set_failsafe(); enter(FSM_FAILSAFE, now_ms); }
static void to_fault(uint32_t now_ms)    { pwm_set_failsafe(); enter(FSM_FAULT, now_ms); }

/* STANDBY is the ONLY door back into operation, so the terminal-fault guard
 * lives here rather than at each call site. With a terminal fault latched every
 * route to STANDBY — host SET_STANDBY/DISARM, FailSafe recovery, pre-host
 * self-heal — is redirected to FAULT. Without this guard the latch could be
 * walked around: FORCE_FAILSAFE from FAULT lands in FAILSAFE, which recovers to
 * STANDBY on a fresh heartbeat, and the board would accept a new PUMP with the
 * unacknowledged circuit still unfixed. Only a power cycle / reset clears it. */
static void to_standby(uint32_t now_ms)
{
    if (fault_get() & FAULT_TERMINAL_MASK) { to_fault(now_ms); return; }
    pwm_set_ch1_us(US_STANDBY);
    pwm_set_ch2_us(US_STANDBY);
    enter(FSM_STANDBY, now_ms);
}

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
        /* Deprecated: arming is automatic on pump confirmation. Kept as an
         * idempotent probe: ACK if already armed, NACK otherwise. */
        if (s_state != FSM_ARMED) result = RES_NACK;
        break;
    case OP_ACTUATE:
        if (s_state == FSM_ARMED && (permit & PERMIT_ACTUATE_ALLOWED)
#if ACTUATE_REQUIRE_KEY
            && cmd->arg == ACTUATE_ARG_KEY
#endif
           ) {
            pwm_set_ch1_us(US_PUMP);
            pwm_set_ch2_us(US_ACTUATE);
            sensor_as_clear();
            enter(FSM_ACTUATING, now_ms);
        } else result = RES_NACK;
        break;
    case OP_DISARM:
        if (s_state != FSM_FAULT) to_standby(now_ms);   /* ARMED incl.: back to standby */
        break;
    case OP_CLEAR_FAULT:
        /* Terminal faults are never cleared by command (see FAULT_TERMINAL_MASK). */
        if (fault_get() & FAULT_TERMINAL_MASK) {
            result = RES_NACK;
        } else if (!fault_source_active(tlm_state)) {
            fault_clear((uint16_t)(0xFFFFu & ~FAULT_CONFIG_MISSING
                                          & ~FAULT_TERMINAL_MASK));
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
static void handle_command(const app_command_t *cmd, uint32_t now_ms,
                           uint8_t permit, uint8_t tlm_state);

static void handle_button(btn_evt_type_t type, uint32_t now_ms,
                          uint8_t tlm_state, uint8_t permit)
{
#if APP_BTN_CTRL
    /* СТЕНД: одиночное = НАКАЧКА, двойное = АКТИВАЦИЯ. Через ту же handle_command,
     * поэтому ВСЕ блокировки FSM сохраняются (кнопка не привилегирована).
     * Синтетическая команда: seq наследует последовательность, чтобы не
     * взводить FAULT_SEQ_GAP; ключ активации подставляется при необходимости. */
    app_command_t bc;
    bc.seq = (uint8_t)(s_last_seq + 1u);
    bc.arg = 0;
    switch (type) {
    case BTN_EVT_PRESSED:
        status_set_button(BTN_CODE_PRESSED);
        bc.opcode = OP_PUMP;
        handle_command(&bc, now_ms, permit, tlm_state);
        break;
    case BTN_EVT_DOUBLE_PRESS:
        status_set_button(BTN_CODE_DOUBLE);
        bc.opcode = OP_ACTUATE;
#if ACTUATE_REQUIRE_KEY
        bc.arg = ACTUATE_ARG_KEY;     /* стендовая кнопка знает ключ */
#endif
        handle_command(&bc, now_ms, permit, tlm_state);
        break;
    case BTN_EVT_LONG_PRESS:
        /* Длинное — аварийный сброс в ФС (безопасное действие, оставляем). */
        status_set_button(BTN_CODE_LONG);
        to_failsafe(now_ms);
        break;
    default: break;
    }
    (void)tlm_state; (void)permit;
    return;
#elif APP_BTN_MODE
    /* Report-only: post the gesture; no local control actions. */
    (void)now_ms; (void)tlm_state; (void)permit;
    switch (type) {
    case BTN_EVT_PRESSED:      status_set_button(BTN_CODE_PRESSED); break;
    case BTN_EVT_DOUBLE_PRESS: status_set_button(BTN_CODE_DOUBLE);  break;
    case BTN_EVT_LONG_PRESS:   status_set_button(BTN_CODE_LONG);    break;
    default: break;
    }
#else
    (void)permit;
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

static uint32_t s_boot0_ms;   /* метка включения питания для грации TLM ERROR */

void actuation_task(void *arg)
{
    (void)arg;
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    pwm_set_ch1_us(US_STANDBY);
    pwm_set_ch2_us(US_STANDBY);
    enter(FSM_BOOT, now);
    s_boot0_ms = now;

    const TickType_t period = pdMS_TO_TICKS(ACTUATION_PERIOD_MS);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        health_stamp(HEALTH_ACTUATION);

        if (s_btn_q == NULL) s_btn_q = App_FsmButtonQueue();  /* dedicated FSM queue */

#if APP_ASSEMBLY_TEST
        /* Сборочный режим: I2C-хоста нет. Каждый тик подаём внутренний «свежий»
         * конверт и один раз — коридор, чтобы кнопка провела полный цикл.
         * envelope_on_heartbeat обновляет метку времени, поэтому HB всегда свеж
         * и сторожевой таймер связи не срабатывает. */
        envelope_on_heartbeat((uint8_t)(now / 100u),
                              ASSEMBLY_ALT_DM, ASSEMBLY_SPD_CMS,
                              (uint8_t)(HB_FLAG_NAV_VALID | HB_FLAG_MISSION));
        {
            uint16_t a0, a1, s0;
            if (!status_get_config(&a0, &a1, &s0)) {
                status_set_config(ASSEMBLY_ALT_MIN_DM, ASSEMBLY_ALT_MAX_DM,
                                  ASSEMBLY_SPD_MAX_CMS);
            }
        }
#endif

        uint8_t tlm    = (uint8_t)telemetry_get_state();
        /* Телеметрия НЕ маскируется даже в сборочном режиме: имитатор/цепь на
         * Кан.4 (PA3) — объект проверки (IDLE -> PUMP_RX -> READY; ERROR с
         * восстановлением). Единственная поблажка сборочного режима живёт в
         * telemetry_tick(): МОЛЧАЩАЯ линия (имитатор выключен/не подключён)
         * деградирует в IDLE без отказа, вместо TLM_TIMEOUT. */
        uint8_t permit = envelope_compute(now, (uint8_t)s_state, tlm);
#if APP_ASSEMBLY_TEST
        uint8_t link_ok = 1u;          /* нет шины — но связь считаем живой */
#else
        uint8_t link_ok = comms_ok(now);
#endif

        if (!link_ok) fault_set(FAULT_COMMS_TIMEOUT);
        else          fault_clear(FAULT_COMMS_TIMEOUT);

        if (sensor_as_triggered()) status_set_sensor(1);

        /* highest-priority safety transitions.
         * TLM ERROR is DEBOUNCED before faulting: at a simultaneous power-up
         * the circuit runs its own startup while our pins settle, and may emit
         * or briefly latch its error state. A single reading must not brick
         * the board. ERROR must persist TLM_ERROR_CONFIRM_MS, and during BOOT
         * the first BOOT_TLM_SETTLE_MS are ignored entirely. */
        uint8_t tlm_err_confirmed = 0;
        if (tlm == TLM_ERROR) {
            if (!s_tlm_err_active) { s_tlm_err_active = 1; s_tlm_err_since = now; }
            /* Грация меряется от ВКЛЮЧЕНИЯ ПИТАНИЯ, а не от пребывания в
             * BOOT: в сборочном режиме (и при автономном старте) BOOT выходит
             * в ДЕЖУРНЫЙ за ~250 мс, и стартовый ERROR самой цепи/имитатора
             * прилетал уже ПОСЛЕ окончания грации -> ложный FAULT на старте
             * с самовосстановлением через 2 с. Задержка защёлки безопасна:
             * пока ERROR активен, permit-цепочка (tlm_alive) всё равно
             * блокирует взведение и активацию. */
            uint8_t settled = ((now - s_boot0_ms) > BOOT_TLM_SETTLE_MS) ? 1u : 0u;
#if APP_ASSEMBLY_TEST
            /* Стенд: имитатор может грузиться дольше любой фиксированной
             * грации. Пока с включения не пришло НИ ОДНОГО валидного импульса
             * телеметрии (реальные IDLE/PUMP_RX/READY), «ERROR с рождения»
             * считаем его собственной загрузкой и защёлку не ставим. После
             * первого валидного импульса действует обычный дебаунс 500 мс. */
            if (!telemetry_valid_pulse_seen()) settled = 0u;
#endif
            tlm_err_confirmed = (settled &&
                (now - s_tlm_err_since) >= TLM_ERROR_CONFIRM_MS) ? 1u : 0u;
        } else {
            s_tlm_err_active = 0;
        }
        if (tlm_err_confirmed && s_state != FSM_FAULT) {
            fault_set(FAULT_POWER_ERROR);
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
                handle_button(ev.type, now, tlm, permit);
            }
        }

        /* per-state behaviour */
        switch (s_state) {
        case FSM_BOOT:
            if (link_ok && tlm == TLM_IDLE) to_standby(now);
            break;
        case FSM_STANDBY:
            break;
        case FSM_PUMPING: {
            /* Hold Ch.1 = 1650 us until the circuit confirms (TLM READY).
             * PUMP_MIN_HOLD_MS is the spec MINIMUM ("hold >= 2.0 s"), not the
             * total: releasing at exactly 2 s while telemetry is still IDLE
             * deadlocks the sequence (circuit never confirms a command that is
             * no longer present). A confirmation timeout returns to standby
             * with FAULT_PUMP_NOACK so the failure is visible, not silent. */
            uint32_t held = now - s_enter_ms;
#if APP_ASSEMBLY_TEST
            /* Цепь может быть не подключена: взводим по таймеру, а не по 1400 мкс.
             * Если реальная цепь ЕСТЬ и подтверждает раньше — используем это. */
            uint8_t confirmed = (tlm == TLM_READY) || (held >= ASSEMBLY_PUMP_MS);
            if (confirmed && held >= (PUMP_MIN_HOLD_MS + PWM_LATCH_MARGIN_MS)) {
#else
            if (tlm == TLM_READY && held >= (PUMP_MIN_HOLD_MS + PWM_LATCH_MARGIN_MS)) {
#endif
                pwm_set_ch1_us(US_STANDBY);
                enter(FSM_ARMED, now);   /* AUTO-ARM on hardware confirmation */
            } else if (held >= PUMP_CONFIRM_TIMEOUT_MS) {
                /* TERMINAL: the circuit never acknowledged the pump command.
                 * Latch and stay in FAULT — safe levels, no CLEAR_FAULT, no
                 * self-heal. Only a power cycle / reset resumes operation. */
                fault_set(FAULT_PUMP_NOACK);
                to_fault(now);
            }
            break;
        }
        case FSM_READY:      /* legacy state, unreachable in the auto-arm model */
            break;
        case FSM_ARMED:
            /* Indefinite: firing is gated by the LIVE permit at ACTUATE time
             * (PERMIT_FLAGS reflects it continuously); leaving the corridor
             * simply blocks ACTUATE, it no longer kicks the state machine. */
            status_set_arm_time_left(0);
            break;
        case FSM_ACTUATING:
            /* +PWM_LATCH_MARGIN_MS guarantees the PHYSICAL 1825 us level meets
             * the >= 200 ms spec despite CCR-preload phase alignment. */
#if APP_ASSEMBLY_TEST
            if ((now - s_enter_ms) >= (ASSEMBLY_ACTUATE_HOLD_MS + PWM_LATCH_MARGIN_MS)) {
#else
            if ((now - s_enter_ms) >= (ACTUATE_MIN_HOLD_MS + PWM_LATCH_MARGIN_MS)) {  /* never cut short */
#endif
                if (sensor_as_triggered()) status_set_sensor(1);
#if APP_ASSEMBLY_TEST
  //              to_standby(now);   /* нет линии: после удержания — возврат в ДЕЖУРНЫЙ */
#else
                if (!link_ok) to_failsafe(now);
                else          to_standby(now);
#endif
            }
            break;
        case FSM_FAILSAFE:
            status_set_arm_time_left(0);
            /* Recovery is attempted via to_standby(), which redirects to FAULT
             * while a terminal fault is latched (see to_standby). */
            if (link_ok && envelope_hb_fresh(now)) to_standby(now);
            break;
        case FSM_FAULT:
            status_set_arm_time_left(0);
            /* Pre-host self-heal: a FAULT entered before ANY host contact
             * (standalone power-up ordering) must not brick the board until a
             * Jetson arrives to send CLEAR_FAULT. If the telemetry error has
             * cleared and stayed clear, recover to STANDBY autonomously.
             * After first host contact, recovery is host-owned as designed. */
            if (!comms_seen() && !s_tlm_err_active &&
                !(fault_get() & FAULT_TERMINAL_MASK) &&
                tlm != TLM_ERROR && tlm != TLM_UNKNOWN) {
                if (s_fault_clear_since == 0u) s_fault_clear_since = now;
                if ((now - s_fault_clear_since) >= FAULT_SELF_HEAL_MS) {
                    fault_clear((uint16_t)(0xFFFFu & ~FAULT_CONFIG_MISSING
                                                  & ~FAULT_TERMINAL_MASK));
                    s_fault_clear_since = 0u;
                    to_standby(now);
                }
            } else {
                s_fault_clear_since = 0u;
            }
            break;
        default:
            to_failsafe(now);
            break;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}
