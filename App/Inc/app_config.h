/**
 * app_config.h — All wiring/timing tunables in one place.
 *
 * ============================ HARDWARE NOTES ===============================
 * PIN MAP (resolved — button moved off PA1 so TIM2_CH2 can use PA1):
 *    PA0  TIM2_CH1  Ch.1 pressure PWM   (out)
 *    PA1  TIM2_CH2  Ch.2 actuate PWM    (out)   <-- was the button
 *    PA3  TIM2_CH4  Ch.4 telemetry      (input capture)
 *    PA4  EXTI4     Ch.3 sensor (ADS)   (in, rising edge)
 *    PA5  GPIO (A4) user button         (in, pull-up)  (SB16/SB18 removed)
 *  TIM2 (PSC=79, ARR=19999) is fully set up in main.c (MX_TIM2_Init +
 *  HAL_TIM_Base_MspInit); App_Init() is called with &htim2.
 *
 * 1) BUTTON is on PA5 (A4), active-low + pull-up, driven by the gesture engine
 *    in main.c. (SB16/SB18 solder bridges removed so PA5 is a normal GPIO.)
 *    in main.c; the supervisor consumes BTN_GetEventQueue().
 *
 * 2) RGB = THREE SEPARATE LEDs, 3.3 V, no PWM, driven independently.
 *    Palette is therefore pure RED / GREEN / BLUE + blink only (no colour
 *    mixing). rgb_led.c lights exactly one colour at a time. Set the pins to
 *    your wiring; rgb_led_init() configures them as push-pull outputs.
 *    (RGB defaults PB4/PB5/PB0 — keep clear of LD3/LED and the PA0..PA5 block.)
 * ===========================================================================
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include "stm32l4xx_hal.h"

/* I2C slave identity (project uses OwnAddress1 = 128 -> 7-bit 0x40) */
#define APP_I2C_INSTANCE              I2C1

/* TIM2 (actuation): 80 MHz -> PSC=79 => 1 us tick; ARR=19999 => 20 ms / 50 Hz.
 * CCRx then equals pulse width in microseconds. Configure in CubeMX. */
#define TIM2_PSC                      79u
#define TIM2_ARR                      19999u
#define PWM_PERIOD_US                 20000u

#define PWM_CH1_TIM_CHANNEL           TIM_CHANNEL_1   /* Ch.1 pressure */
#define PWM_CH2_TIM_CHANNEL           TIM_CHANNEL_2   /* Ch.2 actuate  (see note 2) */
#define TLM_IC_TIM_CHANNEL            TIM_CHANNEL_4   /* Ch.4 telemetry (input capture) */
#define TLM_IC_ACTIVE_FLAG            HAL_TIM_ACTIVE_CHANNEL_4

/* External actuation sensor (ADS), Ch.3 — EXTI rising edge. PA4 is free today. */
#define ADS_GPIO_PORT                 GPIOA
#define ADS_GPIO_PIN                  GPIO_PIN_4

/* --- RGB: three independent GPIOs. >>> SET TO MATCH YOUR WIRING <<< --- */
#define RGB_R_PORT                    GPIOB
#define RGB_R_PIN                     GPIO_PIN_4
#define RGB_G_PORT                    GPIOB
#define RGB_G_PIN                     GPIO_PIN_5
#define RGB_B_PORT                    GPIOB
#define RGB_B_PIN                     GPIO_PIN_0
#define RGB_R_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define RGB_G_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define RGB_B_CLK_ENABLE()            __HAL_RCC_GPIOB_CLK_ENABLE()
#define RGB_ACTIVE_LOW                1   /* 1 if a pin drives its LED when low */

/* Button pin (single source of truth; used by the gesture engine in main.c,
 * the debug helper App_ButtonDebug(), and reference). */
#define BTN_GPIO_PORT                 GPIOA
#define BTN_GPIO_PIN                  GPIO_PIN_5

/* Signal durations (us) — fixed by the actuation-circuit interface */
#define US_STANDBY                    1150u
#define US_PUMP                       1650u
#define US_ACTUATE                    1825u
#define US_FAILSAFE                   1500u
#define PUMP_MIN_HOLD_MS              2000u   /* MINIMUM hold of 1650 us (spec: >= 2.0 s) */
/* CCR preload latches at the next timer update (one 20 ms period @ 50 Hz), so
 * a level change can start up to one period after the timestamp while its
 * release lands immediately on an update. Without margin the PHYSICAL hold can
 * fall one period short of the spec minimum in the worst phase alignment. */
#define PWM_LATCH_MARGIN_MS           20u
#define PUMP_CONFIRM_TIMEOUT_MS       10000u  /* max wait for telemetry READY while pumping */
#define ACTUATE_MIN_HOLD_MS           200u

/* Timeouts / windows (ms) */
#define COMMS_TIMEOUT_MS              300u
#define HB_FRESH_TIMEOUT_MS           300u
/* Arming model: the FSM AUTO-ARMS when the pump stage is confirmed by
 * telemetry (1400 us after the >= 2.0 s hold) and stays ARMED indefinitely.
 * There is no ARM command step and no arming window. Firing requires a
 * single ACTUATE whose ARG carries ACTUATE_ARG_KEY (single-frame
 * confirmation replacing the removed two-step arming; set
 * ACTUATE_REQUIRE_KEY to 0 to accept any ARG). */
#define ACTUATE_REQUIRE_KEY           1
#define ACTUATE_ARG_KEY               0x1825u
#define TLM_TIMEOUT_MS                250u
#define TLM_ERROR_CONFIRM_MS          500u    /* ERROR must persist this long to FAULT */
#define BOOT_TLM_SETTLE_MS            3000u   /* ignore circuit startup states in BOOT */
#define FAULT_SELF_HEAL_MS            2000u   /* pre-host: error clear this long -> recover */

/* Telemetry decode windows (us) */
#define TLM_WIN_CONN_LO               775u
#define TLM_WIN_CONN_HI               825u
#define TLM_WIN_IDLE_LO               875u
#define TLM_WIN_IDLE_HI               925u
#define TLM_WIN_PUMPRX_LO            1275u
#define TLM_WIN_PUMPRX_HI            1345u
#define TLM_WIN_READY_LO            1355u
#define TLM_WIN_READY_HI            1425u
#define TLM_WIN_ERROR_LO           2050u
#define TLM_WIN_ERROR_HI           2150u

/* Task loop periods (ms) */
#define ACTUATION_PERIOD_MS           10u
#define RGBLED_PERIOD_MS              40u
#define TELEMETRY_PERIOD_MS           20u

/* Versions reported over I2C */
#define PROTO_VERSION                 1u
#define FW_VERSION                    1u

/* Button behaviour:
 *   1 = "btn mode": the button ONLY reports its gesture (BTN_EVENT/BTN_SEQ)
 *       and gives an RGB confirmation blip. It does NOT command failsafe or
 *       clear-fault. This matches the original report-only button board.
 *   0 = the button also performs safe local actions (long=failsafe,
 *       double=clear-fault). It still can never pump/arm/actuate. */
#define APP_BTN_MODE                  0

/* APP_BTN_CTRL — СТЕНДОВОЕ управление боевой цепью с кнопки (НЕ для полёта!).
 *   0 = штатно: кнопка НЕ управляет накачкой/активацией (APP_BTN_MODE решает,
 *       докладывать жест или выполнять безопасные локальные действия);
 *   1 = отладка на стенде: одиночное нажатие = НАКАЧКА, двойное = АКТИВАЦИЯ.
 *       Активация по-прежнему проходит ВСЕ блокировки FSM (конверт, ARM и т.д.)
 *       — кнопка лишь подаёт те же команды, что хост. Одиночным/двойным
 *       нажатием нельзя обойти проверки; при снаряжённой БЧ флаг обязан быть 0.
 * Требует APP_BTN_MODE=0 (иначе кнопка компилируется в режиме «только доклад»).
 * Флаг перекрывает поведение handle_button ниже. */
#ifndef APP_BTN_CTRL
#define APP_BTN_CTRL                  1
#endif
#if APP_BTN_CTRL && APP_BTN_MODE
#error "APP_BTN_CTRL=1 requires APP_BTN_MODE=0 (report-only mode blocks local control)"
#endif

/* APP_ASSEMBLY_TEST — СБОРОЧНЫЙ тест платы БЕЗ I2C и (возможно) БЕЗ цепи.
 *   0 = штатно;
 *   1 = автономный стенд ОТК: I2C-хост не нужен. Плата сама формирует «свежий»
 *       конверт (heartbeat/NAV/MISSION/коридор), поэтому кнопка проводит полный
 *       цикл: 1 нажатие -> накачка, 2 нажатия -> активация, RGB сигнализирует.
 *   Что снимается ТОЛЬКО в этом режиме:
 *     - сторожевой таймер связи (нет I2C -> нет ложного ФС);
 *     - требование телеметрии READY для взведения: если цепь не подключена,
 *       накачка завершается по таймеру ASSEMBLY_PUMP_MS (а не по 1400 мкс);
 *     - источник конверта — внутренний, значения ниже.
 *   Требует APP_BTN_CTRL=1 (цикл ведётся кнопкой).
 *   ВНИМАНИЕ: сборочный режим намеренно обходит проверки безопасности —
 *   допустим ТОЛЬКО на столе ОТК с неснаряжённой БЧ; в полёте флаг обязан быть 0. */
#ifndef APP_ASSEMBLY_TEST
#define APP_ASSEMBLY_TEST             1
#endif
#if APP_ASSEMBLY_TEST && !APP_BTN_CTRL
#error "APP_ASSEMBLY_TEST=1 requires APP_BTN_CTRL=1 (button drives the cycle)"
#endif
/* Внутренний конверт и длительность накачки без цепи (сборочный режим). */
#define ASSEMBLY_ALT_DM               350u   /* 35.0 м — внутри коридора */
#define ASSEMBLY_SPD_CMS              120u   /* 1.20 м/с */
#define ASSEMBLY_ALT_MIN_DM           100u
#define ASSEMBLY_ALT_MAX_DM           500u
#define ASSEMBLY_SPD_MAX_CMS          300u
#define ASSEMBLY_PUMP_MS              2200u  /* накачка завершается по таймеру */
#define ASSEMBLY_ACTUATE_HOLD_MS      1000u  /* активация: 1825 мкс держится 1 с,
                                              * чтобы импульс был виден любым
                                              * прибором и медленным имитатором
                                              * (в полёте — ACTUATE_MIN_HOLD_MS) */

/* Enable the debug-console actuation bench-test commands (ext ...) and the
 * heartbeat simulator. Set to 0 for a flight build to compile them out so the
 * hose cannot be fired from the console. */
#define APP_ENABLE_ACT_TEST           1

#endif /* APP_CONFIG_H */
