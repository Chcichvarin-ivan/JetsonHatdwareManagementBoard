#include "rgb_led.h"
#include "app_config.h"
#include "proto.h"

/* Discrete LEDs: exactly one colour lit at a time (no mixing). */
typedef enum { C_OFF, C_RED, C_GREEN, C_BLUE } color_t;
typedef enum { PAT_SOLID, PAT_SLOW, PAT_FAST } pattern_t;

static void pin(GPIO_TypeDef *port, uint16_t p, uint8_t on)
{
#if RGB_ACTIVE_LOW
    on = (uint8_t)!on;
#endif
    HAL_GPIO_WritePin(port, p, on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void drive(color_t c, uint8_t on)
{
    pin(RGB_R_PORT, RGB_R_PIN, (uint8_t)(on && c == C_RED));
    pin(RGB_G_PORT, RGB_G_PIN, (uint8_t)(on && c == C_GREEN));
    pin(RGB_B_PORT, RGB_B_PIN, (uint8_t)(on && c == C_BLUE));
}

void rgb_led_init(void)
{
    RGB_R_CLK_ENABLE();
    RGB_G_CLK_ENABLE();
    RGB_B_CLK_ENABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Mode  = GPIO_MODE_OUTPUT_PP;
    gi.Pull  = GPIO_NOPULL;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin = RGB_R_PIN; HAL_GPIO_Init(RGB_R_PORT, &gi);
    gi.Pin = RGB_G_PIN; HAL_GPIO_Init(RGB_G_PORT, &gi);
    gi.Pin = RGB_B_PIN; HAL_GPIO_Init(RGB_B_PORT, &gi);

    drive(C_OFF, 0);
}

/* Priority resolve (first match wins). Pure colours only.
 *   RED   fast  = FAULT
 *   BLUE  fast  = link/heartbeat lost (also BOOT: no link yet)
 *   RED   solid = ACTUATING (firing now)
 *   RED   slow  = FAILSAFE (commanded/ button, link still OK)
 *   GREEN fast  = ARMED (caution: armed, not yet firing)
 *   GREEN slow  = READY (pumped, awaiting ARM)
 *   BLUE  slow  = PUMPING (working)
 *   GREEN solid = STANDBY (idle, healthy)
 */
static color_t resolve(uint8_t fsm, uint16_t fault, uint8_t hb_fresh, pattern_t *pat)
{
    if (fsm == FSM_FAULT)                                    { *pat = PAT_FAST;  return C_RED;   }
    if (!hb_fresh || (fault & (FAULT_COMMS_TIMEOUT | FAULT_HB_STALE)))
                                                             { *pat = PAT_FAST;  return C_BLUE;  }
    if (fsm == FSM_ACTUATING)                                { *pat = PAT_SOLID; return C_RED;   }
    if (fsm == FSM_FAILSAFE)                                 { *pat = PAT_SLOW;  return C_RED;   }
    if (fsm == FSM_ARMED)                                    { *pat = PAT_FAST;  return C_GREEN; }
    if (fsm == FSM_READY)                                    { *pat = PAT_SLOW;  return C_GREEN; }
    if (fsm == FSM_PUMPING)                                  { *pat = PAT_SLOW;  return C_BLUE;  }
    if (fsm == FSM_STANDBY)                                  { *pat = PAT_SOLID; return C_GREEN; }
    *pat = PAT_FAST; return C_BLUE;    /* BOOT / unknown: treat as no-link */
}

static uint8_t pattern_on(pattern_t pat, uint32_t now_ms)
{
    switch (pat) {
        case PAT_SOLID: return 1;
        case PAT_SLOW:  return (now_ms % 800u) < 400u;   /* ~1.25 Hz */
        case PAT_FAST:  return (now_ms % 200u) < 100u;   /* 5 Hz */
        default:        return 0;
    }
}

void rgb_led_update(uint32_t now_ms, uint8_t fsm_state, uint16_t fault_flags,
                    uint8_t hb_fresh, uint8_t btn_code, uint8_t btn_seq)
{
#if APP_BTN_MODE
    /* Show a short colour blip whenever a new button gesture is reported, so a
     * press is visible on the bench without a Jetson:
     *   single = green, double = blue, long = red (all fast blink). */
    static uint8_t  s_seq;
    static uint8_t  s_seq_init;
    static uint8_t  s_blip_code;
    static uint32_t s_blip_until;

    if (!s_seq_init) { s_seq = btn_seq; s_seq_init = 1; }
    if (btn_seq != s_seq) {
        s_seq = btn_seq;
        s_blip_code = btn_code;
        s_blip_until = now_ms + 300u;           /* 300 ms confirmation */
    }
    if ((int32_t)(s_blip_until - now_ms) > 0) {
        color_t bc = (s_blip_code == BTN_CODE_LONG)   ? C_RED  :
                     (s_blip_code == BTN_CODE_DOUBLE) ? C_BLUE : C_GREEN;
        drive(bc, pattern_on(PAT_FAST, now_ms));
        return;
    }
#else
    (void)btn_code; (void)btn_seq;
#endif
    pattern_t pat;
    color_t c = resolve(fsm_state, fault_flags, hb_fresh, &pat);
    drive(c, pattern_on(pat, now_ms));
}
