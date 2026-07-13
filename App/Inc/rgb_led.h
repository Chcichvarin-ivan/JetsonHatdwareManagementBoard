/**
 * rgb_led.h — Status indicator on THREE separate LEDs (R/G/B), 3.3 V, no PWM.
 *
 * Only one pure colour is lit at a time (no mixing); sub-state is encoded by
 * blink rate (solid / slow / fast). Fault and link-loss override the nominal
 * FSM colour. rgb_led_init() configures the three pins as push-pull outputs.
 */
#ifndef RGB_LED_H
#define RGB_LED_H

#include <stdint.h>

void rgb_led_init(void);
void rgb_led_update(uint32_t now_ms, uint8_t fsm_state, uint16_t fault_flags,
                    uint8_t hb_fresh, uint8_t btn_code, uint8_t btn_seq);

#endif /* RGB_LED_H */
