/**
 * actuation.h — Supervisory state machine. Owns the TIM2 CCRs, enforces the
 * minimum hold times, the flight-envelope interlock and failsafe behaviour.
 * Consumes validated commands from i2c_reg and SAFE button gestures from the
 * project's existing button engine (BTN_GetEventQueue()).
 */
#ifndef ACTUATION_H
#define ACTUATION_H

#include <stdint.h>
#include "FreeRTOS.h"
#include "queue.h"

void actuation_init(QueueHandle_t cmd_queue);
void actuation_task(void *arg);   /* FreeRTOS task entry (created by app.c) */

#endif /* ACTUATION_H */
