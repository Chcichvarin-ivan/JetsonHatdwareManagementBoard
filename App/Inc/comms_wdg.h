/**
 * comms_wdg.h — Link liveness + WWDG health gating.
 *
 * comms: any CRC-valid command or heartbeat refreshes the link timer.
 * health: each critical task stamps its liveness; the WWDG is only refreshed
 *         while every required task is alive, so a hung task forces a reset.
 */
#ifndef COMMS_WDG_H
#define COMMS_WDG_H

#include <stdint.h>

typedef enum {
    HEALTH_ACTUATION = 0,
    HEALTH_RGBLED    = 1,
    HEALTH_TELEMETRY = 2,
    HEALTH_BUTTON    = 3,
    HEALTH_COUNT
} health_id_t;

void    comms_wdg_init(void);

/* Link */
void    comms_note_frame(void);              /* call on any valid frame */
uint8_t comms_ok(uint32_t now_ms);           /* 1 if a frame seen within timeout */

/* Health (per-task heartbeat) */
void    health_stamp(health_id_t id);        /* task calls each loop */
uint8_t health_all_ok(uint32_t now_ms);      /* 1 if all tasks recently stamped */

#endif /* COMMS_WDG_H */
