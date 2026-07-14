#include "comms_wdg.h"
#include "app_config.h"
#include "app_status.h"   /* app_now_ms */

#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t s_last_frame_ms;
static volatile uint8_t  s_link_seen;    /* watchdog arms on first valid frame */
static volatile uint32_t s_health_ms[HEALTH_COUNT];

/* A task is "alive" if it stamped within this window */
#define HEALTH_WINDOW_MS  100u

void comms_wdg_init(void)
{
    uint32_t now = app_now_ms();
    s_last_frame_ms = now;
    s_link_seen = 0;
    for (int i = 0; i < HEALTH_COUNT; ++i) s_health_ms[i] = now;
}

void comms_note_frame(void)
{
    s_last_frame_ms = app_now_ms();   /* called from the I2C ISR */
    s_link_seen = 1;
}

uint8_t comms_ok(uint32_t now_ms)
{
    if (!s_link_seen) return 1;   /* boot grace: never-connected is not "lost" */
    return ((now_ms - s_last_frame_ms) <= COMMS_TIMEOUT_MS) ? 1 : 0;
}

uint8_t comms_seen(void)
{
    return s_link_seen;
}

void health_stamp(health_id_t id)
{
    if (id < HEALTH_COUNT) {
        s_health_ms[id] = app_now_ms();
    }
}

uint8_t health_all_ok(uint32_t now_ms)
{
    for (int i = 0; i < HEALTH_COUNT; ++i) {
        if ((now_ms - s_health_ms[i]) > HEALTH_WINDOW_MS) return 0;
    }
    return 1;
}
