#include "comms_wdg.h"
#include "app_config.h"

#include "FreeRTOS.h"
#include "task.h"

static volatile uint32_t s_last_frame_ms;
static volatile uint32_t s_health_ms[HEALTH_COUNT];

/* A task is "alive" if it stamped within this window */
#define HEALTH_WINDOW_MS  100u

void comms_wdg_init(void)
{
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    s_last_frame_ms = now;
    for (int i = 0; i < HEALTH_COUNT; ++i) s_health_ms[i] = now;
}

void comms_note_frame(void)
{
    s_last_frame_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
}

uint8_t comms_ok(uint32_t now_ms)
{
    return ((now_ms - s_last_frame_ms) <= COMMS_TIMEOUT_MS) ? 1 : 0;
}

void health_stamp(health_id_t id)
{
    if (id < HEALTH_COUNT) {
        s_health_ms[id] = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
}

uint8_t health_all_ok(uint32_t now_ms)
{
    for (int i = 0; i < HEALTH_COUNT; ++i) {
        if ((now_ms - s_health_ms[i]) > HEALTH_WINDOW_MS) return 0;
    }
    return 1;
}
