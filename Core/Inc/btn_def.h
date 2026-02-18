//
// Created by chichvarinivan on 2/18/26.
//

#ifndef JETSONEXTHARDWAREMANAGER_BTN_DEF_H
#define JETSONEXTHARDWAREMANAGER_BTN_DEF_H
#include "cmsis_os2.h"   /* for osDelay() */
#include "queue.h"

/* ---- Button event API provided by your button module ---- */
typedef enum
{
    BTN_EVT_PRESSED = 1,
    BTN_EVT_LONG_PRESS = 2,
    BTN_EVT_DOUBLE_PRESS = 3,
  } btn_evt_type_t;

typedef struct
{
    btn_evt_type_t type;
    uint16_t       pin;
    uint32_t       t_ms;
} btn_event_msg_t;

/* Provided by button module */
extern QueueHandle_t BTN_GetEventQueue(void);
#endif //JETSONEXTHARDWAREMANAGER_BTN_DEF_H