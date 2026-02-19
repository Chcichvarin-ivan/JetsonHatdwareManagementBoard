//
// Created by chichvarinivan on 2/19/26.
//
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "cmsis_os2.h"
#include <stdint.h>
#include "btn_def.h"

#include "i2c_app.h"

/* ===================== Supervisor task (static) ===================== */
static TaskHandle_t hSup = NULL;

#define SUP_STACK_WORDS  384
static StaticTask_t supTcb;
static StackType_t  supStack[SUP_STACK_WORDS];


/* Single-byte RX from master (LED command) */
static uint8_t rx_cmd;

/* Single-byte TX to master (button state) */
static uint8_t tx_btn;

static void vBtnSupervisorTask(void *arg) {
    (void)arg;
    tx_btn = 0x00;
    QueueHandle_t q = NULL;
    btn_event_msg_t ev;
    q = BTN_GetEventQueue();
    for (;;) {
        if (xQueueReceive(q, &ev, pdMS_TO_TICKS(200)) == pdPASS) {
            switch (ev.type) {
                case BTN_EVT_PRESSED:
                    tx_btn = 0x01;
                    break;
                case BTN_EVT_LONG_PRESS:
                    tx_btn = 0x02;
                    break;
                case BTN_EVT_DOUBLE_PRESS:
                    tx_btn = 0x03;
                    break;
                    default:
                    break;
            }
        }
    }
}

void I2C_SlaveApp_Init(I2C_HandleTypeDef *hi2c) {
    HAL_I2C_EnableListen_IT(hi2c);
    hSup = xTaskCreateStatic(
      vBtnSupervisorTask,
      "sup_btn_led",
      SUP_STACK_WORDS,
      NULL,
      tskIDLE_PRIORITY + 2,
      supStack,
      &supTcb
  );
}
extern void HAL_I2C_ListenCpltCallback (I2C_HandleTypeDef *hi2c)
{

    HAL_I2C_EnableListen_IT(hi2c);
}

extern void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
    if(hi2c->Instance==I2C1) {
        if(TransferDirection == I2C_DIRECTION_TRANSMIT)  // if the master wants to transmit the data
        {
            HAL_I2C_Slave_Seq_Receive_IT(hi2c, &rx_cmd, 1, I2C_FIRST_AND_LAST_FRAME);
        }
        else {

            HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &tx_btn, 1, I2C_FIRST_AND_LAST_FRAME);

        }
    }
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    HAL_I2C_EnableListen_IT(hi2c);
}

void HAL_I2C_SlaveRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(hi2c->Instance==I2C1) {
        /* Apply command */
        setLedState(rx_cmd);
    }
}

void HAL_I2C_SlaveTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    tx_btn = 0x00;
    HAL_I2C_EnableListen_IT(hi2c);
}

