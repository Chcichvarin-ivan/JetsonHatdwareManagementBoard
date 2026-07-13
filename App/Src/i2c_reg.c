#include "i2c_reg.h"
#include "app_config.h"
#include "app_status.h"
#include "proto.h"
#include "crc8.h"
#include "envelope.h"
#include "comms_wdg.h"

#include "FreeRTOS.h"
#include "queue.h"

static I2C_HandleTypeDef *s_hi2c;
static QueueHandle_t      s_cmd_q;

/* RX state for the current transaction */
static uint8_t  s_regptr;
static uint8_t  s_have_reg;
static uint8_t  s_payload[16];
static uint8_t  s_plen;

/* TX snapshot for reads (status block, register-addressable with auto-inc) */
static uint8_t  s_tx[STATUS_BLOCK_SIZE];

static uint8_t  s_rx_byte;   /* single-byte landing zone for seq receive */

void i2c_reg_init(I2C_HandleTypeDef *hi2c, QueueHandle_t cmd_queue)
{
    s_hi2c  = hi2c;
    s_cmd_q = cmd_queue;
    s_have_reg = 0;
    s_plen = 0;
    HAL_I2C_EnableListen_IT(hi2c);
}

/* ---- dispatch a completed write transaction ---- */
static void dispatch_write(void)
{
    if (s_plen == 0) return;  /* pointer-only write (precedes a read) */

    if (s_regptr == REG_CMD && s_plen >= CMD_FRAME_LEN) {
        uint8_t crc = crc8_smbus(s_payload, CMD_FRAME_LEN - 1);
        if (crc == s_payload[CMD_FRAME_LEN - 1]) {
            app_command_t c;
            c.opcode = s_payload[0];
            c.seq    = s_payload[1];
            c.arg    = (uint16_t)s_payload[2] | ((uint16_t)s_payload[3] << 8);
            BaseType_t hpw = pdFALSE;
            xQueueSendFromISR(s_cmd_q, &c, &hpw);
            comms_note_frame();
            portYIELD_FROM_ISR(hpw);
        } else {
            fault_set(FAULT_CRC_ERROR);
        }
    } else if (s_regptr == REG_HEARTBEAT && s_plen >= HB_FRAME_LEN) {
        uint8_t crc = crc8_smbus(s_payload, HB_FRAME_LEN - 1);
        if (crc == s_payload[HB_FRAME_LEN - 1]) {
            uint8_t  hb_seq = s_payload[0];
            uint16_t alt    = (uint16_t)s_payload[1] | ((uint16_t)s_payload[2] << 8);
            uint16_t spd    = (uint16_t)s_payload[3] | ((uint16_t)s_payload[4] << 8);
            uint8_t  flags  = s_payload[5];
            envelope_on_heartbeat(hb_seq, alt, spd, flags);
            comms_note_frame();
        } else {
            fault_set(FAULT_HB_CRC_ERROR);
        }
    } else if (s_regptr == REG_CONFIG && s_plen >= CFG_FRAME_LEN) {
        uint8_t crc = crc8_smbus(s_payload, CFG_FRAME_LEN - 1);
        if (crc == s_payload[CFG_FRAME_LEN - 1]) {
            uint16_t amin = (uint16_t)s_payload[0] | ((uint16_t)s_payload[1] << 8);
            uint16_t amax = (uint16_t)s_payload[2] | ((uint16_t)s_payload[3] << 8);
            uint16_t smax = (uint16_t)s_payload[4] | ((uint16_t)s_payload[5] << 8);
            status_set_config(amin, amax, smax);
        } else {
            fault_set(FAULT_CRC_ERROR);
        }
    }
}

void i2c_reg_on_addr(I2C_HandleTypeDef *hi2c, uint8_t direction)
{
    if (hi2c->Instance != APP_I2C_INSTANCE) return;

    if (direction == I2C_DIRECTION_TRANSMIT) {
        /* master writes to us: receive register pointer + payload byte by byte */
        s_have_reg = 0;
        s_plen = 0;
        HAL_I2C_Slave_Seq_Receive_IT(hi2c, &s_rx_byte, 1, I2C_FIRST_FRAME);
    } else {
        /* master reads from us: build a fresh snapshot and stream from regptr */
        status_snapshot(s_tx);
        uint8_t start = s_have_reg ? s_regptr : 0;
        if (start >= STATUS_BLOCK_SIZE) start = 0;
        uint16_t len = (uint16_t)(STATUS_BLOCK_SIZE - start);
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &s_tx[start], len, I2C_LAST_FRAME);
    }
}

void i2c_reg_on_rx_cplt(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != APP_I2C_INSTANCE) return;

    if (!s_have_reg) {
        s_regptr   = s_rx_byte;
        s_have_reg = 1;
    } else if (s_plen < sizeof(s_payload)) {
        s_payload[s_plen++] = s_rx_byte;
    }
    /* keep receiving until the master issues STOP (ListenCplt) */
    HAL_I2C_Slave_Seq_Receive_IT(hi2c, &s_rx_byte, 1, I2C_NEXT_FRAME);
}

void i2c_reg_on_tx_cplt(I2C_HandleTypeDef *hi2c)
{
    (void)hi2c;
    /* nothing extra; ListenCplt re-arms listening */
}

void i2c_reg_on_listen_cplt(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == APP_I2C_INSTANCE) {
        dispatch_write();
    }
    HAL_I2C_EnableListen_IT(hi2c);
}

void i2c_reg_on_error(I2C_HandleTypeDef *hi2c)
{
    /* AF (NACK) at end of a read is normal for the master's last byte */
    HAL_I2C_EnableListen_IT(hi2c);
}
