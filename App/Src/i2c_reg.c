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

static volatile i2c_reg_diag_t s_diag;

void i2c_reg_init(I2C_HandleTypeDef *hi2c, QueueHandle_t cmd_queue)
{
    s_hi2c  = hi2c;
    s_cmd_q = cmd_queue;
    s_have_reg = 0;
    s_plen = 0;
    /* NOTE: listen is armed from task context via i2c_reg_arm_listen();
     * calling EnableListen before the scheduler starts is unreliable. */
}

void i2c_reg_arm_listen(void)
{
    if (!s_hi2c) return;
    HAL_StatusTypeDef st = HAL_I2C_EnableListen_IT(s_hi2c);
    if (st != HAL_OK) {
        /* Recover a stuck peripheral (e.g. after a master-mode call or bus
         * glitch): abort, deinit/reinit, then listen again. */
        HAL_I2C_DeInit(s_hi2c);
        HAL_I2C_Init(s_hi2c);
        st = HAL_I2C_EnableListen_IT(s_hi2c);
    }
    s_diag.listening = (st == HAL_OK) ? 1u : 0u;
}

static void pin_info(GPIO_TypeDef *g, uint32_t pin, uint8_t *mode, uint8_t *af, uint8_t *lvl)
{
    *mode = (uint8_t)((g->MODER >> (pin * 2u)) & 3u);
    *af   = (uint8_t)((g->AFR[pin >> 3u] >> ((pin & 7u) * 4u)) & 0xFu);
    *lvl  = (uint8_t)((g->IDR >> pin) & 1u);
}

void i2c_reg_pin_report(i2c_pin_report_t *o)
{
    if (!o) return;
    pin_info(GPIOA, 10u, &o->pa10_mode, &o->pa10_af, &o->pa10_lvl);
    pin_info(GPIOB, 6u,  &o->pb6_mode,  &o->pb6_af,  &o->pb6_lvl);
    pin_info(GPIOB, 7u,  &o->pb7_mode,  &o->pb7_af,  &o->pb7_lvl);
}

void i2c_reg_get_diag(i2c_reg_diag_t *out)
{
    if (!out) return;
    uint32_t pm = app_crit_enter();
    *out = *(const i2c_reg_diag_t *)&s_diag;
    app_crit_exit(pm);
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
            s_diag.frames_ok++;
            portYIELD_FROM_ISR(hpw);
        } else {
            fault_set(FAULT_CRC_ERROR);
            s_diag.frames_bad++;
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
            s_diag.frames_ok++;
        } else {
            fault_set(FAULT_HB_CRC_ERROR);
            s_diag.frames_bad++;
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

    s_diag.addr_hits++;
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
        s_diag.tx_starts++;
        HAL_I2C_Slave_Seq_Transmit_IT(hi2c, &s_tx[start], len, I2C_LAST_FRAME);
    }
}

void i2c_reg_on_rx_cplt(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance != APP_I2C_INSTANCE) return;

    if (!s_have_reg) {
        s_regptr   = s_rx_byte;
        s_have_reg = 1;
        s_diag.last_regptr = s_regptr;
    } else if (s_plen < sizeof(s_payload)) {
        s_payload[s_plen++] = s_rx_byte;
        s_diag.rx_bytes++;
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
        s_diag.listen_cplt++;
        dispatch_write();
    }
    HAL_I2C_EnableListen_IT(hi2c);
}

void i2c_reg_on_error(I2C_HandleTypeDef *hi2c)
{
    /* AF (NACK) at end of a read is normal for the master's last byte */
    s_diag.errors++;
    HAL_I2C_EnableListen_IT(hi2c);
}
