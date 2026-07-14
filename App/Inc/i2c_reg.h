/**
 * i2c_reg.h — Register-model I2C slave service. Replaces the original
 * single-byte i2c_app. Handles the status read block and the write frames
 * (command / heartbeat / config), validating CRC and SEQ.
 *
 * Wire the project's HAL I2C callbacks to the handlers below (see i2c_reg.c
 * and INTEGRATION.md).
 */
#ifndef I2C_REG_H
#define I2C_REG_H

#include <stdint.h>
#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "queue.h"

/* cmd_queue receives app_command_t; created by app_init */
void i2c_reg_init(I2C_HandleTypeDef *hi2c, QueueHandle_t cmd_queue);

/* (Re)enable slave listen mode. Call once from a running task after the
 * scheduler starts, and any time the bus needs re-arming. Safe to repeat. */
void i2c_reg_arm_listen(void);

/* Diagnostics for bring-up: ISR-side event counters (see `ext i2c`). */
typedef struct {
    uint32_t addr_hits;     /* address-match events */
    uint32_t rx_bytes;      /* payload bytes received */
    uint32_t tx_starts;     /* read transactions served */
    uint32_t listen_cplt;   /* STOP events */
    uint32_t errors;        /* error callback count */
    uint32_t frames_ok;     /* CRC-valid frames dispatched */
    uint32_t frames_bad;    /* CRC failures */
    uint8_t  last_regptr;   /* last register pointer written by master */
    uint8_t  listening;     /* 1 if EnableListen last returned HAL_OK */
} i2c_reg_diag_t;
void i2c_reg_get_diag(i2c_reg_diag_t *out);

/* Live GPIO mux/level readout for the I2C candidate pins (PA10, PB6, PB7).
 * mode: 0=input 1=output 2=alternate 3=analog; af: AFR nibble (I2C1 = 4);
 * lvl: current input data register bit (idle bus should read 1). */
typedef struct {
    uint8_t pa10_mode, pa10_af, pa10_lvl;
    uint8_t pb6_mode,  pb6_af,  pb6_lvl;
    uint8_t pb7_mode,  pb7_af,  pb7_lvl;
} i2c_pin_report_t;
void i2c_reg_pin_report(i2c_pin_report_t *out);

/* HAL callback shims (call these from the project's HAL_I2C_* callbacks) */
void i2c_reg_on_addr(I2C_HandleTypeDef *hi2c, uint8_t direction);
void i2c_reg_on_rx_cplt(I2C_HandleTypeDef *hi2c);
void i2c_reg_on_tx_cplt(I2C_HandleTypeDef *hi2c);
void i2c_reg_on_listen_cplt(I2C_HandleTypeDef *hi2c);
void i2c_reg_on_error(I2C_HandleTypeDef *hi2c);

#endif /* I2C_REG_H */
