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

/* HAL callback shims (call these from the project's HAL_I2C_* callbacks) */
void i2c_reg_on_addr(I2C_HandleTypeDef *hi2c, uint8_t direction);
void i2c_reg_on_rx_cplt(I2C_HandleTypeDef *hi2c);
void i2c_reg_on_tx_cplt(I2C_HandleTypeDef *hi2c);
void i2c_reg_on_listen_cplt(I2C_HandleTypeDef *hi2c);
void i2c_reg_on_error(I2C_HandleTypeDef *hi2c);

#endif /* I2C_REG_H */
