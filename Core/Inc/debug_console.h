//
// Created by chichvarinivan on 2/10/26.
//

#ifndef JETSONEXTHARDWAREMANAGER_DEBUG_CONSOLE_H

#ifndef DEBUG_CONSOLE_H
#define DEBUG_CONSOLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>
#include <stddef.h>

/* ============================================================================
 * Debug Console Module - Public API
 * ============================================================================
 *
 * Purpose:
 *   Provide a UART-based debug console with:
 *     - Non-blocking TX (DMA + internal buffering)
 *     - RX interrupt based input
 *     - Command shell with login token + session manager
 *     - I2C test commands, scripting, and diagnostics
 *
 * Integration pattern:
 *   1) Configure CubeMX:
 *        - USART2 asynchronous, 115200
 *        - Enable DMA for USART2 TX
 *        - Enable USART2 global interrupt (RX)
 *        - Enable I2C1
 *        - Enable FreeRTOS
 *   2) Add debug_console.c/.h to your project.
 *   3) In main.c:
 *        - call DBG_Init() from a task after scheduler starts (StartDefaultTask)
 *        - forward HAL_UART_RxCpltCallback and HAL_UART_TxCpltCallback to module
 *
 * Threading model:
 *   - UART TX: only module TX task touches UART TX hardware.
 *   - UART RX: ISR writes bytes into StreamBuffer; CLI task reads it.
 *   - I2C: only I2C worker task touches the I2C peripheral.
 *
 * Notes:
 *   - DBG_Write/DBG_Printf are safe from task context (not ISR).
 *   - The module defines a HardFault handler inside debug_console.c.
 *     Ensure you do not have a duplicate HardFault_Handler elsewhere.
 * ============================================================================
 */

/* ----------------------------------------------------------------------------
 * dbg_config_t
 *
 * User-provided configuration passed to DBG_Init().
 * This keeps the module reusable across projects/boards.
 * -------------------------------------------------------------------------- */
typedef struct
{
  /* UART used for console. Must be initialized before calling DBG_Init(). */
  UART_HandleTypeDef *huart;

  /* I2C used for test commands. Must be initialized before calling DBG_Init(). */
  I2C_HandleTypeDef  *hi2c;

  /* Optional button input used by "btn" command. */
  GPIO_TypeDef *btn_port;
  uint16_t      btn_pin;

  /* Informational only: printed in banner, does not configure UART. */
  uint32_t baud_hint;

  /* Login token string. Required to unlock console (e.g., "k1-7a"). */
  const char *login_token;

  /* Session inactivity timeout. If no RX activity, auto-stops session. */
  uint32_t session_timeout_ms;

  /* If 1: session starts immediately after successful login. */
  uint8_t  auto_session_on_login;

  /* If 1: echo is enabled immediately after login. */
  uint8_t  default_echo;

  /* If 1: timestamps are enabled for DBG_Log() output after login. */
  uint8_t  default_timestamps;

  /* If 1: CLI prints prompts. If 0: useful for scripts/binary clients. */
  uint8_t  default_prompt;

} dbg_config_t;

/* ----------------------------------------------------------------------------
 * DBG_Init
 *
 * Create internal RTOS buffers/tasks and start UART RX interrupt.
 *
 * Must be called from task context AFTER scheduler start.
 * Typical place: at start of StartDefaultTask().
 * -------------------------------------------------------------------------- */
void DBG_Init(const dbg_config_t *cfg);

/* ----------------------------------------------------------------------------
 * Non-blocking output API
 *
 * These functions enqueue into an internal TX buffer.
 * They do NOT wait for UART to finish and should return quickly.
 * -------------------------------------------------------------------------- */
void DBG_Write(const void *data, size_t len);
void DBG_Puts(const char *s);
void DBG_Printf(const char *fmt, ...);

/* ----------------------------------------------------------------------------
 * Logging API (level 0..3)
 *
 * lvl:
 *   0 = ERR, 1 = WRN, 2 = INF, 3 = DBG
 *
 * DBG_Log() prints only if lvl <= current log level.
 * -------------------------------------------------------------------------- */
void DBG_SetLogLevel(uint8_t lvl);
uint8_t DBG_GetLogLevel(void);
void DBG_Log(uint8_t lvl, const char *fmt, ...);

/* ----------------------------------------------------------------------------
 * Counters (quality features)
 *
 * TX drops:
 *   Number of times data couldn't be fully enqueued (TX buffer full).
 *
 * RX overflows:
 *   Number of times RX ISR couldn't push byte into StreamBuffer.
 * -------------------------------------------------------------------------- */
uint32_t DBG_GetTxDrops(void);
uint32_t DBG_GetRxOverflows(void);

/* ----------------------------------------------------------------------------
 * HAL callback forwarding
 *
 * Call these from your HAL_UART_* callbacks in main.c.
 * -------------------------------------------------------------------------- */
void DBG_OnUartRxCpltCallback(UART_HandleTypeDef *huart);
void DBG_OnUartTxCpltCallback(UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* DEBUG_CONSOLE_H */

#define JETSONEXTHARDWAREMANAGER_DEBUG_CONSOLE_H

#endif //JETSONEXTHARDWAREMANAGER_DEBUG_CONSOLE_H