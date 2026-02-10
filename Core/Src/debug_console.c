#include "debug_console.h"

/* FreeRTOS primitives used by this module */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "stream_buffer.h"
#include "message_buffer.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

/* ============================================================================
 * Compile-time tunables
 * ============================================================================
 *
 * RX_SB_SIZE:
 *   StreamBuffer capacity for incoming UART bytes.
 *
 * TX_MB_SIZE:
 *   MessageBuffer capacity for outgoing debug strings/bytes.
 *
 * TX_CHUNK_SIZE:
 *   How many bytes TX task sends per DMA transfer (tradeoff latency vs overhead).
 *
 * CLI_LINE_MAX:
 *   Maximum length of a typed command line.
 *
 * HISTORY_LEN:
 *   Number of prior command lines stored for up/down history.
 *
 * I2C_QUEUE_LEN:
 *   Depth of queued I2C jobs waiting for worker execution.
 *
 * SCRIPT_*:
 *   I2C script line storage for simple automation.
 */
#define RX_SB_SIZE          256
#define TX_MB_SIZE          2048
#define TX_CHUNK_SIZE       128

#define CLI_LINE_MAX        160
#define CLI_TOK_MAX         24

#define HISTORY_LEN         8

#define I2C_QUEUE_LEN       12
#define I2C_MAX_DATA        32

#define SCRIPT_MAX_LINES    24
#define SCRIPT_LINE_MAX     96

/* Binary protocol constants (not fully implemented in this module version) */
#define BIN_SYNC1           0xAA
#define BIN_SYNC2           0x55
#define BIN_MAX_PAYLOAD     64

/* Security parameters */
#define LOGIN_FAIL_LIMIT       5
#define LOGIN_LOCKOUT_MS       15000
#define LOGIN_BACKOFF_BASE_MS  300

/* ============================================================================
 * Module-global configuration copy
 * ============================================================================
 *
 * We copy user config into g_cfg so module tasks can use it safely later.
 * NOTE: the pointers inside (huart/hi2c/btn_port/login_token) must remain valid.
 */
static dbg_config_t g_cfg;

/* ============================================================================
 * RTOS objects
 * ============================================================================
 *
 * sbUartRx:
 *   Receives bytes from UART RX ISR. CLI task blocks on it.
 *
 * mbUartTx:
 *   Receives bytes from dbg output API. TX task blocks on it.
 *
 * qI2cJobs:
 *   Holds I2C operations to run, executed by I2C worker task.
 */
static StreamBufferHandle_t  sbUartRx;
static MessageBufferHandle_t mbUartTx;
static QueueHandle_t         qI2cJobs;

/* Task handles: allow ISR notifications, suspend/resume worker, etc. */
static TaskHandle_t hTx, hCli, hI2c, hSup, hKeep;

/* UART RX single-byte buffer used by HAL_UART_Receive_IT() */
static uint8_t uart_rx_byte;

/* ============================================================================
 * Session + console state
 * ============================================================================
 *
 * last_rx_tick:
 *   Updated on every received byte, used for session timeout.
 *
 * logged_in:
 *   Set when "login <token>" succeeds, cleared on logout/timeout.
 *
 * session_active:
 *   Set when "session start" is issued, controls I2C worker availability.
 *
 * echo_enabled:
 *   If true, CLI echoes typed characters.
 *
 * prompt_enabled:
 *   If false, prompts are not printed (useful for scripts/binary hosts).
 *
 * timestamps_enabled:
 *   If true, DBG_Log() prefixes output with tick-based timestamp.
 *
 * log_level:
 *   Current filter for DBG_Log().
 *
 * binary_mode:
 *   If true, CLI is disabled and raw bytes are treated as binary protocol.
 *
 * latency_enabled:
 *   If true, prints per-command latency.
 */
static volatile uint32_t last_rx_tick = 0;
static volatile int logged_in = 0;
static volatile int session_active = 0;
static volatile int echo_enabled = 0;
static volatile int prompt_enabled = 1;
static volatile int timestamps_enabled = 0;
static volatile uint8_t log_level = 2;

static volatile int binary_mode = 0;
static volatile int latency_enabled = 1;
static volatile int quiet_mode = 1;

/* Security state */
static volatile int login_fails = 0;
static volatile uint32_t lockout_until_tick = 0;

/* Scripting state */
static char script_lines[SCRIPT_MAX_LINES][SCRIPT_LINE_MAX];
static int  script_len = 0;
static int  script_recording = 0;

/* Command history state */
static char history[HISTORY_LEN][CLI_LINE_MAX];
static int  hist_count = 0;
static int  hist_head  = 0;
static int  hist_nav   = -1;

/* Diagnostics counters */
static volatile uint32_t tx_drops = 0;      /* increments when TX buffer is full */
static volatile uint32_t rx_overflows = 0;  /* increments when RX buffer is full */

/* Keepalive feature: prints '.' periodically during active session */
static volatile int keepalive_enabled = 0;
static volatile uint32_t keepalive_period_ms = 5000;

/* ============================================================================
 * Crash dump storage (HardFault)
 * ============================================================================
 *
 * Stored in ".noinit" section so it survives reset. Ensure linker keeps it.
 * If your linker script does not include .noinit in RAM, add it.
 */
typedef struct
{
  uint32_t magic;
  uint32_t r0,r1,r2,r3,r12,lr,pc,psr;
  uint32_t cfsr,hfsr,dfsr,afsr,mmfar,bfar;
} crash_dump_t;

__attribute__((section(".noinit"))) static crash_dump_t g_crash;
#define CRASH_MAGIC 0xC0DECAFEu

/* ============================================================================
 * Small time helpers
 * ============================================================================
 */
static uint32_t ticks_to_ms(uint32_t ticks) { return ticks * portTICK_PERIOD_MS; }
static uint32_t ms_to_ticks(uint32_t ms) { return pdMS_TO_TICKS(ms); }

static int in_lockout(void)
{
  uint32_t now = xTaskGetTickCount();
  return (lockout_until_tick != 0 && (int32_t)(lockout_until_tick - now) > 0);
}
static uint32_t lockout_remaining_ms(void)
{
  if (!in_lockout()) return 0;
  uint32_t now = xTaskGetTickCount();
  return ticks_to_ms(lockout_until_tick - now);
}

/* ============================================================================
 * Public counter getters (quality diagnostics)
 * ============================================================================
 */
uint32_t DBG_GetTxDrops(void) { return tx_drops; }
uint32_t DBG_GetRxOverflows(void) { return rx_overflows; }

/* ============================================================================
 * Non-blocking TX API
 * ============================================================================
 *
 * DBG_Write:
 *   - task context only
 *   - enqueues bytes into mbUartTx
 *   - returns immediately (0 tick wait)
 *   - updates tx_drops if message buffer couldn't accept all bytes
 */
void DBG_Write(const void *data, size_t len)
{
  if (!mbUartTx || !data || len == 0) return;

  size_t sent = xMessageBufferSend(mbUartTx, data, len, 0);
  if (sent != len) tx_drops++;
}

/* DBG_Puts: convenience wrapper for null-terminated strings */
void DBG_Puts(const char *s)
{
  if (!s) return;
  DBG_Write(s, strlen(s));
}

/* DBG_Printf: formats into a stack buffer, then enqueues */
void DBG_Printf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0) return;
  if ((size_t)n > sizeof(buf)) n = (int)sizeof(buf);

  DBG_Write(buf, (size_t)n);
}

/* ============================================================================
 * Logging API
 * ============================================================================
 */
void DBG_SetLogLevel(uint8_t lvl) { log_level = lvl; }
uint8_t DBG_GetLogLevel(void) { return log_level; }

static const char *lvl_name(uint8_t lvl)
{
  switch (lvl) {
    case 0: return "ERR";
    case 1: return "WRN";
    case 2: return "INF";
    default: return "DBG";
  }
}

/*
 * DBG_Log:
 *   - prints only if lvl <= current log_level
 *   - optional timestamp prefix controlled by timestamps_enabled
 *   - always appends CRLF
 */
void DBG_Log(uint8_t lvl, const char *fmt, ...)
{
  if (lvl > log_level) return;

  if (timestamps_enabled)
    DBG_Printf("[%lu ms] ", (unsigned long)ticks_to_ms(xTaskGetTickCount()));

  DBG_Printf("[%s] ", lvl_name(lvl));

  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  if (n <= 0) return;
  if ((size_t)n > sizeof(buf)) n = (int)sizeof(buf);

  DBG_Write(buf, (size_t)n);
  DBG_Puts("\r\n");
}

/* ============================================================================
 * UART RX startup
 * ============================================================================
 *
 * Starts interrupt-based reception of a single byte.
 * The RX complete callback re-arms reception forever.
 */
static void uart_rx_start(void)
{
  last_rx_tick = xTaskGetTickCount();
  HAL_UART_Receive_IT(g_cfg.huart, &uart_rx_byte, 1);
}

/* ============================================================================
 * HAL callback forwarding
 * ============================================================================
 *
 * These functions are designed to be called by the user’s
 * HAL_UART_* callbacks in main.c.
 */
void DBG_OnUartRxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Ignore callbacks from other UART instances */
  if (!g_cfg.huart || huart != g_cfg.huart) return;

  BaseType_t hpw = pdFALSE;

  /* Push byte into RX stream buffer; if full, count overflow */
  if (sbUartRx)
  {
    size_t ok = xStreamBufferSendFromISR(sbUartRx, &uart_rx_byte, 1, &hpw);
    if (ok != 1) rx_overflows++;
  }

  /* Update activity for session timeout */
  last_rx_tick = xTaskGetTickCountFromISR();

  /* Re-arm RX */
  HAL_UART_Receive_IT(g_cfg.huart, &uart_rx_byte, 1);

  portYIELD_FROM_ISR(hpw);
}

void DBG_OnUartTxCpltCallback(UART_HandleTypeDef *huart)
{
  if (!g_cfg.huart || huart != g_cfg.huart) return;

  /* Notify TX task that DMA completed, so it can send next chunk */
  BaseType_t hpw = pdFALSE;
  if (hTx) vTaskNotifyGiveFromISR(hTx, &hpw);
  portYIELD_FROM_ISR(hpw);
}

/* ============================================================================
 * TX task
 * ============================================================================
 *
 * Responsibility:
 *   - Block on mbUartTx until data exists
 *   - Send a chunk via HAL_UART_Transmit_DMA
 *   - Block until DMA complete callback notifies it
 *
 * Hardware access:
 *   - ONLY this task calls HAL_UART_Transmit_DMA (UART TX ownership)
 */
static void vUartTxTask(void *arg)
{
  (void)arg;

  static uint8_t chunk[TX_CHUNK_SIZE];

  for (;;)
  {
    size_t n = xMessageBufferReceive(mbUartTx, chunk, sizeof(chunk), portMAX_DELAY);
    if (n == 0) continue;

    /* Wait until UART DMA is available */
    while (HAL_UART_Transmit_DMA(g_cfg.huart, chunk, (uint16_t)n) == HAL_BUSY)
      vTaskDelay(pdMS_TO_TICKS(1));

    /* Wait until TX complete interrupt gives notification */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
}

/* ============================================================================
 * I2C worker task and queue
 * ============================================================================
 *
 * All I2C transactions go through this single worker task.
 * This avoids concurrent I2C access from multiple tasks and simplifies debugging.
 */
typedef enum { I2C_OP_SCAN, I2C_OP_RD, I2C_OP_WR, I2C_OP_DELAY } i2c_op_t;

typedef struct
{
  i2c_op_t op;                 /* operation type */
  uint8_t  addr7;              /* 7-bit device address */
  uint8_t  reg;                /* register address (8-bit mem addr) */
  uint8_t  len;                /* length for read/write */
  uint8_t  data[I2C_MAX_DATA]; /* write data buffer */
  uint32_t delay_ms;           /* used if op == I2C_OP_DELAY */
} i2c_job_t;

/* Enqueue an I2C job to be executed by vI2cTask */
static void enqueue_i2c_job(const i2c_job_t *job)
{
  if (!qI2cJobs) return;
  if (xQueueSend(qI2cJobs, job, pdMS_TO_TICKS(50)) != pdPASS)
    DBG_Puts("[i2c] queue full\r\n");
}

/*
 * vI2cTask:
 *   - blocks on qI2cJobs
 *   - ignores jobs if not logged in or session not active
 *   - executes scan/read/write using HAL I2C API
 */
static void vI2cTask(void *arg)
{
  (void)arg;

  i2c_job_t job;

  for (;;)
  {
    if (xQueueReceive(qI2cJobs, &job, portMAX_DELAY) != pdPASS) continue;

    if (!logged_in || !session_active) continue;

    if (job.op == I2C_OP_DELAY)
    {
      vTaskDelay(pdMS_TO_TICKS(job.delay_ms));
      continue;
    }

    if (job.op == I2C_OP_SCAN)
    {
      DBG_Puts("[i2c] scan...\r\n");
      int found = 0;
      for (uint8_t a = 1; a < 127; a++)
      {
        if (HAL_I2C_IsDeviceReady(g_cfg.hi2c, (uint16_t)(a << 1), 2, 20) == HAL_OK)
        {
          DBG_Printf("  found 0x%02X\r\n", a);
          found++;
        }
      }
      if (!found) DBG_Puts("  none\r\n");
      DBG_Puts("[i2c] done\r\n");
    }
    else if (job.op == I2C_OP_RD)
    {
      uint8_t rx[I2C_MAX_DATA] = {0};
      if (job.len > I2C_MAX_DATA) job.len = I2C_MAX_DATA;

      HAL_StatusTypeDef st =
        HAL_I2C_Mem_Read(g_cfg.hi2c, (uint16_t)(job.addr7 << 1), job.reg, I2C_MEMADD_SIZE_8BIT,
                         rx, job.len, 200);

      if (st != HAL_OK) DBG_Puts("[i2c] read error\r\n");
      else
      {
        DBG_Printf("[i2c] rd 0x%02X reg 0x%02X:", job.addr7, job.reg);
        for (uint8_t i = 0; i < job.len; i++) DBG_Printf(" %02X", rx[i]);
        DBG_Puts("\r\n");
      }
    }
    else if (job.op == I2C_OP_WR)
    {
      HAL_StatusTypeDef st =
        HAL_I2C_Mem_Write(g_cfg.hi2c, (uint16_t)(job.addr7 << 1), job.reg, I2C_MEMADD_SIZE_8BIT,
                          job.data, job.len, 200);

      DBG_Puts(st == HAL_OK ? "[i2c] write ok\r\n" : "[i2c] write error\r\n");
    }
  }
}

/* ============================================================================
 * Script helpers
 * ============================================================================
 *
 * Script stores user-entered lines and later converts them into queued I2C jobs.
 * Lines are simple ASCII commands like:
 *   i2c scan
 *   i2c rd 0x68 0x75 1
 *   i2c wr 0x68 0x6B 0
 *   delay 100
 */
static void script_clear(void)
{
  for (int i = 0; i < SCRIPT_MAX_LINES; i++) script_lines[i][0] = 0;
  script_len = 0;
  DBG_Puts("[script] cleared\r\n");
}

static void script_show(void)
{
  DBG_Printf("[script] lines=%d\r\n", script_len);
  for (int i = 0; i < script_len; i++)
    DBG_Printf("  %02d: %s\r\n", i, script_lines[i]);
}

static void script_add_line(const char *line)
{
  if (script_len >= SCRIPT_MAX_LINES) { DBG_Puts("[script] full\r\n"); return; }
  strncpy(script_lines[script_len], line, SCRIPT_LINE_MAX - 1);
  script_lines[script_len][SCRIPT_LINE_MAX - 1] = 0;
  script_len++;
}

/* Parse decimal or hex byte */
static int parse_u8(const char *s, uint8_t *out)
{
  char *end = NULL;
  long v = strtol(s, &end, 0);
  if (end == s || v < 0 || v > 255) return 0;
  *out = (uint8_t)v;
  return 1;
}

/* Convert stored lines into queued I2C jobs */
static void script_run(void)
{
  DBG_Printf("[script] running %d line(s)\r\n", script_len);

  for (int i = 0; i < script_len; i++)
  {
    char linebuf[SCRIPT_LINE_MAX];
    strncpy(linebuf, script_lines[i], sizeof(linebuf)-1);
    linebuf[sizeof(linebuf)-1] = 0;

    char *tok[CLI_TOK_MAX] = {0};
    int n = 0;
    for (char *p = strtok(linebuf, " \t"); p && n < CLI_TOK_MAX; p = strtok(NULL, " \t"))
      tok[n++] = p;
    if (n == 0) continue;

    if (!strcmp(tok[0], "delay") && n >= 2)
    {
      long ms = strtol(tok[1], NULL, 0);
      if (ms < 0) ms = 0;
      i2c_job_t j = {.op = I2C_OP_DELAY, .delay_ms = (uint32_t)ms};
      enqueue_i2c_job(&j);
      continue;
    }

    if (!strcmp(tok[0], "i2c") && n >= 2 && !strcmp(tok[1], "scan"))
    {
      i2c_job_t j = {.op = I2C_OP_SCAN};
      enqueue_i2c_job(&j);
      continue;
    }

    if (!strcmp(tok[0], "i2c") && n >= 5 && !strcmp(tok[1], "rd"))
    {
      i2c_job_t j = {.op = I2C_OP_RD};
      if (!parse_u8(tok[2], &j.addr7) || !parse_u8(tok[3], &j.reg) || !parse_u8(tok[4], &j.len))
      { DBG_Printf("[script] bad rd line %d\r\n", i); continue; }
      if (j.len > I2C_MAX_DATA) j.len = I2C_MAX_DATA;
      enqueue_i2c_job(&j);
      continue;
    }

    if (!strcmp(tok[0], "i2c") && n >= 5 && !strcmp(tok[1], "wr"))
    {
      i2c_job_t j = {.op = I2C_OP_WR};
      if (!parse_u8(tok[2], &j.addr7) || !parse_u8(tok[3], &j.reg))
      { DBG_Printf("[script] bad wr line %d\r\n", i); continue; }
      j.len = 0;
      for (int k = 4; k < n && j.len < I2C_MAX_DATA; k++)
      {
        if (!parse_u8(tok[k], &j.data[j.len])) { DBG_Printf("[script] bad byte line %d\r\n", i); break; }
        j.len++;
      }
      if (j.len) enqueue_i2c_job(&j);
      continue;
    }

    DBG_Printf("[script] unknown line %d: %s\r\n", i, script_lines[i]);
  }

  DBG_Puts("[script] queued\r\n");
}

/* ============================================================================
 * Command history helpers (up/down)
 * ============================================================================
 */
static void history_add(const char *line)
{
  if (!line || !line[0]) return;

  if (hist_count > 0)
  {
    int last = (hist_head - 1 + HISTORY_LEN) % HISTORY_LEN;
    if (strncmp(history[last], line, CLI_LINE_MAX) == 0) return;
  }

  strncpy(history[hist_head], line, CLI_LINE_MAX - 1);
  history[hist_head][CLI_LINE_MAX - 1] = 0;

  hist_head = (hist_head + 1) % HISTORY_LEN;
  if (hist_count < HISTORY_LEN) hist_count++;
  hist_nav = -1;
}

static const char* history_get(int offset)
{
  if (hist_count == 0) return NULL;
  if (offset < 0) return NULL;
  if (offset >= hist_count) offset = hist_count - 1;

  int idx = (hist_head - 1 - offset + HISTORY_LEN) % HISTORY_LEN;
  return history[idx];
}

/* ============================================================================
 * Terminal/UI helpers
 * ============================================================================
 */
static const char *PROMPT_LOCKED  = "locked> ";
static const char *PROMPT_LOGGED  = "dbg> ";
static const char *PROMPT_SESSION = "sess> ";

/* Clear terminal screen using ANSI escape sequence */
static void term_clear(void) { DBG_Puts("\033[2J\033[H"); }

/* Redraw the entire input line with prompt */
static void cli_redraw_line(const char *prompt, const char *content)
{
  DBG_Puts("\r\033[2K");
  if (prompt_enabled) DBG_Puts(prompt);
  DBG_Puts(content);
}

/* ============================================================================
 * Keepalive task
 * ============================================================================
 *
 * Periodically prints '.' when session is active.
 * Useful to confirm the device is alive during long silent periods.
 */
static void vKeepaliveTask(void *arg)
{
  (void)arg;

  for (;;)
  {
    if (keepalive_enabled && session_active && logged_in && !binary_mode)
      DBG_Puts(".");
    vTaskDelay(pdMS_TO_TICKS(keepalive_period_ms));
  }
}

/* ============================================================================
 * Session control + supervisor
 * ============================================================================
 */
static void session_stop(const char *reason)
{
  session_active = 0;
  logged_in = 0;
  echo_enabled = 0;
  binary_mode = 0;
  script_recording = 0;

  if (hI2c) vTaskSuspend(hI2c);

  if (reason) DBG_Printf("\r\n[debug] %s\r\n", reason);
  DBG_Puts("[debug] locked. login required.\r\n");
  if (prompt_enabled) DBG_Puts(PROMPT_LOCKED);
}

static void session_start(void)
{
  session_active = 1;
  if (hI2c) vTaskResume(hI2c);
  DBG_Puts("[debug] session started\r\n");
  if (prompt_enabled) DBG_Puts(PROMPT_SESSION);
}

/*
 * vSupervisorTask:
 *   Enforces session timeout by checking last_rx_tick.
 *   Runs periodically (250ms) and stops session if inactive too long.
 */
static void vSupervisorTask(void *arg)
{
  (void)arg;

  for (;;)
  {
    if (session_active)
    {
      uint32_t idle_ms = ticks_to_ms(xTaskGetTickCount() - last_rx_tick);
      if (idle_ms > g_cfg.session_timeout_ms)
        session_stop("session timeout -> stopped");
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

/* ============================================================================
 * Memory/stat commands
 * ============================================================================
 */
static void cmd_mem(void)
{
  size_t free = xPortGetFreeHeapSize();
  size_t minEver = xPortGetMinimumEverFreeHeapSize();
  DBG_Printf("[mem] heap free: %lu bytes\r\n", (unsigned long)free);
  DBG_Printf("[mem] heap min-ever: %lu bytes\r\n", (unsigned long)minEver);
  DBG_Printf("[mem] this task stack HW: %lu words\r\n", (unsigned long)uxTaskGetStackHighWaterMark(NULL));
}

static void cmd_txstat(void)
{
  DBG_Printf("[tx] drops=%lu\r\n", (unsigned long)tx_drops);
}

static void cmd_rxstat(void)
{
  DBG_Printf("[rx] overflows=%lu\r\n", (unsigned long)rx_overflows);
}

/* ============================================================================
 * Crash dump commands
 * ============================================================================
 */
static void cmd_crash_show(void)
{
  if (g_crash.magic != CRASH_MAGIC) { DBG_Puts("[crash] none\r\n"); return; }

  DBG_Puts("\r\n[crash] HardFault dump:\r\n");
  DBG_Printf("  pc=0x%08lX lr=0x%08lX psr=0x%08lX\r\n",
             (unsigned long)g_crash.pc, (unsigned long)g_crash.lr, (unsigned long)g_crash.psr);
  DBG_Printf("  r0=0x%08lX r1=0x%08lX r2=0x%08lX r3=0x%08lX r12=0x%08lX\r\n",
             (unsigned long)g_crash.r0,(unsigned long)g_crash.r1,(unsigned long)g_crash.r2,(unsigned long)g_crash.r3,(unsigned long)g_crash.r12);
  DBG_Printf("  CFSR=0x%08lX HFSR=0x%08lX BFAR=0x%08lX MMFAR=0x%08lX\r\n",
             (unsigned long)g_crash.cfsr,(unsigned long)g_crash.hfsr,(unsigned long)g_crash.bfar,(unsigned long)g_crash.mmfar);
}

static void cmd_crash_clear(void)
{
  memset((void*)&g_crash, 0, sizeof(g_crash));
  DBG_Puts("[crash] cleared\r\n");
}

/* ============================================================================
 * HardFault handler implementation
 * ============================================================================
 *
 * Captures stacked registers and fault status registers, writes into g_crash,
 * then resets the MCU.
 *
 * IMPORTANT:
 *   Ensure you do not have another HardFault_Handler in your project.
 */
__attribute__((naked)) void HardFault_Handler(void)
{
  __asm volatile(
    "tst lr, #4            \n"
    "ite eq                \n"
    "mrseq r0, msp         \n"
    "mrsne r0, psp         \n"
    "b HardFault_HandlerC  \n"
  );
}

void HardFault_HandlerC(uint32_t *s)
{
  g_crash.magic = CRASH_MAGIC;

  g_crash.r0 = s[0]; g_crash.r1 = s[1]; g_crash.r2 = s[2]; g_crash.r3 = s[3];
  g_crash.r12 = s[4]; g_crash.lr = s[5]; g_crash.pc = s[6]; g_crash.psr = s[7];

  g_crash.cfsr = SCB->CFSR;
  g_crash.hfsr = SCB->HFSR;
  g_crash.dfsr = SCB->DFSR;
  g_crash.afsr = SCB->AFSR;
  g_crash.mmfar = SCB->MMFAR;
  g_crash.bfar = SCB->BFAR;

  NVIC_SystemReset();
  while (1) {}
}

/* ============================================================================
 * Command help and line parsing
 * ============================================================================
 */
static void print_help(void)
{
  DBG_Puts(
    "\r\nCommands:\r\n"
    "  help\r\n"
    "  login <token>\r\n"
    "  logout\r\n"
    "  session start|stop\r\n"
    "  echo on|off\r\n"
    "  prompt on|off\r\n"
    "  time on|off\r\n"
    "  loglvl 0..3\r\n"
    "  log <lvl> <msg...>\r\n"
    "  keepalive on|off [ms]\r\n"
    "  lat on|off\r\n"
    "  bin on|off\r\n"
    "  clear\r\n"
    "  reset\r\n"
    "  btn\r\n"
    "  i2c scan\r\n"
    "  i2c rd <addr7> <reg> <len>\r\n"
    "  i2c wr <addr7> <reg> <b1> [b2...]\r\n"
    "  i2c script start|stop|add|show|run|clear\r\n"
    "  mem\r\n"
    "  txstat\r\n"
    "  rxstat\r\n"
    "  crash show|clear\r\n"
    "\r\n"
  );
}

static void trim_crlf(char *s)
{
  size_t n = strlen(s);
  while (n && (s[n-1] == '\r' || s[n-1] == '\n')) s[--n] = 0;
}

/* ============================================================================
 * CLI command dispatch
 * ============================================================================
 *
 * handle_line():
 *   - takes ownership of `line` buffer (modifies with strtok)
 *   - may enqueue I2C jobs
 *   - may change session/login state
 *   - prints prompt at the end (if enabled)
 */
static void handle_line(char *line)
{
  trim_crlf(line);

  /* If recording, store line unless it's a script control command */
  if (script_recording)
  {
    if (strncmp(line, "i2c script", 10) != 0)
    {
      script_add_line(line);
      DBG_Puts("[script] +\r\n");
      return;
    }
  }

  /* Tokenize by whitespace */
  char *tok[CLI_TOK_MAX] = {0};
  int n = 0;
  for (char *p = strtok(line, " \t"); p && n < CLI_TOK_MAX; p = strtok(NULL, " \t"))
    tok[n++] = p;
  if (n == 0) return;

  uint32_t t0 = xTaskGetTickCount();

  /* always-allowed commands (no login needed) */
  if (!strcmp(tok[0], "help")) { print_help(); goto done; }
  if (!strcmp(tok[0], "clear")) { term_clear(); goto done; }
  if (!strcmp(tok[0], "reset")) { DBG_Puts("[sys] reset\r\n"); NVIC_SystemReset(); goto done; }
  if (!strcmp(tok[0], "txstat")) { cmd_txstat(); goto done; }
  if (!strcmp(tok[0], "rxstat")) { cmd_rxstat(); goto done; }

  if (!strcmp(tok[0], "crash"))
  {
    if (n >= 2 && !strcmp(tok[1], "show")) cmd_crash_show();
    else if (n >= 2 && !strcmp(tok[1], "clear")) cmd_crash_clear();
    else DBG_Puts("usage: crash show|clear\r\n");
    goto done;
  }

  /* login command */
  if (!strcmp(tok[0], "login"))
  {
    if (in_lockout())
    {
      DBG_Printf("[debug] locked out (%lu ms)\r\n", (unsigned long)lockout_remaining_ms());
      goto done;
    }

    if (n >= 2 && g_cfg.login_token && !strcmp(tok[1], g_cfg.login_token))
    {
      logged_in = 1;
      login_fails = 0;
      lockout_until_tick = 0;

      echo_enabled = g_cfg.default_echo ? 1 : 0;
      timestamps_enabled = g_cfg.default_timestamps ? 1 : 0;
      prompt_enabled = g_cfg.default_prompt ? 1 : 0;
      quiet_mode = 0;

      DBG_Puts("[debug] login ok\r\n");
      if (g_cfg.auto_session_on_login) session_start();
      else
      {
        DBG_Puts("[debug] type: session start\r\n");
        if (prompt_enabled) DBG_Puts(PROMPT_LOGGED);
      }
      goto done;
    }

    /* failed login */
    login_fails++;
    DBG_Puts("[debug] login failed\r\n");
    vTaskDelay(pdMS_TO_TICKS((uint32_t)login_fails * LOGIN_BACKOFF_BASE_MS));
    if (login_fails >= LOGIN_FAIL_LIMIT)
    {
      lockout_until_tick = xTaskGetTickCount() + ms_to_ticks(LOGIN_LOCKOUT_MS);
      DBG_Printf("[debug] too many fails -> lockout %d ms\r\n", LOGIN_LOCKOUT_MS);
    }
    goto done;
  }

  /* logout always permitted if you are logged in */
  if (!strcmp(tok[0], "logout")) { session_stop("logout"); goto done; }

  /* require login for everything below */
  if (!logged_in)
  {
    DBG_Puts("login <token>\r\n");
    goto done;
  }

  /* session management */
  if (!strcmp(tok[0], "session"))
  {
    if (n >= 2 && !strcmp(tok[1], "start")) session_start();
    else if (n >= 2 && (!strcmp(tok[1], "stop") || !strcmp(tok[1], "end"))) session_stop("session stopped");
    else DBG_Puts("usage: session start|stop\r\n");
    goto done;
  }

  /* UI toggles */
  if (!strcmp(tok[0], "echo"))
  {
    if (n >= 2 && !strcmp(tok[1], "on")) { echo_enabled = 1; DBG_Puts("[debug] echo on\r\n"); }
    else if (n >= 2 && !strcmp(tok[1], "off")) { echo_enabled = 0; DBG_Puts("[debug] echo off\r\n"); }
    else DBG_Puts("usage: echo on|off\r\n");
    goto done;
  }

  if (!strcmp(tok[0], "prompt"))
  {
    if (n >= 2 && !strcmp(tok[1], "on")) { prompt_enabled = 1; DBG_Puts("[debug] prompt on\r\n"); }
    else if (n >= 2 && !strcmp(tok[1], "off")) { prompt_enabled = 0; DBG_Puts("[debug] prompt off\r\n"); }
    else DBG_Puts("usage: prompt on|off\r\n");
    goto done;
  }

  if (!strcmp(tok[0], "time"))
  {
    if (n >= 2 && !strcmp(tok[1], "on")) { timestamps_enabled = 1; DBG_Puts("[debug] time on\r\n"); }
    else if (n >= 2 && !strcmp(tok[1], "off")) { timestamps_enabled = 0; DBG_Puts("[debug] time off\r\n"); }
    else DBG_Puts("usage: time on|off\r\n");
    goto done;
  }

  /* logging controls */
  if (!strcmp(tok[0], "loglvl"))
  {
    if (n >= 2) { int lv = atoi(tok[1]); if (lv < 0) lv = 0; if (lv > 3) lv = 3; log_level = (uint8_t)lv; DBG_Puts("[debug] loglvl set\r\n"); }
    else DBG_Puts("usage: loglvl 0..3\r\n");
    goto done;
  }

  if (!strcmp(tok[0], "log"))
  {
    if (n >= 3)
    {
      int lv = atoi(tok[1]); if (lv < 0) lv = 0; if (lv > 3) lv = 3;
      char msg[180]; msg[0]=0;
      for (int i=2;i<n;i++){ strncat(msg,tok[i],sizeof(msg)-strlen(msg)-1); if(i!=n-1) strncat(msg," ",sizeof(msg)-strlen(msg)-1); }
      DBG_Log((uint8_t)lv, "%s", msg);
    }
    else DBG_Puts("usage: log <lvl> <msg...>\r\n");
    goto done;
  }

  /* keepalive */
  if (!strcmp(tok[0], "keepalive"))
  {
    if (n >= 2 && !strcmp(tok[1], "on"))
    {
      keepalive_enabled = 1;
      if (n >= 3)
      {
        long ms = strtol(tok[2], NULL, 0);
        if (ms < 100) ms = 100;
        keepalive_period_ms = (uint32_t)ms;
      }
      DBG_Puts("[debug] keepalive on\r\n");
    }
    else if (n >= 2 && !strcmp(tok[1], "off"))
    {
      keepalive_enabled = 0;
      DBG_Puts("[debug] keepalive off\r\n");
    }
    else DBG_Puts("usage: keepalive on|off [ms]\r\n");
    goto done;
  }

  /* latency toggle */
  if (!strcmp(tok[0], "lat"))
  {
    if (n >= 2 && !strcmp(tok[1], "on")) { latency_enabled = 1; DBG_Puts("[debug] latency on\r\n"); }
    else if (n >= 2 && !strcmp(tok[1], "off")) { latency_enabled = 0; DBG_Puts("[debug] latency off\r\n"); }
    else DBG_Puts("usage: lat on|off\r\n");
    goto done;
  }

  /* binary mode toggle (CLI does not decode in this module version) */
  if (!strcmp(tok[0], "bin"))
  {
    if (n >= 2 && !strcmp(tok[1], "on")) { binary_mode = 1; DBG_Puts("[debug] binary ON\r\n"); }
    else if (n >= 2 && !strcmp(tok[1], "off")) { binary_mode = 0; DBG_Puts("[debug] binary OFF\r\n"); }
    else DBG_Puts("usage: bin on|off\r\n");
    goto done;
  }

  /* memory info */
  if (!strcmp(tok[0], "mem")) { cmd_mem(); goto done; }

  /* require active session for hardware operations */
  if (!session_active)
  {
    DBG_Puts("[debug] no active session. type: session start\r\n");
    goto done;
  }

  /* button test */
  if (!strcmp(tok[0], "btn"))
  {
    GPIO_PinState st = HAL_GPIO_ReadPin(g_cfg.btn_port, g_cfg.btn_pin);
    DBG_Puts(st == GPIO_PIN_RESET ? "BTN: pressed\r\n" : "BTN: released\r\n");
    goto done;
  }

  /* I2C commands + script control */
  if (!strcmp(tok[0], "i2c"))
  {
    if (n >= 2 && !strcmp(tok[1], "scan"))
    {
      i2c_job_t j = {.op = I2C_OP_SCAN};
      enqueue_i2c_job(&j);
      goto done;
    }

    if (n >= 5 && !strcmp(tok[1], "rd"))
    {
      i2c_job_t j = {.op = I2C_OP_RD};
      if (!parse_u8(tok[2], &j.addr7) || !parse_u8(tok[3], &j.reg) || !parse_u8(tok[4], &j.len))
      { DBG_Puts("usage: i2c rd <addr7> <reg> <len>\r\n"); goto done; }
      if (j.len == 0) { DBG_Puts("[i2c] len>0\r\n"); goto done; }
      if (j.len > I2C_MAX_DATA) j.len = I2C_MAX_DATA;
      enqueue_i2c_job(&j);
      goto done;
    }

    if (n >= 5 && !strcmp(tok[1], "wr"))
    {
      i2c_job_t j = {.op = I2C_OP_WR};
      if (!parse_u8(tok[2], &j.addr7) || !parse_u8(tok[3], &j.reg))
      { DBG_Puts("usage: i2c wr <addr7> <reg> <b1> [b2...]\r\n"); goto done; }

      j.len = 0;
      for (int i=4;i<n && j.len<I2C_MAX_DATA;i++)
      {
        if (!parse_u8(tok[i], &j.data[j.len])) { DBG_Puts("bad byte\r\n"); goto done; }
        j.len++;
      }
      if (!j.len) { DBG_Puts("[i2c] need data\r\n"); goto done; }
      enqueue_i2c_job(&j);
      goto done;
    }

    if (n >= 3 && !strcmp(tok[1], "script"))
    {
      if (!strcmp(tok[2], "start")) { script_recording = 1; DBG_Puts("[script] rec ON\r\n"); goto done; }
      if (!strcmp(tok[2], "stop"))  { script_recording = 0; DBG_Puts("[script] rec OFF\r\n"); goto done; }
      if (!strcmp(tok[2], "show"))  { script_show(); goto done; }
      if (!strcmp(tok[2], "run"))   { script_recording = 0; script_run(); goto done; }
      if (!strcmp(tok[2], "clear")) { script_recording = 0; script_clear(); goto done; }
      if (!strcmp(tok[2], "add"))
      {
        char tmp[SCRIPT_LINE_MAX]; tmp[0]=0;
        for (int i=3;i<n;i++){ strncat(tmp,tok[i],sizeof(tmp)-strlen(tmp)-1); if(i!=n-1) strncat(tmp," ",sizeof(tmp)-strlen(tmp)-1); }
        script_add_line(tmp);
        DBG_Puts("[script] added\r\n");
        goto done;
      }
      DBG_Puts("usage: i2c script start|stop|add|show|run|clear\r\n");
      goto done;
    }

    DBG_Puts("usage: i2c scan|rd|wr|script\r\n");
    goto done;
  }

  DBG_Puts("unknown cmd. type help\r\n");

done:
  /* Optional per-command latency printout */
  if (latency_enabled)
  {
    uint32_t dt = xTaskGetTickCount() - t0;
    DBG_Printf("[lat] %lu ms\r\n", (unsigned long)ticks_to_ms(dt));
  }

  /* Print prompt (if enabled) */
  if (prompt_enabled)
  {
    if (!logged_in) DBG_Puts(PROMPT_LOCKED);
    else if (!session_active) DBG_Puts(PROMPT_LOGGED);
    else DBG_Puts(PROMPT_SESSION);
  }
}

/* ============================================================================
 * CLI task
 * ============================================================================
 *
 * Responsibilities:
 *   - Reads input bytes from sbUartRx (blocks most of the time)
 *   - Basic line editing:
 *       - typing ASCII appends to buffer
 *       - backspace deletes
 *       - up/down arrow browse history
 *   - When Enter pressed:
 *       - terminate line
 *       - store in history
 *       - pass to handle_line()
 *
 * Note:
 *   - binary_mode in this version simply disables CLI consumption of bytes.
 */
static void vCliTask(void *arg)
{
  (void)arg;

  char line[CLI_LINE_MAX];
  size_t idx = 0;
  line[0] = 0;

  int esc_state = 0;

  DBG_Puts("\r\n[debug] ready. login required.\r\n");
  DBG_Puts("Type: help\r\n");
  DBG_Printf("UART %lu baud\r\n", (unsigned long)g_cfg.baud_hint);

  if (g_crash.magic == CRASH_MAGIC)
    DBG_Puts("[boot] crash dump present: crash show\r\n");

  if (prompt_enabled) DBG_Puts(PROMPT_LOCKED);

  for (;;)
  {
    uint8_t ch;
    if (xStreamBufferReceive(sbUartRx, &ch, 1, portMAX_DELAY) != 1) continue;

    /* If binary mode enabled, ignore CLI input bytes */
    if (binary_mode) continue;

    /* ANSI escape sequence parsing: ESC [ A/B */
    if (esc_state == 0 && ch == 0x1B) { esc_state = 1; continue; }
    if (esc_state == 1) { esc_state = (ch == '[') ? 2 : 0; continue; }
    if (esc_state == 2)
    {
      esc_state = 0;

      if (ch == 'A') /* Up */
      {
        if (hist_count)
        {
          if (hist_nav < hist_count - 1) hist_nav++;
          const char *h = history_get(hist_nav);
          if (h)
          {
            strncpy(line, h, sizeof(line)-1); line[sizeof(line)-1]=0;
            idx = strlen(line);
            cli_redraw_line(logged_in ? (session_active?PROMPT_SESSION:PROMPT_LOGGED) : PROMPT_LOCKED, line);
          }
        }
        continue;
      }

      if (ch == 'B') /* Down */
      {
        if (hist_count)
        {
          if (hist_nav > 0) hist_nav--;
          else hist_nav = -1;

          if (hist_nav >= 0)
          {
            const char *h = history_get(hist_nav);
            if (h) { strncpy(line,h,sizeof(line)-1); line[sizeof(line)-1]=0; idx=strlen(line); }
          }
          else { line[0]=0; idx=0; }

          cli_redraw_line(logged_in ? (session_active?PROMPT_SESSION:PROMPT_LOGGED) : PROMPT_LOCKED, line);
        }
        continue;
      }

      continue;
    }

    /* Echo typed char (if enabled); TAB is treated separately elsewhere */
    if (echo_enabled && ch != '\t') DBG_Write(&ch, 1);

    /* Enter -> execute */
    if (ch == '\r' || ch == '\n')
    {
      if (echo_enabled) DBG_Puts("\r\n");
      line[idx] = 0;

      history_add(line);
      hist_nav = -1;

      handle_line(line);

      idx = 0;
      line[0] = 0;
      continue;
    }

    /* Backspace/Delete */
    if (ch == 0x7F || ch == 0x08)
    {
      if (idx > 0)
      {
        idx--;
        line[idx] = 0;
        if (echo_enabled) DBG_Puts("\b \b");
      }
      continue;
    }

    /* Printable ASCII */
    if (ch >= 0x20 && ch <= 0x7E)
    {
      if (idx + 1 < sizeof(line))
      {
        line[idx++] = (char)ch;
        line[idx] = 0;
      }
      else
      {
        idx = 0; line[0]=0;
        DBG_Puts("\r\n[cli] line too long\r\n");
        if (prompt_enabled) DBG_Puts(logged_in ? (session_active?PROMPT_SESSION:PROMPT_LOGGED) : PROMPT_LOCKED);
      }
    }
  }
}

/* ============================================================================
 * Module initialization
 * ============================================================================
 *
 * DBG_Init:
 *   - copies configuration
 *   - creates RTOS primitives
 *   - creates tasks
 *   - suspends I2C worker until session starts
 *   - starts UART RX interrupt
 */
void DBG_Init(const dbg_config_t *cfg)
{
  if (!cfg || !cfg->huart) return;
  g_cfg = *cfg;

  logged_in = 0;
  session_active = 0;
  quiet_mode = 1;
  echo_enabled = 0;
  prompt_enabled = cfg->default_prompt ? 1 : 0;
  timestamps_enabled = cfg->default_timestamps ? 1 : 0;
  log_level = 2;
  binary_mode = 0;
  latency_enabled = 1;
  keepalive_enabled = 0;

  sbUartRx = xStreamBufferCreate(RX_SB_SIZE, 1);
  mbUartTx = xMessageBufferCreate(TX_MB_SIZE);
  qI2cJobs = xQueueCreate(I2C_QUEUE_LEN, sizeof(i2c_job_t));

  xTaskCreate(vUartTxTask,     "dbg_tx",  384, NULL, tskIDLE_PRIORITY+3, &hTx);
  xTaskCreate(vCliTask,        "dbg_cli", 768, NULL, tskIDLE_PRIORITY+2, &hCli);
  xTaskCreate(vI2cTask,        "dbg_i2c", 640, NULL, tskIDLE_PRIORITY+1, &hI2c);
  xTaskCreate(vSupervisorTask, "dbg_sup", 256, NULL, tskIDLE_PRIORITY+1, &hSup);
  xTaskCreate(vKeepaliveTask,  "dbg_ka",  256, NULL, tskIDLE_PRIORITY+1, &hKeep);

  /* Start with no session: disable I2C worker to save CPU */
  if (hI2c) vTaskSuspend(hI2c);

  uart_rx_start();
}
