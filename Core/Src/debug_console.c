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

#include "cmsis_os2.h"
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
#define TX_MB_SIZE          4096
#define TX_CHUNK_SIZE       1024

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

/* ---------------- Static task storage (TCB + stacks) ---------------- */
/* Stack sizes are in WORDS (not bytes). */
#define DBG_TX_STACK_WORDS   2048
#define DBG_CLI_STACK_WORDS  768
#define DBG_I2C_STACK_WORDS  640
#define DBG_SUP_STACK_WORDS  256
#define DBG_KA_STACK_WORDS   256
/* ===================== Smart RX (line gate) =====================
 * ISR buffers incoming characters into a single line.
 * Only when CR/LF arrives AND the line looks like a valid command,
 * the entire line is forwarded to CLI via StreamBuffer.
 *
 * This reduces CPU usage and prevents CLI from waking on noise/garbage.
 */
#define DBG_RX_LINE_MAX   96  /* keep ISR line buffer small */

static volatile uint8_t g_rx_line[DBG_RX_LINE_MAX];
static volatile uint16_t g_rx_line_len = 0;

/* enable/disable smart gating (set 1 for script-driven Linux use) */
static volatile uint8_t g_smart_rx_enable = 1;
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

/* ============================================================================
 * I2C worker task and queue
 * ============================================================================
 *
 * All I2C transactions go through this single worker task.
 * This avoids concurrent I2C access from multiple tasks and simplifies debugging.
 */
static StreamBufferHandle_t  sbUartRx;
static MessageBufferHandle_t mbUartTx;
static QueueHandle_t         qI2cJobs;

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

/* ---------------- Static queue storage ---------------- */
static StaticQueue_t qI2cStatic;
static uint8_t qI2cStorage[I2C_QUEUE_LEN * sizeof(i2c_job_t)];

static StaticTask_t tcbTx, tcbCli, tcbI2c, tcbSup, tcbKeep;
static StackType_t  stkTx[DBG_TX_STACK_WORDS];
static StackType_t  stkCli[DBG_CLI_STACK_WORDS];
static StackType_t  stkI2c[DBG_I2C_STACK_WORDS];
static StackType_t  stkSup[DBG_SUP_STACK_WORDS];
static StackType_t  stkKeep[DBG_KA_STACK_WORDS];

/* ---------------- Static Stream/Message buffer storage ---------------- */
static StaticStreamBuffer_t sbRxStruct;
static uint8_t sbRxStorage[RX_SB_SIZE];

static StaticStreamBuffer_t mbTxStruct;       /* MessageBuffer uses StaticStreamBuffer_t */
static uint8_t mbTxStorage[TX_MB_SIZE];

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

/* Return 1 if c is acceptable as part of a command line */
static inline int rx_is_cmd_char(uint8_t c)
{
  /* allow printable ASCII + space + tab */
  if (c >= 0x20 && c <= 0x7E) return 1;
  if (c == '\t') return 1;
  return 0;
}

/* Fast “is this line plausibly a command?” check.
 * Keep this VERY cheap (runs in ISR on newline only).
 */
static int rx_line_is_valid_cmd_prefix(const uint8_t *s, uint16_t n)
{
  /* trim leading spaces/tabs */
  uint16_t i = 0;
  while (i < n && (s[i] == ' ' || s[i] == '\t')) i++;
  if (i == n) return 0;

  /* Find token end */
  uint16_t start = i;
  while (i < n && s[i] != ' ' && s[i] != '\t') i++;
  uint16_t toklen = (uint16_t)(i - start);
  if (toklen == 0) return 0;

  /* Compare first token against allowed commands.
   * Keep this list short and match your CLI root commands.
   */
#define TOK_EQ(lit) (toklen == (sizeof(lit)-1) && memcmp((const void*)&s[start], (lit), sizeof(lit)-1) == 0)

  if (TOK_EQ("help")) return 1;
  if (TOK_EQ("login")) return 1;
  if (TOK_EQ("logout")) return 1;
  if (TOK_EQ("session")) return 1;
  if (TOK_EQ("echo")) return 1;
  if (TOK_EQ("prompt")) return 1;
  if (TOK_EQ("time")) return 1;
  if (TOK_EQ("loglvl")) return 1;
  if (TOK_EQ("log")) return 1;
  if (TOK_EQ("keepalive")) return 1;
  if (TOK_EQ("lat")) return 1;
  if (TOK_EQ("bin")) return 1;
  if (TOK_EQ("clear")) return 1;
  if (TOK_EQ("reset")) return 1;
  if (TOK_EQ("btn")) return 1;
  if (TOK_EQ("i2c")) return 1;
  if (TOK_EQ("mem")) return 1;
  if (TOK_EQ("txstat")) return 1;
  if (TOK_EQ("rxstat")) return 1;
  if (TOK_EQ("crash")) return 1;

  return 0;
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
  if (!g_cfg.huart || huart != g_cfg.huart) return;

  BaseType_t hpw = pdFALSE;

  /* Restart RX ASAP at the end (but safe to do here too) */
  uint8_t b = uart_rx_byte;

  /* If smart RX disabled, behave like classic byte-forwarding */
  if (!g_smart_rx_enable)
  {
    if (sbUartRx)
    {
      size_t ok = xStreamBufferSendFromISR(sbUartRx, &b, 1, &hpw);
      if (ok != 1) rx_overflows++;
    }

    /* any byte counts as activity */
    last_rx_tick = xTaskGetTickCountFromISR();

    HAL_UART_Receive_IT(g_cfg.huart, &uart_rx_byte, 1);
    portYIELD_FROM_ISR(hpw);
    return;
  }

  /* ==================== Smart Line Mode ==================== */

  /* Newline -> evaluate buffered line */
  if (b == '\r' || b == '\n')
  {
    uint16_t n = g_rx_line_len;

    /* If empty line: ignore (do NOT wake CLI) */
    if (n == 0)
    {
      /* keep listening */
      HAL_UART_Receive_IT(g_cfg.huart, &uart_rx_byte, 1);
      return;
    }

    /* Validate prefix */
    int valid = rx_line_is_valid_cmd_prefix((const uint8_t*)g_rx_line, n);

    if (valid && sbUartRx)
    {
      /* Forward line to CLI in one shot + CRLF */
      size_t ok1 = xStreamBufferSendFromISR(sbUartRx, (const void*)g_rx_line, n, &hpw);
      size_t ok2 = xStreamBufferSendFromISR(sbUartRx, "\r\n", 2, &hpw);

      if (ok1 != n || ok2 != 2) rx_overflows++;

      /* Only count activity when we got a valid command line */
      last_rx_tick = xTaskGetTickCountFromISR();
    }
    else
    {
      /* Invalid line => silently drop it */
      /* (optional) you could increment a counter here if you want */
    }

    /* Reset for next line */
    g_rx_line_len = 0;

    HAL_UART_Receive_IT(g_cfg.huart, &uart_rx_byte, 1);
    portYIELD_FROM_ISR(hpw);
    return;
  }

  /* Regular character: keep it only if it looks like command content */
  if (rx_is_cmd_char(b))
  {
    /* Append if space available; else drop line */
    if (g_rx_line_len < (DBG_RX_LINE_MAX - 1))
    {
      g_rx_line[g_rx_line_len++] = b;
    }
    else
    {
      /* Line too long -> discard entire line (prevents partial commands) */
      g_rx_line_len = 0;
    }
  }
  else
  {
    /* Non-command character (including binary junk) => discard current line */
    g_rx_line_len = 0;
  }

  /* Re-arm RX for next byte */
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

    /* Start DMA with bounded wait (no spinning) */
    while (HAL_UART_Transmit_DMA(g_cfg.huart, chunk, (uint16_t)n) == HAL_BUSY)
      osDelay(pdMS_TO_TICKS(1));

    /* Sleep until DMA complete ISR notifies us */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  }
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
      osDelay(pdMS_TO_TICKS(job.delay_ms));
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
 * keepalive_enabled = 1;
 * if (hKeep) xTaskNotifyGive(hKeep);

 */
static void vKeepaliveTask(void *arg)
{
  (void)arg;

  for (;;)
  {
    /* Sleep until enabled */
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    while (keepalive_enabled)
    {
      if (logged_in && session_active && !binary_mode)
        DBG_Puts(".");
      osDelay(pdMS_TO_TICKS(keepalive_period_ms));
    }
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

/* Print prompt based on current state (single place) */
static void cli_print_prompt(void)
{
  if (!prompt_enabled) return;

  if (!logged_in) DBG_Puts(PROMPT_LOCKED);
  else if (!session_active) DBG_Puts(PROMPT_LOGGED);
  else DBG_Puts(PROMPT_SESSION);
}

/* Optional: print latency once per command */
static void cli_print_latency(uint32_t t0)
{
  if (!latency_enabled) return;
  uint32_t dt_ticks = xTaskGetTickCount() - t0;
  DBG_Printf("[lat] %lu ms\r\n", (unsigned long)ticks_to_ms(dt_ticks));
}

/* Helper: parse tokens (modifies line) */
static int cli_tokenize(char *line, char *tok[], int tokmax)
{
  int n = 0;
  for (char *p = strtok(line, " \t"); p && n < tokmax; p = strtok(NULL, " \t"))
    tok[n++] = p;
  return n;
}

/* ---- Command handlers return 1 if they handled the command ---- */
static int cmd_help(char *tok[], int n)
{
  if (n == 1 && strcmp(tok[0], "help") == 0) return 0;
  /* (you can keep your existing print_help implementation) */
  print_help();
  return 1;
}

static int cmd_login(char *tok[], int n)
{
  if (strcmp(tok[0], "login") != 0) return 0;

  if (in_lockout())
  {
    DBG_Printf("[debug] locked out (%lu ms)\r\n", (unsigned long)lockout_remaining_ms());
    return 1;
  }

  if (n >= 2 && g_cfg.login_token && strcmp(tok[1], g_cfg.login_token) == 0)
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
    else DBG_Puts("[debug] type: session start\r\n");

    /* Wake supervisor/keepalive logic only when useful */
    if (hSup) xTaskNotifyGive(hSup);

    return 1;
  }

  /* failed */
  login_fails++;
  DBG_Puts("[debug] login failed\r\n");

  /* backoff uses osDelay */
  osDelay((uint32_t)login_fails * LOGIN_BACKOFF_BASE_MS);

  if (login_fails >= LOGIN_FAIL_LIMIT)
  {
    lockout_until_tick = xTaskGetTickCount() + ms_to_ticks(LOGIN_LOCKOUT_MS);
    DBG_Printf("[debug] too many fails -> lockout %d ms\r\n", LOGIN_LOCKOUT_MS);
  }
  return 1;
}

/* ... add cmd_reset/cmd_clear/cmd_echo/cmd_time/cmd_mem/cmd_i2c etc similarly ... */

/* Main line handler */
static void handle_line(char *line)
{
  trim_crlf(line);

  /* Script recording: store line unless it’s script control */
  if (script_recording)
  {
    if (strncmp(line, "i2c script", 10) != 0)
    {
      script_add_line(line);
      DBG_Puts("[script] +\r\n");
      cli_print_prompt();
      return;
    }
  }

  char *tok[CLI_TOK_MAX] = {0};
  int n = cli_tokenize(line, tok, CLI_TOK_MAX);
  if (n == 0)
  {
    cli_print_prompt();
    return;
  }

  uint32_t t0 = xTaskGetTickCount();

  /* Always-allowed commands first */
  if (strcmp(tok[0], "help") == 0) { print_help(); cli_print_latency(t0); cli_print_prompt(); return; }
  if (strcmp(tok[0], "clear") == 0) { term_clear(); cli_print_latency(t0); cli_print_prompt(); return; }
  if (strcmp(tok[0], "reset") == 0) { DBG_Puts("[sys] reset\r\n"); cli_print_latency(t0); NVIC_SystemReset(); return; }
  if (strcmp(tok[0], "txstat") == 0) { cmd_txstat(); cli_print_latency(t0); cli_print_prompt(); return; }
  if (strcmp(tok[0], "rxstat") == 0) { cmd_rxstat(); cli_print_latency(t0); cli_print_prompt(); return; }
  if (strcmp(tok[0], "crash") == 0)
  {
    if (n >= 2 && strcmp(tok[1], "show") == 0) cmd_crash_show();
    else if (n >= 2 && strcmp(tok[1], "clear") == 0) cmd_crash_clear();
    else DBG_Puts("usage: crash show|clear\r\n");
    cli_print_latency(t0);
    cli_print_prompt();
    return;
  }

  /* Login */
  if (strcmp(tok[0], "login") == 0) { (void)cmd_login(tok, n); cli_print_latency(t0); cli_print_prompt(); return; }

  /* Logout */
  if (strcmp(tok[0], "logout") == 0) { session_stop("logout"); cli_print_latency(t0); cli_print_prompt(); return; }

  /* Require login below */
  if (!logged_in)
  {
    DBG_Puts("login <token>\r\n");
    cli_print_latency(t0);
    cli_print_prompt();
    return;
  }

  /* Session mgmt (doesn’t require session_active) */
  if (strcmp(tok[0], "session") == 0)
  {
    if (n >= 2 && strcmp(tok[1], "start") == 0) session_start();
    else if (n >= 2 && (strcmp(tok[1], "stop") == 0 || strcmp(tok[1], "end") == 0)) session_stop("session stopped");
    else DBG_Puts("usage: session start|stop\r\n");

    /* Wake supervisor (starts timeout tracking only when session exists) */
    if (hSup) xTaskNotifyGive(hSup);

    cli_print_latency(t0);
    cli_print_prompt();
    return;
  }

  /* Other toggles like echo/time/prompt/keepalive/bin/lat/mem ... */
  /* ... implement similarly with early returns ... */

  /* Require session for hardware ops */
  if (!session_active)
  {
    DBG_Puts("[debug] no active session. type: session start\r\n");
    cli_print_latency(t0);
    cli_print_prompt();
    return;
  }

  /* Button */
  if (strcmp(tok[0], "btn") == 0)
  {
    GPIO_PinState st = HAL_GPIO_ReadPin(g_cfg.btn_port, g_cfg.btn_pin);
    DBG_Puts(st == GPIO_PIN_RESET ? "BTN: pressed\r\n" : "BTN: released\r\n");
    cli_print_latency(t0);
    cli_print_prompt();
    return;
  }

  /* I2C */
  if (strcmp(tok[0], "i2c") == 0)
  {
    /* parse i2c scan/rd/wr/script and enqueue jobs */
    /* (reuse your existing logic here but with early returns) */
    /* After queueing, I2C worker wakes automatically because it blocks on queue */
    cli_print_latency(t0);
    cli_print_prompt();
    return;
  }

  DBG_Puts("unknown cmd. type help\r\n");
  cli_print_latency(t0);
  cli_print_prompt();
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

  /* Copy config (pointers inside must remain valid) */
  g_cfg = *cfg;

  /* Reset runtime state */
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

  login_fails = 0;
  lockout_until_tick = 0;

  tx_drops = 0;
  rx_overflows = 0;

  script_len = 0;
  script_recording = 0;

  hist_count = 0;
  hist_head = 0;
  hist_nav = -1;

  /* ---------------- Create static RX StreamBuffer ----------------
   * Trigger level = 1 byte (CLI wakes on each byte).
   */
  sbUartRx = xStreamBufferCreateStatic(
      RX_SB_SIZE,
      1,
      sbRxStorage,
      &sbRxStruct
  );

  /* ---------------- Create static TX MessageBuffer ----------------
   * MessageBuffer is a specialization of StreamBuffer.
   * Trigger level = 1 byte (TX task wakes as soon as something exists).
   */
  mbUartTx = xMessageBufferCreateStatic(
      TX_MB_SIZE,
      mbTxStorage,
      &mbTxStruct
  );

  /* ---------------- Create static I2C job queue ---------------- */
  qI2cJobs = xQueueCreateStatic(
      I2C_QUEUE_LEN,
      sizeof(i2c_job_t),
      qI2cStorage,
      &qI2cStatic
  );

  /* Validate creation (optional but recommended) */
  if (!sbUartRx || !mbUartTx || !qI2cJobs)
  {
    /* If any failed, do not proceed (should not fail if sizes are correct) */
    return;
  }

  /* ---------------- Create tasks using xTaskCreateStatic ---------------- */
  hTx = xTaskCreateStatic(
      vUartTxTask,
      "dbg_tx",
      DBG_TX_STACK_WORDS,
      NULL,
      tskIDLE_PRIORITY + 3,
      stkTx,
      &tcbTx
  );

  hCli = xTaskCreateStatic(
      vCliTask,
      "dbg_cli",
      DBG_CLI_STACK_WORDS,
      NULL,
      tskIDLE_PRIORITY + 2,
      stkCli,
      &tcbCli
  );

  hI2c = xTaskCreateStatic(
      vI2cTask,
      "dbg_i2c",
      DBG_I2C_STACK_WORDS,
      NULL,
      tskIDLE_PRIORITY + 1,
      stkI2c,
      &tcbI2c
  );

  hSup = xTaskCreateStatic(
      vSupervisorTask,
      "dbg_sup",
      DBG_SUP_STACK_WORDS,
      NULL,
      tskIDLE_PRIORITY + 1,
      stkSup,
      &tcbSup
  );

  hKeep = xTaskCreateStatic(
      vKeepaliveTask,
      "dbg_ka",
      DBG_KA_STACK_WORDS,
      NULL,
      tskIDLE_PRIORITY + 1,
      stkKeep,
      &tcbKeep
  );

  /* Start with no session: suspend I2C worker to reduce load */
  if (hI2c) vTaskSuspend(hI2c);

  /* Start UART RX interrupt */
  uart_rx_start();
}