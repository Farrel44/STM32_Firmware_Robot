/**
 * @file   ultrasonic.c
 * @brief  HC-SR04 ultrasonic driver — EXTI + DWT CYCCNT implementation.
 *
 * Each sensor pair shares one trig pin and has two echo pins.
 * Trigger fires a 10 us pulse; EXTI ISR captures rising/falling
 * timestamps via DWT->CYCCNT (180 MHz, ~5.6 ns resolution).
 *
 * TaskUltrasonic calls TriggerPair -> osDelay -> ProcessCaptures
 * in a round-robin loop over 4 pairs (~4 Hz full cycle).
 */

// ADDED(phase2-ultrasonic)

#include "ultrasonic.h"
#include "main.h"

#include "FreeRTOS.h"
#include "task.h"

/* ── Shared Data ─────────────────────────────────────────────────── */

volatile UltrasonicData_t g_ultrasonic_data;
volatile EchoCapture_t    g_echo_capture[ULTRASONIC_COUNT];

/* ── Trigger Pin Lookup Table ────────────────────────────────────── */

/** @brief GPIO port for each trigger (indexed by pair_index 0-3). */
static GPIO_TypeDef *const trig_port[4] = {
  TRIG_DEPAN_GPIO_Port,     /* pair 0: DEPAN    → PC2  */
  TRIG_KANAN_GPIO_Port,     /* pair 1: KANAN    → PC3  */
  TRIG_BELAKANG_GPIO_Port,  /* pair 2: BELAKANG → PD2  */
  TRIG_KIRI_GPIO_Port       /* pair 3: KIRI     → PC12 */
};

/** @brief GPIO pin mask for each trigger (indexed by pair_index 0-3). */
static const uint16_t trig_pin[4] = {
  TRIG_DEPAN_Pin,           /* GPIO_PIN_2 on GPIOC */
  TRIG_KANAN_Pin,           /* GPIO_PIN_3 on GPIOC */
  TRIG_BELAKANG_Pin,        /* GPIO_PIN_2 on GPIOD */
  TRIG_KIRI_Pin             /* GPIO_PIN_12 on GPIOC */
};

/* ── Echo Pin Lookup Table (for EchoCallback GPIO read) ──────────── */

/** @brief GPIO port for each echo pin (indexed by echo_index 0-7). */
static GPIO_TypeDef *const echo_port[ULTRASONIC_COUNT] = {
  ECHO1_GPIO_Port,  /* PC8  */
  ECHO2_GPIO_Port,  /* PC9  */
  ECHO3_GPIO_Port,  /* PC10 */
  ECHO4_GPIO_Port,  /* PC11 */
  ECHO5_GPIO_Port,  /* PC4  */
  ECHO6_GPIO_Port,  /* PC5  */
  ECHO7_GPIO_Port,  /* PB12 */
  ECHO8_GPIO_Port   /* PB2  */
};

/** @brief GPIO pin mask for each echo pin (indexed by echo_index 0-7). */
static const uint16_t echo_pin[ULTRASONIC_COUNT] = {
  ECHO1_Pin,   /* GPIO_PIN_8  */
  ECHO2_Pin,   /* GPIO_PIN_9  */
  ECHO3_Pin,   /* GPIO_PIN_10 */
  ECHO4_Pin,   /* GPIO_PIN_11 */
  ECHO5_Pin,   /* GPIO_PIN_4  */
  ECHO6_Pin,   /* GPIO_PIN_5  */
  ECHO7_Pin,   /* GPIO_PIN_12 */
  ECHO8_Pin    /* GPIO_PIN_2  */
};

/* ── Public Functions ────────────────────────────────────────────── */

/**
 * @brief  Enable DWT cycle counter and zero all capture/data state.
 */
void Ultrasonic_Init(void)
{
  /* Enable DWT CYCCNT — requires trace unit enabled first. */
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Clear all echo capture states. */
  for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++)
  {
    g_echo_capture[i].rise_cyccnt = 0;
    g_echo_capture[i].fall_cyccnt = 0;
    g_echo_capture[i].state = 0;
  }

  /* Clear measurement data. */
  for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++)
  {
    g_ultrasonic_data.distance_mm[i] = 0;
    g_ultrasonic_data.valid[i] = 0;
  }
  g_ultrasonic_data.timestamp_ms = 0;
}

/**
 * @brief  Fire the trigger pulse for a sensor pair.
 * @param  pair_index  0=DEPAN, 1=KANAN, 2=BELAKANG, 3=KIRI.
 *
 * Sequence:
 *   1. Reset capture state for both echo pins in this pair.
 *   2. Set trig pin HIGH.
 *   3. DWT busy-wait 10 us (no osDelay — needs microsecond precision).
 *   4. Set trig pin LOW.
 *   5. EXTI ISR handles echo capture asynchronously.
 */
void Ultrasonic_TriggerPair(uint8_t pair_index)
{
  if (pair_index >= 4)
  {
    return;
  }

  const uint8_t idx_a = (uint8_t)(pair_index * 2);
  const uint8_t idx_b = (uint8_t)(pair_index * 2 + 1);

  /* Reset capture states to idle. */
  g_echo_capture[idx_a].state = 0;
  g_echo_capture[idx_b].state = 0;

  /* Fire 10 us trigger pulse using DWT busy-wait. */
  HAL_GPIO_WritePin(trig_port[pair_index], trig_pin[pair_index], GPIO_PIN_SET);

  const uint32_t start = DWT->CYCCNT;
  const uint32_t delay_ticks = ULTRASONIC_TRIG_PULSE_US * (SystemCoreClock / 1000000UL);
  while ((DWT->CYCCNT - start) < delay_ticks)
  {
    /* busy wait — 10 us at 180 MHz = 1800 cycles */
  }

  HAL_GPIO_WritePin(trig_port[pair_index], trig_pin[pair_index], GPIO_PIN_RESET);
}

/**
 * @brief  Convert captured echo timestamps to distance and update shared data.
 * @param  pair_index  0=DEPAN, 1=KANAN, 2=BELAKANG, 3=KIRI.
 *
 * For each echo in the pair:
 *   - If state == 2 (done): compute distance from DWT delta.
 *   - If state != 2: mark as timeout (invalid).
 *   - Validate range [ULTRASONIC_MIN_DIST_MM, ULTRASONIC_MAX_DIST_MM].
 *   - Update g_ultrasonic_data inside a critical section.
 *
 * Distance formula (DWT at SystemCoreClock = 180 MHz):
 *   distance_mm = duration_ticks * 343 / (2 * (SystemCoreClock / 1000))
 *   Sound speed = 343 m/s = 343000 mm/s.  Divide by 2 for round-trip.
 */
void Ultrasonic_ProcessCaptures(uint8_t pair_index)
{
  if (pair_index >= 4)
  {
    return;
  }

  const uint8_t idx_a = (uint8_t)(pair_index * 2);
  const uint8_t idx_b = (uint8_t)(pair_index * 2 + 1);
  const uint32_t clk_khz = SystemCoreClock / 1000UL;

  uint16_t dist[2] = {0, 0};
  uint8_t  ok[2]   = {0, 0};

  for (uint8_t i = 0; i < 2; i++)
  {
    const uint8_t idx = (i == 0) ? idx_a : idx_b;

    if (g_echo_capture[idx].state == 2)
    {
      /* Compute elapsed DWT ticks (handles 32-bit wrap via unsigned subtraction). */
      const uint32_t duration = g_echo_capture[idx].fall_cyccnt
                              - g_echo_capture[idx].rise_cyccnt;

      /* Convert to mm: dist = duration * 343 / (2 * clk_khz).
       * At 180 MHz, max 30 ms echo → ~5.4M ticks → fits uint32. */
      const uint32_t d = (duration * 343UL) / (2UL * clk_khz);

      if (d >= ULTRASONIC_MIN_DIST_MM && d <= ULTRASONIC_MAX_DIST_MM)
      {
        dist[i] = (uint16_t)d;
        ok[i] = 1;
      }
    }
    /* else: state 0 (idle — never triggered) or 1 (timeout — no falling edge) → invalid */
  }

  /* Atomic update of shared data. */
  taskENTER_CRITICAL();
  g_ultrasonic_data.distance_mm[idx_a] = dist[0];
  g_ultrasonic_data.valid[idx_a]       = ok[0];
  g_ultrasonic_data.distance_mm[idx_b] = dist[1];
  g_ultrasonic_data.valid[idx_b]       = ok[1];
  g_ultrasonic_data.timestamp_ms       = HAL_GetTick();
  taskEXIT_CRITICAL();
}

/**
 * @brief  Thread-safe copy of current ultrasonic data.
 * @param  out  Destination buffer (caller-owned, must not be NULL).
 */
void Ultrasonic_GetData(UltrasonicData_t *out)
{
  taskENTER_CRITICAL();
  for (uint8_t i = 0; i < ULTRASONIC_COUNT; i++)
  {
    out->distance_mm[i] = g_ultrasonic_data.distance_mm[i];
    out->valid[i]       = g_ultrasonic_data.valid[i];
  }
  out->timestamp_ms = g_ultrasonic_data.timestamp_ms;
  taskEXIT_CRITICAL();
}

/**
 * @brief  EXTI echo callback — called from HAL_GPIO_EXTI_Callback (ISR context).
 * @param  echo_index  0-7 mapping to ECHO1-ECHO8.
 *
 * Reads the current GPIO level to determine edge direction:
 *   - Rising  (HIGH): record DWT->CYCCNT as rise_cyccnt, set state=1.
 *   - Falling (LOW):  record DWT->CYCCNT as fall_cyccnt, set state=2.
 */
void Ultrasonic_EchoCallback(uint8_t echo_index)
{
  if (echo_index >= ULTRASONIC_COUNT)
  {
    return;
  }

  const uint32_t now = DWT->CYCCNT;

  if (HAL_GPIO_ReadPin(echo_port[echo_index], echo_pin[echo_index]) == GPIO_PIN_SET)
  {
    /* Rising edge — echo pulse started. */
    g_echo_capture[echo_index].rise_cyccnt = now;
    g_echo_capture[echo_index].state = 1;
  }
  else
  {
    /* Falling edge — echo pulse ended. */
    if (g_echo_capture[echo_index].state == 1)
    {
      g_echo_capture[echo_index].fall_cyccnt = now;
      g_echo_capture[echo_index].state = 2;
    }
  }
}
