/**
 * @file   ultrasonic.h
 * @brief  Driver HC-SR04 menggunakan EXTI + DWT CYCCNT.
 *
 * Arsitektur:
 *   - TaskUltrasonic trigger trig pin -> tunggu osDelay -> baca hasil
 *   - EXTI rising edge: catat DWT timestamp start
 *   - EXTI falling edge: hitung durasi, konversi ke mm
 *   - 4 pasang trig/echo, round-robin scheduling
 *
 * Pinout (lihat PROJECT.md):
 *   DEPAN:    Trig=PC2,  Echo1=PC8,  Echo2=PC9
 *   KANAN:    Trig=PC3,  Echo3=PC10, Echo4=PC11
 *   BELAKANG: Trig=PD2,  Echo5=PC4,  Echo6=PC5
 *   KIRI:     Trig=PC12, Echo7=PB12, Echo8=PB2
 */

#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "stm32f4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ───────────────────────────────────────────────────── */

/** @brief Number of ultrasonic echo sensors (2 per direction x 4 directions). */
#define ULTRASONIC_COUNT           8

/** @brief Trigger pulse width in microseconds (HC-SR04 datasheet: min 10 us). */
#define ULTRASONIC_TRIG_PULSE_US   10

/** @brief Echo timeout in microseconds (~5.1 m max range). */
#define ULTRASONIC_ECHO_TIMEOUT_US 30000

/** @brief Minimum valid distance in mm. */
#define ULTRASONIC_MIN_DIST_MM     20

/** @brief Maximum valid distance in mm. */
#define ULTRASONIC_MAX_DIST_MM     4000

/** @brief Delay between successive trigger pairs in ms (allows echo to decay). */
#define ULTRASONIC_CYCLE_DELAY_MS  60

/* ── Data Types ──────────────────────────────────────────────────── */

/**
 * @brief Aggregated ultrasonic measurement data (thread-safe snapshot).
 */
typedef struct {
  uint16_t distance_mm[ULTRASONIC_COUNT]; /**< Distance per echo sensor (index 0-7). */
  uint8_t  valid[ULTRASONIC_COUNT];       /**< 1=valid measurement, 0=timeout/out-of-range. */
  uint32_t timestamp_ms;                  /**< HAL_GetTick() at last update. */
} UltrasonicData_t;

/**
 * @brief Per-echo-pin capture state for EXTI callback.
 */
typedef struct {
  uint32_t        rise_cyccnt;  /**< DWT->CYCCNT value at rising edge. */
  uint32_t        fall_cyccnt;  /**< DWT->CYCCNT value at falling edge. */
  volatile uint8_t state;       /**< 0=idle, 1=waiting_fall, 2=done. */
} EchoCapture_t;

/* ── Shared Data (defined in ultrasonic.c) ───────────────────────── */

/** @brief Latest ultrasonic measurements — written by TaskUltrasonic, read by TaskSerial. */
extern volatile UltrasonicData_t g_ultrasonic_data;

/** @brief Per-echo capture state — written by EXTI ISR, read by TaskUltrasonic. */
extern volatile EchoCapture_t g_echo_capture[ULTRASONIC_COUNT];

/* ── Public Functions ────────────────────────────────────────────── */

/**
 * @brief  Initialise DWT CYCCNT and reset all echo capture states.
 * @note   Must be called once before any trigger/capture operations.
 */
void Ultrasonic_Init(void);

/**
 * @brief  Fire the trigger pin for a sensor pair and prepare echo captures.
 * @param  pair_index  0=DEPAN, 1=KANAN, 2=BELAKANG, 3=KIRI.
 * @note   After calling this, the EXTI ISR will capture echo timestamps.
 *         Wait ULTRASONIC_CYCLE_DELAY_MS before calling ProcessCaptures.
 */
void Ultrasonic_TriggerPair(uint8_t pair_index);

/**
 * @brief  Process captured echo timestamps for a sensor pair.
 * @param  pair_index  0=DEPAN, 1=KANAN, 2=BELAKANG, 3=KIRI.
 * @note   Converts DWT cycle counts to distance_mm, validates range,
 *         and updates g_ultrasonic_data with a critical section.
 */
void Ultrasonic_ProcessCaptures(uint8_t pair_index);

/**
 * @brief  Thread-safe copy of current ultrasonic data.
 * @param  out  Destination buffer (caller-owned).
 */
void Ultrasonic_GetData(UltrasonicData_t *out);

/**
 * @brief  EXTI callback — called from HAL_GPIO_EXTI_Callback in ISR context.
 * @param  echo_index  Index 0-7 corresponding to ECHO1-ECHO8.
 * @note   Reads GPIO level to determine rising/falling edge.
 */
void Ultrasonic_EchoCallback(uint8_t echo_index);

#ifdef __cplusplus
}
#endif

#endif /* ULTRASONIC_H */
