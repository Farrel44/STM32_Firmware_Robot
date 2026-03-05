/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    app_freertos.c
  * @brief   CMSIS-RTOS2 (FreeRTOS) application layer.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "app_freertos.h"

#include "gpio.h"
#include "tim.h"
#include "usart.h"

#include "pid.h"
#include "serial_protocol.h"

#include "control_config.h"
#include "encoder_quad.h"
#include "motor_pwm.h"
#include "mpu6050.h"
#include "i2c.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

/* Private function prototypes -----------------------------------------------*/
static void TaskPid(void *argument);
static void TaskSerial(void *argument);
static void TaskImu(void *argument);
static void TaskUltrasonic(void *argument);

/* Global thread handles ------------------------------------------------------*/
osThreadId_t pidTaskHandle;
static osThreadId_t serialTaskHandle;

/* Command targets — written by UART ISR, read by TaskPid / TaskSerial. */
static volatile int16_t target_rpm1 = 0;
static volatile int16_t target_rpm2 = 0;
static volatile int16_t target_rpm3 = 0;
static volatile TickType_t last_valid_cmd_tick = 0;
static volatile bool comm_healthy = false;

/* Ramped setpoints — slew-rate limited copies of target_rpm (TaskPid only). */
static float actual_rpm1 = 0.0f;
static float actual_rpm2 = 0.0f;
static float actual_rpm3 = 0.0f;

/*
 * Encoder tick accumulators — bridge between 200 Hz PID and 50 Hz feedback.
 * TaskPid increments; TaskSerial reads + resets inside a critical section.
 */
static volatile int32_t tick1_accum = 0;
static volatile int32_t tick2_accum = 0;
static volatile int32_t tick3_accum = 0;

/*
 * Filtered IMU outputs — written by TaskImu, read by TaskSerial.
 * All 6 values form a consistent snapshot; use a critical section when
 * reading or writing the full set to avoid torn reads across task boundaries.
 *
 * Gyro: milli-rad/s (EMA-filtered).  Accel: milli-m/s² (raw, hardware DLPF only).
 */
static volatile int16_t imu_gyro_x = 0;
static volatile int16_t imu_gyro_y = 0;
static volatile int16_t imu_gyro_z = 0;
static volatile int16_t imu_accel_x = 0;
static volatile int16_t imu_accel_y = 0;
static volatile int16_t imu_accel_z = 0;
static volatile bool imu_ready = false;

static SerialProtoRx serial_rx;
static uint8_t uart2_rx_dma_buf[64];

/* CMSIS-RTOS2 init -----------------------------------------------------------*/
void MX_FREERTOS_Init(void)
{
  /* Motor PID — highest priority; woken by TIM6 200 Hz ISR via thread flag. */
  const osThreadAttr_t pidTaskAttributes = {
    .name = "pid",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 1024
  };
  pidTaskHandle = osThreadNew(TaskPid, NULL, &pidTaskAttributes);

  /* Serial — 50 Hz feedback TX + comm watchdog. */
  const osThreadAttr_t serialTaskAttributes = {
    .name = "serial",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  serialTaskHandle = osThreadNew(TaskSerial, NULL, &serialTaskAttributes);

  /* IMU — 100 Hz MPU6050 read + complementary filter (uses atan2f/sqrtf). */
  const osThreadAttr_t imuTaskAttributes = {
    .name = "imu",
    .priority = (osPriority_t) osPriorityBelowNormal,
    .stack_size = 1536
  };
  (void)osThreadNew(TaskImu, NULL, &imuTaskAttributes);

  /* Ultrasonic — placeholder, not yet implemented. */
  const osThreadAttr_t usTaskAttributes = {
    .name = "ultra",
    .priority = (osPriority_t) osPriorityLow,
    .stack_size = 768
  };
  (void)osThreadNew(TaskUltrasonic, NULL, &usTaskAttributes);
}

/* Helper: clamp int16_t to ±limit. */
static int16_t clamp_rpm(int16_t x, int16_t limit)
{
  if (x > limit)
  {
    return limit;
  }
  if (x < -limit)
  {
    return (int16_t)-limit;
  }
  return x;
}

/* USART2 RX-to-idle DMA callback (ISR context).
 * Parses command packets, stores clamped RPM targets, refreshes watchdog. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == USART2)
  {
    CommandPacket cmd;
    (void)SerialProtoRx_ParseBytes(&serial_rx, uart2_rx_dma_buf, (size_t)size, &cmd);

    if (cmd.valid)
    {
      target_rpm1 = clamp_rpm(cmd.rpm1, CONTROL_MAX_RPM);
      target_rpm2 = clamp_rpm(cmd.rpm2, CONTROL_MAX_RPM);
      target_rpm3 = clamp_rpm(cmd.rpm3, CONTROL_MAX_RPM);
      last_valid_cmd_tick = xTaskGetTickCountFromISR();
      comm_healthy = true;
    }

    /* Re-arm DMA reception (half-transfer IRQ disabled — only idle-line fires). */
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buf, sizeof(uart2_rx_dma_buf));
    __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);
  }
}

/* Timer callbacks ------------------------------------------------------------*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM6)
  {
    /* 200Hz PID tick: notify PID task. */
    (void)osThreadFlagsSet(pidTaskHandle, PID_TICK_FLAG);
  }
}

/* Tasks ---------------------------------------------------------------------*/
static bool is_near_zero_rpm(float x)
{
  return (x > -0.3f) && (x < 0.3f);
}

/* Feedforward linearisation — returns signed PWM offset for given target. */
static float compute_feedforward(float target_rpm)
{
  const float abs_rpm = (target_rpm < 0.0f) ? -target_rpm : target_rpm;
  if (abs_rpm < CONTROL_FF_DEADBAND_RPM)
  {
    return 0.0f;
  }
  /* Preserve sign so feedforward and PID agree on direction. */
  const float magnitude = CONTROL_FF_OFFSET + (CONTROL_FF_SLOPE * abs_rpm);
  return (target_rpm < 0.0f) ? -magnitude : magnitude;
}

static int16_t clamp_pwm_255(int32_t pwm)
{
  if (pwm > CONTROL_PWM_MAX)
  {
    return CONTROL_PWM_MAX;
  }
  if (pwm < -CONTROL_PWM_MAX)
  {
    return (int16_t)-CONTROL_PWM_MAX;
  }
  return (int16_t)pwm;
}

static int16_t rate_limit_pwm(int16_t desired, int16_t *prev)
{
  const int16_t diff = (int16_t)(desired - *prev);
  if (diff > CONTROL_PWM_RATE_LIMIT_PER_TICK)
  {
    *prev = (int16_t)(*prev + CONTROL_PWM_RATE_LIMIT_PER_TICK);
    return *prev;
  }
  if (diff < -CONTROL_PWM_RATE_LIMIT_PER_TICK)
  {
    *prev = (int16_t)(*prev - CONTROL_PWM_RATE_LIMIT_PER_TICK);
    return *prev;
  }
  *prev = desired;
  return desired;
}

/* Slew-rate limit a single RPM channel toward its command target.
 * Snaps instantly on sign reversal or near-zero convergence. */
static float ramp_single(float actual, float target)
{
  /* Instant snap on direction change. */
  if ((actual > 0.0f && target < 0.0f) || (actual < 0.0f && target > 0.0f))
  {
    return target;
  }

  /* Snap to zero when target is 0 and close enough. */
  const float abs_actual = (actual < 0.0f) ? -actual : actual;
  if (target == 0.0f && abs_actual < CONTROL_ZERO_THRESHOLD)
  {
    return 0.0f;
  }

  /* Gradual ramp. */
  if (actual < target)
  {
    actual += CONTROL_RAMP_UP_RATE;
    if (actual > target)
    {
      actual = target;
    }
  }
  else if (actual > target)
  {
    actual -= CONTROL_RAMP_DOWN_RATE;
    if (actual < target)
    {
      actual = target;
    }
  }

  return actual;
}

/*
 * TaskPid — 200 Hz closed-loop motor control (feedforward + PID).
 * Priority: osPriorityHigh.  Woken deterministically by TIM6 ISR thread flag.
 *
 * Pipeline per tick:
 *   encoder delta → RPM (EMA-filtered) → ramp → FF+PID → rate-limited PWM.
 *
 * Shared resources:
 *   target_rpm (volatile, written by UART ISR)
 *   tick_accum (volatile, read+reset by TaskSerial under critical section)
 */
static void TaskPid(void *argument)
{
  (void)argument;

  PIDController pid1;
  PIDController pid2;
  PIDController pid3;
  PID_Init(&pid1, CONTROL_PID_KP, CONTROL_PID_KI, CONTROL_PID_KD);
  PID_Init(&pid2, CONTROL_PID_KP, CONTROL_PID_KI, CONTROL_PID_KD);
  PID_Init(&pid3, CONTROL_PID_KP, CONTROL_PID_KI, CONTROL_PID_KD);

  PID_SetOutputLimits(&pid1, (float)-CONTROL_PWM_MAX, (float)CONTROL_PWM_MAX);
  PID_SetOutputLimits(&pid2, (float)-CONTROL_PWM_MAX, (float)CONTROL_PWM_MAX);
  PID_SetOutputLimits(&pid3, (float)-CONTROL_PWM_MAX, (float)CONTROL_PWM_MAX);

  PID_SetIntegralLimits(&pid1, CONTROL_PID_INTEGRAL_LIMIT);
  PID_SetIntegralLimits(&pid2, CONTROL_PID_INTEGRAL_LIMIT);
  PID_SetIntegralLimits(&pid3, CONTROL_PID_INTEGRAL_LIMIT);

  const float dt_sec = 1.0f / 200.0f;

  /* Reconfigure encoder timers from CubeMX 1x default to full 4x quadrature. */
  EncoderQuad_ConfigureFullQuad(&htim3);
  EncoderQuad_ConfigureFullQuad(&htim4);
  EncoderQuad_ConfigureFullQuad(&htim8);

  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  uint16_t enc1_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint16_t enc2_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  uint16_t enc3_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

  MotorPwm_Init();

  /* Start 200 Hz deterministic tick source (TIM6 update interrupt). */
  (void)HAL_TIM_Base_Start_IT(&htim6);

  float rpm1 = 0.0f;
  float rpm2 = 0.0f;
  float rpm3 = 0.0f;
  float rpm1_f = 0.0f;
  float rpm2_f = 0.0f;
  float rpm3_f = 0.0f;

  int16_t last_pwm1 = 0;
  int16_t last_pwm2 = 0;
  int16_t last_pwm3 = 0;

  for (;;)
  {
    (void)osThreadFlagsWait(PID_TICK_FLAG, osFlagsWaitAny, osWaitForever);

    /*
     * ENCODER OVERFLOW SAFETY ANALYSIS (Rec 5)
     *
     * Hardware: TIM3, TIM4, TIM8 — 16-bit counters (0–65535).
     * Encoder resolution: 380 ticks/wheel_revolution.
     * Max motor speed: 600 RPM → max wheel speed ≈ 600 RPM
     *   (gearbox ratio baked into ticks_per_rev definition).
     * Max tick rate: 600 RPM × 380 / 60 = 3800 ticks/second.
     *
     * Overflow time: 65536 / 3800 ≈ 17.2 seconds.
     *
     * SAFETY: TaskPid reads counters at 200 Hz (5 ms period).
     * Max ticks per read: 3800 × 0.005 = 19 ticks  ≪  32768.
     * Conclusion: Counter wrap is handled safely by the 16-bit
     * subtraction below, and overflow cannot occur between
     * consecutive reads at 200 Hz.
     *
     * CAVEAT: If this task is blocked for >17 s (debugger breakpoint,
     * priority inversion), the delta calculation will be incorrect.
     * In production, ensure no higher-priority task can starve TaskPid
     * for that duration.
     */
    const uint16_t enc1_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    const uint16_t enc2_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    const uint16_t enc3_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

    /* 16-bit subtraction handles counter wrap (valid while |delta| < 32768). */
    int32_t d1 = (int32_t)(int16_t)(enc1_now - enc1_prev);
    int32_t d2 = (int32_t)(int16_t)(enc2_now - enc2_prev);
    int32_t d3 = (int32_t)(int16_t)(enc3_now - enc3_prev);

    if (CONTROL_INVERT_ENC1)
    {
      d1 = -d1;
    }
    if (CONTROL_INVERT_ENC2)
    {
      d2 = -d2;
    }
    if (CONTROL_INVERT_ENC3)
    {
      d3 = -d3;
    }

    /* Accumulate for TaskSerial (full delta between 50 Hz feedback sends). */
    tick1_accum += d1;
    tick2_accum += d2;
    tick3_accum += d3;

    enc1_prev = enc1_now;
    enc2_prev = enc2_now;
    enc3_prev = enc3_now;

    /* Delta ticks → RPM. */
    const float rpm_multi = 60.0f / (CONTROL_TICKS_PER_REV * dt_sec);
    rpm1 = (float)d1 * rpm_multi;
    rpm2 = (float)d2 * rpm_multi;
    rpm3 = (float)d3 * rpm_multi;

    /* EMA-filter measured RPM to reduce derivative noise. */
    rpm1_f = (CONTROL_EMA_ALPHA * rpm1) + (CONTROL_EMA_BETA * rpm1_f);
    rpm2_f = (CONTROL_EMA_ALPHA * rpm2) + (CONTROL_EMA_BETA * rpm2_f);
    rpm3_f = (CONTROL_EMA_ALPHA * rpm3) + (CONTROL_EMA_BETA * rpm3_f);

    const float sp1 = (float)target_rpm1;
    const float sp2 = (float)target_rpm2;
    const float sp3 = (float)target_rpm3;

    actual_rpm1 = ramp_single(actual_rpm1, sp1);
    actual_rpm2 = ramp_single(actual_rpm2, sp2);
    actual_rpm3 = ramp_single(actual_rpm3, sp3);

    const bool all_zero = is_near_zero_rpm(actual_rpm1) &&
                          is_near_zero_rpm(actual_rpm2) &&
                          is_near_zero_rpm(actual_rpm3);

    /* Safety: zero PWM when comm is lost or all targets are zero.
     * EN stays HIGH (same as ESP32) — stop = PWM 0, not EN toggle. */
    if (!comm_healthy || all_zero)
    {
      MotorPwm_StopAll();

      /* Flush ALL control state to prevent stale actuation on reconnect:
       *   - ramped setpoints → 0
       *   - rate-limiter history → 0
       *   - EMA-filtered RPM → 0  (prevents derivative spike on reconnect)
       *   - PID integral + prev_error → 0
       */
      actual_rpm1 = 0.0f;
      actual_rpm2 = 0.0f;
      actual_rpm3 = 0.0f;
      last_pwm1 = 0;
      last_pwm2 = 0;
      last_pwm3 = 0;
      rpm1_f = 0.0f;
      rpm2_f = 0.0f;
      rpm3_f = 0.0f;
      PID_Reset(&pid1);
      PID_Reset(&pid2);
      PID_Reset(&pid3);
      continue;
    }

    /* Clear PID integral in dead zone to prevent windup at low setpoints. */
    if (actual_rpm1 > -CONTROL_DEAD_ZONE_RPM && actual_rpm1 < CONTROL_DEAD_ZONE_RPM)
    {
      PID_Reset(&pid1);
    }
    if (actual_rpm2 > -CONTROL_DEAD_ZONE_RPM && actual_rpm2 < CONTROL_DEAD_ZONE_RPM)
    {
      PID_Reset(&pid2);
    }
    if (actual_rpm3 > -CONTROL_DEAD_ZONE_RPM && actual_rpm3 < CONTROL_DEAD_ZONE_RPM)
    {
      PID_Reset(&pid3);
    }

    const float ff1 = compute_feedforward(actual_rpm1);
    const float ff2 = compute_feedforward(actual_rpm2);
    const float ff3 = compute_feedforward(actual_rpm3);

    const float trim1 = PID_Compute(&pid1, rpm1_f, actual_rpm1, dt_sec);
    const float trim2 = PID_Compute(&pid2, rpm2_f, actual_rpm2, dt_sec);
    const float trim3 = PID_Compute(&pid3, rpm3_f, actual_rpm3, dt_sec);

    const int16_t pwm1_raw = clamp_pwm_255((int32_t)(ff1 + trim1));
    const int16_t pwm2_raw = clamp_pwm_255((int32_t)(ff2 + trim2));
    const int16_t pwm3_raw = clamp_pwm_255((int32_t)(ff3 + trim3));

    const int16_t pwm1 = rate_limit_pwm(pwm1_raw, &last_pwm1);
    const int16_t pwm2 = rate_limit_pwm(pwm2_raw, &last_pwm2);
    const int16_t pwm3 = rate_limit_pwm(pwm3_raw, &last_pwm3);


    MotorPwm_Set(1, pwm1);
    MotorPwm_Set(2, pwm2);
    MotorPwm_Set(3, pwm3);
  }
}

/*
 * TaskSerial — 50 Hz feedback TX and communication watchdog.
 * Priority: osPriorityNormal.  Uses osDelayUntil for jitter-free period.
 *
 * Shared resources:
 *   tick_accum   — read+reset under critical section (vs TaskPid).
 *   target_rpm   — zeroed under critical section on watchdog timeout (vs UART ISR).
 *   imu_gyro/accel — read under critical section (vs TaskImu).
 */
static void TaskSerial(void *argument)
{
  (void)argument;

  SerialProtoRx_Init(&serial_rx);
  last_valid_cmd_tick = xTaskGetTickCount();
  comm_healthy = false;

  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buf, sizeof(uart2_rx_dma_buf));
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  TickType_t last_wake = osKernelGetTickCount();

  for (;;)
  {
    /* Comm watchdog — halt motors if no valid command within timeout.
     * Critical section prevents UART ISR from writing new targets mid-clear.
     */
    const TickType_t now = xTaskGetTickCount();
    if ((now - last_valid_cmd_tick) > pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS))
    {
      taskENTER_CRITICAL();
      comm_healthy = false;
      target_rpm1 = 0;
      target_rpm2 = 0;
      target_rpm3 = 0;
      taskEXIT_CRITICAL();
    }

    /* Atomically snapshot and reset tick accumulators. */
    taskENTER_CRITICAL();
    const int32_t t1 = tick1_accum;
    const int32_t t2 = tick2_accum;
    const int32_t t3 = tick3_accum;
    tick1_accum = 0;
    tick2_accum = 0;
    tick3_accum = 0;
    taskEXIT_CRITICAL();

    /* Snapshot IMU values (atomic w.r.t. TaskImu). */
    taskENTER_CRITICAL();
    const int16_t gx = imu_gyro_x;
    const int16_t gy = imu_gyro_y;
    const int16_t gz = imu_gyro_z;
    const int16_t ax = imu_accel_x;
    const int16_t ay = imu_accel_y;
    const int16_t az = imu_accel_z;
    taskEXIT_CRITICAL();

    FeedbackPacket fb = {
      .tick1 = t1,
      .tick2 = t2,
      .tick3 = t3,
      .gyro_x = gx,
      .gyro_y = gy,
      .gyro_z = gz,
      .accel_x = ax,
      .accel_y = ay,
      .accel_z = az,
    };

    uint8_t pkt[FEEDBACK_PACKET_SIZE];
    SerialProto_BuildFeedback(&fb, pkt);
    (void)HAL_UART_Transmit(&huart2, pkt, FEEDBACK_PACKET_SIZE, 10);

    /* Fixed-period 50 Hz — avoids cumulative drift from TX duration. */
    last_wake += 20;
    osDelayUntil(last_wake);
  }
}

/*
 * TaskImu — 100 Hz MPU6050 polling with EMA-filtered gyro output.
 * Priority: osPriorityBelowNormal (non-critical for motor safety).
 *
 * Outputs (milli-unit int16, written under critical section):
 *   imu_gyro_x/y/z  — EMA-filtered angular rate (milli-rad/s)
 *   imu_accel_x/y/z — raw linear acceleration (milli-m/s²)
 *
 * Self-disables after CONTROL_IMU_ERROR_THRESHOLD consecutive I2C failures.
 * When disabled, shared variables freeze at their last valid value; TaskSerial
 * reads zeros only at boot (initial value) which is safe.
 */
static void TaskImu(void *argument)
{
  (void)argument;

  if (!Mpu6050_Init(&hi2c1))
  {
    imu_ready = false;
    for (;;)
    {
      osDelay(1000);
    }
  }
  imu_ready = true;

  uint8_t error_count = 0;
  float filtered_gyro_x = 0.0f;
  float filtered_gyro_y = 0.0f;
  float filtered_gyro_z = 0.0f;

  const float alpha = CONTROL_IMU_FILTER_ALPHA;
  const float one_minus_alpha = 1.0f - alpha;

  for (;;)
  {
    osDelay(10);

    Mpu6050_RawData raw;
    if (!Mpu6050_ReadRaw(&hi2c1, &raw))
    {
      error_count++;
      if (error_count > CONTROL_IMU_ERROR_THRESHOLD)
      {
        imu_ready = false;
      }
      continue;
    }
    error_count = 0;
    imu_ready = true;

    Mpu6050_Data data;
    Mpu6050_ConvertToPhysical(&raw, &data);

    /* EMA low-pass on all 3 gyro axes to reject motor vibration.
     * Accel is left unfiltered — the MPU6050 hardware DLPF at 44 Hz
     * is sufficient, and accel filtering would add lag to tilt estimates. */
    filtered_gyro_x = alpha * filtered_gyro_x + one_minus_alpha * data.gyro_x;
    filtered_gyro_y = alpha * filtered_gyro_y + one_minus_alpha * data.gyro_y;
    filtered_gyro_z = alpha * filtered_gyro_z + one_minus_alpha * data.gyro_z;

    /* Convert to milli-units and write as an atomic snapshot. */
    const int16_t gx = (int16_t)(filtered_gyro_x * 1000.0f);
    const int16_t gy = (int16_t)(filtered_gyro_y * 1000.0f);
    const int16_t gz = (int16_t)(filtered_gyro_z * 1000.0f);
    const int16_t ax_out = (int16_t)(data.accel_x * 1000.0f);
    const int16_t ay_out = (int16_t)(data.accel_y * 1000.0f);
    const int16_t az_out = (int16_t)(data.accel_z * 1000.0f);

    taskENTER_CRITICAL();
    imu_gyro_x = gx;
    imu_gyro_y = gy;
    imu_gyro_z = gz;
    imu_accel_x = ax_out;
    imu_accel_y = ay_out;
    imu_accel_z = az_out;
    taskEXIT_CRITICAL();
  }
}

static void TaskUltrasonic(void *argument)
{
  (void)argument;
  for (;;)
  {
    /* TODO: schedule ultrasonic trigger pulses; echo timing via EXTI + DWT. */
    osDelay(50);
  }
}
