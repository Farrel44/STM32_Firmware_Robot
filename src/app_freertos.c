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

/* Latest command targets (updated on valid command reception). */
static volatile int16_t target_rpm1 = 0;
static volatile int16_t target_rpm2 = 0;
static volatile int16_t target_rpm3 = 0;
static volatile TickType_t last_valid_cmd_tick = 0;
static volatile bool comm_healthy = false;

/* Ramped RPM targets: smoothly approach target_rpm to prevent wheel slip. */
static float actual_rpm1 = 0.0f;
static float actual_rpm2 = 0.0f;
static float actual_rpm3 = 0.0f;

/* Accumulated encoder delta ticks since last feedback send.
 * Why accumulate: PID runs at 200Hz but feedback sends at 50Hz.
 * Without accumulation, TaskSerial would only see the last 5ms delta
 * (1/4 of actual movement), breaking odometry on the Pi side.
 * TaskPid adds deltas; TaskSerial reads + resets atomically.
 */
static volatile int32_t tick1_accum = 0;
static volatile int32_t tick2_accum = 0;
static volatile int32_t tick3_accum = 0;

/* IMU filtered outputs (updated by TaskImu, read by TaskSerial).
 * gyro_z_filtered: EMA-filtered angular velocity (rad/s * 1000 -> milli-rad/s).
 * angle_x_filtered: complementary-filtered pitch angle (rad * 1000 -> milli-rad).
 */
static volatile int16_t imu_gyro_z = 0;
static volatile int16_t imu_angle_x = 0;
static volatile bool imu_ready = false;

static SerialProtoRx serial_rx;
static uint8_t uart2_rx_dma_buf[64];

/* CMSIS-RTOS2 init -----------------------------------------------------------*/
void MX_FREERTOS_Init(void)
{
  /* PID task: deterministic tick from TIM6 ISR via thread flags. */
  const osThreadAttr_t pidTaskAttributes = {
    .name = "pid",
    .priority = (osPriority_t) osPriorityHigh,
    .stack_size = 1024
  };
  pidTaskHandle = osThreadNew(TaskPid, NULL, &pidTaskAttributes);

  /* Serial task: placeholder for UART DMA idle-line RX (later). */
  const osThreadAttr_t serialTaskAttributes = {
    .name = "serial",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  serialTaskHandle = osThreadNew(TaskSerial, NULL, &serialTaskAttributes);

  /* IMU task: placeholder for MPU6050 polling/filter (later). */
  const osThreadAttr_t imuTaskAttributes = {
    .name = "imu",
    .priority = (osPriority_t) osPriorityBelowNormal,
    .stack_size = 1024
  };
  (void)osThreadNew(TaskImu, NULL, &imuTaskAttributes);

  /* Ultrasonic task: placeholder for trigger scheduling (later). */
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

/* USART2 RX-to-idle DMA callback (called from ISR context). */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart->Instance == USART2)
  {
    CommandPacket cmd;
    (void)SerialProtoRx_ParseBytes(&serial_rx, uart2_rx_dma_buf, (size_t)size, &cmd);

    if (cmd.valid)
    {
      /* Clamp RPM to safe operating range before storing. */
      target_rpm1 = clamp_rpm(cmd.rpm1, CONTROL_MAX_RPM);
      target_rpm2 = clamp_rpm(cmd.rpm2, CONTROL_MAX_RPM);
      target_rpm3 = clamp_rpm(cmd.rpm3, CONTROL_MAX_RPM);
      last_valid_cmd_tick = xTaskGetTickCountFromISR();
      comm_healthy = true;
    }

    /* Re-arm reception. */
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
  /* Why: avoid float residue keeping motors enabled/active when command is effectively zero. */
  return (x > -0.3f) && (x < 0.3f);
}

static float compute_feedforward(float target_rpm)
{
  const float abs_rpm = (target_rpm < 0.0f) ? -target_rpm : target_rpm;
  if (abs_rpm < CONTROL_FF_DEADBAND_RPM)
  {
    return 0.0f;
  }
  /* Sign must match direction — without this, feedforward fights PID
   * on reverse commands (ESP32 original had the same sign logic).
   */
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

/*
 * Ramp a single motor's actual target toward its command target.
 *
 * Behavior:
 * - Gradual ramp up/down to prevent wheel slip on sudden commands.
 * - Instant snap on direction change to avoid conflicting motor states.
 * - Snap to zero when target is 0 and actual is near zero (avoids float residue).
 */
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

  /* Configure encoder timers to full quadrature to match CONTROL_TICKS_PER_REV.
   * Why: keeps RPM math consistent with the existing ROS/ESP calibration.
   */
  EncoderQuad_ConfigureFullQuad(&htim3);
  EncoderQuad_ConfigureFullQuad(&htim4);
  EncoderQuad_ConfigureFullQuad(&htim8);

  /* Start encoder timers (HW encoder mode). */
  (void)HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
  (void)HAL_TIM_Encoder_Start(&htim8, TIM_CHANNEL_ALL);

  uint16_t enc1_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
  uint16_t enc2_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
  uint16_t enc3_prev = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

  /* Initialize PWM outputs and keep motors disabled until comm is healthy. */
  MotorPwm_Init();

  /* Start the deterministic 200Hz tick source. */
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

  bool motors_enabled = false;

  for (;;)
  {
    (void)osThreadFlagsWait(PID_TICK_FLAG, osFlagsWaitAny, osWaitForever);

    const uint16_t enc1_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim3);
    const uint16_t enc2_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim4);
    const uint16_t enc3_now = (uint16_t)__HAL_TIM_GET_COUNTER(&htim8);

    /* Signed 16-bit delta handles wrap-around (assumes delta < 32768 per tick). */
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

    /* Accumulate (not overwrite) so TaskSerial gets the full delta
     * between feedback sends, not just the last PID cycle's slice.
     */
    tick1_accum += d1;
    tick2_accum += d2;
    tick3_accum += d3;

    enc1_prev = enc1_now;
    enc2_prev = enc2_now;
    enc3_prev = enc3_now;

    /* Convert delta ticks -> RPM.
     * Why: control loop uses RPM setpoints from the serial protocol.
     */
    const float rpm_multi = 60.0f / (CONTROL_TICKS_PER_REV * dt_sec);
    rpm1 = (float)d1 * rpm_multi;
    rpm2 = (float)d2 * rpm_multi;
    rpm3 = (float)d3 * rpm_multi;

    /* Filter RPM (EMA) to reduce derivative noise. */
    rpm1_f = (CONTROL_EMA_ALPHA * rpm1) + (CONTROL_EMA_BETA * rpm1_f);
    rpm2_f = (CONTROL_EMA_ALPHA * rpm2) + (CONTROL_EMA_BETA * rpm2_f);
    rpm3_f = (CONTROL_EMA_ALPHA * rpm3) + (CONTROL_EMA_BETA * rpm3_f);

    /* Read command targets (clamped on reception). */
    const float sp1 = (float)target_rpm1;
    const float sp2 = (float)target_rpm2;
    const float sp3 = (float)target_rpm3;

    /* Apply smooth ramping toward command targets. */
    actual_rpm1 = ramp_single(actual_rpm1, sp1);
    actual_rpm2 = ramp_single(actual_rpm2, sp2);
    actual_rpm3 = ramp_single(actual_rpm3, sp3);

    const bool all_zero = is_near_zero_rpm(actual_rpm1) &&
                          is_near_zero_rpm(actual_rpm2) &&
                          is_near_zero_rpm(actual_rpm3);

    /* Safety: never drive motors if comm is unhealthy. */
    if (!comm_healthy || all_zero)
    {
      if (motors_enabled)
      {
        MotorPwm_StopAll();
        MotorPwm_SetEnabled(false);
        motors_enabled = false;
      }

      /* Reset ramp state along with PID to avoid stale targets on reconnect. */
      actual_rpm1 = 0.0f;
      actual_rpm2 = 0.0f;
      actual_rpm3 = 0.0f;
      last_pwm1 = 0;
      last_pwm2 = 0;
      last_pwm3 = 0;
      PID_Reset(&pid1);
      PID_Reset(&pid2);
      PID_Reset(&pid3);
      continue;
    }

    /* Reset PID in dead zone to prevent integral windup. */
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

    /* Use ramped targets for feedforward and PID. */
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

    if (!motors_enabled)
    {
      /* Why enable only after we have a non-zero, valid command:
       * prevents motor twitch during boot and makes watchdog behavior deterministic.
       */
      MotorPwm_StopAll();
      MotorPwm_SetEnabled(true);
      motors_enabled = true;
    }

    MotorPwm_Set(1, pwm1);
    MotorPwm_Set(2, pwm2);
    MotorPwm_Set(3, pwm3);
  }
}

static void TaskSerial(void *argument)
{
  (void)argument;

  SerialProtoRx_Init(&serial_rx);
  last_valid_cmd_tick = xTaskGetTickCount();
  comm_healthy = false;

  /* Start UART2 RX-to-idle DMA. */
  (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart2, uart2_rx_dma_buf, sizeof(uart2_rx_dma_buf));
  __HAL_DMA_DISABLE_IT(huart2.hdmarx, DMA_IT_HT);

  for (;;)
  {
    const TickType_t now = xTaskGetTickCount();
    if ((now - last_valid_cmd_tick) > pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS))
    {
      /* Communication watchdog -> stop targets. */
      comm_healthy = false;
      target_rpm1 = 0;
      target_rpm2 = 0;
      target_rpm3 = 0;
    }

    /* Send feedback at 50Hz (same protocol as ESP).
     * Read accumulated ticks then reset — this gives the Pi the full
     * encoder delta since the previous feedback, not just one PID slice.
     */
    taskENTER_CRITICAL();
    const int32_t t1 = tick1_accum;
    const int32_t t2 = tick2_accum;
    const int32_t t3 = tick3_accum;
    tick1_accum = 0;
    tick2_accum = 0;
    tick3_accum = 0;
    taskEXIT_CRITICAL();

    FeedbackPacket fb = {
      .tick1 = t1,
      .tick2 = t2,
      .tick3 = t3,
      .gyro_z = imu_gyro_z,
      .accel_z = imu_angle_x,
    };

    uint8_t pkt[FEEDBACK_PACKET_SIZE];
    SerialProto_BuildFeedback(&fb, pkt);
    (void)HAL_UART_Transmit(&huart2, pkt, FEEDBACK_PACKET_SIZE, 10);

    osDelay(20);
  }
}

static void TaskImu(void *argument)
{
  (void)argument;

  /* Initialize MPU6050. */
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
  float filtered_angle_x = 0.0f;  /* Pitch angle (rad). */
  float filtered_gyro_z = 0.0f;   /* Yaw rate (rad/s). */
  TickType_t last_tick = xTaskGetTickCount();

  const float alpha = CONTROL_IMU_FILTER_ALPHA;
  const float one_minus_alpha = 1.0f - alpha;

  for (;;)
  {
    osDelay(10);  /* 100Hz IMU update rate. */

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

    /* Compute dt from tick delta. */
    const TickType_t now_tick = xTaskGetTickCount();
    float dt = (float)(now_tick - last_tick) / 1000.0f;
    last_tick = now_tick;

    /* Sanity check dt (guard against tick wrap or scheduling jitter). */
    if (dt < 0.001f || dt > 0.1f)
    {
      dt = 0.01f;
    }

    /* Compute pitch angle from accelerometer (gravity reference).
     * atan2(ay, sqrt(ax^2 + az^2)) gives tilt about X axis.
     */
    const float ax2 = data.accel_x * data.accel_x;
    const float az2 = data.accel_z * data.accel_z;
    const float accel_angle_x = atan2f(data.accel_y, sqrtf(ax2 + az2));

    /* Complementary filter for pitch angle:
     * - Gyro integration for short-term (high-pass).
     * - Accelerometer for long-term drift correction (low-pass).
     */
    filtered_angle_x = alpha * (filtered_angle_x + data.gyro_x * dt)
                     + one_minus_alpha * accel_angle_x;

    /* EMA filter for gyro Z (yaw rate).
     * Reduces high-frequency noise for odometry fusion.
     */
    filtered_gyro_z = alpha * filtered_gyro_z + one_minus_alpha * data.gyro_z;

    /* Scale to milli-rad(/s) and store for feedback packet. */
    imu_gyro_z = (int16_t)(filtered_gyro_z * 1000.0f);
    imu_angle_x = (int16_t)(filtered_angle_x * 1000.0f);
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
