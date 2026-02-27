#include "pid.h"

static float clampf(float x, float min, float max)
{
  if (x < min)
  {
    return min;
  }
  if (x > max)
  {
    return max;
  }
  return x;
}

void PID_Init(PIDController *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;

  pid->integral = 0.0f;
  pid->prev_error = 0.0f;

  pid->output_min = -255.0f;
  pid->output_max = 255.0f;
  pid->integral_max = 1000.0f;
}

void PID_SetOutputLimits(PIDController *pid, float min, float max)
{
  pid->output_min = min;
  pid->output_max = max;
}

void PID_SetIntegralLimits(PIDController *pid, float limit)
{
  if (limit < 0.0f)
  {
    limit = -limit;
  }
  pid->integral_max = limit;
}

void PID_SetTuning(PIDController *pid, float kp, float ki, float kd)
{
  pid->kp = kp;
  pid->ki = ki;
  pid->kd = kd;
}

float PID_Compute(PIDController *pid, float input, float setpoint, float dt_sec)
{
  if ((dt_sec <= 0.00001f) || (dt_sec > 1.0f))
  {
    dt_sec = 0.01f;
  }

  const float error = setpoint - input;

  const float p = pid->kp * error;

  pid->integral += (error * dt_sec);
  pid->integral = clampf(pid->integral, -pid->integral_max, pid->integral_max);
  const float i = pid->ki * pid->integral;

  const float derivative = (error - pid->prev_error) / dt_sec;
  const float d = pid->kd * derivative;

  pid->prev_error = error;

  const float out = p + i + d;
  return clampf(out, pid->output_min, pid->output_max);
}

void PID_Reset(PIDController *pid)
{
  pid->integral = 0.0f;
  pid->prev_error = 0.0f;
}
