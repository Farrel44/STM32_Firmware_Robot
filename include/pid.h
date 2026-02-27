#ifndef PID_H
#define PID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  float kp;
  float ki;
  float kd;

  float integral;
  float prev_error;

  float output_min;
  float output_max;
  float integral_max;
} PIDController;

void PID_Init(PIDController *pid, float kp, float ki, float kd);
void PID_SetOutputLimits(PIDController *pid, float min, float max);
void PID_SetIntegralLimits(PIDController *pid, float limit);
void PID_SetTuning(PIDController *pid, float kp, float ki, float kd);

float PID_Compute(PIDController *pid, float input, float setpoint, float dt_sec);
void PID_Reset(PIDController *pid);

#ifdef __cplusplus
}
#endif

#endif /* PID_H */
