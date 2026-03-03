#ifndef PWM_REG_DEBUG_H
#define PWM_REG_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Register-level PWM truth verification.
 * Call from main() BEFORE osKernelStart() for clean, RTOS-free output.
 */

/* Dump all RCC, TIM, GPIO registers relevant to motor PWM */
void PWM_RegDebug_DumpAll(void);

/* Drive Motor 1 at 40% for 2s while dumping live register values */
void PWM_RegDebug_DriveAndDump(void);

#ifdef __cplusplus
}
#endif

#endif /* PWM_REG_DEBUG_H */
