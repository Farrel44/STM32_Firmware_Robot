/*
 * FreeRTOS configuration for stm_firmware_2026 (STM32F446 @ 180MHz)
 *
 * Notes:
 * - CMSIS-RTOS2 wrapper (CMSIS_RTOS_V2) requires several INCLUDE_* macros.
 * - This project uses heap_4 (portable/MemMang/heap_4.c).
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/* CMSIS-RTOS2 wrapper expects CMSIS_device_header to be defined. */
#ifndef CMSIS_device_header
#define CMSIS_device_header "stm32f4xx.h"
#endif

#include <stdint.h>

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * NOTE: configCPU_CLOCK_HZ must match SystemCoreClock.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                    1
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configCPU_CLOCK_HZ                      ( ( unsigned long ) 180000000 )
#define configTICK_RATE_HZ                      ( ( TickType_t ) 1000 )
#define configMAX_PRIORITIES                    ( 56 )
#define configMINIMAL_STACK_SIZE                ( ( uint16_t ) 128 )
#define configTOTAL_HEAP_SIZE                   ( ( size_t ) ( 32 * 1024 ) )
#define configMAX_TASK_NAME_LEN                 ( 16 )
#define configUSE_16_BIT_TICKS                  0
#define configIDLE_SHOULD_YIELD                 1

/* CMSIS-RTOS2 wrapper requirements/knobs. */
#define configUSE_OS2_THREAD_ENUMERATE          0

/* Avoid port-optimized task selection to satisfy freertos_os2.h checks. */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0

#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             1
#define configUSE_COUNTING_SEMAPHORES           1

#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               ( 3 )
#define configTIMER_QUEUE_LENGTH                ( 10 )
#define configTIMER_TASK_STACK_DEPTH            ( 256 )

#define configUSE_TASK_NOTIFICATIONS            1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1

/*-----------------------------------------------------------
 * Memory allocation.
 *----------------------------------------------------------*/

#define configSUPPORT_STATIC_ALLOCATION         0
#define configSUPPORT_DYNAMIC_ALLOCATION        1

/*-----------------------------------------------------------
 * Hook function related definitions.
 *----------------------------------------------------------*/

#define configUSE_MALLOC_FAILED_HOOK            1
#define configCHECK_FOR_STACK_OVERFLOW          2

/*-----------------------------------------------------------
 * Runtime and task stats.
 *----------------------------------------------------------*/

#define configGENERATE_RUN_TIME_STATS           0
#define configUSE_TRACE_FACILITY                0

/*-----------------------------------------------------------
 * Co-routine definitions.
 *----------------------------------------------------------*/

#define configUSE_CO_ROUTINES                   0
#define configMAX_CO_ROUTINE_PRIORITIES         ( 2 )

/*-----------------------------------------------------------
 * API function inclusion.
 * These are required by the CMSIS-RTOS2 wrapper.
 *----------------------------------------------------------*/

#define INCLUDE_vTaskPrioritySet                1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskDelete                     1
#define INCLUDE_vTaskSuspend                    1
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_vTaskDelayUntil                 1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xSemaphoreGetMutexHolder        1
#define INCLUDE_xTimerPendFunctionCall          1

/*-----------------------------------------------------------
 * Cortex-M specific definitions.
 *----------------------------------------------------------*/

#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS                         __NVIC_PRIO_BITS
#else
#define configPRIO_BITS                         4
#endif

/* The lowest interrupt priority that can be used in a call to a
 * FreeRTOS API function that ends in "FromISR".
 *
 * IMPORTANT: This must be set to a value that results in a valid priority
 * when shifted left by (8 - configPRIO_BITS).
 */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY   ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << ( 8 - configPRIO_BITS ) )

/*-----------------------------------------------------------
 * Assertions.
 *----------------------------------------------------------*/

#define configASSERT( x ) if( ( x ) == 0 ) { portDISABLE_INTERRUPTS(); for( ;; ); }

#ifdef __cplusplus
}
#endif

#endif /* FREERTOS_CONFIG_H */
