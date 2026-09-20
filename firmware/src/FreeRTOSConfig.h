#pragma once
#include <stdint.h>

#define configUSE_PREEMPTION            1
#define configUSE_TIME_SLICING          1
#define configTICK_RATE_HZ              1000

#define configMAX_PRIORITIES            5
#define configMINIMAL_STACK_SIZE        256
#define configTOTAL_HEAP_SIZE           (64 * 1024)

#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION  0

#define configUSE_MUTEXES               1
#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0

#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS

#define configUSE_EVENT_GROUPS 1

#define configUSE_TIMERS 1
#define INCLUDE_xTimerPendFunctionCall 1

#define configTIMER_TASK_PRIORITY 2
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH 256

#define INCLUDE_vTaskDelay 1
