#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H
#define configUSE_PREEMPTION 1
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configTICK_RATE_HZ 1000
#define configMAX_PRIORITIES 10
#define configMINIMAL_STACK_SIZE 4096
#define configTOTAL_HEAP_SIZE (4*1024*1024)
#define configMAX_TASK_NAME_LEN 16
#define configUSE_16_BIT_TICKS 0
#define configUSE_MUTEXES 1
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY 5
#define configTIMER_QUEUE_LENGTH 10
#define configTIMER_TASK_STACK_DEPTH 4096
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configCHECK_FOR_STACK_OVERFLOW 0
#define configUSE_TRACE_FACILITY 0
#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_vTaskDelete 1
#define configASSERT(x) do { if (!(x)) { for(;;); } } while(0)
#endif
