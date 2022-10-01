#ifndef __AUX_RTOS_H_
#define __AUX_RTOS_H_

#include "auxiliary/aux_hw/aux_hw.h"

#define TASK_ADC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_ADC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_PROC_STACK_SIZE (1024 * 10 / sizeof(portSTACK_TYPE))
#define TASK_PROC_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TASK_BLUETOOTH_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_BLUETOOTH_STACK_PRIORITY (tskIDLE_PRIORITY)

#define TIMEOUT 1000

xTaskHandle xTask_bte_send_handle, xTask_bte_recieve_handle, xTask_handle_recieve_data_handle, xTask_bte_handshake_handle, xTask_hw_onoff_handle;

#define EOP 'Q'

// rtos init
void init_rtos(void);

// rtos functions
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName);
void vApplicationIdleHook(void);
void vApplicationTickHook(void);
void vApplicationMallocFailedHook(void);
void xPortSysTickHandler(void);

// tasks
void task_bluetooth_send(void);
void task_bluetooth_receive(void);
void task_handle_response(void);
void task_handshake(void);
void task_on_off(void);

#endif