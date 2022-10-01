#include "auxiliary/aux_time/aux_time.h"
#include "auxiliary/aux_bt/aux_bt.h"
#include "aux_rtos.h"

/*==================================================================================================================*/
/*                                                     ROS INIT                                                     */
/*==================================================================================================================*/
void init_rtos(void)
{
	set_bt_send_data_queue(xQueueCreate(10, sizeof(BTData)));
	set_bt_receive_data_queue(xQueueCreate(10, sizeof(BTData)));
	set_on_off_queue(xQueueCreate(10, sizeof(char)));
	
  	if (xTaskCreate(task_bluetooth_send, "BT_SEND", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, &xTask_bte_send_handle) != pdPASS)
    	printf("Failed to create test BT Send task\r\n");

  	if (xTaskCreate(task_bluetooth_receive, "BT_RECEIVE", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, &xTask_bte_recieve_handle) != pdPASS)
    	printf("Failed to create test BT Receive task\r\n");
		
	if (xTaskCreate(task_handle_response, "HANDLE_RESPONSE", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, &xTask_handle_recieve_data_handle) != pdPASS)
		printf("Failed to create test handle response task\r\n");
		
  	if (xTaskCreate(task_handshake, "HANDSHAKE", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, &xTask_bte_handshake_handle) != pdPASS)
		printf("Failed to create task Handshake\r\n");
		
	if (xTaskCreate(task_on_off, "ON_OFF", TASK_BLUETOOTH_STACK_SIZE, NULL, TASK_BLUETOOTH_STACK_PRIORITY, &xTask_hw_onoff_handle) != pdPASS)
		printf("Failed to create task on off\r\n");
		
	vTaskSuspend(xTask_bte_send_handle);
	vTaskSuspend(xTask_bte_recieve_handle);
	vTaskSuspend(xTask_handle_recieve_data_handle);
	vTaskSuspend(xTask_hw_onoff_handle);
	
  	vTaskStartScheduler();
}

/*==================================================================================================================*/
/*                                                  ROS FUNCTIONS                                                   */
/*==================================================================================================================*/
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
	printf("stack overflow %u %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;)
	{
	}
}
void vApplicationIdleHook(void) { pmc_sleep(SAM_PM_SMODE_SLEEP_WFI); }
void vApplicationTickHook(void) {}
void vApplicationMallocFailedHook(void) { configASSERT((volatile void *)NULL); }
	
void data_convert(char *D0, char *D1, int data)
{
	*D0 = data & 0x00ff;
	*D1 = data << 8;
}

/*==================================================================================================================*/
/*                                                      TAKS                                                        */
/*==================================================================================================================*/

/* -------------------------------------------------- BTE SEND ---------------------------------------------------- */

void task_bluetooth_send(void)
{
	char on = 1;
	BTData msg;

	while (1)
	{
		if (xQueueReceive(get_bt_send_data_queue(), &(msg), 1000))
		{
			if (msg.id == 'O'){
				on = on == 1 ? 0 : 1;
				if(on == 0){
					vTaskResume(xTask_hw_onoff_handle);
				}
			} else {
				char D0, D1;
				data_convert(&D0, &D1, msg.data);

				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
				usart_write(USART_COM, msg.id);

				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
				usart_write(USART_COM, msg.data >> 8);

				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
				usart_write(USART_COM, msg.data & 0x00ff);

				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
				usart_write(USART_COM, EOP);
			}
		}
	}
}


/* ------------------------------------------------- BTE RECIEVE -------------------------------------------------- */

void task_bluetooth_receive(void)
{	
	int i = 3;
	char buffer[7];

	char c;
	
	char is_trash_rx = 0;

	while (1)
	{
		char c;
		
		char buffer[7];
		int t = 0;
		while(is_trash_rx == 1){
			if (!usart_read(USART_COM, &c))
			{
				if(c == 'Q'){
					is_trash_rx = 0;
				}
			}
		}
		if (!usart_read(USART_COM, &c))
		{
			char val = c;
			buffer[i] = val;
			i++;
			t = 0;
			if (i == 7)
			{
				if(buffer[6] == 'Q'){
					printf("Received info: %c, %c, %c\n", buffer[3], buffer[4], buffer[5]);
					BTData msg = {buffer[5], buffer[3]};
					xQueueSend(get_bt_receive_data_queue(), &msg, 0);
				} else {
					printf("Received Trash\n");
					is_trash_rx = 1;
				}
				i = 3;
			}
		}
		vTaskDelay(1);
	}
}

/* ----------------------------------------------- HANDLE RESPONSE ------------------------------------------------ */

void task_handle_response(void){	
	BTData msg;
	
	while(1)
	{
		if (xQueueReceive(get_bt_receive_data_queue(), &(msg), 1000))
		{
			if(msg.id == 'N'){
				if(msg.data == 1){
					printf("Recieved N 1\n");
					tc_start(TC0, 2);
				} else {
					printf("Recieved N 0\n");
					tc_stop(TC0, 2);
					pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				}
			} else if(msg.id == 'L'){
				if(msg.data == 1){
					printf("Recieved L 1\n");
					tc_start(TC1, 0);
				} else {
					printf("Recieved L 0\n");
					tc_stop(TC1, 0);
					pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				}
			} else if(msg.id == 'S'){
				if(msg.data == 1){
					printf("Recieved S 1\n");
					tc_start(TC1, 1);
				} else {
					printf("Recieved S 0\n");
					tc_stop(TC1, 1);
					pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				}
			} else if(msg.id == 'O'){
				if(msg.data == 1){
					printf("Recieved O 1\n");
					tc_start(TC1, 2);
				} else {
					printf("Recieved O 0\n");
					tc_stop(TC1, 2);
					pio_clear(LED4_PIO, LED4_PIO_IDX_MASK);
				}
			}
		}
	}
}

/* ------------------------------------------------- HANDSHAKE ---------------------------------------------------- */

void task_handshake(void)
{
	printf("Inicializando HC05\n");
	config_usart0();
	hc05_init();
	
	tc_start(TC0, 0);
	
	char connected = 0;
	
	int i = 3;
	char buffer[7];
	
	while(1)
	{
		printf("try to connect...\n");
		while (!usart_is_tx_ready(USART_COM))
		vTaskDelay(1 / portTICK_PERIOD_MS);
		usart_write(USART_COM, 'C');

		while (!usart_is_tx_ready(USART_COM))
		vTaskDelay(1 / portTICK_PERIOD_MS);
		usart_write(USART_COM, '0');

		while (!usart_is_tx_ready(USART_COM))
		vTaskDelay(1 / portTICK_PERIOD_MS);
		usart_write(USART_COM, '1');

		while (!usart_is_tx_ready(USART_COM))
		vTaskDelay(1 / portTICK_PERIOD_MS);
		usart_write(USART_COM, 'Q');
		
		int t = 0;
		char c;
		while(t < TIMEOUT){
			if (!usart_read(USART_COM, &c))
			{
				char val = c;
				buffer[i] = val;
				i++;
				t = 0;
				if (i == 7)
				{
					i = 3;
					if(buffer[3] == 'C' && buffer[5] == '1')
					{
						printf("CONNECTED\n");
						tc_stop(TC0, 0);
						tc_start(TC0, 1);
						pio_set(LED_POWER_PIO, LED_POWER_PIO_IDX_MASK);
						vTaskResume(xTask_bte_send_handle);
						vTaskResume(xTask_bte_recieve_handle);
						vTaskResume(xTask_handle_recieve_data_handle);
						vTaskSuspend(NULL);
					}
				}
			}
			vTaskDelay(1);
			t++;
		}
	}
}

/* --------------------------------------------------- ON OFF ----------------------------------------------------- */

void task_on_off(void){
	char msg = 0;
	char on = '1';
	while(1){
		if (xQueueReceive(get_on_off_queue(), &(msg), 10)){
			on = on == '1' ? '0' : '1';
			if(on == '1'){
				tc_start(TC0, 0);
				vTaskResume(xTask_bte_handshake_handle);
				vTaskSuspend(NULL);
			} else {
				vTaskSuspend(xTask_bte_send_handle);
				vTaskSuspend(xTask_bte_recieve_handle);
				vTaskSuspend(xTask_handle_recieve_data_handle);
				tc_stop(TC0, 1);
				tc_stop(TC0, 2);
				tc_stop(TC1, 0);
				tc_stop(TC1, 1);
				tc_stop(TC1, 2);
				pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				pio_clear(LED4_PIO, LED4_PIO_IDX_MASK);
				pio_clear(LED_POWER_PIO, LED_POWER_PIO_IDX_MASK);
				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
				usart_write(USART_COM, 'C');

				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
				usart_write(USART_COM, '0');

				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
				usart_write(USART_COM, '0');

				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
				usart_write(USART_COM, 'Q');
				
				while (!usart_is_tx_ready(USART_COM))
					vTaskDelay(1 / portTICK_PERIOD_MS);
			}
		}
		vTaskDelay(100); //SLEEP
	}
}