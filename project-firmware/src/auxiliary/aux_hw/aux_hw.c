#include "aux_hw.h"

// globals
volatile QueueHandle_t xQueueBTSendData, xQueueBTReceiveData, xQueueONOFF;


// getters & setters
QueueHandle_t get_bt_send_data_queue(void)
{
	return xQueueBTSendData;
}
void set_bt_send_data_queue(QueueHandle_t val)
{
	xQueueBTSendData = val;
	if (xQueueBTSendData == NULL)
		printf("falha em criar a queue BT Send Data \n");
}

QueueHandle_t get_bt_receive_data_queue(void)
{
	return xQueueBTReceiveData;
}
void set_bt_receive_data_queue(QueueHandle_t val)
{
	xQueueBTReceiveData = val;
	if (xQueueBTReceiveData == NULL)
		printf("falha em criar a queue BT Receive Data \n");
}

QueueHandle_t get_on_off_queue(void)
{
	return xQueueONOFF;
}
void set_on_off_queue(QueueHandle_t val)
{
	xQueueONOFF = val;
	if (xQueueONOFF == NULL)
		printf("falha em criar a queue BT Receive Data \n");
}

// callbacks
void but_spd_up_callback(void)
{
	BTData btdata = {!pio_get(BUT_SPD_UP_PIO, PIO_INPUT, BUT_SPD_UP_PIO_IDX_MASK), 'U'};
	xQueueSendFromISR(xQueueBTSendData, &btdata, 0);
}
void but_spd_dn_callback(void)
{
	BTData btdata = {!pio_get(BUT_SPD_DN_PIO, PIO_INPUT, BUT_SPD_DN_PIO_IDX_MASK), 'D'};
	xQueueSendFromISR(xQueueBTSendData, &btdata, 0);
}
void but_180_callback(void)
{
	BTData btdata = {!pio_get(BUT_180_PIO, PIO_INPUT, BUT_180_PIO_IDX_MASK), 'R'};
	xQueueSendFromISR(xQueueBTSendData, &btdata, 0);
}
void but_power_callback(void)
{
	BTData btdata = {1, 'O'};
	xQueueSendFromISR(xQueueBTSendData, &btdata, 0);
	char msg = '1';
	xQueueSendFromISR(xQueueONOFF, &msg, 0);
}
void AFEC_an_callback(void)
{
	BTData adc_x = {afec_channel_get_value(AFEC_AN_X, AFEC_AN_X_CHANNEL), 'X'};
	BTData adc_y = {afec_channel_get_value(AFEC_AN_Y, AFEC_AN_Y_CHANNEL), 'Y'};

	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(get_bt_send_data_queue(), &adc_x, &xHigherPriorityTaskWoken);
	xQueueSendFromISR(get_bt_send_data_queue(), &adc_y, &xHigherPriorityTaskWoken);
}

// init basic
void init(void)
{
	// setup
	sysclk_init();
	board_init();
	configure_console();
	delay_init();

	// init clocks
	pmc_enable_periph_clk(ID_PIOA);
	pmc_enable_periph_clk(ID_PIOB);
	pmc_enable_periph_clk(ID_PIOC);
	pmc_enable_periph_clk(ID_PIOD);

	// btn SPD UP
	pio_configure(BUT_SPD_UP_PIO, PIO_INPUT, BUT_SPD_UP_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_SPD_UP_PIO, BUT_SPD_UP_PIO_IDX_MASK, 60);
	pio_handler_set(BUT_SPD_UP_PIO,
					BUT_SPD_UP_PIO_ID,
					BUT_SPD_UP_PIO_IDX_MASK,
					PIO_IT_EDGE,
					but_spd_up_callback);
	pio_enable_interrupt(BUT_SPD_UP_PIO, BUT_SPD_UP_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_SPD_UP_PIO);
	NVIC_EnableIRQ(BUT_SPD_UP_PIO_ID);
	NVIC_SetPriority(BUT_SPD_UP_PIO_ID, 4);

	// btn SPD DN
	pio_configure(BUT_SPD_DN_PIO, PIO_INPUT, BUT_SPD_DN_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_SPD_DN_PIO, BUT_SPD_DN_PIO_IDX_MASK, 60);
	pio_handler_set(BUT_SPD_DN_PIO,
					BUT_SPD_DN_PIO_ID,
					BUT_SPD_DN_PIO_IDX_MASK,
					PIO_IT_EDGE,
					but_spd_dn_callback);
	pio_enable_interrupt(BUT_SPD_DN_PIO, BUT_SPD_DN_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_SPD_DN_PIO);
	NVIC_EnableIRQ(BUT_SPD_DN_PIO_ID);
	NVIC_SetPriority(BUT_SPD_DN_PIO_ID, 4);

	// btn 180
	pio_configure(BUT_180_PIO, PIO_INPUT, BUT_180_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_180_PIO, BUT_180_PIO_IDX_MASK, 60);
	pio_handler_set(BUT_180_PIO,
					BUT_180_PIO_ID,
					BUT_180_PIO_IDX_MASK,
					PIO_IT_EDGE,
					but_180_callback);
	pio_enable_interrupt(BUT_180_PIO, BUT_180_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_180_PIO);
	NVIC_EnableIRQ(BUT_180_PIO_ID);
	NVIC_SetPriority(BUT_180_PIO_ID, 4);

	// btn power
	pio_configure(BUT_POWER_PIO, PIO_INPUT, BUT_POWER_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_POWER_PIO, BUT_POWER_PIO_IDX_MASK, 60);
	pio_handler_set(BUT_POWER_PIO,
					BUT_POWER_PIO_ID,
					BUT_POWER_PIO_IDX_MASK,
					PIO_IT_FALL_EDGE,
					but_power_callback);
	pio_enable_interrupt(BUT_POWER_PIO, BUT_POWER_PIO_IDX_MASK);
	pio_get_interrupt_status(BUT_POWER_PIO);
	NVIC_EnableIRQ(BUT_POWER_PIO_ID);
	NVIC_SetPriority(BUT_POWER_PIO_ID, 4);

	// led
	pio_configure(LED1_PIO, PIO_OUTPUT_0, LED1_PIO_IDX_MASK, PIO_DEFAULT);

	// led2
	pio_configure(LED2_PIO, PIO_OUTPUT_0, LED2_PIO_IDX_MASK, PIO_DEFAULT);

	// led3
	pio_configure(LED3_PIO, PIO_OUTPUT_0, LED3_PIO_IDX_MASK, PIO_DEFAULT);

	// led4
	pio_configure(LED4_PIO, PIO_OUTPUT_0, LED4_PIO_IDX_MASK, PIO_DEFAULT);

	// led POWER
	pio_configure(LED_POWER_PIO, PIO_OUTPUT_0, LED_POWER_PIO_IDX_MASK, PIO_DEFAULT);

	// turn off led's
	pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
	pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
	pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
	pio_clear(LED4_PIO, LED4_PIO_IDX_MASK);
	pio_clear(LED_POWER_PIO, LED_POWER_PIO_IDX_MASK);

	// AFEC an for JOYSTICK
	config_AFEC_an(AFEC_AN_X, AFEC_AN_X_ID, AFEC_AN_X_CHANNEL, AFEC_an_callback);
	config_AFEC_an(AFEC_AN_Y, AFEC_AN_Y_ID, AFEC_AN_Y_CHANNEL, AFEC_an_callback);
	TC_init(TC0, ID_TC1, 1, 10);
	tc_start(TC0, 1);

	TC_init(TC0, ID_TC2, 2, 5);
	TC_init(TC1, ID_TC3, 0, 5);
	TC_init(TC1, ID_TC4, 1, 5);
	TC_init(TC1, ID_TC5, 2, 5);
	
	// TC for power button:
	TC_init(TC0, ID_TC0, 0, 4);
}

// configure com
void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
#if (defined CONF_UART_CHAR_LENGTH)
		.charlength = CONF_UART_CHAR_LENGTH,
#endif
		.paritytype = CONF_UART_PARITY,
#if (defined CONF_UART_STOP_BITS)
		.stopbits = CONF_UART_STOP_BITS,
#endif
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

/* Specify that stdout should not be buffered. */
#if defined(__GNUC__)
	setbuf(stdout, NULL);
#else
/* Already the case in IAR's Normal DLIB default configuration: printf()
 * emits one character at a time.
 */
#endif
}

void config_AFEC_an(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback)
{
	/*************************************
	 * Ativa e configura AFEC
	 *************************************/
	/* Ativa AFEC - 0 */
	afec_enable(afec);

	/* struct de configuracao do AFEC */
	struct afec_config afec_cfg;

	/* Carrega parametros padrao */
	afec_get_config_defaults(&afec_cfg);

	/* Configura AFEC */
	afec_init(afec, &afec_cfg);

	/* Configura trigger por software */
	afec_set_trigger(afec, AFEC_TRIG_SW);

	/*** Configuracao espec?fica do canal AFEC ***/
	struct afec_ch_config afec_ch_cfg;
	afec_ch_get_config_defaults(&afec_ch_cfg);
	afec_ch_cfg.gain = AFEC_GAINVALUE_0;
	afec_ch_set_config(afec, afec_channel, &afec_ch_cfg);

	/*
	* Calibracao:
	* Because the internal ADC offset is 0x200, it should cancel it and shift
	down to 0.
	*/
	afec_channel_set_analog_offset(afec, afec_channel, 0x200);

	/***  Configura sensor de temperatura ***/
	struct afec_temp_sensor_config afec_temp_sensor_cfg;

	afec_temp_sensor_get_config_defaults(&afec_temp_sensor_cfg);
	afec_temp_sensor_set_config(afec, &afec_temp_sensor_cfg);

	/* configura IRQ */
	afec_set_callback(afec, afec_channel, callback, 1);
	NVIC_SetPriority(afec_id, 4);
	NVIC_EnableIRQ(afec_id);
}

// other
void pin_toggle(Pio *pio, uint32_t mask)
{
	pio_get_output_data_status(pio, mask) ? pio_clear(pio, mask) : pio_set(pio, mask);
}