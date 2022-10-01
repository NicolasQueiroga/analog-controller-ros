#ifndef __AUX_HW_H_
#define __AUX_HW_H_

#include "asf.h"
#include "conf_board.h"
#include "auxiliary/aux_time/aux_time.h"
#include "auxiliary/aux_rtos/aux_rtos.h"

#define LED_PIO PIOC
#define LED_PIO_ID ID_PIOC
#define LED_PIO_IDX 8
#define LED_IDX_MASK (1 << LED_PIO_IDX)

#define BUT_PIO PIOA
#define BUT_PIO_ID ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

// ui
#define LED1_PIO PIOA
#define LED1_PIO_ID ID_PIOA
#define LED1_PIO_IDX 6
#define LED1_PIO_IDX_MASK (1u << LED1_PIO_IDX)

#define LED2_PIO PIOD
#define LED2_PIO_ID ID_PIOD
#define LED2_PIO_IDX 11
#define LED2_PIO_IDX_MASK (1u << LED2_PIO_IDX)

#define LED3_PIO PIOC
#define LED3_PIO_ID ID_PIOC
#define LED3_PIO_IDX 19
#define LED3_PIO_IDX_MASK (1u << LED3_PIO_IDX)

#define LED4_PIO PIOD
#define LED4_PIO_ID ID_PIOD
#define LED4_PIO_IDX 26
#define LED4_PIO_IDX_MASK (1u << LED4_PIO_IDX)

#define BUT_180_PIO PIOA
#define BUT_180_PIO_ID ID_PIOA
#define BUT_180_PIO_IDX 2
#define BUT_180_PIO_IDX_MASK (1u << BUT_180_PIO_IDX)

#define BUT_POWER_PIO PIOA
#define BUT_POWER_PIO_ID ID_PIOA
#define BUT_POWER_PIO_IDX 0
#define BUT_POWER_PIO_IDX_MASK (1u << BUT_POWER_PIO_IDX)

#define LED_POWER_PIO PIOC
#define LED_POWER_PIO_ID ID_PIOC
#define LED_POWER_PIO_IDX 30
#define LED_POWER_PIO_IDX_MASK (1u << LED_POWER_PIO_IDX)

#define BUT_SPD_UP_PIO PIOB
#define BUT_SPD_UP_PIO_ID ID_PIOB
#define BUT_SPD_UP_PIO_IDX 2
#define BUT_SPD_UP_PIO_IDX_MASK (1u << BUT_SPD_UP_PIO_IDX)

#define BUT_SPD_DN_PIO PIOB
#define BUT_SPD_DN_PIO_ID ID_PIOB
#define BUT_SPD_DN_PIO_IDX 3
#define BUT_SPD_DN_PIO_IDX_MASK (1u << BUT_SPD_DN_PIO_IDX)

#define AFEC_AN_X AFEC1
#define AFEC_AN_X_ID ID_AFEC1
#define AFEC_AN_X_CHANNEL 1

#define AFEC_AN_Y AFEC0
#define AFEC_AN_Y_ID ID_AFEC0
#define AFEC_AN_Y_CHANNEL 0

//#define DEBUG_SERIAL
#ifdef DEBUG_SERIAL
#define USART_COM USART1
#define USART_COM_ID ID_USART1
#else
#define USART_COM USART0
#define USART_COM_ID ID_USART0
#endif

typedef struct
{
	int data;
	char id;
} BTData;

// getters & setters
QueueHandle_t get_bt_send_data_queue(void);
void set_bt_send_data_queue(QueueHandle_t val);

QueueHandle_t get_bt_receive_data_queue(void);
void set_bt_receive_data_queue(QueueHandle_t val);

QueueHandle_t get_on_off_queue(void);
void set_on_off_queue(QueueHandle_t val);


// callbacks
void but_spd_up_callback(void);
void but_spd_dn_callback(void);
void but_180_callback(void);
void but_power_callback(void);

// init basic
void init(void);

// configure com
void configure_console(void);
void config_AFEC_an(Afec *afec, uint32_t afec_id, uint32_t afec_channel, afec_callback_t callback);
void AFEC_an_callback(void);

// other
void pin_toggle(Pio *pio, uint32_t mask);

#endif