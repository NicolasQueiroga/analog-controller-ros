#ifndef __AUX_BT_H_
#define __AUX_BT_H_

#include "auxiliary/aux_hw/aux_hw.h"

uint32_t usart_puts(uint8_t *pstring);
void usart_put_string(Usart *usart, char *str);
int usart_get_string(Usart *usart, char *buffer, int bufferlen, uint timeout_ms);
void usart_send_command(Usart *usart, char buffer_rx[], int bufferlen, char buffer_tx[], int timeout);
void config_usart0(void);
int hc05_init(void);

#endif