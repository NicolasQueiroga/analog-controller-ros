#ifndef __AUX_TIME_H_
#define __AUX_TIME_H_

#include "auxiliary/aux_hw/aux_hw.h"

typedef struct
{
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} Calendar;

// getters & setters
_Bool get_tc0_flag(void);
void set_tc0_flag(_Bool val);

_Bool get_tc1_flag(void);
void set_tc1_flag(_Bool val);

_Bool get_rtc_second_flag(void);
void set_rtc_second_flag(_Bool val);

_Bool get_rtc_alarm_flag(void);
void set_rtc_alarm_flag(_Bool val);

// inits
void TC_init(Tc *TC, int ID_TC, int TC_CHANNEL, int freq);
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
void RTC_init(Rtc *rtc, uint32_t id_rtc, Calendar t, uint32_t irq_type);

// handlers
void TC0_Handler(void);
void TC1_Handler(void);
void TC2_Handler(void);
void TC3_Handler(void);
void TC4_Handler(void);
void TC5_Handler(void);
void RTT_Handler(void);
void RTC_Handler(void);

#endif