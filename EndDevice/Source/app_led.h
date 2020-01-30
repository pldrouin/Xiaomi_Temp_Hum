/*
 * app_led.h
 *
 *  Created on: Jan 29, 2020
 *      Author: Pierre-Luc Drouin
 */

#ifndef APP_LED_H_
#define APP_LED_H_

#define DIO_LED_PIN (1<<9)

#include <stdlib.h>
#include "AppHardwareApi.h"
#include "ZTimer.h"
#include "dbg.h"

typedef struct {
	uint16_t const* seq;
	bool_t state;
} tsLedStruct;

extern uint8 u8TimerLED;
extern tsLedStruct tsLed;

PUBLIC void led_initialise();
PUBLIC void APP_cbTimerLED(void *pvParam);
PUBLIC void led_connect();
PUBLIC void led_pulse();
PUBLIC void led_device_reset();

#endif /* APP_SHT3X_H_ */
