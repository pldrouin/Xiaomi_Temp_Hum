/*
 * app_led.c
 *
 *  Created on: Jan 29, 2020
 *      Author: Pierre-Luc Drouin
 */

#include "app_led.h"

#ifdef DEBUG_LED
    #define TRACE_LED      TRUE
#else
    #define TRACE_LED      FALSE
#endif

PRIVATE const uint16_t led_connect_seq[]={100, 250, 100, 250, 100, 0};
PRIVATE const uint16_t led_pulse_seq[]={100, 0};
PRIVATE const uint16_t led_device_reset_seq[]={1000, 0};

uint8 u8TimerLED;
tsLedStruct tsLed;

PUBLIC void led_initialise()
{
	/* Turn on pull-ups for LED DIO line */
	vAHI_DioSetPullup(DIO_LED_PIN, 0);

	/* Turn off the LED */
	vAHI_DioSetOutput(DIO_LED_PIN,0);

	/* Set LED DIO line to output */
	vAHI_DioSetDirection(0, DIO_LED_PIN);
}

PUBLIC void APP_cbTimerLED(void *pvParam)
{
	tsLedStruct* ptsled=(tsLedStruct*)pvParam;
	uint16_t const* cseq;
	bool_t newstate;

	if(*(cseq=++(ptsled->seq))) {
		newstate=(ptsled->state=!ptsled->state);
		vAHI_DioSetOutput((!newstate)*DIO_LED_PIN,newstate*DIO_LED_PIN);
		ZTIMER_eStart(u8TimerLED, ZTIMER_TIME_MSEC(*cseq));

	} else vAHI_DioSetOutput(DIO_LED_PIN,0);
}

PUBLIC void led_connect()
{
    ZTIMER_eStop(u8TimerLED);
    tsLed.state=TRUE;
    tsLed.seq=led_connect_seq;
    vAHI_DioSetOutput(0,DIO_LED_PIN);
    ZTIMER_eStart(u8TimerLED, ZTIMER_TIME_MSEC(*tsLed.seq));
}

PUBLIC void led_pulse()
{
    ZTIMER_eStop(u8TimerLED);
    tsLed.state=TRUE;
    tsLed.seq=led_pulse_seq;
    vAHI_DioSetOutput(0,DIO_LED_PIN);
    ZTIMER_eStart(u8TimerLED, ZTIMER_TIME_MSEC(*tsLed.seq));
}


PUBLIC void led_device_reset()
{
    ZTIMER_eStop(u8TimerLED);
    tsLed.state=TRUE;
    tsLed.seq=led_device_reset_seq;
    vAHI_DioSetOutput(0,DIO_LED_PIN);
    ZTIMER_eStart(u8TimerLED, ZTIMER_TIME_MSEC(*tsLed.seq));
}
