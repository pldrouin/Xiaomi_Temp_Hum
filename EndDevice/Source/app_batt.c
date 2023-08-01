/*
 * app_batt.c
 *
 *  Created on: Oct 25, 2020
 *      Author: Pierre-Luc Drouin
 */

#include "app_batt.h"

#ifdef DEBUG_SI
    #define TRACE_SI      TRUE
#else
    #define TRACE_SI      FALSE
#endif

uint16_t batt_read_voltage()
{
	/* General ADC initialisation */
	vAHI_ApConfigure(E_AHI_AP_REGULATOR_ENABLE, E_AHI_AP_INT_DISABLE, E_AHI_AP_SAMPLE_2, E_AHI_AP_CLOCKDIV_500KHZ, E_AHI_AP_INTREF);
	
	/* Wait for ADC to power up */
	while (!bAHI_APRegulatorEnabled());
	vAHI_AdcEnable(E_AHI_ADC_SINGLE_SHOT, E_AHI_AP_INPUT_RANGE_2, E_AHI_ADC_SRC_VOLT);
	
	/* Start ADC sampling  */
	vAHI_AdcStartSample();
	while(bAHI_AdcPoll());

	uint32_t adcread=u16AHI_AdcRead();

	vAHI_AdcDisable();
	vAHI_ApConfigure(E_AHI_AP_REGULATOR_DISABLE, E_AHI_AP_INT_DISABLE, E_AHI_AP_SAMPLE_2, E_AHI_AP_CLOCKDIV_500KHZ, E_AHI_AP_INTREF);
	return (uint16_t)((adcread * 7410)>>11);
}
