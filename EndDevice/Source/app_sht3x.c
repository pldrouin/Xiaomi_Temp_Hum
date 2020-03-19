/*
 * app_sht3x.c
 *
 *  Created on: Nov 28, 2019
 *      Author: root
 */

#include "app_sht3x.h"

#ifdef DEBUG_SI
    #define TRACE_SI      TRUE
#else
    #define TRACE_SI      FALSE
#endif

#define SHT3X_ADDR	(0x44)

uint16_t sht3x_lasthumtemp=0;

PRIVATE bool_t sht3x_active=FALSE;

static const uint8_t crc8_table[] = {
  0, 0x31, 0x62, 0x53, 0xC4, 0xF5, 0xA6, 0x97, 0xB9,
  0x88, 0xDB, 0xEA, 0x7D, 0x4C, 0x1F, 0x2E, 0x43, 0x72,
  0x21, 0x10, 0x87, 0xB6, 0xE5, 0xD4, 0xFA, 0xCB, 0x98,
  0xA9, 0x3E, 0xF, 0x5C, 0x6D, 0x86, 0xB7, 0xE4, 0xD5,
  0x42, 0x73, 0x20, 0x11, 0x3F, 0xE, 0x5D, 0x6C, 0xFB,
  0xCA, 0x99, 0xA8, 0xC5, 0xF4, 0xA7, 0x96, 1, 0x30,
  0x63, 0x52, 0x7C, 0x4D, 0x1E, 0x2F, 0xB8, 0x89, 0xDA,
  0xEB, 0x3D, 0xC, 0x5F, 0x6E, 0xF9, 0xC8, 0x9B, 0xAA,
  0x84, 0xB5, 0xE6, 0xD7, 0x40, 0x71, 0x22, 0x13, 0x7E,
  0x4F, 0x1C, 0x2D, 0xBA, 0x8B, 0xD8, 0xE9, 0xC7, 0xF6,
  0xA5, 0x94, 3, 0x32, 0x61, 0x50, 0xBB, 0x8A, 0xD9,
  0xE8, 0x7F, 0x4E, 0x1D, 0x2C, 2, 0x33, 0x60, 0x51,
  0xC6, 0xF7, 0xA4, 0x95, 0xF8, 0xC9, 0x9A, 0xAB, 0x3C,
  0xD, 0x5E, 0x6F, 0x41, 0x70, 0x23, 0x12, 0x85, 0xB4,
  0xE7, 0xD6, 0x7A, 0x4B, 0x18, 0x29, 0xBE, 0x8F, 0xDC,
  0xED, 0xC3, 0xF2, 0xA1, 0x90, 7, 0x36, 0x65, 0x54,
  0x39, 8, 0x5B, 0x6A, 0xFD, 0xCC, 0x9F, 0xAE, 0x80,
  0xB1, 0xE2, 0xD3, 0x44, 0x75, 0x26, 0x17, 0xFC, 0xCD,
  0x9E, 0xAF, 0x38, 9, 0x5A, 0x6B, 0x45, 0x74, 0x27,
  0x16, 0x81, 0xB0, 0xE3, 0xD2, 0xBF, 0x8E, 0xDD, 0xEC,
  0x7B, 0x4A, 0x19, 0x28, 6, 0x37, 0x64, 0x55, 0xC2,
  0xF3, 0xA0, 0x91, 0x47, 0x76, 0x25, 0x14, 0x83, 0xB2,
  0xE1, 0xD0, 0xFE, 0xCF, 0x9C, 0xAD, 0x3A, 0xB, 0x58,
  0x69, 4, 0x35, 0x66, 0x57, 0xC0, 0xF1, 0xA2, 0x93,
  0xBD, 0x8C, 0xDF, 0xEE, 0x79, 0x48, 0x1B, 0x2A, 0xC1,
  0xF0, 0xA3, 0x92, 5, 0x34, 0x67, 0x56, 0x78, 0x49,
  0x1A, 0x2B, 0xBC, 0x8D, 0xDE, 0xEF, 0x82, 0xB3, 0xE0,
  0xD1, 0x46, 0x77, 0x24, 0x15, 0x3B, 0xA, 0x59, 0x68,
  0xFF, 0xCE, 0x9D, 0xAC
};

void sht3x_initialise()
{
	/* Set DIO pin connected to SHT3X VDD pin as an output */
	vAHI_DioSetDirection(0, 1<<DIO_SHT_VDD);

	/* Set SHT ALERT DIO line to input */
	vAHI_DioSetDirection(1<<DIO_SHT_ALERT, 0);

	/* Turn off pull-up for SHT ALERT DIO line */
	vAHI_DioSetPullup(0, 1<<DIO_SHT_ALERT);

	/* Set the edge detection for rising edge */
	//vAHI_DioInterruptEdge(1<<DIO_SHT_ALERT, 0);

	/* Enable interrupt to occur on selected edge */
	//vAHI_DioInterruptEnable(1<<DIO_SHT_ALERT, 0);
}

bool_t sht3x_i2c_initialise()
{
	if(!sht3x_active) {
		DBG_vPrintf(TRACE_SI, "Initialising SHT3X\n");
		vAHI_DioSetOutput(1<<DIO_SHT_VDD,0);
		vAHI_SiMasterConfigure(TRUE,FALSE,63);

		while(sht3x_send_command(0x20,0x32,FALSE)){}
		sht3x_active = TRUE;
		return TRUE;

	} else return FALSE;
}

bool_t sht3x_i2c_enable()
{
	if(sht3x_active) {
		DBG_vPrintf(TRACE_SI, "Enabling SHT3X\n");
		vAHI_SiMasterConfigure(TRUE,FALSE,63);

		while(sht3x_send_command(0x20,0x32,FALSE)){}
		return TRUE;

	} else return FALSE;
}

bool_t sht3x_i2c_disable()
{
	if(sht3x_active) {
		DBG_vPrintf(TRACE_SI, "Disabling SHT3X\n");

		while(sht3x_i2c_clear_alerts());
		vAHI_SiMasterDisable();
		return TRUE;

	} else return FALSE;
}

bool_t sht3x_i2c_turnoff()
{
	if(sht3x_active) {
		DBG_vPrintf(TRACE_SI, "Turning off SHT3X\n");
		vAHI_SiMasterDisable();
		vAHI_DioSetOutput(0,1<<DIO_SHT_VDD);
		sht3x_active = FALSE;
		return TRUE;

	} else return FALSE;
}

void crc8_update(uint8_t* crc, const uint8_t byte)
{
	*crc = crc8_table[*crc ^ byte];
}

int sht3x_transfer_wait(bool_t stop_on_nack)
{
	//Wait for the transfer to be completed
	while(bAHI_SiMasterPollTransferInProgress()) {
		//DBG_vPrintf(TRACE_SI, "Transfer in progress...\n");
	}

	//Check for NACK
	if(bAHI_SiMasterCheckRxNack()) {
		bAHI_SiMasterSetCmdReg(FALSE, stop_on_nack, FALSE, FALSE, FALSE, FALSE);
		DBG_vPrintf(TRACE_SI, "Nack received\n");
		return -1;
	}

	//Check for loss of arbitration
	if(bAHI_SiMasterPollArbitrationLost()){
		bAHI_SiMasterSetCmdReg(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE);
		DBG_vPrintf(TRACE_SI, "Master lost arbitration\n");
		return -1;
	}

	return 0;
}

bool_t sht3x_read_value(uint16_t* value, bool_t last)
{
	//DBG_vPrintf(TRACE_SI, "Reading MSB\n");

	do {
		bAHI_SiMasterSetCmdReg(FALSE, FALSE, TRUE, FALSE, FALSE, FALSE);
	} while(sht3x_transfer_wait(FALSE));

	uint8_t byte=u8AHI_SiMasterReadData8();
	uint8_t crc;
	*value = byte<<8;

	crc8_init(&crc);
	crc8_update(&crc, byte);

	//DBG_vPrintf(TRACE_SI, "Reading LSB\n");

	do {
		bAHI_SiMasterSetCmdReg(FALSE, FALSE, TRUE, FALSE, FALSE, FALSE);
	} while(sht3x_transfer_wait(FALSE));

	byte=u8AHI_SiMasterReadData8();
	*value|=byte;
	crc8_update(&crc, byte);

	do {
		bAHI_SiMasterSetCmdReg(FALSE, last, TRUE, FALSE, last, FALSE);
	} while(sht3x_transfer_wait(FALSE));

	byte=u8AHI_SiMasterReadData8();

	if(byte==crc) {
		DBG_vPrintf(TRACE_SI, "CRC 0x%02x matches\n",crc);
		return TRUE;
	}
	DBG_vPrintf(TRACE_SI, "CRC 0x%02x does not match expected 0x%02x\n",byte,crc);
	return FALSE;
}

int sht3x_send_command(uint8_t msb, uint8_t lsb, bool_t stopafterlsb)
{
	DBG_vPrintf(TRACE_SI, "Sending command 0x%02x%02x\n",msb,lsb);
	vAHI_SiMasterWriteSlaveAddr(SHT3X_ADDR, FALSE);
	//Send the Start command, followed by the 7-bit address and the write command
	bAHI_SiMasterSetCmdReg(TRUE, FALSE, FALSE, TRUE, FALSE, FALSE);

	if(!sht3x_transfer_wait(FALSE)) {
		//Write command MSB to buffer
		//DBG_vPrintf(TRACE_SI, "Writing MSB\n");
		vAHI_SiMasterWriteData8(msb);
		bAHI_SiMasterSetCmdReg(FALSE, FALSE, FALSE, TRUE, FALSE, FALSE);

		if(!sht3x_transfer_wait(FALSE)) {
			//Write command LSB to buffer
			//DBG_vPrintf(TRACE_SI, "Writing LSB\n");
			vAHI_SiMasterWriteData8(lsb);
			bAHI_SiMasterSetCmdReg(FALSE, stopafterlsb, FALSE, TRUE, FALSE, FALSE);

			if(!sht3x_transfer_wait(FALSE)) return 0;
		}
	}
	return -1;
}

int sht3x_write_alert_limit(uint8_t lsb, uint16_t humtemp)
{
	uint8_t byte;
	uint8_t crc;
	DBG_vPrintf(TRACE_SI, "Write alert with reduced value 0x%04x\n",humtemp);

	if(!sht3x_send_command(0x61,lsb,FALSE)) {
		byte=(uint8_t)(humtemp>>8);
		vAHI_SiMasterWriteData8(byte);
		crc8_init(&crc);
		crc8_update(&crc, byte);
		bAHI_SiMasterSetCmdReg(FALSE, FALSE, FALSE, TRUE, FALSE, FALSE);

		if(!sht3x_transfer_wait(FALSE)) {
			byte=(uint8_t)humtemp;
			vAHI_SiMasterWriteData8(byte);
			crc8_update(&crc, byte);
			bAHI_SiMasterSetCmdReg(FALSE, FALSE, FALSE, TRUE, FALSE, FALSE);

			if(!sht3x_transfer_wait(FALSE)) {
				vAHI_SiMasterWriteData8(crc);
				bAHI_SiMasterSetCmdReg(FALSE, TRUE, FALSE, TRUE, FALSE, FALSE);

				if(!sht3x_transfer_wait(FALSE)) return 0;
			}
		}
	}
	return -1;
}

int sht3x_read_single_value(uint8_t msb, uint8_t lsb, uint16_t* value)
{
	if(!sht3x_send_command(msb,lsb,FALSE)) {

		do {
			DBG_vPrintf(TRACE_SI, "Adressing slave to read data\n");
			vAHI_SiMasterWriteSlaveAddr(SHT3X_ADDR, TRUE);
			bAHI_SiMasterSetCmdReg(TRUE, FALSE, FALSE, TRUE, FALSE, FALSE);
		} while(sht3x_transfer_wait(TRUE));

		DBG_vPrintf(TRACE_SI, "Ready to read data\n");

		if(sht3x_read_value(value, TRUE)) return 0;
	}
	return -1;
}


int sht3x_read_two_values(uint8_t msb, uint8_t lsb, uint16_t* value0, uint16_t* value1)
{
	if(!sht3x_send_command(msb,lsb,FALSE)) {

		do {
			DBG_vPrintf(TRACE_SI, "Adressing slave to read data\n");
			vAHI_SiMasterWriteSlaveAddr(SHT3X_ADDR, TRUE);
			bAHI_SiMasterSetCmdReg(TRUE, FALSE, FALSE, TRUE, FALSE, FALSE);
		} while(sht3x_transfer_wait(TRUE));

		DBG_vPrintf(TRACE_SI, "Ready to read data\n");

		if(sht3x_read_value(value0, FALSE) && sht3x_read_value(value1, TRUE)) return 0;
	}
	return -1;
}

int sht3x_get_measurements(uint16_t* temp, uint16_t* hum)
{
	if(!sht3x_read_two_values(0xE0,0x00,temp,hum)) {
		uint16_t humtemp=sht3x_humtemp_reduced_value(*hum, *temp);
		DBG_vPrintf(TRACE_SI, "Read values are 0x%04x 0x%04x, reduced value is 0x%04x\n",*temp,*hum,humtemp);


		if(humtemp==sht3x_lasthumtemp) return 0;

		if(!sht3x_send_command(0x30,0x93,TRUE)) {

			//while(sht3x_i2c_clear_alerts()){}

			if(!sht3x_write_alert_limit(0x16, humtemp) &&  //High clear
					!sht3x_write_alert_limit(0x0B, humtemp) &&   //Low clear
					!sht3x_write_alert_limit(0x1D, sht3x_humtemp_reduced_value(*hum+2048, *temp+160)) && //Minimum for hum is 512, temp is 128. Add 1/4*LSB for average alarm with 0.75*LSB delta, from 0.25*LSB delta to 1.25*LSB delta.
					!sht3x_write_alert_limit(0x00, sht3x_humtemp_reduced_value(*hum-2048, *temp-160))) { //Remove 1.25*LSB for average alarm with 0.75*LSB delta, from 0.25*LSB delta to 1.25*LSB delta.

				while(sht3x_send_command(0x20,0x32,FALSE)){}

				/*uint16_t hs, hc, ls, lc;
				if(!sht3x_read_alert_high_set(&hs))	DBG_vPrintf(TRUE, "Alert high set is 0x%04x\n",hs);
				if(!sht3x_read_alert_high_clear(&hc))	DBG_vPrintf(TRUE, "Alert high clear is 0x%04x\n",hc);
				if(!sht3x_read_alert_low_clear(&ls))	DBG_vPrintf(TRUE, "Alert low clear is 0x%04x\n",ls);
				if(!sht3x_read_alert_low_set(&lc))		DBG_vPrintf(TRUE, "Alert low set is 0x%04x\n",lc);*/
				sht3x_lasthumtemp=humtemp;
				return 1;
			}
		}

	}

	//while(sht3x_send_command(0x20,0x32,FALSE)){};
	return -1;
}
