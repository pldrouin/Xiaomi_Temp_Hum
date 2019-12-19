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

int sht3x_transfer_wait()
{
	//Wait for the transfer to be completed
	while(bAHI_SiMasterPollTransferInProgress()) {
		DBG_vPrintf(TRACE_SI, "Transfer in progress...\n");
	}

	//Check for NACK
	if(bAHI_SiMasterCheckRxNack()) {
		bAHI_SiMasterSetCmdReg(FALSE, FALSE, FALSE, FALSE, FALSE, FALSE);
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

uint8_t sht3x_read_value(uint16_t* value)
{
	DBG_vPrintf(TRACE_SI, "Reading MSB\n");

	*value=u8AHI_SiMasterReadData8()<<8;
	DBG_vPrintf(TRACE_SI, "Reading LSB\n");
	*value|=u8AHI_SiMasterReadData8();
	return u8AHI_SiMasterReadData8();
}

int sht3x_get_measurements(uint16_t* temp, uint16_t* hum)
{
	vAHI_SiMasterWriteSlaveAddr(0x44, FALSE);
	//Send the Start command, followed by the 7-bit address and the write command
	bAHI_SiMasterSetCmdReg(TRUE, FALSE, FALSE, TRUE, FALSE, FALSE);

	if(!sht3x_transfer_wait()) {
		//Write command MSB to buffer
		DBG_vPrintf(TRACE_SI, "Writing MSB\n");
		vAHI_SiMasterWriteData8(0x24);
		bAHI_SiMasterSetCmdReg(FALSE, FALSE, FALSE, TRUE, FALSE, FALSE);

		if(!sht3x_transfer_wait()) {
			//Write command LSB to buffer
			DBG_vPrintf(TRACE_SI, "Writing LSB\n");
			vAHI_SiMasterWriteData8(0x06);
			bAHI_SiMasterSetCmdReg(FALSE, TRUE, FALSE, TRUE, FALSE, FALSE);

			if(!sht3x_transfer_wait()) {
				DBG_vPrintf(TRACE_SI, "Adressing slave to read data\n");

				do {
					vAHI_SiMasterWriteSlaveAddr(0x44, TRUE);
					bAHI_SiMasterSetCmdReg(TRUE, FALSE, TRUE, FALSE, TRUE, FALSE);
				} while(sht3x_transfer_wait());

				sht3x_read_value(temp);
				sht3x_read_value(hum);
				bAHI_SiMasterSetCmdReg(FALSE, TRUE, FALSE, FALSE, FALSE, FALSE);
			}
		}
	}
	return 0;
}
