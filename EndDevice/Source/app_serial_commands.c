/*****************************************************************************
 *
 * MODULE:             JN-AN-1217
 *
 * COMPONENT:          app_serial_commands.c
 *
 * DESCRIPTION:        Base Device Serial Commands: Coordinator application
 *
 ****************************************************************************
 *
 * This software is owned by NXP B.V. and/or its supplier and is protected
 * under applicable copyright laws. All rights are reserved. We grant You,
 * and any third parties, a license to use this software solely and
 * exclusively on NXP products [NXP Microcontrollers such as JN5168, JN5179].
 * You, and any third parties must reproduce the copyright and warranty notice
 * and any other legend of ownership on each copy or partial copy of the
 * software.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <stdlib.h>
#include "dbg.h"
#include "pdum_apl.h"
#include "pwrm.h"
#include "app_end_device_node.h"
#include "app_zcl_task.h"
#include "app_buttons.h"
#include "app_common.h"
#include "app_serial_commands.h"
#include "app_events.h"
#include "app_main.h"
#include "app_sht3x.h"
#include "ZQueue.h"
#include "ZTimer.h"

#define apduZDP &pdum_apduZDP
#define apduZCL &pdum_apduZCL

/* APDUs */
extern const struct pdum_tsAPdu_tag pdum_apduZDP;
extern const struct pdum_tsAPdu_tag pdum_apduZCL;

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#ifdef DEBUG_SERIAL
    #define TRACE_SERIAL      TRUE
#else
    #define TRACE_SERIAL      FALSE
#endif


/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/
#if BDB_JOIN_USES_INSTALL_CODE_KEY == TRUE
    #define COMMAND_BUF_SIZE   80
#else
    #define COMMAND_BUF_SIZE   80
#endif
typedef struct
{
    uint8  au8Buffer[COMMAND_BUF_SIZE];
    uint8  u8Pos;
}tsCommand;

#if (JENNIC_CHIP_FAMILY == JN517x)
#define stricmp strcasecmp
#endif

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/
PRIVATE void vProcessRxChar(uint8 u8Char);
PRIVATE void vProcessCommand(void);
PRIVATE void vPrintAPSTable(void);
#if (BDB_JOIN_USES_INSTALL_CODE_KEY == TRUE)
    PRIVATE bool_t bValidateHexStr(uint8 *HexStr);
    PRIVATE uint64 u64HexStrToValue(uint8 *HexStr);
    PRIVATE bool_t bValidateKeyStr(uint8 *KeyStr);
#endif
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/
PRIVATE tsCommand sCommand;
char * strings[] = {
        "START UP",
        "NFN START",
        "RUNNING"
        };
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/******************************************************************************
 * NAME: APP_taskAtSerial
 *
 * DESCRIPTION:
 *
 *
 * PARAMETERS:      Name            Usage
 *
 * RETURNS:
 * None
 ****************************************************************************/
PUBLIC void APP_taskAtSerial( void)
{
    uint8 u8RxByte;
    if (ZQ_bQueueReceive(&APP_msgSerialRx, &u8RxByte) == TRUE)
    {
        DBG_vPrintf(TRACE_SERIAL, "Rx Char %02x\n", u8RxByte);
        u8KeepAliveTime = KEEP_ALIVETIME;
        vProcessRxChar(u8RxByte);
    }
}

/******************************************************************************
 * NAME: vProcessRxChar
 *
 * DESCRIPTION:
 * Processes the received character
 *
 * PARAMETERS:      Name            Usage
 * uint8            u8Char          Character
 *
 * RETURNS:
 * None
 ****************************************************************************/
PRIVATE void vProcessRxChar(uint8 u8Char)
{

    if ((u8Char >= 'a' && u8Char <= 'z'))
    {
	u8Char -= ('a' - 'A');
    }
    if((u8Char == 0x08 || u8Char == 0x7f) && sCommand.u8Pos > 0)
    {
	--sCommand.u8Pos;
    }
    else
    {
	if ((sCommand.u8Pos < COMMAND_BUF_SIZE)  && (u8Char != 0x0d))
	{
	    sCommand.au8Buffer[sCommand.u8Pos++] = u8Char;
	}
	else if (sCommand.u8Pos >= COMMAND_BUF_SIZE)
	{
	    DBG_vPrintf(TRACE_SERIAL, "OverFlow\n");
	    memset(sCommand.au8Buffer, 0, COMMAND_BUF_SIZE);
	    sCommand.u8Pos = 0;
	}

	if (u8Char == 0x0d)
	{
	    vProcessCommand();
	}
    }
}

/******************************************************************************
 * NAME: vProcessCommand
 *
 * DESCRIPTION:
 * Processed the received command
 *
 * PARAMETERS:      Name            Usage
 *
 * RETURNS:
 * None
 ****************************************************************************/
PRIVATE void vProcessCommand(void)
{
    uint8 Delimiters[] = " ";
    uint8 *token = NULL;
    uint16 u16ClusterId;
    uint8 u8SrcEndpoint;
    uint16 u16DstAddr;
    uint64 u64DstIeeeAddr;
    uint8 u8DstEndpoint;

    DBG_vPrintf(TRACE_SERIAL, "Command string = '%s'\n", sCommand.au8Buffer);

    token = (uint8 *)strtok((char *)sCommand.au8Buffer, (char *)Delimiters);

    DBG_vPrintf(TRACE_SERIAL, "Command = %s\n", token);

    APP_tsEvent sButtonEvent;
    sButtonEvent.eType = APP_E_EVENT_NONE;

    if (0 == stricmp((char*)token, "steer")) //Used
    {
        DBG_vPrintf(TRACE_SERIAL, "Steer\n");
        BDB_eNsStartNwkSteering();
    }
    else if (0 == stricmp((char*)token, "bind")) //Used
        {
    		token = (uint8 *)strtok( NULL, (char *)Delimiters);
    		u16ClusterId=strtoul((char*)token,NULL,16);
    		token = (uint8 *)strtok( NULL, (char *)Delimiters);
    		u8SrcEndpoint=strtoul((char*)token,NULL,16);
    		token = (uint8 *)strtok( NULL, (char *)Delimiters);
    		u64DstIeeeAddr = strtoull((char*)token,NULL,16);
    		token = (uint8 *)strtok( NULL, (char *)Delimiters);
    		u8DstEndpoint=strtoul((char*)token,NULL,16);
            //u16DstAddr = ZPS_u16AplZdoLookupAddr(u64DstIeeeAddr);
    		token = (uint8 *)strtok( NULL, (char *)Delimiters);
    		u16DstAddr=strtoul((char*)token,NULL,16);

            if(u16DstAddr == ZPS_NWK_INVALID_NWK_ADDR)
	    {
		PDUM_thAPduInstance hAPduInst;

		hAPduInst =  PDUM_hAPduAllocateAPduInstance ( apduZDP );

		if (PDUM_INVALID_HANDLE != hAPduInst)
		{
		    ZPS_tsAplZdpNwkAddrReq    sAplZdpNwkAddrReq;
		    ZPS_tuAddress             uDstAddr;

		    /* always send to node of interest rather than a cache */

		    uDstAddr.u16Addr                =  0xFFFF;
		    sAplZdpNwkAddrReq.u64IeeeAddr   =  u64DstIeeeAddr;
		    sAplZdpNwkAddrReq.u8RequestType =  0x00;
		    sAplZdpNwkAddrReq.u8StartIndex  =  0;
		    uint8 pu8Seq=0;

		    u16DstAddr = ZPS_eAplZdpNwkAddrRequest ( hAPduInst,
			    uDstAddr,
			    FALSE,
			    &pu8Seq,
			    &sAplZdpNwkAddrReq );
		}

	    }

            DBG_vPrintf(TRUE, "Bind 0x%x 0x%x 0x%x 0x%llx 0x%x\n",u16ClusterId,u8SrcEndpoint,u16DstAddr,u64DstIeeeAddr,u8DstEndpoint);

            if(ZPS_eAplZdoBind(u16ClusterId,u8SrcEndpoint,u16DstAddr,u64DstIeeeAddr,u8DstEndpoint) != ZPS_E_SUCCESS)
            {
            	DBG_vPrintf(TRUE,"ZPS_eAplZdoBind failed!\n");
            } else {
            	DBG_vPrintf(TRUE,"ZPS_eAplZdoBind succeeded!\n");
            }
        }
    else if (0 == stricmp((char*)token, "factory")) //Used?
    {
        token = (uint8 *)strtok( NULL, (char *)Delimiters);
        if (0 == stricmp((char*)token, "reset"))
        {
            DBG_vPrintf(TRACE_SERIAL, "Factory reset\n");
            APP_vFactoryResetRecords();
            vAHI_SwReset();
        }

    }
    else if (0 == stricmp((char*)token, "soft")) //Used?
    {
        token = (uint8 *)strtok( NULL, (char *)Delimiters);
        if (0 == stricmp((char*)token, "reset"))
        {
            vAHI_SwReset();
        }
    }
    else if (0 == stricmp((char*)token, "print"))
    {
        DBG_vPrintf(TRACE_SERIAL, "Key Table:\n");
        vPrintAPSTable();
    }
#if BDB_JOIN_USES_INSTALL_CODE_KEY == TRUE
    else if (0 == stricmp((char*)token, "code"))
    {
        DBG_vPrintf(TRACE_SERIAL, "Code\n");
        token = (uint8 *)strtok( NULL, (char *)Delimiters);
        if ((token) && ( bValidateHexStr(token) ))
        {
            uint64 u64Addr;
            u64Addr = u64HexStrToValue( token);
            uint8 u8Status;
            DBG_vPrintf(TRACE_SERIAL, "u64Addr %016llx\n", u64Addr);

            token = (uint8 *)strtok( NULL, (char *)Delimiters);
            if (0 == stricmp((char*)token, "vector"))
            {
                uint8 au8TestVector[16] = {0x83,0xFE,0xD3,0x40,0x7A,0x93,0x97,0x23,0xA5,0xC6,0x39,0xB2,0x69,0x16,0xD5,0x05};
                u8Status = ZPS_eAplZdoAddReplaceInstallCodes( u64Addr, au8TestVector, 16, ZPS_APS_UNIQUE_LINK_KEY);
                DBG_vPrintf(TRACE_SERIAL, "Key Addred for %016llx Status %02x\n", u64Addr, u8Status);
                vPrintAPSTable();
            }
            else if (bValidateKeyStr(token))
            {
                uint8 Key[16];
                DBG_vPrintf(TRACE_SERIAL, "Adjusted String %s\n", token);
                uint8 val;
                int i;
                for (i=0; i<16; i++)
                {
                    val = (*token >= 'A')? *token - 'A' + 10: *token - '0';
                    Key[i] = 16  * val;
                    token++;
                    val = (*token >= 'A')? *token - 'A' + 10: *token - '0';
                    Key[i] += val;
                    DBG_vPrintf(TRACE_SERIAL, "%02x ", Key[i]);
                    token++;
                }
                DBG_vPrintf(TRACE_SERIAL, "\n");
                u8Status = ZPS_eAplZdoAddReplaceInstallCodes( u64Addr, Key, 16, ZPS_APS_UNIQUE_LINK_KEY);
                DBG_vPrintf(TRACE_SERIAL, "Key Addred for %016llx Status %02x\n", u64Addr, u8Status);
                vPrintAPSTable();


            } else { DBG_vPrintf(TRACE_SERIAL, "KEY VALIDATION FAILED\n"); }


        }

    }
#endif  /* define uses install codes */
    else if (0 == stricmp((char*)token, "togglesht"))
    {
    	   uint32_t oldval = u32AHI_DioReadInput() & (1<<DIO_SHT_VDD);
    	   uint32_t newval = (~oldval) & (1<<DIO_SHT_VDD);

    	   if(oldval) sht3x_i2c_turnoff();
    	   vAHI_DioSetOutput(newval,oldval);

    	   if(newval) sht3x_i2c_initialise();
           DBG_vPrintf(TRUE, "SHT power set to %i\n",newval>>DIO_SHT_VDD);
    }

    else if (0 == stricmp((char*)token, "temphum"))
    {
           DBG_vPrintf(TRUE, "Temperature and humidity\n");
           uint16_t temp, hum;
           while(sht3x_get_measurements(&temp, &hum)<0){}
           DBG_vPrintf(TRUE, "Values are %i/100 C, %i/100 %%\n",((uint32_t)(temp)*17500)/65535-4500,((uint32_t)hum)*10000/65535);
    }

    if (sButtonEvent.eType != APP_E_EVENT_NONE)
    {
        ZQ_bQueueSend(&APP_msgAppEvents, &sButtonEvent);
    }

    memset(sCommand.au8Buffer, 0, COMMAND_BUF_SIZE);
    sCommand.u8Pos = 0;

}

#if (BDB_JOIN_USES_INSTALL_CODE_KEY == TRUE)
/****************************************************************************
 *
 * NAME: u64HexStrToValue
 *
 * DESCRIPTION:
 * Convert a hex string to its value
 *
 * PARAMETERS:      Name           RW  Usage
 * uint8*           HexStr         R   String to convert
 *
 * RETURNS:
 * 64 bit value
 *
 ****************************************************************************/
PRIVATE uint64 u64HexStrToValue(uint8 *HexStr)
{
    uint64 u64Value = 0;

    /*Get rid of '{' and '}', if present */
    if(*HexStr == '{')
    {
        HexStr++;
        HexStr[strlen((char *)HexStr) - 1] = '\0'; // Assume last character will be '}' and needs to be stripped off
    }

    /* Get rid of any 0x prefix */
    if('0' == *(HexStr+0) && 'x' == *(HexStr+1))
    {
        HexStr+=2;
    }

    /* skip any leading 0s */
    while ('0' == *HexStr)
    {
        HexStr++;
    }

    if (*HexStr)
    {
        unsigned int uShift = 0;
        unsigned int n = strlen((char *)HexStr) - 1;;

        do
        {
            if (HexStr[n] >= 'A')
            {
                u64Value |= (uint64)(HexStr[n] - 'A' + 10) << uShift;
            }
            else
            {
                u64Value |= (uint64)(HexStr[n] - '0') << uShift;
            }
            uShift += 4;
        } while (n-- != 0);
    }

    return u64Value;
}

/****************************************************************************
 *
 * NAME: bValidateHexStr
 *
 * DESCRIPTION:
 * Check a hex string is valid
 *
 * PARAMETERS:      Name           RW  Usage
 * uint8*           HexStr         R   String to validate
 *
 * RETURNS:
 * TRUE if the string is valid.
 *
 ****************************************************************************/
PRIVATE bool_t bValidateHexStr(uint8 *HexStr)
{
    bool_t bValid = TRUE;

    /* skip any leading 0s */
    while ('0' == *HexStr)
    {
        HexStr++;
    }

    if (strlen((char *)HexStr) <= 16 )
    {
        uint8 c;
        while ( (c = *HexStr++) )
        {
            if (! ( (c >= 'A' && c <= 'F') /*|| (c >= 'a' && c <= 'f')*/ || (c >= '0' && c <= '9')) )
            {
                bValid = FALSE;
                break;
            }
        }
    }
    else
    {
        bValid = FALSE;
    }

    return bValid;
}

/******************************************************************************
 * NAME: bValidateKeyStr
 *
 * DESCRIPTION:
 * Validate the received string for allowable char in a Key string
 * converts a to f to upper case, strips out the separators (':' or ',') if present
 *
 * PARAMETERS:      Name            Usage
 * uint8           *KeyStr          Key to validate
 *
 * RETURNS: True if valid key string, without separators in the string
 * None
 ****************************************************************************/
PRIVATE bool_t bValidateKeyStr(uint8 *KeyStr)
{
    int len, i, j;
    len = strlen((char*)KeyStr);
    for (i=0; i<len; i++)
    {
        if ((KeyStr[i] == ':') || (KeyStr[i] == ','))
        {
            for (j=i; j<len; j++)
            {
                KeyStr[j] = KeyStr[j+1];
            }
            len--;
        }
        if ((KeyStr[i] >= 'a' && KeyStr[i] <= 'f'))
        {
            KeyStr[i] -= ('a' - 'A');
        }
        if (! ( (KeyStr[i] >= 'A' && KeyStr[i] <= 'F') || (KeyStr[i] >= '0' && KeyStr[i] <= '9')))
        {
            DBG_vPrintf(TRACE_SERIAL, "Bad char %c at pos %d\n", KeyStr[i], i);
            return FALSE;
        }
    }
    return (32 == len);

}
#endif

/****************************************************************************
 *
 * NAME: vPrintAPSTable
 *
 * DESCRIPTION:
 * Prints the content of APS table
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vPrintAPSTable(void)
{
    uint8 i;
    uint8 j;

    ZPS_tsAplAib * tsAplAib;

    tsAplAib = ZPS_psAplAibGetAib();

    for ( i = 0 ; i < (tsAplAib->psAplDeviceKeyPairTable->u16SizeOfKeyDescriptorTable + 1) ; i++ )
    {
        DBG_vPrintf(TRUE, "%d MAC: %016llx Key: ", i, ZPS_u64NwkNibGetMappedIeeeAddr(ZPS_pvAplZdoGetNwkHandle(),tsAplAib->psAplDeviceKeyPairTable->psAplApsKeyDescriptorEntry[i].u16ExtAddrLkup));
        for(j=0; j<16;j++)
        {
            DBG_vPrintf(TRUE, "%02x ", tsAplAib->psAplDeviceKeyPairTable->psAplApsKeyDescriptorEntry[i].au8LinkKey[j]);
        }
        DBG_vPrintf(TRUE, "\n");
    }
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
