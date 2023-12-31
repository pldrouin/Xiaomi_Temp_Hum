/*****************************************************************************
 *
 * MODULE:             JN-AN-1217
 *
 * COMPONENT:          app_start.c
 *
 * DESCRIPTION:        Base Device Demo: Router Initialisation
 *
 *****************************************************************************
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
 * Copyright NXP B.V. 2017. All rights reserved
 *
 ****************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include "pwrm.h"
#include "pdum_nwk.h"
#include "pdum_apl.h"
#include "pdum_gen.h"
#include "PDM.h"
#include "dbg.h"
#include "dbg_uart.h"
#include "zps_gen.h"
#include "zps_apl.h"
#include "zps_apl_af.h"
#include "zps_apl_zdo.h"
#include "string.h"
#include "AppApi.h"
#include "app_end_device_node.h"
#include "zcl_options.h"
#include "app_common.h"
#include "app_main.h"
#include "uart.h"
#include "ZTimer.h"
#include "app_buttons.h"
#include "app_sht3x.h"
#ifdef APP_NTAG_ICODE
#include "ntag_nwk.h"
#include "app_ntag_icode.h"
#endif
#ifdef APP_NTAG_AES
#include "ntag_nwk.h"
#include "app_ntag_aes.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

#ifdef DEBUG_APP
    #define TRACE_APP   TRUE
#else
    #define TRACE_APP   FALSE
#endif

#ifdef DEBUG_SLEEP
    #define TRACE_SLEEP  TRUE
#else
    #define TRACE_SLEEP   FALSE
#endif

#define HALT_ON_EXCEPTION   FALSE

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void APP_vInitialise(void);
PRIVATE void vfExtendedStatusCallBack(ZPS_teExtendedStatus eExtendedStatus);
PRIVATE void vSetUpWakeUpConditions(bool_t bDeeperSleep);

/**
 * Power manager Callback.
 * Called just before the device is put to sleep
 */

static PWRM_DECLARE_CALLBACK_DESCRIPTOR(PreSleep);
/**
 * Power manager Callback.
 * Called just after the device wakes up from sleep
 */
static PWRM_DECLARE_CALLBACK_DESCRIPTOR(Wakeup);
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

extern void *_stack_low_water_mark;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

/****************************************************************************
 *
 * NAME: vAppMain
 *
 * DESCRIPTION:
 * Entry point for application from a cold start.
 *
 * RETURNS:
 * Never returns.
 *
 ****************************************************************************/
PUBLIC void vAppMain(void)
{

#if JENNIC_CHIP_FAMILY == JN516x
    /* Wait until FALSE i.e. on XTAL - otherwise UART data will be at wrong speed */
    while (bAHI_GetClkSource() == TRUE);

    /* Move CPU to 32 MHz  vAHI_OptimiseWaitStates automatically called */
    bAHI_SetClockRate(3);
#endif

    /*
     * Don't use RTS/CTS pins on UART0 as they are used for buttons
     * */
#ifdef DBG_ENABLE
    vAHI_UartSetRTSCTS(E_AHI_UART_0, FALSE);
    /* Initialise the debug diagnostics module to use UART0 at 115K Baud;
     * Do not use UART 1 if LEDs are used, as it shares DIO with the LEDS */
    DBG_vUartInit(DBG_E_UART_0, DBG_E_UART_BAUD_RATE_115200);
    #ifdef DEBUG_921600
    {
        /* Bump baud rate up to 921600 */
        vAHI_UartSetBaudDivisor(DBG_E_UART_0, 2);
        vAHI_UartSetClocksPerBit(DBG_E_UART_0, 8);
    }
    #endif
#endif

#if (JENNIC_CHIP_FAMILY == JN516x)
    /* Initialise the stack overflow exception to trigger if the end of the
     * stack is reached. See the linker command file to adjust the allocated
     * stack size. */
    vAHI_SetStackOverflow(TRUE, (uint32)&_stack_low_water_mark);
#endif

    /* Catch resets due to watchdog timer expiry. Comment out to harden code. */
    if (bAHI_WatchdogResetEvent())
    {
        DBG_vPrintf(TRUE, "APP: Watchdog timer has reset device!\n");
        #if HALT_ON_EXCEPTION
            vAHI_WatchdogStop();
            while (1);
        #endif
    }

    //uint8 array[]={0xD2, 0x85, 0x01, 0x2A, 0xEC, 0xF4, 0x80, 0xDE, 0x37, 0xCB, 0x19, 0x5E, 0x3B, 0x39, 0xC5, 0xD4};

    /* idle task commences here */
    DBG_vPrintf(TRUE,"\n");
    DBG_vPrintf(TRUE, "***********************************************\n");
    DBG_vPrintf(TRUE, "* END DEVICE RESET                            *\n");
    DBG_vPrintf(TRUE, "***********************************************\n");

    DBG_vPrintf(TRACE_APP, "APP: Entering APP_vSetUpHardware()\n");
    APP_vSetUpHardware();

    DBG_vPrintf(TRACE_APP, "APP: Entering APP_vInitResources()\n");
    APP_vInitResources();

    DBG_vPrintf(TRACE_APP, "APP: Entering APP_vInitialise()\n");
    APP_vInitialise();

#if (defined APP_NTAG_ICODE) || (defined APP_NTAG_AES)
    DBG_vPrintf(TRACE_APP, "\nAPP: Entering APP_vNtagPdmLoad()");
    /* Didn't start BDB using PDM data ? */
    if (FALSE == APP_bNtagPdmLoad())
#endif
    {
        DBG_vPrintf(TRACE_APP, "APP: Entering BDB_vStart()\n");
        BDB_vStart();

#ifdef APP_NTAG_AES
        DBG_vPrintf(TRACE_APP, "\nAPP: Entering APP_vNtagStart()");
        APP_vNtagStart(NFC_NWK_NSC_DEVICE_ZIGBEE_ROUTER_DEVICE);
#endif
    }

#ifdef APP_NTAG_ICODE
    /* Not waking from deep sleep ? */
    if (0 == (u16AHI_PowerStatus() & (1 << 11)))
    {
        DBG_vPrintf(TRACE_APP, "\nAPP: Entering APP_vNtagStart()");
        APP_vNtagStart(ENDDEVICE_APPLICATION_ENDPOINT);
    }
#endif

    //ZPS_teStatus status=ZPS_vAplSecSetInitialSecurityState(ZPS_ZDO_PRECONFIGURED_LINK_KEY, array, 0, ZPS_APS_GLOBAL_LINK_KEY);

    //DBG_vPrintf(TRACE_APP, "APP: Set key returned %d\n",status);

    DBG_vPrintf(TRACE_APP, "APP: Entering APP_vMainLoop()\n");
    APP_vMainLoop();

}

/****************************************************************************
 *
 * NAME: vAppRegisterPWRMCallbacks
 *
 * DESCRIPTION:
 * Power manager callback.
 * Called to allow the application to register
 * sleep and wake callbacks.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
void vAppRegisterPWRMCallbacks(void)
{
    PWRM_vRegisterPreSleepCallback(PreSleep);
    PWRM_vRegisterWakeupCallback(Wakeup);
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/
/****************************************************************************
 *
 * NAME: PreSleep
 *
 * DESCRIPTION:
 *
 * PreSleep call back by the power manager before the controller put into sleep.
 *
 * PARAMETERS:      Name            RW  Usage
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PWRM_CALLBACK(PreSleep)
{
    DBG_vPrintf(TRACE_SLEEP,"Sleeping mode %d...\n", bDeeperSleep);
    /* If the power mode is with RAM held do the following
     * else not required as the entry point will init everything*/
    if(bSleepRamOn)
    {
        /* sleep memory held */
       vAppApiSaveMacSettings();
    }

    ZTIMER_vSleep();

    /* Set up wake up dio input */
    vSetUpWakeUpConditions(bDeeperSleep);

    sht3x_i2c_disable();

#ifdef DBG_ENABLE
    /* Wait for the UART to complete any transmission */
    DBG_vUartFlush();

    /* Disable UART (if enabled) */
    vAHI_UartDisable(E_AHI_UART_0);
#endif

    vAHI_WatchdogStop();
}

/****************************************************************************
 *
 * NAME: Wakeup
 *
 * DESCRIPTION:
 *
 * Wakeup call back by the power manager after the controller wakes up from sleep.
 *
 * PARAMETERS:      Name            RW  Usage
 *
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PWRM_CALLBACK(Wakeup)
{
	vAHI_WatchdogStart(12);

    /*Stabilise the oscillator*/
#if JENNIC_CHIP_FAMILY == JN516x
    // Wait until FALSE i.e. on XTAL  - otherwise uart data will be at wrong speed
    while (bAHI_GetClkSource() == TRUE);
    // Now we are running on the XTAL, optimise the flash memory wait states.
    vAHI_OptimiseWaitStates();
#endif

    uint32_t diowake=u32AHI_DioWakeStatus();

#if (defined SERIAL_COMMS && defined RX_WAKE)
    vAHI_DioInterruptEnable(0, (1<<UART_RXD_DIO));
#endif

#ifdef DBG_ENABLE
    /* Don't use RTS/CTS pins on UART0 as they are used for buttons */
    vAHI_UartSetRTSCTS(E_AHI_UART_0, FALSE);
    DBG_vUartInit(DBG_E_UART_0, DBG_E_UART_BAUD_RATE_115200);
    #ifdef DEBUG_921600
    {
        /* Bump baud rate up to 921600 */
        vAHI_UartSetBaudDivisor(DBG_E_UART_0, 2);
        vAHI_UartSetClocksPerBit(DBG_E_UART_0, 8);
    }
    #endif
#endif
    DBG_vPrintf(TRACE_SLEEP, "\n\nAPP: Woken up (CB)");
    DBG_vPrintf(TRACE_SLEEP, "\nAPP: Warm Waking powerStatus = 0x%x\n", u8AHI_PowerStatus());
    DBG_vPrintf(TRACE_SLEEP, "DioWake: 0x%08x\n",diowake);

    /* If the power status is OK and RAM held while sleeping
     * restore the MAC settings
     * */
    if( (u8AHI_PowerStatus()) && bSleepRamOn )
    {
        // Restore Mac settings (turns radio on)
        vMAC_RestoreSettings();
        DBG_vPrintf(TRACE_SLEEP, "\nAPP: MAC settings restored");
    }

    APP_vSetUpHardware();

    ZTIMER_vWake();

#ifdef SERIAL_COMMS
    UART_vInit();
    UART_vRtsStartFlow();
#endif

    if(diowake&(APP_BUTTONS_DIO_MASK)) {
        DBG_vPrintf(TRACE_SLEEP, "Button wake\n");
    	u8WakeUpCondition = APP_WAKE_BUTTON;
    	u8KeepAliveTime = KEEP_ALIVETIME;
    	APP_cbTimerButtonScan(NULL);
    }
#if (defined SERIAL_COMMS && defined RX_WAKE)
    else if(diowake&(1<<UART_RXD_DIO)) {
        DBG_vPrintf(TRACE_SLEEP, "Uart RX wake\n");
    	u8WakeUpCondition = APP_WAKE_NONE;
    	u8KeepAliveTime = KEEP_ALIVETIME;
    }
#endif
    else if(diowake&(1<<DIO_SHT_ALERT)) {
        DBG_vPrintf(TRACE_SLEEP, "SHT3x wake\n");
    	u8WakeUpCondition = APP_WAKE_SHT3X;

    } else { //Timer wake
        DBG_vPrintf(TRACE_SLEEP, "Timer wake\n");
    	u8WakeUpCondition = APP_WAKE_TIMER;
    	u8KeepAliveTime = 2;
    }

    sht3x_i2c_enable();

    /* Activate the SleepTask, that would start the SW timer and polling would continue
     * */
    APP_vStartUpHW();
}
/****************************************************************************
 *
 * NAME: vSetUpWakeUpConditions
 *
 * DESCRIPTION:
 *
 * Set up the wake up inputs while going to sleep  or preserve as an output
 * to drive the LHS led indicator if never sleeping
 *
 ****************************************************************************/
PRIVATE void vSetUpWakeUpConditions(bool_t bDeeperSleep)
{
    u32AHI_DioWakeStatus();                         /* clear interrupts */
    //vAHI_DioSetDirection(APP_BUTTONS_DIO_MASK|(1<<DIO_SHT_ALERT),0);   /* Set Power Button(DIO0) and SHT DIO line as Input */
    vAHI_DioSetDirection(APP_BUTTONS_DIO_MASK|(1<<DIO_SHT_ALERT)
#if (defined SERIAL_COMMS && defined RX_WAKE)
    		|(1<<UART_RXD_DIO)
#endif
    		,0);   /* Set Power Button(DIO0) and SHT DIO line as Input */


    DBG_vPrintf(TRACE_SLEEP, "Going to sleep: Buttons:%08x Mask:%08x\n", u32AHI_DioReadInput() & APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP, APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP);
    if(bDeeperSleep)
    {
    	/* Not waking up on SHT DIO line from deep sleep as SHT is to be disabled during deep sleep */
        vAHI_DioWakeEdge(0,APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP);
        vAHI_DioWakeEnable(APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP,131071^APP_BUTTONS_DIO_MASK_FOR_DEEP_SLEEP);
    }
    else
    {
        //vAHI_DioWakeEdge(1<<DIO_SHT_ALERT,APP_BUTTONS_DIO_MASK);       /* Set the wake up DIO Edge - Falling Edge and SHT DIO line Edge - Rising Edge */
        vAHI_DioWakeEdge(1<<DIO_SHT_ALERT,APP_BUTTONS_DIO_MASK
#if (defined SERIAL_COMMS && defined RX_WAKE)
        		|(1<<UART_RXD_DIO)
#endif
        		);       /* Set the wake up DIO Edge - Falling Edge and SHT DIO line Edge - Rising Edge */
        //vAHI_DioWakeEnable(APP_BUTTONS_DIO_MASK|(1<<DIO_SHT_ALERT),0);     /* Set the Wake up DIO Power Button */
        vAHI_DioWakeEnable(APP_BUTTONS_DIO_MASK|(1<<DIO_SHT_ALERT)
#if (defined SERIAL_COMMS && defined RX_WAKE)
        		|(1<<UART_RXD_DIO)
#endif
        		,131071^(APP_BUTTONS_DIO_MASK|(1<<DIO_SHT_ALERT)
#if (defined SERIAL_COMMS && defined RX_WAKE)
        		|(1<<UART_RXD_DIO)
#endif
        		));     /* Set the Wake up DIO Power Button */

    }
}
/****************************************************************************
 *
 * NAME: APP_vInitialise
 *
 * DESCRIPTION:
 * Initialises Zigbee stack, hardware and application.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_vInitialise(void)
{
    /* Initialise Power Manager even on non-sleeping nodes as it allows the
     * device to doze when in the idle task */
    PWRM_vInit(E_AHI_SLEEP_OSCON_RAMON);

    /* Initialise the Persistent Data Manager */
    PDM_eInitialise(63);

    /* Initialise Protocol Data Unit Manager */
    PDUM_vInit();

#ifdef SERIAL_COMMS
    UART_vInit();
    UART_vRtsStartFlow();
#endif

    ZPS_vExtendedStatusSetCallback(vfExtendedStatusCallBack);

    /* Initialise application */
    APP_vInitialiseNode();
}

/****************************************************************************
 *
 * NAME: vfExtendedStatusCallBack
 *
 * DESCRIPTION:
 * Callback from stack on extended error situations.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vfExtendedStatusCallBack (ZPS_teExtendedStatus eExtendedStatus)
{
    DBG_vPrintf(TRUE,"ERROR: Extended status 0x%02x\n", eExtendedStatus);
}

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
