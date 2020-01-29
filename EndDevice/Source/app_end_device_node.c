/*****************************************************************************
 *
 * MODULE:             JN-AN-1217
 *
 * COMPONENT:          app_end_device_node.c
 *
 * DESCRIPTION:        Thermostat Device Demo: End Device Application
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
 * Copyright NXP B.V. 2017. All rights reserved
 *
 ***************************************************************************/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/

#include <jendefs.h>
#include <string.h>
#include "dbg.h"
#include "pdum_apl.h"
#include "pdum_nwk.h"
#include "pdum_gen.h"
#include "pwrm.h"
#include "PDM.h"
#include "zps_gen.h"
#include "zps_apl.h"
#include "zps_apl_af.h"
#include "zps_apl_zdo.h"
#include "zps_apl_zdp.h"
#include "zps_apl_aib.h"
#include "zps_apl_aps.h"
#include "bdb_api.h"
#include "app_common.h"
#include "app_main.h"
#include "app_buttons.h"
#include "app_sht3x.h"
#include "ZTimer.h"
#include "app_events.h"
#include <rnd_pub.h>

#include "app_zcl_task.h"
#include "app_end_device_node.h"
#include "zps_nwk_nib.h"
#include "PDM_IDs.h"
#include "zcl_options.h"
#include "zcl.h"
#ifdef JN517x
#include "AHI_ModuleConfiguration.h"
#endif
#ifdef APP_NTAG_ICODE
#include "app_ntag_icode.h"
#include "nfc_nwk.h"
#endif
#ifdef APP_NTAG_AES
#include "app_ntag_aes.h"
#include "nfc_nwk.h"
#endif

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/


#ifndef DEBUG_APP
    #define TRACE_APP   FALSE
#else
    #define TRACE_APP   TRUE
#endif

#ifdef DEBUG_APP_EVENT
    #define TRACE_APP_EVENT   TRUE
#else
    #define TRACE_APP_EVENT   FALSE
#endif

#ifdef DEBUG_APP_BDB
    #define TRACE_APP_BDB     TRUE
#else
    #define TRACE_APP_BDB     FALSE
#endif

#ifdef DEBUG_POLL_SLEEP
    #define TRACE_POLL_SLEEP   TRUE
#else
    #define TRACE_POLL_SLEEP   FALSE
#endif


/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Local Function Prototypes                                     ***/
/****************************************************************************/

PRIVATE void vAppHandleAfEvent( BDB_tsZpsAfEvent *psZpsAfEvent);
PRIVATE void vAppSendIdentifyStop( uint16 u16Address, uint8 u8Endpoint);
PRIVATE void vAppHandleZdoEvents( BDB_tsZpsAfEvent *psZpsAfEvent);
PRIVATE void vStartPolling(void);
PRIVATE void vWakeCallBack(void);
PRIVATE void vStopAllTimers(void);
PRIVATE void APP_vBdbInit(void);
PRIVATE void vPrintAPSTable(void);
#if BDB_JOIN_USES_INSTALL_CODE_KEY == TRUE
PRIVATE void APP_vSetICDerivedLinkKey(void);
#endif
/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/
const uint8 u8MyEndpoint = ENDDEVICE_APPLICATION_ENDPOINT;
PUBLIC bool_t bDeepSleep;
PUBLIC uint8 u8KeepAliveTime = KEEP_ALIVETIME;
PUBLIC uint8 u8DeepSleepTime = DEEP_SLEEPTIME;

/****************************************************************************/
/***        Local Variables                                               ***/
/****************************************************************************/

PRIVATE teNodeState eNodeState;
PRIVATE uint16 u16FastPoll;
PRIVATE bool_t bBDBJoinFailed = FALSE;
PRIVATE bool_t bFailToJoin = FALSE;
PRIVATE pwrm_tsWakeTimerEvent    sWake;
PRIVATE bool_t bFnBInProgress = FALSE;
PRIVATE PDUM_thAPduInstance hAPduInst;
PRIVATE bool_t bButtonDown = FALSE;
PRIVATE uint8_t u8ButtonDownCount = 0;
/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/

#ifdef PDM_EEPROM
extern uint8 u8PDM_CalculateFileSystemCapacity();
extern uint8 u8PDM_GetFileSystemOccupancy();
#endif

/****************************************************************************
 *
 * NAME: APP_vInitialiseNode
 *
 * DESCRIPTION:
 * Initialises the application related functions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vInitialiseNode(void)
{
    uint16 u16ByteRead;

    APP_bButtonInitialise();

    sht3x_initialise();

    eNodeState = E_STARTUP;
    PDM_eReadDataFromRecord(PDM_ID_APP_END_DEVICE,
                            &eNodeState,
                            sizeof(teNodeState),
                            &u16ByteRead);

    ZPS_u32MacSetTxBuffers  (4);
#ifdef JN517x
    /* Default module configuration: change E_MODULE_DEFAULT as appropriate */
      vAHI_ModuleConfigure(E_MODULE_DEFAULT);
#endif

    /* Initialise ZBPro stack */
    ZPS_eAplAfInit();

    /* Initialise ZCL */
    APP_ZCL_vInitialise();

    ZPS_bAplAfSetEndDeviceTimeout(ZPSENDDEVICETIMEOUT);

    /* Initialise other software modules
     * HERE
     */
    APP_vBdbInit();

    /* Always initialise any peripherals used by the application
     * HERE
     */

#if (defined PDM_EEPROM) && (defined TRACE_APP)
    /* The functions u8PDM_CalculateFileSystemCapacity and u8PDM_GetFileSystemOccupancy
     * may be called at any time to monitor space available in  the eeprom  */
    DBG_vPrintf(TRACE_APP, "PDM: Capacity %d\n", u8PDM_CalculateFileSystemCapacity() );
    DBG_vPrintf(TRACE_APP, "PDM: Occupancy %d\n", u8PDM_GetFileSystemOccupancy() );
#endif

#if BDB_JOIN_USES_INSTALL_CODE_KEY == TRUE
    APP_vSetICDerivedLinkKey();
    vPrintAPSTable();
#endif

    hAPduInst = PDUM_hAPduAllocateAPduInstance(apduZCL);
}


/****************************************************************************
 *
 * NAME: APP_vBdbCallback
 *
 * DESCRIPTION:
 * Callback from the BDB
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vBdbCallback(BDB_tsBdbEvent *psBdbEvent)
{
    static uint8 u8NoQueryCount;

    switch(psBdbEvent->eEventType)
    {
        case BDB_EVENT_NONE:
            break;
        case BDB_EVENT_ZPSAF:                // Use with BDB_tsZpsAfEvent
            vAppHandleAfEvent(&psBdbEvent->uEventData.sZpsAfEvent);
            break;

        case BDB_EVENT_INIT_SUCCESS:
            break;

        case BDB_EVENT_REJOIN_FAILURE:
            DBG_vPrintf(TRACE_APP_BDB,"APP: BDB_EVENT_REJOIN_FAILURE\n");
            bBDBJoinFailed = TRUE;
            sht3x_i2c_turnoff();
            break;

        case BDB_EVENT_REJOIN_SUCCESS:
            vStartPolling();
            eNodeState = E_RUNNING;
            bBDBJoinFailed = FALSE;
            DBG_vPrintf(TRACE_APP_BDB,"APP: BDB_EVENT_REJOIN_SUCCESS\n");

            sht3x_i2c_initialise();
            sht3x_alert=TRUE;
            break;


        case BDB_EVENT_NWK_STEERING_SUCCESS:
            DBG_vPrintf(TRACE_APP_BDB,"APP: NwkSteering Success 0x%016llx\n",ZPS_psAplAibGetAib()->u64ApsUseExtendedPanid);
            ZPS_vSaveAllZpsRecords();
            PDM_eSaveRecordData(PDM_ID_APP_END_DEVICE,
                                &eNodeState,
                                sizeof(teNodeState));
            sht3x_i2c_initialise();
            sht3x_alert=TRUE;
            vStartPolling();
            break;

        case BDB_EVENT_APP_START_POLLING:
            DBG_vPrintf(TRACE_APP_BDB,"APP: Start polling\n");
            vStartPolling();
            break;


        case BDB_EVENT_FB_CHECK_BEFORE_BINDING_CLUSTER_FOR_TARGET:
            DBG_vPrintf(TRACE_APP_BDB,"Check For Binding Cluster %d \n",psBdbEvent->uEventData.psFindAndBindEvent->uEvent.u16ClusterId);

            break;

        case BDB_EVENT_FB_CLUSTER_BIND_CREATED_FOR_TARGET:
            DBG_vPrintf(TRACE_APP_BDB,"Bind Created for cluster %d \n",psBdbEvent->uEventData.psFindAndBindEvent->uEvent.u16ClusterId);
            break;

        case BDB_EVENT_FB_BIND_CREATED_FOR_TARGET:
        {
            DBG_vPrintf(TRACE_APP_BDB,"APP-BDB: Bind Created for target EndPt %d\n",
                    psBdbEvent->uEventData.psFindAndBindEvent->u8TargetEp);
#ifndef USE_GROUPS
            vAppSendIdentifyStop( psBdbEvent->uEventData.psFindAndBindEvent->u16TargetAddress,
                                  psBdbEvent->uEventData.psFindAndBindEvent->u8TargetEp);
#endif
            break;
        }

        case BDB_EVENT_FB_GROUP_ADDED_TO_TARGET:
        {
            DBG_vPrintf(TRACE_APP_BDB,"APP-BDB: Group Added with ID %04x\n",
                                psBdbEvent->uEventData.psFindAndBindEvent->uEvent.u16GroupId);
                        u8NoQueryCount = 0;
#ifdef USE_GROUPS
                            //Example to ask to Stop identification to that group
                        vAppSendIdentifyStop(psBdbEvent->uEventData.psFindAndBindEvent->u16TargetAddress,
                                             psBdbEvent->uEventData.psFindAndBindEvent->u8TargetEp);
#endif

            break;
        }

        case BDB_EVENT_FB_ERR_BINDING_TABLE_FULL:
            DBG_vPrintf(TRACE_APP_BDB,"ERR: Bind Table Full\n");
            break;

        case BDB_EVENT_FB_ERR_BINDING_FAILED:
            DBG_vPrintf(TRACE_APP_BDB,"ERR: Bind\n");
            break;

        case BDB_EVENT_FB_ERR_GROUPING_FAILED:
            DBG_vPrintf(TRACE_APP_BDB,"ERR: Group\n");
            break;

        case BDB_EVENT_FB_NO_QUERY_RESPONSE:
            DBG_vPrintf(TRACE_APP_BDB,"ERR: No Query response\n");
            //Example to stop further query repeating
            if(u8NoQueryCount >= 2)
            {
                u8NoQueryCount = 0;
                BDB_vFbExitAsInitiator();
                bFnBInProgress = FALSE;
                u8KeepAliveTime = KEEP_ALIVETIME;
            }
            else
            {
                u8NoQueryCount++;
            }
            break;

        case BDB_EVENT_FB_TIMEOUT:
            DBG_vPrintf(TRACE_APP_BDB,"ERR: TimeOut\n");
            bFnBInProgress = FALSE;
            u8KeepAliveTime = KEEP_ALIVETIME;
            break;

        default:
            break;
    }
}


/****************************************************************************
 *
 * NAME: APP_taskEndDevice
 *
 * DESCRIPTION:
 * Task that handles application related functions
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_taskEndDevice(void)
{
	APP_tsEvent sAppEvent;
	sAppEvent.eType = APP_E_EVENT_NONE;

	if (ZQ_bQueueReceive(&APP_msgAppEvents, &sAppEvent) == TRUE) {
		DBG_vPrintf(TRACE_APP_EVENT, "ZPR: App event %d, NodeState=%d\n", sAppEvent.eType, eNodeState);

		if (sAppEvent.eType == APP_E_EVENT_BUTTON_UP) {
			bButtonDown = FALSE;
			u8ButtonDownCount = 0;

		} else if (sAppEvent.eType == APP_E_EVENT_BUTTON_DOWN) {
			bButtonDown = TRUE;

			if ((eNodeState == E_RUNNING) && (bBDBJoinFailed)) {
				bBDBJoinFailed = FALSE;
				/* trigger a new round of rejoin attempts */
				DBG_vPrintf(TRACE_APP_EVENT, "Call BDB vStart\n");
				sBDB.sAttrib.bbdbNodeIsOnANetwork = TRUE;
				BDB_vStart();

			} else {
				/* reset the sleep counts on key press */

				if ((eNodeState == E_RUNNING) && (bFnBInProgress == FALSE)) {
					u8KeepAliveTime = KEEP_ALIVETIME;
					u8DeepSleepTime = DEEP_SLEEPTIME;

				} else if(eNodeState != E_RUNNING) {
					u8KeepAliveTime = KEEP_ALIVE_FACTORY_NEW;
				}
			}
		}
	}
}

/****************************************************************************
 *
 * NAME: vAppHandleAfEvent
 *
 * DESCRIPTION:
 * Application handler for stack events
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vAppHandleAfEvent( BDB_tsZpsAfEvent *psZpsAfEvent)
{

    if (psZpsAfEvent->u8EndPoint == ENDDEVICE_APPLICATION_ENDPOINT)
    {
        if (psZpsAfEvent->sStackEvent.eType == ZPS_EVENT_APS_DATA_INDICATION)
        {
            APP_ZCL_vEventHandler( &psZpsAfEvent->sStackEvent);
        }
    }
    else if(psZpsAfEvent->u8EndPoint == ENDDEVICE_ZDO_ENDPOINT)
    {
        vAppHandleZdoEvents( psZpsAfEvent);
    }

    /* Ensure Freeing of Apdus */
    if (psZpsAfEvent->sStackEvent.eType == ZPS_EVENT_APS_DATA_INDICATION)
    {
        PDUM_eAPduFreeAPduInstance(psZpsAfEvent->sStackEvent.uEvent.sApsDataIndEvent.hAPduInst);
    }
    else if ( psZpsAfEvent->sStackEvent.eType == ZPS_EVENT_APS_INTERPAN_DATA_INDICATION )
    {
        PDUM_eAPduFreeAPduInstance(psZpsAfEvent->sStackEvent.uEvent.sApsInterPanDataIndEvent.hAPduInst);
    }

}


/****************************************************************************
 *
 * NAME: vAppHandleZdoEvents
 *
 * DESCRIPTION:
 * Application handler for stack events for end point 0 (ZDO)
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vAppHandleZdoEvents( BDB_tsZpsAfEvent *psZpsAfEvent)
{
    ZPS_tsAfEvent *psAfEvent = &(psZpsAfEvent->sStackEvent);

    switch(psAfEvent->eType)
    {
        case ZPS_EVENT_APS_DATA_INDICATION:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Data Indication Status %02x from %04x Src Ep Dst %d Ep %d Profile %04x Cluster %04x\n",
                    psAfEvent->uEvent.sApsDataIndEvent.eStatus,
                    psAfEvent->uEvent.sApsDataIndEvent.uSrcAddress.u16Addr,
                    psAfEvent->uEvent.sApsDataIndEvent.u8SrcEndpoint,
                    psAfEvent->uEvent.sApsDataIndEvent.u8DstEndpoint,
                    psAfEvent->uEvent.sApsDataIndEvent.u16ProfileId,
                    psAfEvent->uEvent.sApsDataIndEvent.u16ClusterId);
            break;

        case ZPS_EVENT_APS_DATA_CONFIRM:
            break;

        case ZPS_EVENT_APS_DATA_ACK:
            break;
            break;

        case ZPS_EVENT_NWK_JOINED_AS_ENDDEVICE:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Joined Network Addr %04x Rejoin %d\n",
                    psAfEvent->uEvent.sNwkJoinedEvent.u16Addr,
                    psAfEvent->uEvent.sNwkJoinedEvent.bRejoin);

            /* set the aps use pan id */
            ZPS_eAplAibSetApsUseExtendedPanId( ZPS_u64NwkNibGetEpid(ZPS_pvAplZdoGetNwkHandle()) );

            eNodeState = E_RUNNING;
            bFailToJoin = FALSE;
            bBDBJoinFailed = FALSE;
            u8KeepAliveTime = KEEP_ALIVETIME;
            PDM_eSaveRecordData(PDM_ID_APP_END_DEVICE,
                                &eNodeState,
                                sizeof(teNodeState));
            #ifdef APP_NTAG_ICODE
            {
                /* Not a rejoin ? */
                if (FALSE == psAfEvent->uEvent.sNwkJoinedEvent.bRejoin)
                {
                    /* Write network data to tag */
                    APP_vNtagStart(ENDDEVICE_APPLICATION_ENDPOINT);
                }
            }
            #endif
            break;

        case ZPS_EVENT_NWK_FAILED_TO_JOIN:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Failed To Join 0x%02x Rejoin %d\n",
                    psAfEvent->uEvent.sNwkJoinFailedEvent.u8Status,
                    psAfEvent->uEvent.sNwkJoinFailedEvent.bRejoin);
            bFailToJoin = TRUE;
            if (ZPS_psAplAibGetAib()->u64ApsUseExtendedPanid != 0)
            {
                DBG_vPrintf(TRACE_APP, "Restore epid %016llx\n", ZPS_psAplAibGetAib()->u64ApsUseExtendedPanid);
                ZPS_vNwkNibSetExtPanId(ZPS_pvAplZdoGetNwkHandle(), ZPS_psAplAibGetAib()->u64ApsUseExtendedPanid);
            }
            break;

        case ZPS_EVENT_NWK_DISCOVERY_COMPLETE:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Discovery Complete 0x%02x\n",
                    psAfEvent->uEvent.sNwkDiscoveryEvent.eStatus);
            vPrintAPSTable();
            break;

        case ZPS_EVENT_NWK_LEAVE_INDICATION:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Leave Indication %016llx Rejoin %d\n",
                    psAfEvent->uEvent.sNwkLeaveIndicationEvent.u64ExtAddr,
                    psAfEvent->uEvent.sNwkLeaveIndicationEvent.u8Rejoin);
            if ( (psAfEvent->uEvent.sNwkLeaveIndicationEvent.u64ExtAddr == 0UL) &&
                 (psAfEvent->uEvent.sNwkLeaveIndicationEvent.u8Rejoin == 0) )
            {
                /* We sare asked to Leave without rejoin */
                DBG_vPrintf(TRACE_APP, "LEAVE IND -> For Us No Rejoin\n");
                APP_vFactoryResetRecords();
                vAHI_SwReset();
            }
            break;

        case ZPS_EVENT_NWK_LEAVE_CONFIRM:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Leave Confirm status %02x Addr %016llx\n",
                    psAfEvent->uEvent.sNwkLeaveConfirmEvent.eStatus,
                    psAfEvent->uEvent.sNwkLeaveConfirmEvent.u64ExtAddr);
            if (( psAfEvent->uEvent.sNwkLeaveConfirmEvent.eStatus == ZPS_E_SUCCESS) &&
                ( psAfEvent->uEvent.sNwkLeaveConfirmEvent.u64ExtAddr == 0UL))
            {
                DBG_vPrintf(TRACE_APP, "Leave -> Reset Data Structures\n");
                APP_vFactoryResetRecords();
                vAHI_SwReset();
            }
            break;

        case ZPS_EVENT_NWK_STATUS_INDICATION:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Network status Indication %02x addr %04x\n",
                    psAfEvent->uEvent.sNwkStatusIndicationEvent.u8Status,
                    psAfEvent->uEvent.sNwkStatusIndicationEvent.u16NwkAddr);
            break;

        case ZPS_EVENT_NWK_POLL_CONFIRM:
                    break;

        case ZPS_EVENT_NWK_ED_SCAN:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Energy Detect Scan %02x\n",
                    psAfEvent->uEvent.sNwkEdScanConfirmEvent.u8Status);
            break;
        case ZPS_EVENT_ZDO_BIND:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Zdo Bind event\n");
            break;

        case ZPS_EVENT_ZDO_UNBIND:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Zdo Unbiind Event\n");
            break;

        case ZPS_EVENT_ZDO_LINK_KEY:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Zdo Link Key Event Type %d Addr %016llx\n",
                        psAfEvent->uEvent.sZdoLinkKeyEvent.u8KeyType,
                        psAfEvent->uEvent.sZdoLinkKeyEvent.u64IeeeLinkAddr);
            break;

        case ZPS_EVENT_BIND_REQUEST_SERVER:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Bind Request Server Event\n");
            break;

        case ZPS_EVENT_ERROR:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: AF Error Event %d\n", psAfEvent->uEvent.sAfErrorEvent.eError);
            break;

        case ZPS_EVENT_TC_STATUS:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Trust Center Status %02x\n", psAfEvent->uEvent.sApsTcEvent.u8Status);
            break;

        default:
            DBG_vPrintf(TRACE_APP, "APP-ZDO: Unhandled Event %d\n", psAfEvent->eType);
            break;
    }
}

/****************************************************************************/
/***        Local Functions                                               ***/
/****************************************************************************/


/****************************************************************************
 *
 * NAME: APP_vFactoryResetRecords
 *
 * DESCRIPTION:
 * Resets persisted data structures to factory new state
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vFactoryResetRecords(void)
{
    /* clear out the stack */
    ZPS_vDefaultStack();
    ZPS_eAplAibSetApsUseExtendedPanId(0);
    ZPS_vSetKeys();
#if BDB_JOIN_USES_INSTALL_CODE_KEY == TRUE
    APP_vSetICDerivedLinkKey();
#endif
    /* save everything */
    eNodeState = E_STARTUP;
    PDM_eSaveRecordData(PDM_ID_APP_END_DEVICE, &eNodeState, sizeof(teNodeState));
    ZPS_vSaveAllZpsRecords();
}

/****************************************************************************
 *
 * NAME: vStartPolling
 *
 * DESCRIPTION:
 * start the remote polling, initial at a fast rate
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vStartPolling(void)
{
    u16FastPoll = (uint16)(0.5/0.25);
    ZTIMER_eStop(u8TimerPoll);
    if(ZTIMER_eStart(u8TimerPoll, POLL_TIME_FAST) != E_ZTIMER_OK)
    {
        DBG_vPrintf(TRACE_APP, "APP: Failed to Poll Tick Timer\n");
    }
}

/****************************************************************************
 *
 * NAME: APP_cbTimerPoll
 *
 * DESCRIPTION: Manages polling, sleeping and rejoin retries
 *              activated by timer
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_cbTimerPoll(void *pvParam)
{

    uint32 u32Time = ZTIMER_TIME_MSEC(1000);
    DBG_vPrintf(TRACE_POLL_SLEEP,"Initial activity %d\n",PWRM_u16GetActivityCount());

    if(bButtonDown && !u16FastPoll) {
    	++u8ButtonDownCount;

    	if(u8ButtonDownCount >= BUTTON_PRESS_RESET_TIME) {

			if (eNodeState == E_RUNNING) {
				if (ZPS_eAplZdoLeaveNetwork(0UL, FALSE, FALSE) != ZPS_E_SUCCESS ) {
					DBG_vPrintf(TRACE_APP, "Resetting from running state\n");
					u8ButtonDownCount=0;
					APP_vFactoryResetRecords();
					vAHI_SwReset();
				}

			} else {
				DBG_vPrintf(TRACE_APP, "Resetting from state %u\n",eNodeState);
				u8ButtonDownCount=0;
				APP_vFactoryResetRecords();
				vAHI_SwReset();
			}
    	}
    }

    switch (eNodeState)
    {
        case E_STARTUP:
        	DBG_vPrintf(TRACE_APP," Start Steering \n");
        	BDB_eNsStartNwkSteering();

            u32Time = NETWORK_RESTART_TIME;
            DBG_vPrintf(TRACE_POLL_SLEEP, "Poll Sleep Not Running Keep Alive %d\n", u8KeepAliveTime);

            if ((u8KeepAliveTime--) == 0)
            {
                DBG_vPrintf(TRACE_POLL_SLEEP, "Go Deep...\n");
                vStopAllTimers();
                PWRM_vInit(E_AHI_SLEEP_DEEP);
                bDeepSleep = TRUE;
                return;

            }
            break;

        case E_RUNNING:
            DBG_vPrintf(TRACE_POLL_SLEEP, "Poll & Sleep Keepalive %d Deep Sleep %d BDB Rejoin Failed %d Failed to Join %d\n",
                    u8KeepAliveTime, u8DeepSleepTime, bBDBJoinFailed, bFailToJoin);

            if (bBDBJoinFailed) {

                /* Manage rejoin attempts,then short wait, then deep sleep */
                u32Time = ZTIMER_TIME_MSEC(1000);

                if (u8DeepSleepTime) {
                    u8DeepSleepTime--;

                } else {
                    vStopAllTimers();
                    DBG_vPrintf(TRACE_POLL_SLEEP, "join failed: go deep... %d\n", PWRM_u16GetActivityCount());
                    PWRM_vInit(E_AHI_SLEEP_DEEP);
                    bDeepSleep = TRUE;
                    return;
                }

            } else { // (!bBDBJoinFailed)

                if (bFailToJoin) {
                    /* stop sleeping for the duration of rejoin attempts */
                    u8KeepAliveTime = KEEP_ALIVETIME;

                } else if(sht3x_alert) {
            		DBG_vPrintf(TRACE_APP, "\nSHT ALERT!\n");
            		/*uint16_t status;

            		while(sht3x_read_status(&status)){}
            		DBG_vPrintf(TRUE, "Status is 0x%04x\n",status);

            		if(status&SHT3X_STATUS_ALERT) {*/
            			uint16_t temp, hum;

            			while(sht3x_get_measurements(&temp, &hum)){}
            			DBG_vPrintf(TRACE_APP, "Values are %i/100 C, %i/100 %%\n",((uint32_t)(temp)*17500)/65535-4500,((uint32_t)hum)*10000/65535);

            			tsZCL_Address destaddr={E_ZCL_AM_SHORT, {0x0000}};

            			sThermostatDevice.sTemperatureMeasurementServerCluster.i16MeasuredValue=temp*17500/65535-4500;
            			sThermostatDevice.sRelativeHumidityMeasurementServerCluster.u16MeasuredValue=hum*10000/65535;
            			//PDUM_eAPduInstanceSetPayloadSize(hAPduInst, 0);

            			int32_t i;

            			for(i=ATTRREPORTNUMTRIALS-1; i>=0; --i) {
            				if(!eZCL_ReportAttribute(&destaddr, MEASUREMENT_AND_SENSING_CLUSTER_ID_TEMPERATURE_MEASUREMENT, 0x0000, 0x01, 0x01, hAPduInst))
            					break;
            				DBG_vPrintf(TRACE_APP, "Failed to send temperature measurement\n");
            			}


            			for(i=ATTRREPORTNUMTRIALS-1; i>=0; --i) {
            				if(!eZCL_ReportAttribute(&destaddr, MEASUREMENT_AND_SENSING_CLUSTER_ID_RELATIVE_HUMIDITY_MEASUREMENT, 0x0000, 0x01, 0x01, hAPduInst))
            					break;
            				DBG_vPrintf(TRACE_APP, "Failed to send relative humidity measurement\n");
            			}

            		/*}*/
                	sht3x_alert=FALSE;
                	break; //Skip polling
                }

                /* Manage polling, then warm sleep, then deep sleep */
                if(u8KeepAliveTime == 0) {

                	vStopAllTimers();
                	//if (u8DeepSleepTime)
                	//{
                	uint8 u8Status;
                	u8Status = PWRM_eScheduleActivity(&sWake, (SLEEP_DURATION_MS * SLEEP_TIMER_TICKS_PER_MS) , vWakeCallBack);
                	DBG_vPrintf(TRACE_POLL_SLEEP, "poll task: schedule sleep status %02x\n",
                			u8Status );
                	/*}

                		else
                		{
                			PWRM_vInit(E_AHI_SLEEP_DEEP);
                			bDeepSleep = TRUE;
                			DBG_vPrintf(TRACE_POLL_SLEEP, "poll task: go deep\n");
                		}*/

                    DBG_vPrintf(TRACE_POLL_SLEEP,"Activity %d\n",PWRM_u16GetActivityCount());
                    return;

                } else { // (u8KeepAliveTime>0)
                   uint8 u8PStatus;

                   if (bBDBJoinFailed == FALSE) {
                	   DBG_vPrintf(TRACE_POLL_SLEEP, "Data polling...\n");
                       u8PStatus = ZPS_eAplZdoPoll();

                       if ( u8PStatus) {
                           DBG_vPrintf(TRACE_POLL_SLEEP, "\nPOLL status %d\n", u8PStatus);
                       }
                   }

                   if (u16FastPoll) {
                       u16FastPoll--;
                       u32Time = POLL_TIME_FAST;

                       if (u16FastPoll == 0) {
                           DBG_vPrintf(TRACE_POLL_SLEEP, "\nStop fast poll");
                       }

                   } else { // (!u16FastPoll)
                       /* Decrement the keep alive in the normal operation mode
                        * Not in active scan mode or while fast polling
                        */

                       if(0 < u8KeepAliveTime) {
                           u8KeepAliveTime--;
                       }
                       /*Start Poll Timer to continue normal polling */
                       u32Time = ZTIMER_TIME_MSEC(1000);
                   }
                }
            }

            break;

        default:
            /* shouldn't happen, but... */
            u32Time = POLL_TIME;
            break;

    }

    ZTIMER_eStop(u8TimerPoll);
    if(ZTIMER_eStart(u8TimerPoll, u32Time) != E_ZTIMER_OK)
    {
        DBG_vPrintf(TRACE_APP, "APP: Failed to Poll Tick Timer\n");
    }
}

/****************************************************************************
 *
 * NAME: vAppSendIdentifyStop
 *
 * DESCRIPTION:
 * Sends an Identify stop command to the target address
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vAppSendIdentifyStop( uint16 u16Address, uint8 u8Endpoint)
{
    uint8 u8Seq;
    tsZCL_Address sAddress;
    tsCLD_Identify_IdentifyRequestPayload sPayload;

    sPayload.u16IdentifyTime = 0;
    sAddress.eAddressMode = E_ZCL_AM_SHORT_NO_ACK;
    sAddress.uAddress.u16DestinationAddress = u16Address;
    eCLD_IdentifyCommandIdentifyRequestSend(
                            COORDINATOR_APPLICATION_ENDPOINT,
                            u8Endpoint,
                            &sAddress,
                            &u8Seq,
                            &sPayload);
}


/****************************************************************************
 *
 * NAME: vWakeCallBack
 *
 * DESCRIPTION:
 * Wake up call back called upon wake up by the schedule activity event.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vWakeCallBack(void)
{
    /*Decrement the deepsleep count so that if there is no activity for
     * DEEP_SLEEPTIME then the module is put to deep sleep.
     * */
    if (0 < u8DeepSleepTime)
{
#if NEVER_DEEP_SLEEP
        u8DeepSleepTime = DEEP_SLEEPTIME;
#else
        u8DeepSleepTime--;
#endif
    }

}

/****************************************************************************
 *
 * NAME: vStopAllTimers
 *
 * DESCRIPTION:
 * Stops all the timers
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void vStopAllTimers(void)
{
    ZTIMER_eStop(u8TimerPoll);
    ZTIMER_eStop(u8TimerZCL);
    ZTIMER_eStop(u8TimerId);

}

/****************************************************************************
 *
 * NAME: APP_vStartUpHW
 *
 * DESCRIPTION:
 * Os Task activated by the wake up event to manage sleep
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PUBLIC void APP_vStartUpHW(void)
{

    //uint8 u8Status;


    /* Restart the keyboard scanning timer as we've come up through */
    /* warm start via the Power Manager if we get here              */

    /*DBG_vPrintf(TRACE_POLL_SLEEP, "\nWoken: start poll timer,");
    u8Status = ZPS_eAplZdoPoll();
    DBG_vPrintf(TRACE_POLL_SLEEP, " Wake poll %02x\n", u8Status);*/
    if(ZTIMER_eStart(u8TimerPoll, ZTIMER_TIME_MSEC(200)) != E_ZTIMER_OK)
    {
        DBG_vPrintf(TRACE_APP, "APP: Failed to start Poll Tick Timer\n");
    }
    if ( ZTIMER_eStart(u8TimerZCL, ZCL_TICK_TIME) != E_ZTIMER_OK )
    {
        DBG_vPrintf(TRACE_APP, "APP: Failed to start ZCL Timer\n");
    }

}

/****************************************************************************
 *
 * NAME: APP_vBdbInit
 *
 * DESCRIPTION:
 * Function to initialize BDB attributes and message queue
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_vBdbInit(void)
{
    BDB_tsInitArgs sInitArgs;

    sBDB.sAttrib.bbdbNodeIsOnANetwork = ((eNodeState >= E_RUNNING)? (TRUE): (FALSE));
    if(sBDB.sAttrib.bbdbNodeIsOnANetwork)
    {
        DBG_vPrintf(TRACE_APP, "APP: NFN Start\n");
        bFailToJoin = TRUE;
        ZTIMER_eStart(u8TimerPoll, ZTIMER_TIME_MSEC(1000) );
    }
    else
    {
        DBG_vPrintf(TRACE_APP, "APP: FN Start\n");
        u8KeepAliveTime = KEEP_ALIVE_FACTORY_NEW;
    }
    ZTIMER_eStart(u8TimerPoll, ZTIMER_TIME_MSEC(1000) );
    sInitArgs.hBdbEventsMsgQ = &APP_msgBdbEvents;
    BDB_vInit(&sInitArgs);
}

#if BDB_JOIN_USES_INSTALL_CODE_KEY == TRUE
/****************************************************************************
 *
 * NAME: APP_vSetICDerivedLinkKey
 *
 * DESCRIPTION:
 * Setting up an install code derived link key
 * Install code is 8-byte MAC address repeated once.
 *
 * RETURNS:
 * void
 *
 ****************************************************************************/
PRIVATE void APP_vSetICDerivedLinkKey()
{
    int i;
    uint8 au8ICode[16];
    uint64 u64MyMacAddr = ZPS_u64NwkNibGetExtAddr(ZPS_pvAplZdoGetNwkHandle());
    /* Test Vector  = {0x83,0xFE,0xD3,0x40,0x7A,0x93,0x97,0x23,0xA5,0xC6,0x39,0xB2,0x69,0x16,0xD5,0x05};   */

    /* Copy uint64 MAC into array twice */
    for (i=0; i<=15; i++)
    {
        au8ICode[15-i]= ((u64MyMacAddr >> (i%8)*8) & (0xFF));
    }

    for (i=0; i<16; i++)
    {
        DBG_vPrintf(TRACE_APP, "%02x", au8ICode[i]);
    }
    DBG_vPrintf(TRACE_APP, "\n");

    ZPS_vAplSecSetInitialSecurityState( ZPS_ZDO_PRCONFIGURED_INSTALLATION_CODE,
                                        au8ICode,
                                        16,
                                        ZPS_APS_GLOBAL_LINK_KEY);

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
PUBLIC void vPrintAPSTable(void)
{
    uint8 i;
    uint8 j;

    ZPS_tsAplAib * tsAplAib;

    tsAplAib = ZPS_psAplAibGetAib();

    for ( i = 0 ; i < (tsAplAib->psAplDeviceKeyPairTable->u16SizeOfKeyDescriptorTable + 1) ; i++ )
    {
        DBG_vPrintf(TRACE_APP, "MAC: %016llx Key: ", ZPS_u64NwkNibGetMappedIeeeAddr(ZPS_pvAplZdoGetNwkHandle(),tsAplAib->psAplDeviceKeyPairTable->psAplApsKeyDescriptorEntry[i].u16ExtAddrLkup));
        for(j=0; j<16;j++)
        {
            DBG_vPrintf(TRACE_APP, "%02x ", tsAplAib->psAplDeviceKeyPairTable->psAplApsKeyDescriptorEntry[i].au8LinkKey[j]);
        }
        DBG_vPrintf(TRACE_APP, "\n");
        DBG_vPrintf(TRACE_APP, "Incoming FC: %d\n", tsAplAib->pu32IncomingFrameCounter[i]);
        DBG_vPrintf(TRACE_APP, "Outgoing FC: %d\n", tsAplAib->psAplDeviceKeyPairTable->psAplApsKeyDescriptorEntry[i].u32OutgoingFrameCounter);
    }
}
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
