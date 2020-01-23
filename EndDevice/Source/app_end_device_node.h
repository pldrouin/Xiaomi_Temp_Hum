/*****************************************************************************
 *
 * MODULE:             JN-AN-1217
 *
 * COMPONENT:          app_end_device_node.h
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
 * Copyright NXP B.V. 2016. All rights reserved
 *
 ***************************************************************************/

#ifndef APP_END_DEVICE_NODE_H_
#define APP_END_DEVICE_NODE_H_

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include "zcl_options.h"

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define NUMBER_DEVICE_TO_BE_DISCOVERED 8 // Should be same as Binding table size

#define KEEP_ALIVE_FACTORY_NEW  (45)
#define KEEP_ALIVETIME (10)
#define FIND_AND_BIND_IME (182)
#define DEEP_SLEEPTIME (10)
#define SLEEP_DURATION_MS (3480000)
#define SLEEP_TIMER_TICKS_PER_MS (32)

#define NEVER_DEEP_SLEEP   FALSE
//#define NEVER_DEEP_SLEEP   TRUE
#define ZCL_TICK_TIME           ZTIMER_TIME_MSEC(100)

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void APP_vInitialiseNode(void);
PUBLIC void APP_vFactoryResetRecords(void);
PUBLIC void APP_cbTimerPoll(void *pvParam);
PUBLIC void APP_vStartUpHW(void);

/****************************************************************************/
/***        External Variables                                            ***/
/****************************************************************************/
extern PUBLIC bool_t bDeepSleep;
extern uint8 u8KeepAliveTime;
extern uint8 u8DeepSleepTime;
/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

#endif /*APP_END_DEVICE_NODE_H_*/
