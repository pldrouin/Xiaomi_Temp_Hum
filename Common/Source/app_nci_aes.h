/*****************************************************************************
 *
 * MODULE:          JN-AN-1217 Base Device application
 *
 * COMPONENT:       app_nci_aes.h
 *
 * DESCRIPTION:     Base Device - Application layer for NCI (AES encryption)
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
#ifndef APP_NCI_AES_H_
#define APP_NCI_AES_H_

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/
#include <jendefs.h>
#include <nci.h>

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
#define APP_NCI_ADDRESS       0xFFU           /* I2C Address (0xFF for automatic detection) */
#define APP_NCI_I2C_LOCATION  FALSE           /* TRUE uses alternate I2C lines (DIO16, 17) instead of (DIO14, 15) */
#define APP_NCI_TICK_MS       5               /* Interval of tick timer in ms */
#define APP_NCI_VEN_PIN       0               /* Sets DIO connected to VEN pin */

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
PUBLIC void APP_vNciStart(uint8 u8NscType);
PUBLIC void APP_vNciStop(void);
PUBLIC void APP_cbNciTimer(void *pvParams);
PUBLIC void APP_cbNciEvent(             /* Called when an event takes place */
       teNciEvent   eNciEvent,          /* Event raised */
       uint32       u32Address,
       uint32       u32Length,
       uint8        *pu8Data);          /* Event data (NULL if no data) */

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#endif /* APP_NCI_AES_H_ */
/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/
