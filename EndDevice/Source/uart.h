/*****************************************************************************
 *
 * MODULE:             JN-AN-1217
 *
 * COMPONENT:          uart.h
 *
 * DESCRIPTION:        UART interface for Base Device application
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
 ****************************************************************************/


#ifndef  UART_H_INCLUDED
#define  UART_H_INCLUDED

#ifdef SERIAL_COMMS

/* default to uart 0 */
#ifndef UART
#define UART E_AHI_UART_0
#endif

#if (UART != E_AHI_UART_0 && UART != E_AHI_UART_1)
#error UART must be either 0 or 1
#endif

#if (UART == E_AHI_UART_0)
#define UART_RXD_DIO	(7)
#elif (UART == E_AHI_UART_1)
#define UART_RXD_DIO	(15)
#endif

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************/
/***        Include Files                                                 ***/
/****************************************************************************/

/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/

/****************************************************************************/
/***        Type Definitions                                              ***/
/****************************************************************************/

/****************************************************************************/
/***        Exported Functions                                            ***/
/****************************************************************************/
#if (JENNIC_CHIP_FAMILY == JN516x)
PUBLIC void APP_isrUart(void);
#endif
#if (JENNIC_CHIP_FAMILY == JN517x)
PUBLIC void APP_isrUart (uint32 u32Device, uint32 u32ItemBitmap);
#endif
PUBLIC void UART_vInit(void);
PUBLIC void UART_vTxChar(uint8 u8TxChar);
PUBLIC bool_t UART_bTxReady(void);
PUBLIC void UART_vRtsStartFlow(void);
PUBLIC void UART_vRtsStopFlow(void);
PUBLIC void UART_vSetTxInterrupt(bool_t bState);
PUBLIC void UART_vSetBaudRate(uint32 u32BaudRate);

/****************************************************************************/
/***        Exported Variables                                            ***/
/****************************************************************************/

#if defined __cplusplus
}
#endif

#endif

#endif  /* UART_H_INCLUDED */

/****************************************************************************/
/***        END OF FILE                                                   ***/
/****************************************************************************/


