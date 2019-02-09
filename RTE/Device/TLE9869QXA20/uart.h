/**
 * @cond
 ***********************************************************************************************************************
 *
 * Copyright (c) 2015, Infineon Technologies AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,are permitted provided that the
 * following conditions are met:
 *
 *   Redistributions of source code must retain the above copyright notice, this list of conditions and the  following
 *   disclaimer.
 *
 *   Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *   following disclaimer in the documentation and/or other materials provided with the distribution.
 *
 *   Neither the name of the copyright holders nor the names of its contributors may be used to endorse or promote
 *   products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE  FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY,OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT  OF THE
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 **********************************************************************************************************************/


/*******************************************************************************
**                      Revision Control History                              **
*******************************************************************************/
/* See uart.c */

#ifndef UART_H
#define UART_H

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include <tle_device.h>
#include <Types.h>
#include <int.h>
#include "uart_defines.h"
#include "scu_defines.h"

/*******************************************************************************
**                      Global Macro Definitions                              **
*******************************************************************************/
#define UART_MASK_INT_RI (0x01u)
#define UART_MASK_INT_TI (0x02u)

#define UART1_RX_Sts()          ((UART1->SCON.reg & UART_MASK_INT_RI) >> 0u)
#define UART1_TX_Sts()          ((UART1->SCON.reg & UART_MASK_INT_TI) >> 1u)
#define UART2_RX_Sts()          ((UART2->SCON.reg & UART_MASK_INT_RI) >> 0u)
#define UART2_TX_Sts()          ((UART2->SCON.reg & UART_MASK_INT_TI) >> 1u)

#define UART1_Receiver_En()     (UART1->SCON.reg |= (uint8)1u << 4u)
#define UART1_Receiver_Dis()    (UART1->SCON.reg &= (uint8)~(1u << 4u))
#define UART1_BaudRateGen_En()  (SCU->BCON1.reg |= (uint8)1u << 0u)
#define UART1_BaudRateGen_Dis() (SCU->BCON1.reg &= (uint8)~(1u << 0u))

#define UART2_Receiver_En()     (UART2->SCON.reg |= (uint8)1u << 4u)
#define UART2_Receiver_Dis()    (UART2->SCON.reg &= (uint8)~(1u << 4u))
#define UART2_BaudRateGen_En()  (SCU->BCON2.reg |= (uint8)1u << 0u)
#define UART2_BaudRateGen_Dis() (SCU->BCON2.reg &= (uint8)~(1u << 0u))

/* UART Interrupt Clear Macros */
#define UART1_RX_Int_Clr()      (UART1->SCONCLR.reg = (uint8)1u << 0u)
#define UART1_TX_Int_Clr()      (UART1->SCONCLR.reg = (uint8)1u << 1u)
#define UART2_RX_Int_Clr()      (UART2->SCONCLR.reg = (uint8)1u << 0u)
#define UART2_TX_Int_Clr()      (UART2->SCONCLR.reg = (uint8)1u << 1u)

/* UART Interrupt Enable/Disable Macros */
#define UART1_RX_Int_En()       (SCU->MODIEN1.bit.RIEN1 = 1u)
#define UART1_RX_Int_Dis()      (SCU->MODIEN1.bit.RIEN1 = 0u)
#define UART1_TX_Int_En()       (SCU->MODIEN1.bit.TIEN1 = 1u)
#define UART1_TX_Int_Dis()      (SCU->MODIEN1.bit.TIEN1 = 0u)
#define UART2_RX_Int_En()       (SCU->MODIEN2.bit.RIEN2 = 1u)
#define UART2_RX_Int_Dis()      (SCU->MODIEN2.bit.RIEN2 = 0u)
#define UART2_TX_Int_En()       (SCU->MODIEN2.bit.TIEN2 = 1u)
#define UART2_TX_Int_Dis()      (SCU->MODIEN2.bit.TIEN2 = 0u)

/*******************************************************************************
**                      Global Function Declarations                          **
*******************************************************************************/
void UART1_Init(void);
void UART2_Init(void);
__STATIC_INLINE void UART1_Send_Byte(uint8 c);
__STATIC_INLINE uint8 UART1_Get_Byte(void);
__STATIC_INLINE bool UART1_isByteReceived(void);
__STATIC_INLINE bool UART1_isByteTransmitted(void);
void UART1_BaudRate_Set(uint32 baudrate);

__STATIC_INLINE void UART2_Send_Byte(uint8 c);
__STATIC_INLINE uint8 UART2_Get_Byte(void);
__STATIC_INLINE bool UART2_isByteReceived(void);
__STATIC_INLINE bool UART2_isByteTransmitted(void);
void UART2_BaudRate_Set(uint32 baudrate);

#if ((UART1_STD_EN == 1) || (UART2_STD_EN == 1))
sint32 stdout_putchar(sint32 Char);
sint32 stdin_getchar(void);
void ttywrch(int ch);
#endif /* ((UART1_STD_EN == 1) || (UART2_STD_EN == 1)) */


/*******************************************************************************
**                      Global Inline static Function Definitions             **
*******************************************************************************/
__STATIC_INLINE void UART1_Send_Byte(uint8 c)
{
  UART1_TX_Int_Clr();
  UART1->SBUF.reg = c;
}

__STATIC_INLINE uint8 UART1_Get_Byte(void)
{
  UART1_RX_Int_Clr();
  return(UART1->SBUF.reg);
}

__STATIC_INLINE bool UART1_isByteReceived(void)
{
	return((bool)(UART1_RX_Sts() == 1u));
}

__STATIC_INLINE bool UART1_isByteTransmitted(void)
{
	return((bool)(UART1_TX_Sts() == 1u));
}

__STATIC_INLINE void UART2_Send_Byte(uint8 c)
{
  UART2_TX_Int_Clr();
  UART2->SBUF.reg = c;
}

__STATIC_INLINE uint8 UART2_Get_Byte(void)
{
  UART2_RX_Int_Clr();
  return(UART2->SBUF.reg);
}

__STATIC_INLINE bool UART2_isByteReceived(void)
{
	return((bool)(UART2_RX_Sts() == 1u));
}

__STATIC_INLINE bool UART2_isByteTransmitted(void)
{
	return((bool)(UART2_TX_Sts() == 1u));
}

#endif
