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
**                      Author(s) Identity                                    **
********************************************************************************
**                                                                            **
** Initials     Name                                                          **
** ---------------------------------------------------------------------------**
** DM           Daniel Mysliwitz                                              **
** TA           Thomas Albersinger                                            **
**                                                                            **
**                                                                            **
*******************************************************************************/

/*******************************************************************************
**                      Revision Control History                              **
*******************************************************************************/
/* 
 * V0.1.4: 2017-02-15, DM:   GetByte, isByteReceived and isByteTransmitted
 *                           inline functions added
 *                           Baudrate_Set function added
 * V0.1.3: 2015-07-15, DM:   STDIN/STDOUT function added
 * V0.1.2: 2015-02-10, DM:   individual header file added
 * V0.1.0: 2014-05-11, DM:   Initial version
 * V0.1.1: 2014-07-22, DM:   BGL baudrate calculation fixed
 */

/*******************************************************************************
**                      Includes                                              **
*******************************************************************************/
#include <uart.h>
#include <wdt1.h>
#include <stdio.h>

/*******************************************************************************
**                      External CallBacks                                    **
*******************************************************************************/

/*******************************************************************************
**                      Global Function Definitions                           **
*******************************************************************************/
/** \brief Initializes the UART module.
 *
 * \param None
 * \return None
 *
 * \ingroup drv_api
 */
void UART1_Init(void)
{
#if (UART1_Configuration_En == 1)
  UART1->SCON.reg = (uint8) UART1_SCON;
  SCU->BCON1.reg = (uint8) 0;
  SCU->BGH1.reg = (uint8) (UART1_BRVAL >> 3u);
  SCU->BGL1.reg = (uint8) (((UART1_BRVAL & 7u) << 5u) | UART1_FD);
  SCU->BCON1.reg = (uint8) SCU_BCON1;
#endif
}

/** \brief Sets the baudrate for UART1.
 *
 * \param baudrate, e.g. 19200, or 115200
 * \return None
 *
 * \ingroup drv_api
 */
void UART1_BaudRate_Set(uint32 baudrate)
{
  uint16 BRVAL;
  uint16 FD;
  
  BRVAL = (uint16)(((uint32)UART1_CLK * (uint32)1E6) / (16 * baudrate));
  FD =    (uint16)(((((uint32)UART1_CLK * (uint32)1E6) / (16 * baudrate)) - BRVAL) * 32);
  
  UART1_BaudRateGen_Dis();

  SCU->BGH1.reg = (uint8)(BRVAL >> 3u);
  SCU->BGL1.reg = (uint8)((BRVAL << 5u) | (FD & 0x1F));
  
  UART1_BaudRateGen_En();
}

/** \brief Initializes the UART module.
 *
 * \param None
 * \return None
 *
 * \ingroup drv_api
 */
void UART2_Init(void)
{
#if (UART2_Configuration_En == 1)
  UART2->SCON.reg = (uint8) UART2_SCON;
  SCU->BCON2.reg = (uint8) 0;
  SCU->BGH2.reg = (uint8) (UART2_BRVAL >> 3u);
  SCU->BGL2.reg = (uint8) (((UART2_BRVAL & 7u) << 5u) | UART2_FD);
  SCU->BCON2.reg = (uint8) SCU_BCON2;
#endif
}
/** \brief Sets the baudrate for UART2.
 *
 * \param baudrate, e.g. 19200, or 115200
 * \return None
 *
 * \ingroup drv_api
 */
void UART2_BaudRate_Set(uint32 baudrate)
{
  uint16 BRVAL;
  uint16 FD;
  
  BRVAL = (uint16)(((uint32)UART2_CLK * (uint32)1E6) / (16 * baudrate));
  FD =    (uint16)(((((uint32)UART2_CLK * (uint32)1E6) / (16 * baudrate)) - BRVAL) * 32);
  
  UART2_BaudRateGen_Dis();

  SCU->BGH2.reg = (uint8)(BRVAL >> 3u);
  SCU->BGL2.reg = (uint8)((BRVAL << 5u) | (FD & 0x1F));
  
  UART2_BaudRateGen_En();
}

#if (UART1_STD_EN == 1)
/** \brief Sends a character via UART1.
 *
 * \param[in] Char Character to send
 * \return Sent character
 *
 * \ingroup uart_api
 */
sint32 stdout_putchar(sint32 Char)
{
  UART1_Send_Byte(Char);
  
  if (Char == (sint32)'\n')
  {
    /* line feed: */
    while (UART1->SCON.bit.TI == 0u)
    {
      /* Execute wait function until character is transmitted */
      WDT1_Service();
    }
    /* Clear TI bit */
    UART1->SCONCLR.reg = UART_MASK_INT_TI;

    /* Send additional carriage return */
    UART1->SBUF.reg = (uint8)'\r';
  }
  while (UART1->SCON.bit.TI == 0u)
  {
    /* Execute wait function until character is transmitted */
    WDT1_Service();
  }
  return Char;
} /* End of stdout_putchar */ 


/** \brief Receives a character via UART2.
 *
 * \return Received character
 *
 * \ingroup uart_api
 */
sint32 stdin_getchar(void)
{
  while (UART1->SCON.bit.RI == 0u)
  {
    /* Execute wait function until character is received */
    WDT1_Service();
  }

  /* Clear RI bit */
  UART1->SCONCLR.reg = UART_MASK_INT_RI;

  /* Get character */
  return (sint32)UART1->SBUF.reg;
} /* End of stdin_getchar */ 

void ttywrch(int ch)
{
  stdout_putchar(ch);
}
#endif /* UART1_STD_EN == 1 */

#if (UART2_STD_EN == 1)
/** \brief Sends a character via UART1.
 *
 * \param[in] Char Character to send
 * \return Sent character
 *
 * \ingroup uart_api
 */
sint32 stdout_putchar(sint32 Char)
{
  UART2_Send_Byte(Char);
  
  if (Char == (sint32)'\n')
  {
    /* line feed: */
    while (UART2->SCON.bit.TI == 0u)
    {
      /* Execute wait function until character is transmitted */
      WDT1_Service();
    }
    /* Clear TI bit */
    UART2->SCONCLR.reg = UART_MASK_INT_TI;

    /* Send additional carriage return */
    UART2->SBUF.reg = (uint8)'\r';
  }
  while (UART2->SCON.bit.TI == 0u)
  {
    /* Execute wait function until character is transmitted */
    WDT1_Service();
  }
  return Char;
} /* End of stdout_putchar */ 


/** \brief Receives a character via UART2.
 *
 * \return Received character
 *
 * \ingroup uart_api
 */
sint32 stdin_getchar(void)
{
  while (UART2->SCON.bit.RI == 0u)
  {
    /* Execute wait function until character is received */
    WDT1_Service();
  }

  /* Clear RI bit */
  UART2->SCONCLR.reg = UART_MASK_INT_RI;

  /* Get character */
  return (sint32)UART2->SBUF.reg;
} /* End of stdin_getchar */ 

void ttywrch(int ch)
{
  stdout_putchar(ch);
}
#endif /* UART2_STD_EN == 1 */
