/*sha256=45D90A753A2926802E65B0E7C5D0911AA6A7C0A4C48BA55964C3250EFAC69468*/
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

/*------------------------------------------------------------------------------
IFXConfigWizard output file
created on:Mi Aug 2 14:10:14 2017
------------------------------------------------------------------------------*/

#ifndef _BDRV_DEFINES_H
#define _BDRV_DEFINES_H

#ifndef IFXConfigWizard_Version
  #define IFXConfigWizard_Version 1.8.7
#endif

/* XML Version 1.3.1 */
#ifndef BDRV_XML_VERSION
  #define BDRV_XML_VERSION 10301
#endif

#ifndef BDRV_CP_CLK
  #define BDRV_CP_CLK 225
#endif

/*BDRV_CP_CLK_CTRL: (1<<15)|(2<<13)|(10<<8)|22*/
#ifndef BDRV_CP_CLK_CTRL
  #define BDRV_CP_CLK_CTRL (0xCA16u)
#endif

#ifndef BDRV_CP_CLK_SRC
  #define BDRV_CP_CLK_SRC 1.8e+07
#endif

/*BDRV_CP_CTRL_STS: 0|(3<<26)|(0<<24)|(0<<22)|(0<<20)|(0<<18)|(0<<16)|(0<<8)|(0\
<<25)*/
#ifndef BDRV_CP_CTRL_STS
  #define BDRV_CP_CTRL_STS (0xC000000u)
#endif

/*BDRV_CTRL3: (0<<16)|(0<<24)|(0<<6)|17|(0<<14)|(17<<8)|(0<<7)|(0<<15)|(0<<26)*/
#ifndef BDRV_CTRL3
  #define BDRV_CTRL3 (0x1111u)
#endif

#ifndef BDRV_EFF_CRG_CURR
  #define BDRV_EFF_CRG_CURR 89.9609
#endif

#ifndef BDRV_EFF_DISCRG_CURR
  #define BDRV_EFF_DISCRG_CURR 89.9609
#endif

#ifndef BDRV_LO_DITH
  #define BDRV_LO_DITH 209.302
#endif

#ifndef BDRV_LO_DITH_FREQ
  #define BDRV_LO_DITH_FREQ 209
#endif

/*BDRV_OFF_SEQ: (0<<27)|(0<<24)|(0<<19)|(0<<16)|(0<<11)|(0<<8)|(0<<3)|0*/
#ifndef BDRV_OFF_SEQ
  #define BDRV_OFF_SEQ (0x0u)
#endif

/*BDRV_ON_SEQ: (0<<27)|(0<<24)|(0<<19)|(0<<16)|(0<<11)|(0<<8)|(0<<3)|0*/
#ifndef BDRV_ON_SEQ
  #define BDRV_ON_SEQ (0x0u)
#endif

/*BDRV_TRIM_DRVx: (0<<24)|(1<<10)|(1<<11)|(1<<18)|(1<<19)|(1<<13)|(1<<14)|(1<<2\
1)|(1<<22)|(0<<16)|(0<<8)|0|(0<<5)*/
#ifndef BDRV_TRIM_DRVx
  #define BDRV_TRIM_DRVx (0x6C6C00u)
#endif

#ifndef BDRV_UP_DITH
  #define BDRV_UP_DITH 243.243
#endif

#ifndef BDRV_UP_DITH_FREQ
  #define BDRV_UP_DITH_FREQ 243
#endif


#endif
