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
#include "tle_device.h"
#include "eval_board.h"
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>

#include "asm_prototype.h"
#include "main.h"

#define	RNG_TEST_NUM	100000

__IO uint32_t g_Ticks;

void User_SysTick_Handler(void)
{
  g_Ticks++;
}

void SimpleDelay(uint32_t t)
{
	uint32_t d = t*1000;
	while(--d)
	{
		__NOP();
		(void)WDT1_Service();	
	}  
}

uint32_t HAL_GetTick(void)
{
	return g_Ticks;
}

void HAL_Delay(uint32_t t)
{
	uint32_t d = t + HAL_GetTick();
	while(d > HAL_GetTick())
	{
		(void)WDT1_Service();	
	}  
}

void TestFunct(void){
	printf("%08X %s\n", TestFunct, __func__);
	printf("CPUID:%08X\n", SCB->CPUID);
}

/*****************************************************************************/
/** UART2: sends "Hello World!" through the STDIO @ 115200 baud             **/
/** STDIO uses UART2 P1.1/P1.2 through on-board debugger and VCOM           **/
/*****************************************************************************/
extern int __main(void);
int main(void)
{
	uint32_t tmpTick;

  /*****************************************************************************
  ** initialization of the hardware modules based on the configuration done   **
  ** by using the IFXConfigWizard                                             **
  *****************************************************************************/
  TLE_Init();
	
  /* System timer configuration */
  SysTick_Config(SystemFrequency / 1000);
	
  printf("Hello World! %u %p %p\n", SystemFrequency, main, __main);
	
	//Part 1: Move
	printf("Part 1\n");
	printf("ASM Test 1 Result:%u\n", asm_get_8bit_number());
	printf("ASM Test 2 Result:%08X\n", asm_get_xor(0x12345678, 0x34567890));
	printf("ASM Test 3 Direct Jump\n");
	printf("Before Jump.%08X\n", __get_MSP());
	asm_direct_jump_2(TestFunct);
	printf("Jump over.%08X\n", __get_MSP());
	
	//Part 2: Add
	printf("Part 2\n");
	printf("ASM Test 4 Result:%u\n", asm_add2(34));
	printf("ASM Test 5 Result:%u\n", asm_simple_add(123, 456));
	printf("ASM Test 6 Result:%u\n", asm_pc_add());
	
	//Part 3: Sub
	printf("Part 3\n");
	printf("ASM Test 7 Result:%d\n", asm_sub20(34));
	printf("ASM Test 8 Result:%d\n", asm_simple_sub(123, 456));
	printf("ASM Test 9 Result:%d\n", asm_get_neg(1024));

	//Part 4: Multiply, Compare, Logic
	printf("Part 4\n");
	printf("ASM Test 10 Result:%u\n", asm_simple_mul(123, 456));
	printf("ASM Test 11 Result:%u\n", asm_test_cmp(123, 456));
	printf("ASM Test 12 Result:%u\n", asm_test_cmn(123, 456));
	printf("ASM Test 13 Result:%08X\n", asm_get_and(0x12345678, 0x34567890));
	printf("ASM Test 14 Result:%08X\n", asm_get_or(0x12345678, 0x34567890));
	printf("ASM Test 15 Result:%08X\n", asm_get_not(0x12345678));

	//Test Addition/Mulitiplication Cycles
#define	TEST_ADD_MUL_NUM	100000
	//If the muliplication takes similar cycles, it is a single cycle multiplication implementation
	tmpTick = HAL_GetTick();
	for(uint32_t i=0; i<TEST_ADD_MUL_NUM; ++i)
	{
		asm_simple_add(i, 456);
	}
	tmpTick = HAL_GetTick()-tmpTick;
	printf("A:%u\n", tmpTick);
	tmpTick = HAL_GetTick();
	for(uint32_t i=0; i<TEST_ADD_MUL_NUM; ++i)
	{
		asm_simple_mul(i, 456);
	}
	tmpTick = HAL_GetTick()-tmpTick;
	printf("M:%u\n", tmpTick);
	
	printf("Part 5\n");
	//Part 5: Shift, Rotate
	printf("ASM Test 16 Result:%08X\n", asm_logic_left(0x80000001, 2));
	printf("ASM Test 17 Result:%08X\n", asm_logic_right(0x80000001, 2));
	printf("ASM Test 18 Result:%08X\n", asm_arithm_right(0x80000001, 2));
	printf("ASM Test 19 Result:%08X\n", asm_rotate_right(0x80000001, 2));

	//Part 6: Test Load, Store
	printf("Part 6\n");
	uint32_t g_TestVar32 = 0x12345678;
	printf("ASM Test 20 Result:%08X\n", asm_ldr32(&g_TestVar32));
	asm_str32(&g_TestVar32, 0x78904563);	
	printf("ASM Test 21 Result:%08X\n", asm_ldr32(&g_TestVar32));
	printf("ASM Test 22 Result:%u\n", asm_test_push_pop(123, 456));

	//Part 7: Test Extend, Reverse
	printf("Part 7\n");
	printf("ASM Test 23 Result:%08X\n", asm_s16ext((int16_t)0x8001));
	printf("ASM Test 24 Result:%08X\n", asm_s8ext((int8_t)0xC4));
	printf("ASM Test 25 Result:%08X\n", asm_u16ext((uint16_t)0x8001));
	printf("ASM Test 26 Result:%08X\n", asm_rev(0x123456C8));
	printf("ASM Test 27 Result:%08X\n", asm_rev16(0x123456C8));
	printf("ASM Test 28 Result:%08X\n", asm_revsh(0x123456C8));
	
	//Part 8: Test SVC, MSR, MRS
	printf("Part 8\n");	
	printf("ASM Test 29, Before SVC #1\n");
	asm_svc_1(1000);
	printf("After SVC #1\n");
	
	printf("ASM Test 30 Result:%08X\n", asm_test_mrs());
	printf("ASM Test 31 Tick:%u\n", SysTick->VAL);
	asm_test_msr(0x00000001);
	uint32_t p1 = asm_test_mrs();
	asm_test_msr(0x00000000);
	uint32_t p2 = asm_test_mrs();
	printf("%08X\t%08X\n", p1, p2);
	printf("CM0 Instruction Test End\n");	
	
  while (1)
  {
    WDT1_Service();
  }
}
