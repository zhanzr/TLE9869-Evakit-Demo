	AREA    |.text|, CODE, READONLY

	ALIGN
asm_dot_prod_32   FUNCTION    
	EXPORT asm_dot_prod_32
	
	push {r4,r6}

	MOVS r3, #0
	
innerloop
	ldr r4,[r0,#0]
	ldr r5,[r1,#0]
	ADDS r0, #4
	ADDS r1, #4

#if(__CORTEX_M == 0)
	mov r6, r4
	MULS r6, r5, r6
	ADDS r3, r6, r3
#else
	mla r3,r4,r5,r3
#endif
	
	subs r2,r2,#1
	bne innerloop
	
	mov r0,r3
	
	pop {r4, r6}
	
	bx lr
	
	ENDP
		
	END
