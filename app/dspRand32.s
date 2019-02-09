;	Implements a linear-congruental random number generator
;	y = (a.Xn + c) mod m
;
;	From Knuth: a=16644525, c=32767
;

	AREA    |.text|, CODE, READONLY

	ALIGN
asm_rand_32   FUNCTION    
	EXPORT asm_rand_32

	ldr r1,=16644525	;a
	ldr r2,=32767		;c
	
	;(a*seed) + c	
#if(__CORTEX_M == 0)
	MULS r0, r1, r0
	ADDS r0, r0, r2
#else
	mla r0,r1,r0,r2		
#endif
	
	bx lr
	
	ENDP
		
	END
