; Stop Go C/ASM Mix Example
; Jason Losh

;-----------------------------------------------------------------------------
; Hardware Target
;-----------------------------------------------------------------------------

; Target Platform: EK-TM4C123GXL Evaluation Board
; Target uC:       TM4C123GH6PM
; System Clock:    40 MHz

; Hardware configuration:
; Red LED:
;   PF1 drives an NPN transistor that powers the red LED
; Green LED:
;   PF3 drives an NPN transistor that powers the green LED
; Pushbutton:
;   SW1 pulls pin PF4 low (internal pull-up is used)

;-----------------------------------------------------------------------------
; Device includes, defines, and assembler directives
;-----------------------------------------------------------------------------


   .def pspset
   .def swpush
   .def swpop
   .def idle2XPSR
   .def idel2forPC
   .def idel2forR0toLR
   .def get_SV_val
   .def getR0



;-----------------------------------------------------------------------------
; Register values and large immediate values
;-----------------------------------------------------------------------------

.thumb
.const
GPIO_PORTF_DATA_R       .field   0x400253FC
n						.field	 0x01000000
w						.field   0xFFFFFFFE
;-----------------------------------------------------------------------------
; Subroutines
;-----------------------------------------------------------------------------

.text

pspset:


		MRS R1, CONTROL
		ORR R1 , #2
		MSR CONTROL , R1
		MSR PSP, R0
		BX LR


swpop:

			   MSR     PSP, R0
			   LDMFD   r0!, {r4-r11}
			   MSR	   PSP, R0
			   BX	   LR

swpush:
	   		   MRS     R0, PSP
			   STMDB   R0!, {r4-r11}
			   MSR	   PSP, R0
			   BX	   LR



idle2XPSR:

		MRS  R0, PSP
		SUB  R0, #4
		MOV  R1, #0x61000000
		STR  R1, [R0]
		MSR  PSP, R0
		BX   LR

idel2forPC:

		MOV  R1, R0
		MRS  R0, PSP
		SUB  R0, #4
		STR  R1, [R0]
		MSR  PSP, R0
		BX	 LR



idel2forR0toLR:
		MRS  R0, PSP
		MOV  R1, #1
		SUB  R0, #4
		STR  R1, [R0];  // for LR
		SUB  R0, #4
		STR  R1, [R0];  // for r12
		SUB  R0, #4
		STR  R1, [R0];  // for r3
		SUB  R0, #4
		STR  R1, [R0];  // for r2
        SUB  R0, #4
		STR  R1, [R0];  // for r1
		SUB  R0, #4
		STR  R1, [R0];  // for r0
		MSR  PSP, R0
		BX 	 LR



get_SV_val:
      MRS R0, PSP
      ADD R0, #24
      LDR R1, [R0]
      SUB R1, #2
      LDR R0, [R1]
      BX  LR

getR0:
     MRS R0, PSP
     LDR R0,[R0]
     BX  LR



.endm
