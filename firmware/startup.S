#if defined (__AVR_TINY__)
#define __zero_reg__ r17
#define __tmp_reg__ r16
#else
#define __zero_reg__ r1
#define __tmp_reg__ r0
#endif
#define __SREG__ 0x3f
#if defined (__AVR_HAVE_SPH__)
#define __SP_H__ 0x3e
#endif
#define __SP_L__ 0x3d
#define __RAMPZ__ 0x3B
#define __EIND__  0x3C


.section .vectors,"ax",@progbits

.global	__vectors
.func	__vectors

// Define a vector table with only the reset vector
// This frees up some code space as no interrupts are used in this firmware
__vectors:
	rjmp	__init

.endfunc


.section .init4,"ax",@progbits

.global	__do_clear_bss
.func	__do_clear_bss

// Define memory initialisation sequence
// This is normally provided by GCC, but with -nostartfiles we need to provide it
// (Borrowed from libgcc/config/avr/lib1funcs.S)
__init:
__do_clear_bss:
	ldi	r18, hi8(__bss_end)
	ldi	r26, lo8(__bss_start)
	ldi	r27, hi8(__bss_start)
	rjmp	.do_clear_bss_start
.do_clear_bss_loop:
	st	X+, __zero_reg__
.do_clear_bss_start:
	cpi	r26, lo8(__bss_end)
	cpc	r27, r18
	brne	.do_clear_bss_loop

.endfunc


.section .init9,"ax",@progbits

// Jump to the program, which will run forever
// This saves a couple of instructions from the default startup code that allows
// the program to "exit" and go into an infinite loop. We know our main() function
// will never return, so there's no need for this
rjmp main