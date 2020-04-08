/*
 * handlers.S
 *
 * This file supports the population of the mtvec vector table,
 * which is populated with opcodes containing a jump instruction
 * to a specific handler used for CLINT vectored mode.
 */

.weak default_exception_handler
.balign 4, 0
.global default_exception_handler

.weak software_handler
.balign 4, 0
.global software_handler

.weak timer_handler
.balign 4, 0
.global timer_handler

.weak external_handler
.balign 4, 0
.global external_handler

.option norvc
.weak __mtvec_clint_vector_table
#if __riscv_xlen == 32
.balign 128, 0
#else
.balign 256, 0
#endif
.global __mtvec_clint_vector_table
__mtvec_clint_vector_table:

IRQ_0:
        j default_exception_handler
IRQ_1:
        j default_vector_handler
IRQ_2:
        j default_vector_handler
IRQ_3:
        j default_vector_handler /* software_handler */
IRQ_4:
        j default_vector_handler
IRQ_5:
        j default_vector_handler
IRQ_6:
        j default_vector_handler
IRQ_7:
        j default_vector_handler /* timer_handler */
IRQ_8:
        j default_vector_handler
IRQ_9:
	j default_vector_handler
IRQ_10:
        j default_vector_handler
IRQ_11:
        j default_vector_handler /* external_handler */
IRQ_12:
        j default_vector_handler
IRQ_13:
        j default_vector_handler
IRQ_14:
        j default_vector_handler
IRQ_15:
        j default_vector_handler
IRQ_16:
        j default_vector_handler
IRQ_17:
        j default_vector_handler
IRQ_18:
        j default_vector_handler
IRQ_19:
        j default_vector_handler
IRQ_20:
        j default_vector_handler
IRQ_21:
        j default_vector_handler
IRQ_22:
        j default_vector_handler
IRQ_23:
        j default_vector_handler
IRQ_24:
        j default_vector_handler
IRQ_25:
        j default_vector_handler
IRQ_26:
        j default_vector_handler
IRQ_27:
        j default_vector_handler
IRQ_28:
        j default_vector_handler
IRQ_29:
        j default_vector_handler
IRQ_30:
        j default_vector_handler
IRQ_31:
        j default_vector_handler
