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

.weak lc0_handler
.balign 4, 0
.global lc0_handler

.weak lc1_handler
.balign 4, 0
.global lc1_handler

.weak lc2_handler
.balign 4, 0
.global lc2_handler

.weak lc3_handler
.balign 4, 0
.global lc3_handler

.weak lc4_handler
.balign 4, 0
.global lc4_handler

.weak lc5_handler
.balign 4, 0
.global lc5_handler

.weak lc6_handler
.balign 4, 0
.global lc6_handler

.weak lc7_handler
.balign 4, 0
.global lc7_handler

.weak lc8_handler
.balign 4, 0
.global lc8_handler

.weak lc9_handler
.balign 4, 0
.global lc9_handler

.weak lc10_handler
.balign 4, 0
.global lc10_handler

.weak lc11_handler
.balign 4, 0
.global lc11_handler

.weak lc12_handler
.balign 4, 0
.global lc12_handler

.weak lc13_handler
.balign 4, 0
.global lc13_handler

.weak lc14_handler
.balign 4, 0
.global lc14_handler

.weak lc15_handler
.balign 4, 0
.global lc15_handler

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
        j software_handler /* software_handler */
IRQ_4:
        j default_vector_handler
IRQ_5:
        j default_vector_handler
IRQ_6:
        j default_vector_handler
IRQ_7:
        j timer_handler /* timer_handler */
IRQ_8:
        j default_vector_handler
IRQ_9:
	j default_vector_handler
IRQ_10:
        j default_vector_handler
IRQ_11:
        j external_handler /* external_handler */
IRQ_12:
        j default_vector_handler
IRQ_13:
        j default_vector_handler
IRQ_14:
        j default_vector_handler
IRQ_15:
        j default_vector_handler
IRQ_16:
        j lc0_handler /* local_ext_irq0  */
IRQ_17:
        j lc1_handler /* local_ext_irq1  */
IRQ_18:
        j lc2_handler /* local_ext_irq2  */
IRQ_19:
        j lc3_handler /* local_ext_irq3  */
IRQ_20:
        j lc4_handler /* local_ext_irq4  */
IRQ_21:
        j lc5_handler /* local_ext_irq5  */
IRQ_22:
        j lc6_handler /* local_ext_irq6  */
IRQ_23:
        j lc7_handler /* local_ext_irq7  */
IRQ_24:
        j lc8_handler /* local_ext_irq8  */
IRQ_25:
        j lc9_handler /* local_ext_irq9  */
IRQ_26:
        j lc10_handler /* local_ext_irq10  */
IRQ_27:
        j lc11_handler /* local_ext_irq11  */
IRQ_28:
        j lc12_handler /* local_ext_irq12  */
IRQ_29:
        j lc13_handler /* local_ext_irq13  */
IRQ_30:
        j lc14_handler /* local_ext_irq14  */
IRQ_31:
        j lc15_handler /* local_ext_irq15  */
