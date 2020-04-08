/* Copyright 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */

/*
 * This example sets up the CPU to service local interrupts using
 * the CLINT mode of operation. SiFive GPIO's are configured as inputs
 * to support a hardware platform like the Arty 100T with buttons
 * that are connected to the local interrupt lines.
 */

#include <stdio.h>
#include <stdlib.h>

/* These includes get created at build time, and are based on the contents
 * in the bsp folder.  They are useful since they allow us
 * to use auto generated symbols and base addresses which may change
 * based on the design, and every design has it's own unique bsp.
 */
#include <metal/machine.h>
#include <metal/machine/platform.h>
#include <metal/machine/inline.h>

/*
 * This test demonstrates how to enable and handle local interrupts,
 * like the software interrupt using Interrupt ID #3, the
 * timer interrupt using Interrupt ID #7, and buttons on the
 * Arty 100T platform, which are typically in the #16-31 range.
 *
 * This example uses the CLINT vectored mode of operation, which
 * uses a vector table and is lower latency than CLINT direct mode,
 * due to the software overhead.
 *
 * CLINT direct mode does not use a vector table, so all
 * interrupts and exceptions trap to mtvec.base address and software
 * is responsible for dispatching interrupts based on the contents
 * in the mcause CSR.  CLINT direct mode of operation is not supported
 * in this example.  In CLINT direct mode (mtvec.mode = 0), all interrupts
 * and exceptions would trap to address in mtvec.base.
 */

#define DISABLE                 0
#define ENABLE                  1
#define TRUE                    1
#define FALSE                   0
#define INPUT                   0x100    /* something other than 0 or 1 */
#define OUTPUT                  0x101    /* something other than 0 or 1 */
#define RTC_FREQ                32768

#define MCAUSE_INTR                         0x80000000UL
#define MCAUSE_CAUSE                        0x000003FFUL
#define MCAUSE_CODE(cause)                  (cause & MCAUSE_CAUSE)

/* Compile time options to determine which interrupt modules we have */
#define CLINT_PRESENT                           (METAL_MAX_CLINT_INTERRUPTS > 0)
#define PLIC_PRESENT                            (METAL_MAX_PLIC_INTERRUPTS > 0)

/* Interrupt Specific defines - used for mtvec.mode field, which is bit[0] for
 * designs with CLINT, or [1:0] for designs with a CLIC */
#define MTVEC_MODE_CLINT_DIRECT                 0x00
#define MTVEC_MODE_CLINT_VECTORED               0x01
#define MTVEC_MODE_CLIC_DIRECT                  0x02
#define MTVEC_MODE_CLIC_VECTORED                0x03

/* Offsets for multi-core systems */
#define MSIP_PER_HART_OFFSET                             0x4
#define MTIMECMP_PER_HART_OFFSET                         0x8

#if CLINT_PRESENT
#define CLINT_BASE_ADDR                                 METAL_RISCV_CLINT0_0_BASE_ADDRESS
#define MSIP_BASE_ADDR(hartid)                          (CLINT_BASE_ADDR + METAL_RISCV_CLINT0_MSIP_BASE + (hartid * MSIP_PER_HART_OFFSET))
#define MTIMECMP_BASE_ADDR(hartid)                      (CLINT_BASE_ADDR + METAL_RISCV_CLINT0_MTIMECMP_BASE + (hartid * MTIMECMP_PER_HART_OFFSET))
#define MTIME_BASE_ADDR                                 (CLINT_BASE_ADDR + METAL_RISCV_CLINT0_MTIME)
#endif

#define NUM_TICKS_ONE_S                         RTC_FREQ            // it takes this many ticks of mtime for 1s to elapse
#define NUM_TICKS_ONE_MS                        (RTC_FREQ/1000)     // it takes this many ticks of mtime for 1ms to elapse
#define DEMO_TIMER_INTERVAL                     5000                // 5s timer interval
#define SET_TIMER_INTERVAL_MS(ms_ticks)         write_dword(MTIMECMP_BASE_ADDR(read_csr(mhartid)), (read_dword(MTIME_BASE_ADDR) + (ms_ticks * NUM_TICKS_ONE_MS)))

/* Setup prototypes */
void interrupt_global_enable (void);
void interrupt_global_disable (void);
void interrupt_software_enable (void);
void interrupt_software_disable (void);
void interrupt_timer_enable (void);
void interrupt_timer_disable (void);
void interrupt_external_enable (void);
void interrupt_external_disable (void);
void interrupt_local_enable (int id);
void interrupt_local_disable (int id);

/* Defines to access CSR registers within C code */
#define read_csr(reg) ({ unsigned long __tmp; \
  asm volatile ("csrr %0, " #reg : "=r"(__tmp)); \
  __tmp; })

#define write_csr(reg, val) ({ \
  asm volatile ("csrw " #reg ", %0" :: "rK"(val)); })

#define write_dword(addr, data)                 ((*(uint64_t *)(addr)) = data)
#define read_dword(addr)                        (*(uint64_t *)(addr))
#define write_word(addr, data)                  ((*(uint32_t *)(addr)) = data)
#define read_word(addr)                         (*(uint32_t *)(addr))
#define write_byte(addr, data)                  ((*(uint8_t *)(addr)) = data)
#define read_byte(addr)                         (*(uint8_t *)(addr))

/* Globals */
void __attribute__((weak, interrupt)) __mtvec_clint_vector_table(void);
void __attribute__((weak, interrupt)) software_handler (void);
void __attribute__((weak, interrupt)) timer_handler (void);
void __attribute__((weak, interrupt)) external_handler (void);
void __attribute__((weak, interrupt)) default_vector_handler (void);
void __attribute__((weak)) default_exception_handler(void);

uint32_t plic_interrupt_lines[METAL_MAX_GLOBAL_EXT_INTERRUPTS];
uint32_t clint_interrupt_lines[__riscv_xlen];
uint32_t timer_isr_counter = 0;

/* Main - Setup CLINT interrupt handling and describe how to trigger interrupt */
int main() {

    uint32_t i, mode = MTVEC_MODE_CLINT_VECTORED;
    uintptr_t mtvec_base;

    /* Write mstatus.mie = 0 to disable all machine interrupts prior to setup */
    interrupt_global_disable();

    /* Setup mtvec to point to our exception handler table using mtvec.base,
     * and assign mtvec.mode = 1 for CLINT vectored mode of operation. The
     * mtvec.mode field is bit[0] for designs with CLINT, or [1:0] using CLIC */
    mtvec_base = (uintptr_t)&__mtvec_clint_vector_table;
    write_csr (mtvec, (mtvec_base | mode));

#if CLINT_PRESENT
    /* Get numeric list of CLINT interrupt lines and enable those at the CPU */
    for (i = 0; i < METAL_MAX_LOCAL_EXT_INTERRUPTS; i++) {
        clint_interrupt_lines[i] = __metal_driver_sifive_local_external_interrupts0_interrupt_lines(NULL, i);

        /* enable */
        interrupt_local_enable(clint_interrupt_lines[i]);
    }
#else
#error "This design does not have a CLINT...Exiting.\n");
    exit(0x77);
#endif

    /* enable software interrupts */
    interrupt_software_enable ();

    /* enable timer interrupts */
    SET_TIMER_INTERVAL_MS(DEMO_TIMER_INTERVAL);
    interrupt_timer_enable();

    /* Write mstatus.mie = 1 to enable all machine interrupts */
    interrupt_global_enable();

    /* Allow timer interrupt to fire before we continue, running at ~5s intervals */
    while (!timer_isr_counter);
    interrupt_timer_disable();

    /* write msip and display message that s/w handler was hit */
    write_word(MSIP_BASE_ADDR(read_csr(mhartid)), 0x1);

   exit (0);
}

/* External Interrupt ID #11 - handles all global interrupts */
void __attribute__((weak, interrupt)) external_handler (void) {

    /* The external interrupt is usually used for a PLIC, which handles global
     * interrupt dispatching.  If no PLIC is connected, then custom IP can connect
     * to this interrupt line, and this is where interrupt handling
     * support would reside.  This demo does not use the PLIC.
     */
}

void __attribute__((weak, interrupt)) software_handler (void) {

    uintptr_t mip, code = MCAUSE_CODE(read_csr(mcause));
    uintptr_t int_bit = read_csr(mip);

    /* Clear Software Pending Bit which clears mip.msip bit */
    write_word(MSIP_BASE_ADDR(read_csr(mhartid)), 0x0);
}

void __attribute__((weak, interrupt)) timer_handler (void) {

    uintptr_t code = MCAUSE_CODE(read_csr(mcause));
    uintptr_t mtime, mip;
    uintptr_t int_bit = read_csr(mip);

    /* set our next interval */
    SET_TIMER_INTERVAL_MS(DEMO_TIMER_INTERVAL);
}

void __attribute__((weak, interrupt)) default_vector_handler (void) {
    /* Add functionality if desired */
    while (1);
}

void __attribute__((weak)) default_exception_handler(void) {
    while (1);
}

void interrupt_global_enable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrs %0, mstatus, %1" : "=r"(m) : "r"(METAL_MIE_INTERRUPT));
}

void interrupt_global_disable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrc %0, mstatus, %1" : "=r"(m) : "r"(METAL_MIE_INTERRUPT));
}

void interrupt_software_enable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrs %0, mie, %1" : "=r"(m) : "r"(METAL_LOCAL_INTERRUPT_SW));
}

void interrupt_software_disable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrc %0, mie, %1" : "=r"(m) : "r"(METAL_LOCAL_INTERRUPT_SW));
}

void interrupt_timer_enable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrs %0, mie, %1" : "=r"(m) : "r"(METAL_LOCAL_INTERRUPT_TMR));
}

void interrupt_timer_disable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrc %0, mie, %1" : "=r"(m) : "r"(METAL_LOCAL_INTERRUPT_TMR));
}

void interrupt_external_enable (void) {
    uintptr_t m;
    __asm__ volatile ("csrrs %0, mie, %1" : "=r"(m) : "r"(METAL_LOCAL_INTERRUPT_EXT));
}

void interrupt_external_disable (void) {
    unsigned long m;
    __asm__ volatile ("csrrc %0, mie, %1" : "=r"(m) : "r"(METAL_LOCAL_INTERRUPT_EXT));
}

void interrupt_local_enable (int id) {
    uintptr_t b = 1 << id;
    uintptr_t m;
    __asm__ volatile ("csrrs %0, mie, %1" : "=r"(m) : "r"(b));
}

void interrupt_local_disable (int id) {
    uintptr_t b = 1 << id;
    uintptr_t m;
    __asm__ volatile ("csrrc %0, mie, %1" : "=r"(m) : "r"(b));
}
