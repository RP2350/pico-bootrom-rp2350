/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

.cpu cortex-m33
.thumb
.syntax unified

#include "bootrom.h"
#include "bootram.h"
#include "hardware/regs/accessctrl.h"
#include "hardware/regs/intctrl.h"
#include "hardware/regs/m33.h"

.section .text.s_native_crit_launch_nsboot
.global s_native_crit_launch_nsboot
.thumb_func
s_native_crit_launch_nsboot:
.p2align 2 // so adr is 2 byte - we should be aligned already
    // note r3 is MULTIPLEX_s_native_crit_launch_nsboot on entry
    movs r0, #M33_MPU_CTRL_PRIVDEFENA_BITS | M33_MPU_CTRL_ENABLE_BITS
    movw r7, P16(nsboot_vm_constants)
    ldmia r7!, {r4, r5, r6}
    // r4 = PPB_NONSEC_BASE + M33_VTOR_OFFSET
    // r5 = ACCESSCTRL_PASSWORD_BITS | ACCESSCTRL_UART0_CORE0_BITS | ACCESSCTRL_UART0_SP_BITS | ACCESSCTRL_UART0_NSP_BITS | ACCESSCTRL_USBCTRL_DBG_BITS
    // r6 = ACCESSCTRL_BASE + ACCESSCTRL_USBCTRL_OFFSET

    // install NS VTOR
    // ===============

    movw r1, #NSBOOT_START + NSBOOT_VTOR_OFFSET
    str r1, [r4]
    adds r4, #M33_MPU_CTRL_OFFSET - M33_VTOR_OFFSET

    // setup NS MPU to make everything beyond NSBOOT to end of address space X
    // =======================================================================
#if NSBOOT_END <= BOOTRAM_ARM_END
#error
#endif
    // note that NSBOOT END does not have to be on a 32 byte boundary, so we round up, and allow execution of the first
    // few words of RISC-V. Since this is not user writable, I think this is fine
#define NSBOOT_END_ROUND_UP ((NSBOOT_END + 31) & ~31u)
    movs r1, #0
    movw r2, #NSBOOT_END_ROUND_UP + M33_MPU_RBAR_XN_BITS
    subs r3, #(32 - M33_MPU_RLAR_EN_BITS + MULTIPLEX_s_native_crit_launch_nsboot)
    stmia r4!, {r0-r3}

#if (((SRAM_END - SRAM_BASE) << 16) & 0xffffffff) != SRAM_BASE
#error
#endif
    // set r0 = SRAM_BASE
    mov.w r2, #SRAM_END - SRAM_BASE
    lsls r0, r2, #16

    // check MPU is in place
    // =====================
    tta r4, r0
    mov.w r3, #0x4d0000
    rcp_iequal r4, r3

    // erase RAM
    // =========
    // r0 == SRAM_BASE
    // r1 == 0
    // r2 == SRAM_END - SRAM_BASE
    // has regular calling convention
    bl s_native_step_safe_crit_mem_erase_by_words_impl
    mov.w r3, #SRAM_END - SRAM_BASE
    rcp_iequal_nodelay r0, r3
    rcp_iequal_nodelay r3, r0

    // setup NS access ctrl and NS GPIO
    // ================================

    // r5 = ACCESSCTRL_PASSWORD_BITS | ACCESSCTRL_UART0_CORE0_BITS | ACCESSCTRL_UART0_SP_BITS | ACCESSCTRL_UART0_NSP_BITS | ACCESSCTRL_USBCTRL_DBG_BITS
    // r6 = ACCESSCTRL_BASE + ACCESSCTRL_USBCTRL_OFFSET

    // enable NS access to USB
    str r5, [r6]
#if MINI_PRINTF
    str r5, [r6, #ACCESSCTRL_UART0_OFFSET - ACCESSCTRL_USBCTRL_OFFSET]
#endif

    ldmia r7!, {r2, r3, r4}
    // r2 = PPB_BASE + M33_NVIC_ITNS0_OFFSET + 4 * (USBCTRL_IRQ / 32)
    // r3 = NSBOOT_RAM_START (nsboot_config)
    // r4 = SRAM_END - 32 + M33_SAU_RLAR_ENABLE_BITS

    // itns = 1 << (USBCTRL_IRQ % 32)
    movs r0, #1
    lsls r1, r0, #(USBCTRL_IRQ % 32)
    str r1, [r2]

    ldrb r1, [r3]
    lsls r2, r1, #32 - 6
    bcs nsboot_vm_no_gpio

    // enable NS access to IO bank 0
    str r5, [r6, #ACCESSCTRL_IO_BANK0_OFFSET - ACCESSCTRL_USBCTRL_OFFSET]
    bpl 1f
    // hi pin
    adds r6, #4
1:
    subs r6, #ACCESSCTRL_USBCTRL_OFFSET - ACCESSCTRL_GPIO_NSMASK0_OFFSET
    bics r1, #32
    lsls r0, r1
    // hw_set_bits(&accessctrl_hw->gpio_nsmask[gpio/32], 1u << (gpio & 0x1fu));
    // r6 = &accessctrl_hw->gpio_nsmask[gpio/32]
    str r0, [r6]
nsboot_vm_no_gpio:

    // Give NS access to RAM via SAU region 2
    // ======================================

    ldmia r7, {r2,r3,r5,r6,r7}
    msr msp_ns, r7

    // r2 = secure_stack
    // r3 = PPB_BASE + M33_SAU_RNR_OFFSET
    // r5 = XIP_SRAM_BASE
    // r6 = XIP_SRAM_END - 32 + M33_SAU_RLAR_ENABLE_BITS

    // r4 = SRAM_END - 32 + M33_SAU_RLAR_ENABLE_BITS
    // set r0 = 2
    // set r1 = SRAM_BASE
    lsrs r0, r4, #28
    lsls r1, r0, #28
    stmia r3, {r0, r1, r4}

    // Give NS access to XIP_SRAM via SAU region 3
    // ===========================================

    movs r1, #3
    // r5 = XIP_SRAM_BASE
    // r6 = XIP_SRAM_END - 32 + M33_SAU_RLAR_ENABLE_BITS
    stmia r3!, {r1, r5, r6}

    // r2 = secure_stack_base
    adds r1, r2, #BOOTRAM_NSBOOT_SECURE_STACK_SIZE
    // r5 = secure_stack_end

    // beginning of old s_arm8_usb_client_ns_call_thunk // (secure_stack_base, secure_stack_size)

    rcp_count_check STEPTAG_NSBOOT_OTP_ADVANCE // otp should have been advcance
    // Set secure stack limit. Note setting SPLIM to a value greater than SP
    // does not trigger a fault -- the fault is always on an SP-setting
    // instruction that sets SP < SPLIM.
    msr msplim, r2
    // Secure SP, presumed MSP as we do not set SPSEL. (mov sp is smaller than msr msp)
    mov sp, r1

    mov.w r1, #HX_BIT_PATTERN_FALSE
    str r1, [r2] // we steal the very bottom word of the stack for a flag (which is hx_false() in NSBOOT)

    // does not return
    movw r0, #NSBOOT_ENTRY_POINT & ~1
    // downgrade MPU at last minute (skipping these will cause crash)
#if BOOTROM_MPU_REGION_BOOTRAM_CORE1 != 0
#error
#endif
    // r3 = PPB_BASE + M33_SAU_RNR_OFFSET
    subs r3, #M33_SAU_RNR_OFFSET + 12 - M33_MPU_RNR_OFFSET
    // 1. clear region 0 which was protecting core1 region of bootram (will be used by nsboot stacks)

    // note we only require the bottom 3 bits of the write value to be 0, because
    // RNR has only 3 bits - we want to set region 0
    // RLAR we only care about bit 0 (enable)
#if (NSBOOT_ENTRY_POINT & 7) || (M33_MPU_RLAR_EN_BITS != 1) || (M33_MPU_RNR_BITS != 7)
#error
#endif
    str r0, [r3, #M33_MPU_RNR_OFFSET - M33_MPU_RNR_OFFSET] // note must set RNR to access regular RBAR register
    // RLAR we just care about clearing te enable bit
    str r0, [r3, #M33_MPU_RLAR_OFFSET - M33_MPU_RNR_OFFSET]

    // 3. re-enable execute of SG region
#if BOOTROM_MPU_REGION_SECURE_XN != 3
#error
#endif
    movw r1, #BOOTROM_SG_START - 32 + M33_MPU_RLAR_EN_BITS
    str r1, [r3, #M33_MPU_RLAR_A3_OFFSET - M33_MPU_RNR_OFFSET]

    rcp_count_check STEPTAG_NSBOOT_OTP_ADVANCE + 1
    // clear all regs
    ldmia r0, {r1-r12,lr}
    // hardening: done
    bxns  r0
.section .rodata.nsboot_vm_constants
.p2align 2
.global nsboot_vm_constants
nsboot_vm_constants:
    .word PPB_NONSEC_BASE + M33_VTOR_OFFSET
    .word ACCESSCTRL_PASSWORD_BITS | ACCESSCTRL_UART0_CORE0_BITS | ACCESSCTRL_UART0_SP_BITS | ACCESSCTRL_UART0_NSP_BITS | ACCESSCTRL_USBCTRL_DBG_BITS
    .word ACCESSCTRL_BASE + ACCESSCTRL_USBCTRL_OFFSET

    .word PPB_BASE + M33_NVIC_ITNS0_OFFSET + 4 * (USBCTRL_IRQ / 32)
    .word NSBOOT_RAM_START
    .word SRAM_END - 32 + M33_SAU_RLAR_ENABLE_BITS

// if 0 we can use the BOOTRAM_BASE constant from elsewhere
#if BOOTRAM_XIP_SETUP_CODE_OFFSET == BOOTRAM_NSBOOT_SECURE_STACK_OFFSET
.global xip_setup_func_ptr
xip_setup_func_ptr:
#endif
    .word BOOTRAM_BASE + BOOTRAM_NSBOOT_SECURE_STACK_OFFSET
    .word PPB_BASE + M33_SAU_RNR_OFFSET
    .word XIP_SRAM_BASE
    .word XIP_SRAM_END - 32 + M33_SAU_RLAR_ENABLE_BITS
    .word USBCTRL_DPRAM_BASE + USBCTRL_DPRAM_SIZE
