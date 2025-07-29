/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// ----------------------------------------------------------------------------
// Bootrom Runtime 0
// ----------------------------------------------------------------------------
// This is not a full crt0 -- in particular, no .bss or .data initialisation
// (use of .data/.bss is disallowed via linker script assertions).
// The bootrom is not permitted to use statically-allocated memory, as parts of
// it are called into by user code.
// The purpose of this file is:
// - Provide initial entry point for both cores
// - Provide holding pen and launch code for core 1
// - Provide direct-boot entry for core 0, mainly intended for running
//   ARM code during ATE
// - Pass core 0 control over to the main flash boot sequence
// - Thunks and other ARM only assembly functions

#include "bootram.h"
#include "bootrom.h"
#include "bootrom_layout.h"
#include "hardware/regs/accessctrl.h"
#include "hardware/regs/addressmap.h"
#include "hardware/regs/bootram.h"
#include "hardware/regs/clocks.h"
#include "hardware/regs/m33.h"
#include "hardware/regs/m33.h"
#include "hardware/regs/m33_eppb.h"
#include "hardware/regs/otp.h"
#include "hardware/regs/pads_bank0.h"
#include "hardware/regs/powman.h"
#include "hardware/regs/resets.h"
#include "hardware/regs/sio.h"
#include "hardware/regs/sysinfo.h"
#include "hardware/regs/trng.h"
#include "hardware/regs/watchdog.h"
#include "nsboot_secure_calls.h"

.cpu cortex-m33
.thumb
.syntax unified

.section .vectors, "ax"
.balign 2

.global __vectors
__vectors:
.word INVALID_STACK_PTR // Invalid stack; neither core should use any before setting their own SP
.word s_arm8_entry_point // Reset
// NMI goes to varm_dead which does an rcp hang which will now stall forever
.word varm_dead     // NMI
.word varm_dead     // HardFault

.global _magic
_magic:
// magic
.byte 'M', 'u'
_major_version:
.byte 2
_minor_version:
.byte 3

_arm_well_known_rom_table_base:
.hword BOOTROM_ROMTABLE_START
_arm_well_known_func_table_lookup_val:
.hword arm8_table_lookup_val + 1
_arm_well_known_func_table_lookup_entry:
.hword arm8_table_lookup_entry + 1

copyright:
.string "(C) 2024 Raspberry Pi Ltd"

.global cpacr
cpacr:
.word PPB_BASE + M33_CPACR_OFFSET

#define TT_VALUE_OFFSET 0x38
.global tt_value
tt_value:
.word 0x2c70703

// Args: 2-char symbol in r0, flag query in r1.
// Each table entry is a 2-char symbol, an hword of flags, and then 1 hword of
// data for each bit set in flags.
// Search the table for a symbol and a flag mask, and if *any* of those flags
// are found under the correct symbol, return a pointer to the first
// corresponding table data.
.global arm8_table_lookup_entry
.thumb_func
arm8_table_lookup_entry:
#if FEATURE_CANARIES
    rcp_count_set_nodelay STEPTAG_ARM_TABLE_LOOKUP_ENTRY
#endif
    ldr r3, =BOOTROM_ROMTABLE_START
    mov ip, r0
    b _lookup_next_symbol
#if USE_TINY_ROM_TABLE_LOOKUP || BOOTROM_32BIT_FUNC_POINTERS
1:
    adds r3, #2
_lookup_skip_to_next:
    lsrs r2, #1
    bcs 1b
    bne _lookup_skip_to_next
    adds r3, #4
#else
_lookup_skip_to_next:
    // Assume DATA and ARM_NONSEC entries never share a symbol.
    // Assume no long DATA or ARM_NONSEC entries.
    cmp r2, #0x10
    blo 1f
    adds r3, #2
1:
    // Assume no long ARM_SEC entries. (bit 3)
    ldr r0, =0x4a886866
    lsls r2, #2
    rors r0, r2
    lsrs r0, #28
    adds r3, r0
#endif
_lookup_next_symbol:
    ldrh r0, [r3]
    ldrh r2, [r3, #2]
    cbz r0, _lookup_return_ptr_r0
    cmp ip, r0
    bne _lookup_skip_to_next
    tst r2, r1
    beq _lookup_skip_to_next
_lookup_seek_result:
    ands r1, r2
1:
    lsrs r1, #1
    bcs _lookup_found
    lsrs r2, #1
    bcc 1b
    adds r3, #2
    b 1b
_lookup_found:
    adds r0, r3, #4
_lookup_return_ptr_r0:
#if FEATURE_CANARIES
    rcp_count_check_nodelay STEPTAG_ARM_TABLE_LOOKUP_ENTRY
#endif
    // fall thru
.global _noop, varm_noop
.thumb_func
varm_noop:
    // hardening: skip (previous instruction above is canary)
    bx lr

.global arm8_table_lookup_val
.thumb_func
arm8_table_lookup_val:
#if !FEATURE_CANARIES
    push {lr}
#else
    rcp_canary_get_nodelay r3, CTAG_ARM8_TABLE_LOOKUP_VAL
    push {r3, lr}
#endif
    bl arm8_table_lookup_entry
#if BOOTROM_32BIT_FUNC_POINTERS
    cbz r0, 1f
    ldr r0, [r0]
1:
#else
    // Note the first halfword of ROM is 0, so NULLs propagate naturally:
    ldrh r0, [r0]
#endif
#if !FEATURE_CANARIES
    pop {pc}
#else
    pop {r2, r3}
    rcp_canary_check_nodelay r2, CTAG_ARM8_TABLE_LOOKUP_VAL
    // hardening: done
    bx r3
#endif

// ----------------------------------------------------------------------------
// Entry point for both cores
// ----------------------------------------------------------------------------

.section .vectors, "ax"

.global s_arm8_entry_point
.type s_arm8_entry_point,%function
.thumb_func
s_arm8_entry_point:

    ldr r7, =SIO_BASE
    ldr r7, [r7, #SIO_CPUID_OFFSET] // r7 = core_num

    // Note on constants: we have
    //   ed08 VTOR
    //   ed94 MPU_CTRL
    //   ed88 CPACR
    //   edd0 SAU_CTRL
    // So to avoid near-misses in the literal pool, we always start by
    // materialising CPACR and then do arithmetic on that

    // enable canary processor
    // CPACR = M33_CPACR_CP7_BITS
    movs r2, #0 // done here not below for alignment

    .p2align 2 // this is the case
    // lets delay one core a cycle (requires alignment so nop is free on fall thru)
    // since you'd have to glitch both during the SAU checks below)
    cbz r7, 1f
    nop
1:
    ldr r0, [r2, #CPACR_OFFSET]
    movs r1, #M33_CPACR_CP7_BITS
    str r1, [r0]

    // we could do this here to trap if we didn't enable the RCP, but that is no better
    // really than trapping later - we still don't do a real RCP panic.
    // mrc p7, #1, r3, c0, c0, #0

    // good a place as any to enable it for NS too
    movs r3, #M33_NSACR_CP7_BITS
    str r3, [r0, #M33_NSACR_OFFSET - M33_CPACR_OFFSET]
    // NSACR = M33_NSACR_CP7_BITS
    add r3, r0, #PPB_NONSEC_BASE - PPB_BASE
    str r1, [r3]
    // NS.CPACR = M33_CPACR_CP7_BITS

enable_sau:

    // Enable SAU region 7, which will remain "reserved by us" through and
    // post boot. This sets everything past the end of the Secure-only text
    // to SAU-NS. When combined with the IDAU, the final attribute map is:
    //
    //   Start             End               Attribute (I)  Attribute (D)  Contents
    //   0000           -> 42ff              Exempt         Exempt         Shared .text
    //   4300           -> sonly_text_end-1  Secure         Exempt         Not-NonSecure .text
    //   sonly_text_end -> 7dff              NonSecure      Exempt         .rodata, RISC-V code
    //   7e00           -> 7fff              Secure NSC     Secure NSC     SGs and SG->S wrappers
    //
    // Notes:
    //
    // - The 4300 here is actually 9300 for 64k ROM development FPGA builds
    //
    // - We intended for the 4300 to be tie-cell-programmable, it's a bit of a
    //   funny story (got case-analysed during STA)
    //
    // - The "Not-NonSecure .text" has no security requirement to be
    //   Secure-only, it's just that we can't make all of our text shared due
    //   to lack of IDAU Exempt->S watermark flexibility, so we have to keep
    //   shared text (such as ROM table lookup func) out of this region
    //
    // - The difference between fetch and load/store was a late-in-the-day
    //   synthesis hack, and isn't supposed to exist in v8-M (sorry Joseph).
    //   Having all of ROM be Exempt for load/stores is fine, we just don't
    //   want all of it to all be Secure-executable, since this would mean a
    //   larger ROP surface.

    // r0 = PPB_CPACR
    adds r0, #M33_SAU_CTRL_OFFSET - M33_CPACR_OFFSET
    // r0 = PPB_SAU_CTRL

    // we will do an stmia to set all 5 registers (not we write garbage to SAU_TYPE which is read-only)
#if M33_SAU_TYPE_OFFSET-M33_SAU_CTRL_OFFSET != 4 || M33_SAU_RNR_OFFSET-M33_SAU_CTRL_OFFSET != 8 || \
    M33_SAU_RBAR_OFFSET-M33_SAU_CTRL_OFFSET != 12 || M33_SAU_RLAR_OFFSET-M33_SAU_CTRL_OFFSET != 16
#error expected 5 registers in order
#endif
    movs r1, #M33_SAU_CTRL_ENABLE_BITS; // ALLNS=0 ENABLE=1
    movs r3, #7 // RNR
    // note we can't add 7 in the constant as it is a P16_ index
    movw r4, #P16(sonly_text_end_plus_7) // RBAR we have 0b111 or-ed into this, since those bits are RES0 and won't be read back below
    // note `+` because this is a linker relocation:
#if BOOTROM_SG_END & 31
#error SG_END should be 32 byte aligned
#endif
    ldr  r5, =BOOTROM_SG_END - 32 + M33_SAU_RLAR_ENABLE_BITS
    stmia r0, {r1-r5}

    // r3 is loop counter (which is 7 - the region number)... if that was not initialized,
    // then we may loop forever, but also the region won't have been set correctly

    // Note: the first beq is false due to movs above clearing the Z flag.

    // note r2 should still be 0
check_sau_loop:
    movw r10, P16(sonly_text_end)
    ldr r4, [r0, #12]
    cmp r4, r10
    beq 1f
0:
    rcp_panic
1:
    // we can only test in the SG region; everything else in the bootrom returns 004c0000

    // note tt must be at least 1 cycle away from the stmia above as we have no explicit barrier for space

    // removed this code, as if r5 is not set correctly; it should
    // be impossible to get the correct value
    tt r2, r5
    mov r4, r2
    lsrs r2, r2, #16 // if we skip the tt multiple times (or skipped first tt), r2 will be zero

    // removed to save space
//    // note double check of Z flag
//    cmp r2, #0
//    bne 1f
//    rcp_panic
//1:
    beq 0b

    ldr r6, initial_sau_value
    cmp r4, r6
    beq 1f // opposite test to above
    rcp_panic
1:
    // Test bit 25: should be set (expected value: 0x02ce0700)
    lsls r6, r4, #6
    bmi 1f
    rcp_panic
1:
    ldr r6, initial_sau_value
    subs r6, r4
    beq 1f
    rcp_panic
1:
    adds r3, r6
    lsls r4, #6
    // removed to save space
//    bmi 1f
//    rcp_panic
//1:
    bpl 0b
    subs r3, #1
    cmp r3, #0
    bne check_sau_loop
    cmp r3, #0
    beq 1f
    rcp_panic
1:

#if HACK_RAM_BOOTROM_AT
    // todo this is likely now bit-rotted
    ldr r0, =PPB_BASE + M33_VTOR_OFFSET
    adds r0, #M33_SAU_CTRL_OFFSET - M33_VTOR_OFFSET
    // Second region is required to get SGs to be NSC, as there are no IDAU
    // entries over SRAM.
    movs r1, #6
    str  r1, [r0, #M33_SAU_RNR_OFFSET-M33_SAU_CTRL_OFFSET]
    // Note overlap is always SAU-NSC (which, if IDAU is NS, is NSC overall)
    ldr  r1, =BOOTROM_SG_START
    ldr  r2, =BOOTROM_SG_END - 32 + M33_SAU_RLAR_ENABLE_BITS + M33_SAU_RLAR_NSC_BITS
    str  r1, [r0, #M33_SAU_RBAR_OFFSET-M33_SAU_CTRL_OFFSET]
    str  r2, [r0, #M33_SAU_RLAR_OFFSET-M33_SAU_CTRL_OFFSET]
#endif

#if (M33_MPU_CTRL_PRIVDEFENA_BITS | M33_MPU_CTRL_ENABLE_BITS) != 5
#error
#endif

enable_mpu:
    subs r0, #M33_SAU_CTRL_OFFSET - M33_MPU_CTRL_OFFSET
    //
    adr r6, bootrom_mpu_regs
    ldmia r6, {r1, r2, r3, r4, r5, r8, r9, r10, r11, ip}
    stmia r0, {r1, r2, r3, r4, r5, r8, r9, r10, r11, ip}
check_mpu:
    // note r5 is a large number from above if this is skipped
    movs r5, #5 // note 5 is also the enable value
    mov ip, r5
check_mpu_loop1:
    adr r6, bootrom_mpu_regs + 8
    movs r3, #0xf0 // we start with something that won't neatly count up to BOOTROM_MPU_REGION_COUNT
check_mpu_loop2:
    ldmia r6!, {r1, r4}
    // r1 = expected rbar
    // r4 = expected rlar
    str r3, [r0, #4] // region number possibly with extra bits, read back below unmasked
    ldmia r0, {r2, r3, r10, r11}
    // r2 = ctrl
    // r3 = rnr
    // r10 = rbar
    // r11 = rlar
    cmp r2, ip // MPU enable
    beq 1f
    rcp_panic
1:
    cmp r0, #0
    cmp r1, r10 // RBAR correct
    beq 1f
    rcp_panic
1:
    subs r4, #0x10 // correct for the bit we set which is RES0 in the RLAR
    cmp r4, r11 // RLAR correct
    beq 1f
    rcp_panic
1:
    adds r3, #1
    cmp r3, #BOOTROM_MPU_REGION_COUNT
    bne check_mpu_loop2 // do another check
    beq 1f
    rcp_panic
1:
    adds r5, r3
    subs r5, #BOOTROM_MPU_REGION_COUNT + 1
    bne check_mpu_loop1
    cmp r5, #0
    beq 1f
    rcp_panic
1:

    // r5 should be zero
    ldr r3, [r5, #TT_VALUE_OFFSET]
    tt r4, r4
    movs r2, #1
    cmp r4, r3
    beq 1f
    rcp_panic
1:
    movs r2, #1
    cmp r4, r3
    beq 1f
    rcp_panic
1:

    // Check if this is core 0, and go to holding pen if not enter main booth path
check_core:
    // NOTE: We DO NOT use any stack prior to possible watchdog entry
    cbnz r7, core1_boot_path

set_secure_stack:
    // r3 is currently invalid (mostz likely 0x2c70703)
    ldr r3, =BOOTRAM_PREBOOT_STACK_TOP
    subs r0, r3, #BOOTRAM_PREBOOT_STACK_SIZE
    msr msplim, r0
#if !(BOOTRAM_PREBOOT_STACK_TOP & 4)
#error expect un-aligned stack top for now to keep stack alignment once we enter C code
#endif

zero_secure_stack:
    // zero bootram up to preboot stack top (i.e. everything except core1 and always)
    // note r5 should be 0, but could be 0x01 0x02 0x3 or 0x04 0x05 if the check_mpu_loop1 check is skipped above
1:
    stmia r0!, {r5}
    cmp r0, r3
    bne 1b
    // since we set sp to r0, there is a good chance that the stack was cleared, or that
    // we will subsequently overflow the stack.
    mov sp, r0

    // continue in C code (note the weird call is because we skip the callee-saving which wastes
    // our limited stack space for a function that will never return
    b s_varm_crit_core0_boot_path_entry_p2 - 2

#define MPU_REGION_RO_XN(n, rbar, rlar) \
   .word rbar + M33_MPU_RBAR_XN_BITS + (2 << M33_MPU_RBAR_AP_LSB), \
    (rlar) + M33_MPU_RLAR_EN_BITS + 0x10 // note 0x10 will be written but not read back
.p2align 2
bootrom_mpu_regs:
    /* ctrl */ .word M33_MPU_CTRL_PRIVDEFENA_BITS | M33_MPU_CTRL_ENABLE_BITS
initial_sau_value:
    /* rnr */  .word 0x02ce0700 // note RNR is only 3 bits, so this value is 0 for that purpose (we save 4 bytes by putting constant here)
#if BOOTROM_MPU_REGION_BOOTRAM_CORE1 != 0 || \
    BOOTROM_MPU_REGION_RAM != 1 || \
    BOOTROM_MPU_REGION_FLASH != 2 || \
    BOOTROM_MPU_REGION_SECURE_XN != 3
#error mpu regions should be in order
#endif
    // note this range isn't exact due to alignment (we round inwards) but it is still between the core 0 stack and our always data
    MPU_REGION_RO_XN(BOOTROM_MPU_REGION_BOOTRAM_CORE1,
                     (BOOTRAM_BASE + BOOTRAM_RUNTIME_CORE1_OFFSET + HACK_STACK_WORDS * 4 + 0x1fu) & ~0x1fu,
                     (BOOTRAM_BASE + BOOTRAM_ALWAYS_OFFSET - 0x1fu) & ~0x1fu)
    MPU_REGION_RO_XN(BOOTROM_MPU_REGION_RAM,
                     SRAM_BASE,
                     SRAM_END - 0x20)
    MPU_REGION_RO_XN(BOOTROM_MPU_REGION_FLASH,
                     XIP_BASE,
                     SRAM_BASE - 0x20)
    // note this must be at the end as we don't reset RNR before updating it for core1
    // (we move the base to _end_of_core1_boot_path)
    MPU_REGION_RO_XN(BOOTROM_MPU_REGION_SECURE_XN,
                     __start_of_secure_xn,
                     // note we make all of SG XN as well since we dont need it until NS BOOT
                     // except for a little bit at the end. (note we want the round down, which is
                     // the first -32, then -0x20 for inclusive)
                     __sg_bootpath_needed_start_roundup_32 - 32 - 0x20)
mpu_regions_end:

// ----------------------------------------------------------------------------
// Core 1 boot path
// ----------------------------------------------------------------------------

// This code does not use any stack, and runs with the invalid stack pointer
// from the vector table, to ensure that not even faults cause RAM writes,
// until we have been given a valid stack by core 0. A fault before that point
// will fail to write its exception frame, and be promoted to a lockup.

// This code also runs with eXecute-Never (XN) permissions on all of memory
// except for the small part of the ROM containing this code. The restriction
// is lifted at the point we commit to launching core 1 into user code.

// takes r4 = SIOB_BASE
// returns r6 = word received
.thumb_func
receive_and_check_zero:
#if FEATURE_CANARIES
    rcp_canary_get_nodelay ip, CTAG_RECEIVE_AND_CHECK_ZERO
#endif
receive_and_check_zero_lp:
    wfe
    ldr r6, [r4, #SIO_FIFO_ST_OFFSET]
    lsrs r6, #SIO_FIFO_ST_VLD_LSB + 1
    bcc receive_and_check_zero_lp

    ldr r6, [r4, #SIO_FIFO_RD_OFFSET]
    // if we received 0, we reset back to main loop
    cbz r6, core_0_handshake_loop
#if FEATURE_CANARIES
    rcp_canary_check_nodelay ip, CTAG_RECEIVE_AND_CHECK_ZERO
#endif
    // hardening: done
    bx  lr

send_and_then_again:
    // in case of multiple core 1 resets, we can keep pushing and fill the FIFO
    // we should wait for an event if the FIFO is full to avoid busy wait
    wfe
// takes r6 = word to send, r4 = SIOB_BASE, r5 link register
send_and_then:
#if FEATURE_CANARIES
    // note nodelay as we loop over it, and we could easily expose a lot of delay timings
    rcp_canary_get_nodelay ip, CTAG_SEND_AND_THEN
#endif
    ldr r1, [r4, #SIO_FIFO_ST_OFFSET]
    lsrs r1, #SIO_FIFO_ST_RDY_LSB + 1
    bcc send_and_then_again
    str r6, [r4, #SIO_FIFO_WR_OFFSET]
    sev
#if FEATURE_CANARIES
    rcp_canary_check_nodelay ip, CTAG_SEND_AND_THEN
#endif
    // hardening: done
    bx  r5

// First step: block until core 0 initialises this core's RCP salt value.
// (Note if core 0 is RISC-V then core 1's RCP salt will be unconditionally
// marked as valid, since RISC-V cores can't access the coprocessors.)

wait_for_rcp_salt:
    wfe
core1_boot_path:
    // r0 = MPU_CTRL
    // make more of bootrom RO XN
    movw r2, #P16(_end_of_core1_boot_path_roundup_32_plus_5)
    str r2, [r0, #M33_MPU_RBAR_OFFSET - M33_MPU_CTRL_OFFSET]

#if !NO_EXTRA_SAU_MPU_CHECKS
    // r5 should be 0 if the check loop of SAU completed
    ldr r3, [r5, #TT_VALUE_OFFSET]
#endif
    // Keep SCB pointer in r7
    sub r7, r0, #(M33_MPU_CTRL_OFFSET - M33_VTOR_OFFSET)

    // The rcp.canarystatus instruction returns hx_true (0xa500a500) if the
    // salt has been loaded, and hx_false (0x00c300c3) if it has not:
    mrc p7, #1, APSR_nzcv, c0, c0, #0
    bpl wait_for_rcp_salt
    // Note RCP instructions after this point will fault if the RCP is not
    // initialised, so the branch above is just a friendly check.

// Wait for core 0 to provide an entry point.
wait_for_vector:
    ldr r4, =SIO_BASE
    // Enable SCR.SLEEPDEEP before WFE -- this allows NVIC to be fully gated
    // during sleep, as well as releasing the system-level clock request.
    movs r1, #M33_SCR_SLEEPDEEP_BITS
    str r1, [r7, #(M33_SCR_OFFSET - M33_VTOR_OFFSET)]
    // note core_0_handshake_loop is the intended next instruction, but the read is harmless
    // as we're about to drain, so don't waste an instruction branching
1:
    ldr r1, [r4, #SIO_FIFO_RD_OFFSET]
core_0_handshake_loop:
    rcp_count_set STEPTAG_ASM_C1_BOOTPATH + 0

#if !NO_EXTRA_SAU_MPU_CHECKS
    // note this is hopefully superfluous due to checks above so can be considered for removal if we need space
    movw r1, #BOOTROM_SG_START
    tt r2, r1
    rcp_iequal r2, r3
#endif

    // drain the FIFO before sending 0
    ldr r1, [r4, #SIO_FIFO_ST_OFFSET]
    lsrs r1, #SIO_FIFO_ST_VLD_LSB + 1
    bcs 1b

    // ...and_then = receive_and_check_zero (which jmps to core_0_handshake_loop on 0)
    adr r5, receive_and_check_zero
    // send 0
    movs r6, #0
    bl send_and_then
    rcp_count_check STEPTAG_ASM_C1_BOOTPATH + 0
    // check for cmd 1
    cmp r6, #1
    bne core_0_handshake_loop
    // ack and receive VTOR
    bl send_and_then
    rcp_count_check STEPTAG_ASM_C1_BOOTPATH + 1
    str r6, [r7, #(M33_VTOR_OFFSET - M33_VTOR_OFFSET)]
    // ack and receive SP
    bl send_and_then
    rcp_count_check STEPTAG_ASM_C1_BOOTPATH + 2
    // initialize
    mov sp, r6
    bl send_and_then
    rcp_count_check STEPTAG_ASM_C1_BOOTPATH + 3
    adr r5, core1_launch
    // receive IP (0 sends us back into handshake loop)
    bl send_and_then
.thumb_func
core1_launch:
    // Disable SLEEPDEEP before exiting, as it affects wake latency
    movs r1, #0
    str r1, [r7, #(M33_SCR_OFFSET - M33_VTOR_OFFSET)]
    // Zero out MPU (can't use memset as MPU has XN on that part of bootrom)
    movs r2, #2 + BOOTROM_MPU_REGION_COUNT * 2
1:
    stmia r0!, {r1}
    subs r2, #1
    bne 1b
    bne 1b

    // Enter user-supplied vector
    rcp_count_check STEPTAG_ASM_C1_BOOTPATH + 4
    // hardening: done
    blx r6

// Low power hang on return. Reset the core if you want to provide another entry point
// (There is no need to return though)
//
// alternatively you could return directly to wait_for_vector (available in the function table)
// if you know core 1 is still in a good state
    b.n varm_dead_quiet

// ----------------------------------------------------------------------------
// Past this point is not executable by core 1 until it receives a vector.
.global _end_of_core1_boot_path
_end_of_core1_boot_path:

// ----------------------------------------------------------------------------
// Trampolines and trampoline-shaped objects
// ----------------------------------------------------------------------------

// note we do the copy here in assembly, so we don't need to set aside space for stage2 during the preboot stage (and
// so have more stack then), and here we reset the stack now we're comitted to entering the flash code without
// returning to bootrom C code

.section .text.s_native_crit_init_default_xip_setup_and_enter_image_thunk

// inline version of VARM_TO_PREAMBLE which saves us 2 bytes as we're in the same section
.global varm_to_s_native_crit_init_default_xip_setup_and_enter_image_thunk
.thumb_func
varm_to_s_native_crit_init_default_xip_setup_and_enter_image_thunk:
varm_hint HINT_s_native_crit_init_default_xip_setup_and_enter_image_thunk

.global s_native_crit_init_default_xip_setup_and_enter_image_thunk
s_native_crit_init_default_xip_setup_and_enter_image_thunk:
    ldrd r9, r10, [sp, #0]
    // r0 - XIP mode enum
    // r1 - XIP clkdiv
    // r2 = pc
    // r3 = sp
    // r9 = sp_lim
    // r10 = vector_table

    // double check the entry-point again the image_def stored in the parsed_block_loop
    ldr r5, [sp, #8] // parsed_block_loop
    ldr r5, [r5, #PARSED_BLOCK_LOOP_IMAGE_DEF_ROLLED_ENTRY_POINT_ADDR_OFFSET]
    rcp_iequal r2, r5

    // Note this wipes our stack, which is ok since we aren't returning. Don't
    // call memset, because it may save the callee saves on the stack and then
    // trash them. (Environmental storytelling in code comments!)
    ldr r4, =BOOTRAM_BASE
    add r5, r4, #BOOTRAM_SIZE - BOOTRAM_ALWAYS_SIZE
    movs r6, #0
1:
    stmia r4!, {r6}
    cmp r4, r5
    blo 1b
    rcp_iequal r4, r5

    // ---------------- DO NOT CALL ANY FUNCTIONS USING STACK BELOW THIS POINT  ----------------
    // We have just erased the stack, and are about to copy XIP setup into former stack space.

    // Clear USB RAM -- anything left here by nsboot will not have been verified or cleared. Skip
    // clearing when secure boot is disabled, as a courtesy to core 1 running as USB debug probe
    // (the upper ~3k is already trashed, but 1k is enough to use the peripheral)
#if BOOTRAM_ALWAYS_OFFSET != BOOTRAM_SIZE - BOOTRAM_ALWAYS_SIZE
#error
#endif
    ldr r4, [r5, #BOOTRAM_ALWAYS_SECURE_OFFSET - BOOTRAM_ALWAYS_OFFSET]
    // need to set flags, and take the opportunity to XOR out the "secure" XOR
    eors r4, #HX_XOR_SECURE
    // branch if secure (top bit set)
    bmi 1f
    // if we're skipping the clear, assert this is actually the secure "false" value
    rcp_bfalse r4
    b 2f
1:
    ldr r4, =USBCTRL_DPRAM_BASE
    adds r5, r4, #USBCTRL_DPRAM_SIZE
    // r6 is still 0 at this point
1:
    stmia r4!, {r6}
    cmp r4, r5
    blo 1b
    rcp_iequal r4, r5
2:

    ldr r4, =(PPB_BASE + M33_VTOR_OFFSET)
    str r10, [r4]

    // Copy default xip setup code into boot ram -- this code just restores the XIP mode found by try_flash_boot.
    ldr r7, =BOOTRAM_BASE + BOOTRAM_XIP_SETUP_CODE_OFFSET
    movw r6, P16(s_native_default_xip_setup)
#if DEFAULT_ARM_XIP_SETUP_SIZE_BYTES == 12
    ldmia r6, {r4-r6}
    stmia r7!, {r4-r6}
    stmia r7!, {r0, r1}
#else
#warning "You should probably update the specialised version"
    adds r5, r7, #DEFAULT_ARM_XIP_SETUP_SIZE_BYTES
1:
    ldmia r6!, {r4}
    stmia r7!, {r4}
    cmp r7, r5
    blo 1b
    // Write arguments for s_varm_flash_select_xip_read_mode after the default XIP code
    stmia r7!, {r0, r1}
#endif

    // enter binary
    msr msplim, r9
    msr msp, r3
#if FEATURE_CANARIES
    rcp_count_check STEPTAG_S_VARM_CRIT_RAM_TRASH_LAUNCH_IMAGE_THUNK_BASE
#endif
    blx r2 // we don't expect the binary to return, but safe for us to hang (slightly better than no link register)

.global native_wait_rescue, varm_wait_rescue
.global varm_dead_quiet
.type native_wait_rescue,%function
.type varm_wait_rescue,%function
.type varm_dead_quiet,%function
.thumb_func
native_wait_rescue:
varm_wait_rescue:
varm_dead_quiet:
    cpsid i // disable IRQs
    // fall thru
// Die without taking out the opposite core:
1:
    wfi
    b 1b
    // Fall through if branch fails

.global native_dead, varm_dead
.type native_dead,%function
.type varm_dead,%function
.thumb_func
native_dead:
varm_dead:
#if BREAKPOINT_AT_DEAD
    bkpt #0
#endif
    rcp_panic
#if BOOTROM_HARDENING
    rcp_panic
#endif

// This entry point is called from the secure gateway under ARM
// on Entry r0, r1, r2 args are stacked (along with a dummy reg and lr)
// r0 == ip, r1==ip
.global s_from_nsboot_service_call
.thumb_func
s_from_nsboot_service_call:
#if FEATURE_CANARIES
    rcp_count_set_nodelay STEPTAG_S_FROM_NS_NSBOOT_SERVICE_CALL
#endif
    rcp_canary_check_nodelay r2, CTAG_SG_CALL
    // under ARM we need to make sure we have not taken a boot path other than NSBOOT (at which point this API is not allowed)
    ldr r2, =BOOTRAM_BASE + BOOTRAM_WRITE_ONCE0_OFFSET
    ldr r1, [r2]
    ldr r0, =BOOTRAM_BASE + BOOTRAM_WRITE_ONCE0_OFFSET
    eors r2, r0 // r2 should be zero
    ldr r0, [r0]
    cmp r3, #SC_max_secure_call_num + 1
    adcs r0, r2
    rcp_iequal r0, r1 // two reads are the same (and we didn't exceed max number)
    orrs r0, r1
#if BOOT_ONCE_NSBOOT_API_DISABLED == 0
    lsls r0, #31 // 16-bit encoding
#else
    ands r0, r0, #(1u << BOOT_ONCE_NSBOOT_API_DISABLED)
#endif
1:
    rcp_iequal r0, r2 // boot_once_disabled bit == false
    ldr r2, =BOOTRAM_BASE + BOOTRAM_NSBOOT_SECURE_STACK_OFFSET
    ldr r2, [r2, r0]
    rcp_bfalse r2
    rcp_bfalse r2
    pop {r0, r1, r2}

// RISC-V code enters here under varmulet; we use .cpu to make sure everything other than RCP instructions are v8-M baseline
.cpu cortex-m23
.global s_from_nsboot_varm_service_call_no_boot_once_check
.thumb_func
s_from_nsboot_varm_service_call_no_boot_once_check:
    push {r2, r3, lr}
#if FEATURE_CANARIES
.cpu cortex-m33
    rcp_canary_get_nodelay r2, CTAG_S_FROM_NS_NSBOOT_SERVICE_CALL
    str r2, [sp, #4]
.cpu cortex-m23
#endif
#if SC_max_secure_call_num > 15
#error
#endif
    lsls r3, #28
#if BOOTROM_32BIT_FUNC_POINTERS
    lsrs r3, #26
#else
    lsrs r3, #27
#endif
    cmp r3, #varm_from_nsboot_func_table_end - varm_from_nsboot_func_table
    movw r2, #P16(varm_from_nsboot_func_table)
    bhs.n varm_dead // note we checked this before in the ARM case
    // we don't need to preserve r3, but we maintain stack alignment, and provide a place for our canary
#if BOOTROM_32BIT_FUNC_POINTERS
    ldr r3, [r2, r3]
#else
    ldrh r3, [r2, r3]
#endif
    ldr r2, [sp]
#if FEATURE_CANARIES
    // Assert that we ran the boot-once check on Arm
.cpu cortex-m33
    rcp_count_check_nodelay STEPTAG_S_FROM_NS_NSBOOT_SERVICE_CALL
.cpu cortex-m23
#endif
    blx r3
#if FEATURE_CANARIES
    pop {r2, r3}
.cpu cortex-m33
    rcp_canary_check_nodelay r3, CTAG_S_FROM_NS_NSBOOT_SERVICE_CALL
    // note under ARM our SG caller takes care of clearing r1-r3
.cpu cortex-m23
    // hardening: done
    pop {pc} // note we are returning a canary here in r3, however this should be cleared on return to NS
#else
    pop {r2, r3, pc}
#endif

.cpu cortex-m33
// Single function table, shared by Arm nsboot SG, and RISC-V SG hint.
// The order of this table is determined by the SC_xxx constants in nsboot_secure_calls.h.
// note: where these have from from_nsboot, or from_ns, the raw secure function performs
//       parameter checking or passes some context about the caller to the internal method
#if BOOTROM_32BIT_FUNC_POINTERS
.macro .varm_funcptr x
.word \x
.endm
.p2align 2
#else
.macro .varm_funcptr x
// note: +1 for thumb bit as we are using 16-bit pointers
.hword \x + 1
.endm
#endif

// put this in ASM, since i saw the GCC wasted a bunch of space formulating the table address
// int s_arm8_api_set_ns_api_permission(uint api_num, bool enabled)
// r0: api index
// r1: allowed (bool)
.global s_arm8_api_set_ns_api_permission
s_arm8_api_set_ns_api_permission:
#if FEATURE_CANARIES
    rcp_count_set_nodelay STEPTAG_S_VARM_API_SET_NS_API_PERMISSION
#endif
    movs   r3, #0xc3
    cbz r1, 1f
    movs   r3, #0xa5
1:
    ldr  r2, =BOOTRAM_BASE + BOOTRAM_NS_API_PERMISSIONS_OFFSET
#if HX_BIT_PATTERN_TRUE != 0xa500a500 || HX_BIT_PATTERN_FALSE != 0x00c300c3
#error
#endif
#if BOOTROM_NS_API_COUNT != 8
#error
#endif
    lsrs r1, r0, #3
    ite eq
    strbeq r3, [r0, r2]
    movne r1, #-BOOTROM_ERROR_INVALID_ARG
    negs r0, r1
#if FEATURE_CANARIES
    rcp_count_check_nodelay STEPTAG_S_VARM_API_SET_NS_API_PERMISSION
#endif
    // hardening: done
    bx lr

.global s_from_ns_arm8_api_secure_call
// call number passed in r4 (can't use IP as it is used by SG)
s_from_ns_arm8_api_secure_call:
#if FEATURE_CANARIES
    rcp_count_set_nodelay STEPTAG_S_FROM_NS_ARM8_API_SECURE_CALL
#endif
    // ip is actually the addresss of this function on entry
    ldr ip, =BOOTRAM_BASE
    ldr ip, [ip, #BOOTRAM_ALWAYS_CALLBACKS_OFFSET + BOOTROM_API_CALLBACK_secure_call * 4]
    cmp ip, #0
    itt eq
    ldreq r0, =BOOTROM_ERROR_INVALID_STATE
    moveq ip, lr
#if FEATURE_CANARIES
    rcp_count_check_nodelay STEPTAG_S_FROM_NS_ARM8_API_SECURE_CALL
#endif
    bx ip

.global s_varm_step_safe_reset_unreset_block_wait_noinline
.type s_varm_step_safe_reset_unreset_block_wait_noinline,%function
.thumb_func
s_varm_step_safe_reset_unreset_block_wait_noinline:
#if FEATURE_CANARIES
    // use regular canary as we are called from boot path
    rcp_canary_get ip, CTAG_S_VARM_UNRESET_BLOCK_WAIT_NOINLINE
#endif
    ldr r1, =RESETS_BASE + REG_ALIAS_SET_BITS
    str r0, [r1, #RESETS_RESET_OFFSET]
    // fall thru

// seems unused -- move above canary_get down if this entry point is reinstated.
// .global s_varm_step_safe_unreset_block_wait_noinline
// .type s_varm_step_safe_unreset_block_wait_noinline,%function
// .thumb_func
// s_varm_step_safe_unreset_block_wait_noinline:
    ldr r1, =RESETS_BASE + REG_ALIAS_CLR_BITS
    str r0, [r1, #RESETS_RESET_OFFSET]
    // Remove alias bits (note we're avoiding v8-M Main instructions here)
    lsrs r1, #14
    lsls r1, #14
1:
    ldr r2, [r1, #RESETS_RESET_DONE_OFFSET]
    bics r0, r2
    bne 1b
#if FEATURE_CANARIES
    rcp_canary_check ip, CTAG_S_VARM_UNRESET_BLOCK_WAIT_NOINLINE
#endif
    // hardening: done
    bx lr

// we clear USB SRAM (aka .bss and stack), and switch stack
.global varm_to_s_native_secure_call_pc_sp
.thumb_func
varm_to_s_native_secure_call_pc_sp: // (pc, sp or 0, a, b)
    eors r2, r3
    movs r3, #MULTIPLEX_s_native_secure_call_pc_sp
    varm_hint HINT_MULTIPLEX
//.global s_varm_secure_call // same function but called directly under varmulet
//.thumb_func
//s_varm_secure_call:
    push {r4, r5, r6, lr} // maintain stack align to call into user code
#if FEATURE_CANARIES
    rcp_canary_get_nodelay r6, CTAG_S_VARM_SECURE_CALL // used to check after call returns
    push {r6, r7} // maintain stack align to call into user code
    // canary value is now at [sp, #0]
#endif
    mov r4, sp
    mrs r5, msplim
    cbz r1, 1f // 0 sp means use current
    // Clear splim before setting sp, since our splim is likely higher than their sp
    movs r6, #0
    msr msplim, r6
    mov sp, r1
1:
#if FEATURE_CANARIES
    rcp_count_check_nodelay STEPTAG_S_VARM_SECURE_CALL
#endif
    rcp_iequal r0, r2
    // This will trap if you didn't set the Thumb bit! (Deliberate safeguard
    // against accidentally entering a RISC-V function pointer under Arm)
    // This also deliberately causes a trap with an invalid PC (e.g. 0)

    blx r0

    msr msplim, r5
    mov sp, r4
#if !FEATURE_CANARIES
    pop {r4, r5, r6, pc}
#else
    // canary is popped into r2, r3 is trashed, r4-r6  get their values
    pop {r2, r3, r4, r5, r6}
    rcp_count_set STEPTAG_STEP8_TRY_VECTOR // used in the boot path
    rcp_canary_check_nodelay r2, CTAG_S_VARM_SECURE_CALL
    pop {pc}
#endif


// DON'T FORGET TO UPDATE THE LINKER SCRIPT TOO!
VARM_TO_MULTIPLEX_PREAMBLE(s_native_crit_launch_nsboot)
VARM_TO_MULTIPLEX_PREAMBLE(s_native_crit_xip_cache_maintenance)


#if !USE_64K_BOOTROM
.section .sg_fillers
#else
.section .vectors, "ax"
#endif

.section .rodata.keep
.p2align 2
.global partition_table_ptr
partition_table_ptr:
.word BOOTRAM_BASE + BOOTRAM_ALWAYS_PARTITION_TABLE_OFFSET

.global flash_devinfo16_ptr
flash_devinfo16_ptr:
.word BOOTRAM_BASE + BOOTRAM_ALWAYS_FLASH_DEVINFO_OFFSET

// if 0 we can use the BOOTRAM_BASE constant from elsewhere
#if 1 || BOOTRAM_XIP_SETUP_CODE_OFFSET != 0
xip_setup_func_ptr:
.word BOOTRAM_BASE + BOOTRAM_XIP_SETUP_CODE_OFFSET
#endif

.global varm_from_nsboot_func_table
varm_from_nsboot_func_table:
    .varm_funcptr s_varm_flash_abort_clear// s_varm_api_crit_connect_internal_flash
    .varm_funcptr s_from_nsboot_varm_flash_page_program
    .varm_funcptr s_from_nsboot_varm_flash_sector_erase
    .varm_funcptr s_from_nsboot_varm_flash_read_data
    .varm_funcptr s_varm_flash_abort
    .varm_funcptr s_from_nsboot_varm_reboot
    .varm_funcptr s_from_nsboot_varm_otp_access
    .varm_funcptr s_from_nsboot_varm_ram_trash_get_uf2_target_partition
    .varm_funcptr s_from_ns_varm_api_get_partition_table_info
    .varm_funcptr s_from_ns_varm_api_get_sys_info
#if FEATURE_EXEC2
    .varm_funcptr s_from_ns_varm_picoboot_exec2
#endif
varm_from_nsboot_func_table_end:
