/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

.cpu cortex-m33
.thumb
.syntax unified

#include "bootrom.h"
#include "hardware/regs/accessctrl.h"

// Validate a NonSecure buffer.
//
// Entire buffer must fit in range XIP_BASE -> SRAM_END, and must be
// accessible from NS caller according to SAU + NS MPU (privileged or not
// based on current processor IPSR and NS CONTROL flag). We also allow
// buffers in USB RAM if this is granted to NS via ACCESSCTRL -- note USB RAM
// is IDAU-Exempt so will fail tt* checks.
//
// sets *ok = hx_false() on failure, and returns BOOTROM_ERROR_INVALID_ADDRESS

.section .sg_fillers

.global s_native_api_validate_ns_buffer
s_native_api_validate_ns_buffer:
    push {r4, lr}
    mov r4, r3
    bl s_native_validate_ns_buffer_internal
    str r1, [r4, #0]
    rcp_count_check STEPTAG_S_NATIVE_API_VALIDATE_NS_BUFFER_BASE + 3
    pop {r4, pc}

.section .text.s_native_validate_ns_buffer_internal
.global varm_to_s_native_validate_ns_buffer_internal
varm_to_s_native_validate_ns_buffer_internal:
    VARM_HINT HINT_s_native_validate_ns_buffer_internal

.global s_native_validate_ns_buffer_internal
// note: this function returns an rcp_bool in r1
// note: this functinon flags as per `tst r1, r1` (the returned r1) (`tst r1, r1` is used instead of
//       `cmp r1, #0` to simplify the RISC-V impl of this function. Therefore the caller should use
//       `bpl` rather than `bge` to detect an invalid address
// HOWEVER RISC-V does nto set r3, and sets flags as per tst r1,
// uint64_t __used s_native_validate_ns_buffer_internal(const void *addr, uint32_t size, hx_bool write) {
s_native_validate_ns_buffer_internal:
    rcp_count_set_nodelay STEPTAG_S_NATIVE_API_VALIDATE_NS_BUFFER_BASE
    // save r0, r1 as we need it at the end
    push {r0, r1, r4, r5, r6, r7, lr}

    // calculate is_privileged
    // =======================
    movs r3, #0 // r3 = 0, Z=1
    ldmia r3!, {r4, r5, r6, r7} // trash regs
    // r4 = 0xf0000000
    // r5 = 0x00000091 (or so)
    // r6 = 0x000003ad (or so)
    // r7 = r5

    // No need to zero-extend the Exception field of IPSR, as we know
    // for *this version* of v8-M that all other bits are RES0 (reason for PICO_R2350 check above)
    // r3 = 16, Z=1

    // if either or both of these are skipped, Z should still be 1
    mrs r4, ipsr
    lsls r7, r4, #4

    beq 1f
    beq return_invalid
    movs r4, #1
1:
    mrs ip, control_ns
    orn r3, r4, ip
    ubfx r3, r3, #0, #1

    // calculate is_unprivileged
    // =========================
    //
    // No need to zero-extend the Exception field of IPSR, as we know
    // for *this version* of v8-M that all other bits are RES0.
    lsrs r5, #31 // r5 shyould now be zero
    mrs r5, ipsr
    cbz r5, 1f
    cbz r5, return_invalid
    movs r5, #1
1:
    rcp_iequal r4, r5 // check (bool)ipsr copies against eachother
    mrs r6, control_ns
    rcp_iequal r6, ip // check control_ns copies against eachother
    bics r6, r5
    ubfx ip, r6, #0, #1

    // setup pstart / pend, and poison pend if size < 0
    // ================================================

    // --- r0=addr, r1=size, r2=hx_write, r3=is_privileged, r4=ipsr, r5=ipsr, ip=is_unprivileged
    cbz r1, 1f
    // Note -1 because bounds are inclusive: we test the byte at each end. 0-byte buffers are tested
    // as though they are 1-byte.
    subs r1, #1
1:

    // --- r0=addr, r1=adj_size | 0, r2=hx_write, r3=is_privileged, r4=ipsr, r5=ipsr, ip=is_unprivileged
    adds r4, r0, r1             // pend = addr + size
    subs r5, r4, r0             // size2 =  pend - addr
    rcp_iequal_nodelay r1, r5   // assert size == size2
    // repeat as it is important that r4 be correct
    rcp_iequal_nodelay r1, r5   // assert size == size2

    asrs r5, #29                // r5 is 0 if 0 <= size <= 0x20000000, r5 is above 0xfffffff8 if size is negative
    asrs r1, #29                // r1 is 0 if 0 <= size <= 0x20000000, r1 is above 0xfffffff8 if size is negative

    // we want to OR into pstart or pend so that they become invalid, but we can actually
    // only do it to pend because ORing will increase the value in an unknown way, so we want to leave
    // the pend and pstart at least as far apart as they started so they don't somehow
    // move into the same region when they weren't before
    orrs r4, r5
    orrs r4, r1
    orrs r4, r5
    orrs r4, r1

#define r_unprivileged ip
#define r_privileged r3

#define r_flags_start2 r1
#define r_flags_start r5
#define r_flags_end r6
#define r_flags_end2 r7

    // get redundant TTA(T) values based on privileged level
    // =====================================================

    // --- r0=pstart, r1=0 hopefully, r2=hx_write, r3=is_privileged, r4=pend, r5=0 hopefully, ip=is_unprivileged

    ands r5, r_privileged, r_unprivileged // should be zero, otherwise we'll get an alignment violation
    // we load 2 of the 4 flags with different values.
    // note that both of the below don't match writable buffers, or have any permission bits set
    // r6 and r7 should be different from above code anyway; r6 being a CONTROL_NS read which doesn't have permission bits set
    // and r7 being IPSR << 4 which doesn't either

    // note the values are the vector table, so:

    // 0xf0000000
    // 0x00000091 (or so)

    ldmia r5!, {r_flags_end, r_flags_end2}
    // r5 should be 0x8, so...
    lsrs r5, #3 // now 1
    movs r1, #0 // this should already be 0 from above except if size is negative, in twhich case if this
                // zeroing is skipped, there will likely be a violation anyway

    // r5 should be 1 and C flag should be cleaer
    cmp r_privileged, r5
    // note skipping of most combinations of the above instruction should leave carry set,
    // e.g. if r5 was 0 before the cmp, or if cmp was skipped)
    bcs privileged

unprivileged:
    ttat r_flags_start2, r0
    ttat r_flags_end, r4
    ttat r_flags_end2, r4
    rcp_iequal r_unprivileged, r5 // r5 should be 1
    ttat r_flags_start, r0        // r_flags_start is r5
    rcp_count_check STEPTAG_S_NATIVE_API_VALIDATE_NS_BUFFER_BASE
    b 1f
privileged:
    tta r_flags_start, r0
    tta r_flags_end, r4
    tta r_flags_end2, r4
    rcp_iequal r_unprivileged, r1
    tta r_flags_start2, r0        // r_flags_start2 is r1
    rcp_count_check STEPTAG_S_NATIVE_API_VALIDATE_NS_BUFFER_BASE
1:

    // r0=pstart, r1=flags_start2, r2=hx_write, r3=is_privileged, r4=pend, r5=flags_start, r6=flags_end, r7=flags_end2, ip=is_unprivileged

    // check first pair of flags against eachother
    // ===========================================

    cmp r_flags_start, r_flags_end
    beq flags_match

    // inline failure path
    // ===================
return_invalid:
    rcp_count_check_nodelay STEPTAG_S_NATIVE_API_VALIDATE_NS_BUFFER_BASE + 1
return_invalid_step_done:
    mov.w r1, #HX_BIT_PATTERN_FALSE
    // removed this as I think r1 is sufficiently trashed on the way here not to be pre-populated
    // rcp_bfalse_nodelay r1

    // r0=addr, r1=hx_false r2=hx_write, r3=is_privileged, r4=pend, r5=flags_start, r6=hx_false, r7=flags_end2, ip=?
    ldr r0, =BOOTROM_ERROR_INVALID_ADDRESS
    b done
    // if we fall thru, hopefully the second pair will cause failure

flags_match:
    // check second pair of flags against eachother
    // ============================================
    rcp_iequal_nodelay r_flags_start2, r_flags_end2

    // r0=addr, r1=flags_start2, r2=write, r3=is_privileged, r4=pend, r5=flags_start, r6=flags_end, r7=flags_end2, ip=is_unprivileged

    // ip is very much trashed above, so won't be a valid bool if this instruction is skipped, and that
    // is checked further below
    mov ip, r2

#undef r_flags_start2 // we don't use this any more
#define r_ok2 r1
#define r_ok1 r2
#undef r_write
#define r_write ip
    // r0=addr, r1=ok2, r2=ok1, r3=is_privileged, r4=pend, r5=flags_start, r6=flags_end, r7=flags_end2, ip=write

#define r_pstart r0
#define r_pend r4

    // =======================================
    // Deterimine what address range we are in
    //
    // a) 0x10000000 -> 0x20082000
    // b) 0x50100000 -> 0x50101000
    //
    // we do a loop with sort-of-out-codes because otherwise it is easy
    // to skip a single instruction and break a bounds check
    asrs r_ok1, r_pstart, #31 // short be zero
    adds r_ok1, #8 // offset so we can't skip looping AND the shift right 3 after the loop
outcode_loop:
    cmp r_pstart, XIP_BASE
    ldr r_ok2, =SRAM_END
    bcc 1f
    // p_start >= XIP_BASE
    adds r_ok1, #0x3f
1:
    cmp r_pend, r_ok2
    ldr r_ok2, =USBCTRL_DPRAM_BASE
    bcs 1f
    // p_end < SRAM_END
    adds r_ok1, #0x14
1:
    cmp r_pstart, r_ok2
    bcc 1f
    // p_start >= USBCTRL_DPRAM_BASE
    adds r_ok1, #0x87
1:
    subs r_ok2, r_pend, r_ok2
#if USBCTRL_DPRAM_SIZE != (1u << 12)
#error
#endif
    lsrs r_ok2, #12
    bne 1f
    bmi return_invalid // extra check in case r_ok2 is a500a500 by skipping the lsrs
    // pend < USBCTRL_DPRAM_BASE + 0x1000
    adds r_ok1, #0x28
1:
    adds r_ok1, #0x20000000 // loop 8 times
    bcc outcode_loop // early breakout will leave r_ok1 wrong
outcode_loop_end:
    lsrs r_ok1, #3
2:
    // r0=addr, r1=ok2, r2=ok1, r3=is_privileged, r4=pend, r5=flags_start, r6=flags_end, r7=flags_end2, ip=write

    // at this point we should have
    //
    // ok1 == 0x54 -> high probability of 0x10000000 -> 0x20082000
    // ok1 == 0xef -> high probability of 0x50100000 -> 0x50101000

    // we don't want to go down the usb_ram path unless it really is USB RAM; r_ok1 should be non-zero
    // at this point
    // at this point,
    cmp r_ok1, #0xef // deliberate branch to in_usb_ram, as if we get there by mistake, we don't check TT result
    beq in_usb_ram
    cmp r_ok1, #0x54
    // if we skip this, we will be checking the TT result, so will just return whether the buffer is accessible to NS
    // rather than that and whether it is a valid location for a buffer
    bne return_invalid
in_xip_or_sram:
    // XIP and SRAM are the valid spaces for buffers in non-Exempt regions.

#define TT_RESP_NSR_LSB 20
#define TT_RESP_NSRW_LSB 21
    // r0=pstart, r1=ok2, r2=ok1, r3=is_privileged, r4=pend, r5=flags_start, r6=flags_end, r7=flags_end2, ip=hx_write

#define r_t0 r3
    // check permissions using r_flags_start
    // =====================================
    rcp_bvalid_nodelay r_write
    // note sufficxes _w & _r whether we're doing a write or a read
    // flags_start         = 0000 0000 0SWR xxxx xxxx xxxx xxxx xxxx
    // write_w             = 1010 0101 0000 0000 1010 0101 0000 0000
    // write_r             = 0000 0000 1100 0011 0000 0000 1100 0011
    // write_w >> 1        = 0101 0010 1000 0000 0101 0010 1000 0000
    // write_r >> 1        = 0000 0000 0110 0001 1000 0000 0110 0001
    orrs r_ok2, r_flags_start, r_write, LSR #1
    // ok2_w               = 0101 0010 1SWR xxxx x1x1 xx1x 1xxx xxxx
    // ok2_w >> 1          = 0010 1001 01SW Rxxx xx1x 1xx1 x1xx xxxx
    // ok2_r               = 0000 0000 011R xxx1 1xxx xxxx x11x xxx1
    // ok2_r >> 1          = 0000 0000 0011 Rxxx 11xx xxxx xx11 xxxx
    ands r_ok2, r_ok2, r_ok2, LSR #1
    // ok2_w               = 0000 0000 0xxA xxxx xxxx xxxx xxxx xxxx (A = R & W)
    // ok2_r               = 0000 0000 001R xxxx xxxx xxxx xxxx xxxx
    lsrs r_ok2, r_ok2, #TT_RESP_NSR_LSB
    // ok2_w               = 0000 0000 0000 0000 0000 0000 0000 0xxA
    // ok2_r               = 0000 0000 0000 0000 0000 0000 0000 001R

    // 0x54 -> 0x56 // we need to clear bits 1 & 2, so this is a good value (and in range for a 16-bit add instruction)
    // 0xef -> 0xf1 // this will clear bit 0 which will make ok2 != 1
    adds r_t0, r_ok1, #0x2
    bics r_ok2, r_t0

    // mix in 0x54 (+ 0x2) check
    adds r_ok2, r_t0
    subs r_ok2, #0x56

    // check permissions using r_flags_end
    // ===================================
    orrs r_flags_end, r_flags_end, r_write, LSR #1
    ands r_ok1, r_flags_end, r_flags_end, LSR #1
    lsrs r_ok1, r_ok1, #TT_RESP_NSR_LSB
    bics r_ok1, r_t0

#undef r_t0

combine_results:
    // r0=pstart, r1=ok2, r2=ok1, r3=is_privileged, r4=pend, r5=flags_start, r6=flags_end, r7=flags_end2, ip=hx_write

    // combine both ok1 & ok2 into an hx_bool (which will most likely be invalid if ok1 != ok2 or ok1/ok2 are not 0/1)
    movs r0, r_ok1
    rcp_count_check_nodelay STEPTAG_S_NATIVE_API_VALIDATE_NS_BUFFER_BASE + 1
    // r1 = r_ok2
#undef r_is_uprivileged // call trashes r1 & ip
    bl sonly_varm_make_hx_bool_impl
    // r0 = hx_ok
    // N flags is corret on return from sonly_varm_make_hx_bool_impl
    bpl return_invalid_step_done

    rcp_iequal_nodelay r_flags_end2, r_flags_start // cross compare flag pairs
    mov r1, r0                                     // return hx_ok in r1
    ldr r0, [sp]                                   // and restore addr
done:
    tst r1, r1 // set hx_bool flags for caller
    rcp_count_check_nodelay STEPTAG_S_NATIVE_API_VALIDATE_NS_BUFFER_BASE + 2
    // r1 should be set to a valid hx_bool in all paths
    rcp_bvalid_nodelay r1
    pop {r2, r3, r4, r5, r6, r7, pc} // note we return original r1 in r3 (and r0 in r2 FWIW)

in_usb_ram:
    // r0=addr, r1=ok2, r2=ok1, r3=is_privileged, r4=pend, r5=flags_start, r6=flags_end, r7=flags_end2, ip=hx_write

    // USB RAM is the only valid space for a buffer in an Exempt region. Exempt regions will
    // always fail TTA checks, so we allow NS buffers in USB RAM if and only if the USB
    // peripheral is granted to NS at the caller's privilege level via ACCESSCTRL.
    // (Note: this means we ignore NS MPU regions in USB RAM)
#if ACCESSCTRL_USBCTRL_NSU_BITS != 1
#error
#endif
    // r0=addr, r1=ok2, r2=ok1, r3=is_privileged, r4=pend, r5=flags_start, r6=flags_end, r7=flags_end2, ip=hx_write

#define r_is_priv r3
#define r_t1 r6

    // r_ok2 is likely an invalid address or in ROM at this point
    ldr r_ok2, =ACCESSCTRL_BASE + ACCESSCTRL_USBCTRL_OFFSET
    ldrb r_ok2, [r_ok2]
    // note skipping either of these shifts will leave 0 in the low bits
    lsls r_ok2, #30
    lsrs r_ok2, #30
    adds r_ok2, r_is_priv
    adds r_ok2, #5
    lsrs r_ok2, #3
    adds r_ok2, #0xef
    subs r_ok2, r_ok1

    ldr r_t1, =ACCESSCTRL_BASE + ACCESSCTRL_USBCTRL_OFFSET
    ldrb r_t1, [r_t1]
    // note skipping either of these shifts will leave 0 in the low bits
    lsls r_t1, #30
    lsrs r_t1, #30
    adds r_t1, r_is_priv
    adds r_t1, #5
    lsrs r_ok1, r_t1, #3
    b combine_results

    // note this falls thru to s_varm_api_enter_cmd_xip, which may return to caller but not with correct rcp values
