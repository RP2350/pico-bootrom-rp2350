/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootrom.h"
#include "bootram.h"
#include "boot/bootrom_constants.h"

.cpu cortex-m33
.thumb
.section .secure_gateways.first, "ax"
.syntax unified

.set num_of_sg_table_funcs, 0
.macro sg_table_func name idx prefix last
    // The order is important here because the entry PC is used to get the
    // table index
    .if num_of_sg_table_funcs != \idx
        .error "Wrong SG table func order, compare the bootrom_constants.h BOOTROM_NS_API constants to the asm table"
    .endif

    // Insert function table entry
    .section .secure_gateways, "ax"
    .if num_of_sg_table_funcs == 0
        .global sg_api_table
        sg_api_table:
    .endif
    sg_functable_\name:
#if BOOTROM_32BIT_FUNC_POINTERS
    .word s_from_ns_\prefix\()_api_\name + 1
#else
    .hword s_from_ns_\prefix\()_api_\name + 1
#endif
    .if num_of_sg_table_funcs == BOOTROM_NS_API_COUNT - 1
        .global sg_api_table_end
        sg_api_table_end:
    .endif

    // Insert gateway with same index
    .section .secure_gateways.first, "ax"
    // 8 bytes per gateway: 32-bit SG, 16-bit mov, 16-bit branch. Note the mov
    // points ip at the *next slot*, not the one we entered.
    .if num_of_sg_table_funcs == 0
        .p2align 2
    .endif
    .global sg_api_\name
    sg_api_\name:
    .if num_of_sg_table_funcs == 1
        .global sg_second_table_entry_point
        sg_second_table_entry_point:
    .endif
    sg
    mov ip, pc
    .if \last != 1
        b.n sg_table_entry_longbranch
    .endif
    .set num_of_sg_table_funcs, num_of_sg_table_funcs + 1
.endm

// catch all entry point for nsboot - it is permanently disabled (until bootram
// reset, which implies proc reset) if nsboot path is not taken
// returns a 32 bit value (note all args must be in registers)
.global sg_nsboot_service_call
sg_nsboot_service_call:
    sg
#if !FEATURE_CANARIES
    push {r0, r1, r2, r4, lr} // save r4 for stack alignment (once we've popped r0, r1, r2)
#else
    rcp_canary_get_nodelay ip, CTAG_SG_CALL
    push {r0, r1, r2, ip, lr}
#endif
    // r0 & r2 are used as pointers in s_from_nsboot_service_call, so initialize them with likely invalid values;
    // r2 gets the canary (which is checked); r0 is just different from r2
    mov r2, ip
    adds r0, r2, #1
    adr lr, return_to_ns_preserve_r0 + 1
    b.w s_from_nsboot_service_call


// The BOOTROM_NS_API_xxx numbers here are defined in bootrom_constants.h
#define SG_TABLE_FUNC(name, prefix, last) sg_table_func name BOOTROM_NS_API_ ## name prefix last

// The order here must match the SG_API_xxx numbers defined in bootrom.h
// (and we check this and .error on mismatch)
// NOTE: these functions must all return a value, or secure r0 is exposed
SG_TABLE_FUNC(get_sys_info, varm, 0)
SG_TABLE_FUNC(checked_flash_op, arm8, 0)
SG_TABLE_FUNC(flash_runtime_to_storage_addr, arm8, 0)
SG_TABLE_FUNC(get_partition_table_info, varm, 0)
SG_TABLE_FUNC(secure_call, arm8, 0)
SG_TABLE_FUNC(otp_access, varm, 0)
SG_TABLE_FUNC(reboot, varm, 0)
SG_TABLE_FUNC(get_b_partition, varm, 1)

sg_table_entry_longbranch:
    // b.w sg_table_entry
    // fall-through:

// ip has table index
.global sg_table_entry
sg_table_entry: // ip holds pc
#if FEATURE_CANARIES
    rcp_count_set_nodelay STEPTAG_SG_CALL
#endif
    // save r0, r1 as temporaries before we call the function
    //    note: we use r0, and r1 as temporaries because this makes it harder in general for the user
    //    to pass known values in r0, r1 that will help if instruction in this functions are skipped
    //    since r0, r1 are actual arguments to the function
    // save r4 as space for canary (we don't trash r4)
    // save lr for return
    push {r0, r1, r4, lr}

    // note we use lr here in the calculation to make it harder to pass in a rcp_true lr value
#if FEATURE_CANARIES
    rcp_canary_get_nodelay lr, CTAG_SG_CALL
    str lr, [sp, #8]
#endif

    // The tabled SGs are 8 bytes apart, in SG_API index order, so we can
    // calculate the API index from the PC which was stashed in IP. Note the
    // captured PC actually points at slot + 1 (because reading PC does not
    // point to the current instruction on Arm) so we take the difference
    // starting at the *second* slot to cancel out the off-by-one.
    adr r1, sg_second_table_entry_point
    subs r0, ip, r1 // r0 = ip - sg_second_table_entry_point
    lsrs r0, #3
    cmp r0, #BOOTROM_NS_API_COUNT
    bhs 1f // bhs is unsigned-greater-or-equal
#if BOOTROM_NS_API_COUNT != 8
#error expected 8 APIs
#endif
    // re-check API num range again
    lsrs r1, r0, #3
    bne 1f
    bne 1f
    // permission check
    ldr ip, =BOOTRAM_BASE + BOOTRAM_NS_API_PERMISSIONS_OFFSET
    adds ip, r1 // clear EQ most likely; also extra check that r1 == 0
    ldr r1, =BOOTRAM_BASE + BOOTRAM_NS_API_PERMISSIONS_OFFSET
    ldrb r1, [r1, r0]
    ldrb ip, [ip, r0]
    // r1 is a5 if valid
    cmp r1, #0xa5
    beq 2f
1:
    ldr r0, =BOOTROM_ERROR_NOT_PERMITTED
    // pop temporaries from stack trashing regs
    pop {r1, r2}
    // fall thru
.global return_to_ns_preserve_r0
.thumb_func
return_to_ns_preserve_r0:
    // note this is a common return from all SG calls, and expects canary, LR on stack
    pop {r1, lr} // restore canary_value, lr
#if FEATURE_CANARIES
    rcp_canary_check_nodelay r1, CTAG_SG_CALL
#endif
    // we clear the non-callee saved regs on exit from secure code (other than r0 which is the return code)
    movs r1, #0
    msr APSR_nzcvqg, r1
    ldmia r1!, {r2, r3}
    ldmia r1!, {r2, r3}
    bxns lr

2: // we are good to go
    rcp_iequal r1, ip
    lsls r1, #8
    rcp_canary_check_nodelay lr, CTAG_SG_CALL
    add.w lr, r1, ip, lsl #24
    // lr is 0xa500a500 if permission was 0xa5
    rcp_btrue_nodelay lr

    ldr r1, =P16(sg_api_table)
#if BOOTROM_HARDENING
    cmp r0, #BOOTROM_NS_API_COUNT
    bhs 1b // bhs is unsigned-greater-or-equal
#endif

#if BOOTROM_32BIT_FUNC_POINTERS
    ldr ip, [r1, r0, lsl #2]
#else
    ldrh ip, [r1, r0, lsl #1]
#endif
    // restore r0, r1 prior to call
    pop {r0, r1}
    // recheck validity
    rcp_btrue_nodelay lr
    // adr seems to get confused with negative offsets
    //adr  lr, return_to_ns_preserve_r0 + 1
adr_loc:
    subw lr, pc, #adr_loc + 2 - return_to_ns_preserve_r0 - 1
#if FEATURE_CANARIES
    rcp_count_check_nodelay STEPTAG_SG_CALL
#endif
    bx   ip

// Assert that the bootrom.h constant matches the number of table entries, as
// it's used for bounds checking in arm8_bootrom_rt0.S
.if num_of_sg_table_funcs != BOOTROM_NS_API_COUNT
.error "Number of SG table funcs does not match BOOTROM_NS_API_COUNT from bootrom_constants.h"
.endif

.section .secure_gateways, "ax"
.global s_from_ns_arm8_api_checked_flash_op
// int __sg_impl_nsc s_from_ns_arm8_api_checked_flash_op(cflash_flags_t flags, uintptr_t addr, uint32_t size_bytes, uint8_t *buf)
s_from_ns_arm8_api_checked_flash_op:
    push {r0, r1, r2, r3, r4, r5, lr}
    rcp_canary_get_nodelay r4, CTAG_S_FROM_NS_ARM8_API_CHECKED_FLASH_OP
    str r4, [sp, #0]

#if CFLASH_OP_BITS != 0x00070000
#error
#endif
    // r1 = flash op
    ubfx r1, r0, #CFLASH_OP_LSB, #3

    // r4 = flags with no security level
    bics r4, r0, #CFLASH_SECLEVEL_BITS
#if (CFLASH_SECLEVEL_VALUE_SECURE != 1) || (CFLASH_SECLEVEL_VALUE_NONSECURE !=2 ) || (CFLASH_SECLEVEL_VALUE_BOOTLOADER != 3)
#error
#endif
    // check for SECURE or none
    lsls r5, r0, #32 - CFLASH_SECLEVEL_LSB - 1
    bcc cfo_error
    // check for BOOTLOADER
    bmi cfo_error

    // explicitly set non-secure
    orrs r4, #CFLASH_SECLEVEL_VALUE_NONSECURE << CFLASH_SECLEVEL_LSB
    rcp_iequal_nodelay r0, r4
    rcp_iequal_nodelay r4, r0

#if CFLASH_OP_VALUE_ERASE != 0
#error
#endif
    cbnz r1, op_not_erase
    // buf should be zero
    cbnz r3, cfo_error
    rcp_iequal_nodelay r1, r3 // op and buf should be zero
    adds r1, #HX_BIT_PATTERN_TRUE
    b op_check_ok

op_not_erase:
    // ===== OP != ERASE
    // NS flash operations with RAM buffers must point to NS-accessible RAM
    // note NULL is not a valid buffer, so will fail validation

#if CFLASH_OP_MAX != 2 || CFLASH_OP_VALUE_PROGRAM != 1
#error
#endif
    // hx_bool write = make_hx_bool(op == CFLASH_OP_VALUE_PROGRAM);
    and r0, r1, #1
    and r1, #1
    bl sonly_varm_make_hx_bool_impl // preserves r2, r3
    // buf,addr_ok = call_s_native_validate_ns_buffer(buf, size_bytes, write);
    mov r1, r2
    mov r1, r2
    mov r2, r0
    mov r0, r3
    movs r3, #1 // poison r3 and flags
    bl s_native_validate_ns_buffer_internal
    bpl op_return
    // reload size
    ldr r2, [sp, #8]
    rcp_iequal_nodelay r2, r3
    rcp_iequal_nodelay r3, r2
    ldr r3, [sp, #12]
    rcp_iequal r0, r3
op_check_ok:
    rcp_btrue_nodelay r1
    rcp_btrue_nodelay r1
    mov r0, r4
    ldr r1, [sp, #4]
    // rc = s_varm_api_checked_flash_op((cflash_flags_t) {flags2}, addr, size_bytes, buf);
    bl s_varm_api_checked_flash_op
op_return:
    pop {r1}
    rcp_canary_check_nodelay r1, CTAG_S_FROM_NS_ARM8_API_CHECKED_FLASH_OP
    pop {r1, r2, r3, r4, r5, pc}

cfo_error:
    mov r0, #BOOTROM_ERROR_INVALID_ARG
    b op_return
