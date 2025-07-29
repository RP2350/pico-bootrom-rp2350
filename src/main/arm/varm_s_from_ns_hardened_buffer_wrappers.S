/**
 * Copyright (c) 2024 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootrom.h"

.cpu cortex-m23
// note using near (.n) for branches as GCC still seems to do things like bcs.w for m23!
.thumb
.syntax unified

#define FLASH_SECTOR_SIZE 4096
#define FLASH_PAGE_SIZE 256

.section .textend.s_from_ns_varm_api_otp_access, "ax"
.global s_from_ns_varm_api_otp_access
s_from_ns_varm_api_otp_access:
    // copy buffer to arg 4
    mov r3, r0
.cpu cortex-m33
    // load false value (this isn't an xbool value, but we don't care as we only assert on the value deeper down the call if it is true)
    mrc p7, #7, r0, c0, c1, #0 // load false value no delay
    mcr2 p7, #3, r0, c0, c0, #1 // bfalse no delay
    mcr2 p7, #3, r0, c0, c0, #1 // bfalse no delay
.cpu cortex-m23
    // fall trhu

// int __noinline s_from_ns_varm_api_otp_access_internal(hx_xbool secure, uint32_t buf_len, otp_cmd_t cmd, aligned4_uint8_t *buf) {
.global s_from_ns_varm_api_otp_access_internal
s_from_ns_varm_api_otp_access_internal:
    push {r0, r1, r2, r4, lr}
.cpu cortex-m33
    rcp_canary_get_nodelay r4, CTAG_S_FROM_NS_VARM_API_OTP_ACCESS_INTERNAL
.cpu cortex-m23
    // hx_bool write = make_hx_bool(!(cmd.flags & OTP_CMD_WRITE_BITS));
    movs r1, #1
    lsrs r0, r2, #OTP_CMD_WRITE_LSB
    bics r1, r0
    movs r0, r1
    // preserves r2, r3
    bl sonly_varm_make_hx_bool_impl
    mov r2, r0
    mov r0, r3
    movs r3, #1 // poison r3 and flags
    ldr r1, [sp, #4]
    ldr r1, [sp, #4]
    bl varm_to_s_native_validate_ns_buffer_internal // (buf, buf_len, write)
    // r0 = buf or error
    // r1 = rcp_ok
    // r3 = buf_len (on ARM only)
    bpl 1f
    bpl 1f
.cpu cortex-m33
    rcp_btrue r1
    rcp_btrue r1
.cpu cortex-m23
    add r2, sp, #4
    ldmia r2, {r1, r2}
    // re-check buf_len is correct
.cpu cortex-m33
    rcp_iequal r1, r3
    rcp_iequal r3, r1
.cpu cortex-m23
    // checked in s_varm_api_hx_otp_access
    //    if ((uintptr_t)buf & 0x3u) {
    //        return BOOTROM_ERROR_INVALID_ADDRESS;
    //    }
    //    if ((uintptr_t)io->row & ~(OTP_CMD_ROW_BITS >> OTP_CMD_ROW_LSB)) {
    //        return BOOTROM_ERROR_INVALID_ARG;
    //    }
    ldr r3, [sp, #0]
    // rc = s_varm_api_hx_otp_access(buf, buf_len, cmd, secure);
    bl s_varm_api_hx_otp_access
1:
.cpu cortex-m33
    rcp_canary_check_nodelay r4, CTAG_S_FROM_NS_VARM_API_OTP_ACCESS_INTERNAL
.cpu cortex-m23
    pop {r1, r2, r3, r4, pc}


.section .sg_fillers, "ax"

.global s_from_ns_varm_api_get_sys_info
s_from_ns_varm_api_get_sys_info:
// int __noinline __sg_filler s_from_ns_varm_api_get_sys_info(uint32_t *buffer, uint32_t buffer_size_words, uint flags) {
    push {r1, r2, r4, r5, lr}
    movs r4, #0
    movs r4, #0
    movw r4, P16(s_varm_api_get_sys_info)
    // fall thru

// common handler for get_sys_info or get_partition_info which take
// (buffer, buffer_size_words, arg2) and want to write to the buffer
// ip = rcp_true for get_sys_info
// ip = rcp_false for get_partition_table_info
s_from_ns_varm_common_get_sys_or_partition_table_info:
.cpu cortex-m33
    rcp_canary_get_nodelay r5, CTAG_S_FROM_NS_VARM_COMMON_GET_SYS_OR_PARTITION_TABLE_INFO
.cpu cortex-m23
    // note if r1 ends up with the wrong value (e.g. this instruction is skipped) that is
    // ok, as we make sure we use the same value for the subsequent secure call
    lsls r1, r1, #2 // buffer_size_words to bytes
.cpu cortex-m33
    hx_bit_pattern_true r2
    rcp_btrue_nodelay r2
.cpu cortex-m23
    movs r3, #1 // poison r3 and flags
    movs r3, #1 // poison r3 and flags
    bl varm_to_s_native_validate_ns_buffer_internal
    // r0 = buffer
    // r1 = rcp_ok
    // r3 = buffer_size_bytes (on ARM only)
    // flags are tst r1, r1 (we don't do that here since we don't want skipping the function call
    //   to allow a passed in r5 to spoof
    bpl 1f
    bpl 1f
.cpu cortex-m33
    rcp_btrue r1
    rcp_btrue r1
.cpu cortex-m23
    // Note we use buffer_size_bytes / 4, not buffer_size_words, since it's not necessarily true
    // that x * 4 >= x when accounting for unsigned wrapping
    ldr r1, [sp, #0]
    lsrs r3, #2
    // note these won't match if size had either of the top 2 bits set, but that is an invalid size anyway
.cpu cortex-m33
    rcp_iequal r1, r3
    rcp_iequal r3, r1
.cpu cortex-m23
    ldr r2, [sp, #4]
    // call secure func in r4(buffer, buf_size_words, r2)
    blx r4
1:
.cpu cortex-m33
    rcp_canary_check_nodelay r5, CTAG_S_FROM_NS_VARM_COMMON_GET_SYS_OR_PARTITION_TABLE_INFO
.cpu cortex-m23
    pop {r2, r3, r4, r5, pc}

.global s_from_ns_varm_api_get_partition_table_info
s_from_ns_varm_api_get_partition_table_info:
    push {r1, r2, r4, r5, lr}
    movs r4, #0
    movs r4, #0
    movw r4, P16(s_varm_api_get_partition_table_info)
    b.n s_from_ns_varm_common_get_sys_or_partition_table_info

.section .text.s_from_nsboot_varm_flash_sector_erase
.global s_from_nsboot_varm_flash_sector_erase
s_from_nsboot_varm_flash_sector_erase:
// int __attribute__((naked)) s_from_nsboot_varm_flash_sector_erase(__unused uint32_t addr) {
#if (XIP_BASE & 0xffffff) || (FLASH_SECTOR_SIZE != (XIP_BASE >> 16))
#error
#endif
#define SECTOR_ERASE_FLAGS ((CFLASH_ASPACE_VALUE_STORAGE     << CFLASH_ASPACE_LSB  ) | \
                           (CFLASH_SECLEVEL_VALUE_BOOTLOADER << CFLASH_SECLEVEL_LSB) | \
                           (CFLASH_OP_VALUE_ERASE            << CFLASH_OP_LSB      ))
    movs r1, #XIP_BASE >> 24
    lsls r1, #24
    lsrs r2, r1, #16
    adds r1, r0
    ldr r0, =SECTOR_ERASE_FLAGS
    ldr r0, =SECTOR_ERASE_FLAGS

.global s_from_nsboot_varm_flash_sector_erase_end
s_from_nsboot_varm_flash_sector_erase_end:
    // NOTE: fall trhue to s_varm_api_checked_flash_op via linker script

.section .text.s_from_nsboot_varm_flash_read_data

.global s_from_nsboot_varm_flash_read_data
s_from_nsboot_varm_flash_read_data:
// int s_from_nsboot_varm_flash_read_data(uint8_t *rx, uint32_t addr, size_t count) {
#define READ_DATA_FLAGS (CFLASH_ASPACE_VALUE_STORAGE      << CFLASH_ASPACE_LSB  ) | \
                        (CFLASH_SECLEVEL_VALUE_BOOTLOADER << CFLASH_SECLEVEL_LSB) | \
                        (CFLASH_OP_VALUE_READ             << CFLASH_OP_LSB      )
    push {r4, r5, r6, r7, lr}
    mov r4, r2
    ldr r5, =READ_DATA_FLAGS
    ldr r5, =READ_DATA_FLAGS
    b.n s_from_nsboot_varm_common_flash_read_data_or_page_program
    // note if we fall thru here, we're just calling the wrong function
    // which isn't an attack since the NSBOOT code could have called it anyway
    // (these methods are not shared with _ns_)

.global s_from_nsboot_varm_flash_page_program
s_from_nsboot_varm_flash_page_program:
// int s_from_nsboot_varm_flash_page_program(const uint8_t *data, uint32_t addr)
#define PAGE_PROGRAM_FLAGS  (CFLASH_ASPACE_VALUE_STORAGE      << CFLASH_ASPACE_LSB  ) | \
                            (CFLASH_SECLEVEL_VALUE_BOOTLOADER << CFLASH_SECLEVEL_LSB) | \
                            (CFLASH_OP_VALUE_PROGRAM          << CFLASH_OP_LSB      )
    push {r4, r5, r6, r7, lr}
    ldr r5, =PAGE_PROGRAM_FLAGS
    ldr r5, =PAGE_PROGRAM_FLAGS
    movw r4, #FLASH_PAGE_SIZE
    // fall thru

// common handler for flash_read_data or flash_page_program
// r0 = buffer
// r1 = addr
//
// r4 = buffer_size
// r5 = op
// stack has {r4, r5, r6, r7, lr}
s_from_nsboot_varm_common_flash_read_data_or_page_program:
    mov r6, r0
    mov r7, r1
.cpu cortex-m33
    rcp_canary_get_nodelay r1, CTAG_S_FROM_NSBOOT_VARM_COMMON_FLASH_READ_DATA_OR_PAGE_PROGRAM
.cpu cortex-m23
    push {r1}
    mov r1, r4
    mov r1, r4
    movs r3, #1 // poison flags and r3
    // because this function is exclusively used by NSBOOT, and we know that the buffers used are both R/W, we'll
    // just execlusively ask for write access to save space
.cpu cortex-m33
    hx_bit_pattern_true r2
    rcp_btrue_nodelay r2
.cpu cortex-m23

    bl varm_to_s_native_validate_ns_buffer_internal
    // r0 = buffer
    // r1 = rcp_ok
    // r3 = buffer_size_bytes (on ARM only)
    bpl 1f
.cpu cortex-m33
    rcp_btrue r1
    rcp_btrue r1
.cpu cortex-m23
    mov r2, r4 // buffer_size
.cpu cortex-m33
    rcp_iequal r2, r3 // check against validate-returned size
    rcp_iequal r3, r2
.cpu cortex-m23
    mov r1, r7 // addr (we do't care if this is skipped, since it is checked in the callee)
    movs r7, #1
#if XIP_BASE != 0x10000000
#error
#endif
    lsls r7, #28
    adds r1, r7
    mov r3, r0 // buffer
.cpu cortex-m33
    rcp_iequal r3, r6
    rcp_iequal r3, r0
.cpu cortex-m23
    mov r0, r5
    // s_varm_api_checked_flash_op(cmd, XIP_BASE + addr, size, buffer)
    bl s_varm_api_checked_flash_op
1:
.cpu cortex-m33
    pop {r4}
    rcp_canary_check_nodelay r4, CTAG_S_FROM_NSBOOT_VARM_COMMON_FLASH_READ_DATA_OR_PAGE_PROGRAM
.cpu cortex-m23
    pop {r4, r5, r6, r7, pc}

.section .text.s_from_nsboot_varm_ram_trash_get_uf2_target_partition
.global s_from_nsboot_varm_ram_trash_get_uf2_target_partition
// int s_from_nsboot_varm_ram_trash_get_uf2_target_partition(resident_partition_t *partition_out, uint family_id) {
s_from_nsboot_varm_ram_trash_get_uf2_target_partition:
    push {r0, r1, r4, lr}
.cpu cortex-m33
    rcp_canary_get_nodelay r4, CTAG_S_FROM_NSBOOT_VARM_RAM_TRASH_GET_UF2_TARGET_PARTITION
    hx_bit_pattern_true r2
.cpu cortex-m23
    movs r1, #RESIDENT_PARTITION_SIZE
    movs r1, #RESIDENT_PARTITION_SIZE
    bl varm_to_s_native_validate_ns_buffer_internal
    bpl 1f
.cpu cortex-m33
    rcp_btrue r1
    rcp_btrue r1
.cpu cortex-m23
    ldr r1, [sp]
.cpu cortex-m33
    rcp_iequal r0, r1
.cpu cortex-m23
    ldr r0, [sp, #4]
    ldr r2, =SRAM0_BASE
    // s_varm_ram_trash_get_uf2_target_partition(family_id, partition_out)
    // aka s_varm_ram_trash_get_uf2_target_partition_workarea(family_id, partition_out, SRAM0_BASE);
    bl s_varm_ram_trash_get_uf2_target_partition_workarea
1:
.cpu cortex-m33
    rcp_canary_check_nodelay r4, CTAG_S_FROM_NSBOOT_VARM_RAM_TRASH_GET_UF2_TARGET_PARTITION
.cpu cortex-m23
    pop {r1, r2, r4, pc}

