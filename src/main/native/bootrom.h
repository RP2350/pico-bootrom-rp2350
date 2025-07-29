/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "pico.h"
#include "bootrom_common.h"
#include "bootrom_assert.h"
#include "hardening.h"
#include "rcp_tags.h"
#include "varm_to_riscv_hints.h"
#include "boot/bootrom_constants.h"
#include "boot/picobin.h"
#ifndef SB_TEST
#include "bootrom_layout.h"
#ifndef __riscv
#include "p16.h"
#endif
#endif
#ifndef __ASSEMBLER__
#include "bootrom_error.h"
#include "boot/picoboot.h"
#include "hardware/structs/bootram.h"
#include "hardware/structs/otp.h"
#include "hardware/structs/sau.h"
#include "hardware/structs/mpu.h"
#include "hardware/sync.h"
#include "native_exports.h"
#endif

// store some variables at well known offsets so we can load them via offset from a zero register
#define PARSED_BLOCK_LOOP_IMAGE_DEF_ROLLED_ENTRY_POINT_ADDR_OFFSET 0x4c
#define RESIDENT_PARTITION_SIZE 8

// CLK_SYS FREQ ON STARTUP for DIV=8 (in MHz)
// +-----------------------
// | min    |  3.6        |
// | typ    |  13.0       |
// | max    |  22.6       |
// +----------------------+
// (multiply x4 for DIV=2)
#define ROSC_MHZ_MAX 92
#define ROSC_MHZ_TYP 52

#define BOOTROM_SHORT_REBOOT_MS 1

#define BOOT_ONCE_NSBOOT_API_DISABLED 0
// detect that OTP boot was used (nice to know if someone has injected some code in the boot path)
#define BOOT_ONCE_OTP_BOOT_TAKEN 1

#define INVALID_STACK_PTR 0xf0000000 // chosen to make things easier for varmulet_hooks_bootrom which has this in a reg already

#define VECTORED_BOOT_MAGIC          0xb007c0d3
// we reuse the same pattern to save on code/data space
#define REBOOT_TO_MAGIC_PC           VECTORED_BOOT_MAGIC

// Must match the definition of s_native_default_xip_setup in varm_misc.S
#define DEFAULT_ARM_XIP_SETUP_SIZE_BYTES 12

// Must match the definition of s_native_default_xip_setup in riscv_bootrom_rt0.S
#define DEFAULT_RISCV_XIP_SETUP_SIZE_BYTES 16

// Static region assignment: 0=bootram_core1, 1=SRAM, 2=XIP, 3=ROM:rodata+ns,
#define BOOTROM_MPU_REGION_BOOTRAM_CORE1 0
#define BOOTROM_MPU_REGION_RAM           1
#define BOOTROM_MPU_REGION_FLASH         2
#define BOOTROM_MPU_REGION_SECURE_XN     3

#define BOOTROM_MPU_REGION_COUNT         4

#define XIP_CACHE_MAINTENANCE_OP_INVALIDATE_BY_SET_WAY 0
#define XIP_CACHE_MAINTENANCE_OP_CLEAN_BY_SET_WAY 1
#define XIP_CACHE_MAINTENANCE_OP_INVALIDATE_BY_ADDRESS 2
#define XIP_CACHE_MAINTENANCE_OP_CLEAN_BY_ADDRESS 3
#define XIP_CACHE_MAINTENANCE_OP_PIN_AT_ADDRESS 7

#ifndef __ASSEMBLER__
static_assert(sizeof(resident_partition_t) == RESIDENT_PARTITION_SIZE, "");
#if !USE_64K_BOOTROM
#define __sg_filler __attribute__((section(".sg_fillers")))
#else
#define __sg_filler
#endif

#define uint32s_to_uint64(a, b) (((uint64_t)(b) << 32)|(a))
#define uint64_to_uint32_0(l) ((uint32_t)l)
#define uint64_to_uint32_1(l) ((uint32_t)((l)>>32u))

#define __exported_from_arm __used __noinline

// GCC likes to sometimes do
// movw rx, #offset
// ldr ry, =base_ptr
// str rz, [ry, rx]
//
// even if base_ptr is only used once... in this case it is better to skip the movw obviously and save 4 bytes:
//
// ldr ry, =base_ptr+offset
// str rz, [ry]
#define gcc_avoid_single_movw_plus_ldr(x) (*__get_opaque_ptr(&(x)))

// Similar to above: force the compiler to generate a struct member pointer by first materialising a
// pointer to base_member, then accessing actual_member at an offset. Profitable when a pointer to
// base_member is already pooled, and actual_member is within load/store offset range.
#define struct_member_ptr_shared_base(inst, base_member, actual_member) ((typeof(&((inst)->actual_member))) \
    ((uintptr_t)__get_opaque_ptr(&(inst)->base_member) + offsetof(typeof(*(inst)), actual_member) - offsetof(typeof(*(inst)), base_member)))

#if TAIL_CALL_HACKS
// need to manually mark symbols tail-called from inline asm as used
#define __used_if_tail_call_hacks __used
#else
#define __used_if_tail_call_hacks
#endif

#define debug_label(l) ({asm volatile ( "___" __STRING(l) ":");})

// note:: we chose a small number to keep this small, though we still want to be relatively confident that
// the pt data in flash hasn't changed. 32 bits seems fine for this
#define PARTITION_TABLE_SHA256_HASH_WORDS 1

// rp2040, rp2350 arm, rp2350 riscv, global, data
static_assert((int8_t)PARTITION_TABLE_NO_PARTITION_INDEX == -1, "");

typedef struct uf2_target_workarea uf2_target_workarea_t;

// sp can be 0 to use current stack
void varm_to_s_native_secure_call_pc_sp(uint32_t pc, uint32_t sp, uint32_t pc_xor, uint32_t xor);
// void s_varm_secure_call(uint32_t pc, uint32_t sp);

int s_varm_api_get_partition_table_info(uint32_t *out_buffer, uint32_t out_buffer_word_size, uint32_t flags);

// this is the internal method used by the previous
int s_varm_crit_get_pt_partition_info(uint32_t *out_buffer, uint32_t out_buffer_word_size, uint32_t flags_and_partition, const uint32_t *pt_item_data, uint partition_count, bool first_load_from_buffer);
int s_varm_ram_trash_get_uf2_target_partition_workarea(uint32_t family_id, resident_partition_t *partition_out, uf2_target_workarea_t *uf2_target_workarea);

int s_varm_api_get_sys_info(uint32_t *buffer, uint32_t buffer_size_words, uint32_t flags);
int s_varm_api_reboot(uint32_t flags, uint32_t delay_ms, uint32_t p0, uint32_t p1);
uint s_varm_step_safe_api_crit_bootrom_state_reset(uint reset_flags);

void varm_callable(s_native_busy_wait_at_least_cycles)(uint32_t cycles);
struct parsed_block_loop;
void varm_callable(s_native_crit_init_default_xip_setup_and_enter_image_thunk)(/*bootrom_xip_mode_t*/int8_t mode, uint clkdiv,
    uint32_t pc, uint32_t sp, uint32_t sp_lim, uint32_t vector_table,
    struct parsed_block_loop *parsed_block_loop);
void varm_callable(s_native_crit_launch_nsboot)(void);

void __attribute__((noreturn)) varm_and_native(dead)(void);
void __attribute__((noreturn)) varm_and_native(wait_rescue)(void);

static __force_inline int inline_s_lock_check(uint lock_type) {
    bootrom_assert(MISC, lock_type <= BOOTROM_LOCK_ENABLE);
    // note bits are 1 for unowned, 0 for owned
    uint stat = (~bootram_hw->bootlock_stat) & ((1u << BOOTROM_LOCK_ENABLE) | (1u << lock_type));
    if (stat < (1u << BOOTROM_LOCK_ENABLE) || stat == ((1u << BOOTROM_LOCK_ENABLE) | (1u << lock_type))) {
        return BOOTROM_OK;
    }
    // don't ask me how i figured this out, but this saves 28 bytes ;-)
    pico_default_asm_volatile("" : : : "memory");
    return BOOTROM_ERROR_LOCK_REQUIRED;
}

static __force_inline void __attribute__((noreturn)) sudden_death(void) {
#ifndef __riscv
    pico_default_asm_volatile(
            ".cpu cortex-m33\n"
            "cdp p7, #0, c0, c0, c0, #1\n" // rcp_panic
            ".cpu cortex-m23\n"
            "b.w native_dead\n"
    );
#else
    native_dead();
#endif
    __builtin_unreachable();
}

static __force_inline bool call_varm_is_sram_or_xip_ram(uint32_t addr) {
    register union {
        uint32_t u;
        bool b;
    } r0 asm ("r0");
    r0.u = addr;
    pico_default_asm_volatile(
        "bl varm_is_sram_or_xip_ram\n"
        : "+l" (r0)
        :
        : "cc", "ip", "lr"
        );
    return r0.b;
}

static __force_inline uint32_t call_s_varm_step_safe_otp_read_rbit3_guarded(uint row) {
    register uint r0 asm ("r0") = row;
    pico_default_asm_volatile(
        "bl s_varm_step_safe_otp_read_rbit3_guarded\n"
        : "+l" (r0)
        :
        : "ip", "lr", "cc"
        );
    return r0;
}

#endif // __ASSEMBLER__
