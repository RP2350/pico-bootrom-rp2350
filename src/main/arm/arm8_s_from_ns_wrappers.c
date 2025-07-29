/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "bootrom.h"
#include "hardening.h"
#include "varm_checked_flash.h"

// The functions in this file are ARM8 because they are only used by SG on
// ARM. If the API is available on RISC-V under varmulet, these wrappers are
// bypassed and the varm_ function is called directly

#include "arm8_validate_ns_buffer.h"

// There is no functional difference between an SG implementation in main text
// vs in NSC text, since the actual SG instruction is separated from the
// implementation due to shared permission/return code. So, push them to
// whichever section to satisfy code layout constraints.
#define __sg_impl_exempt

#if !ASM_SIZE_HACKS
int __attribute__((noinline, section(".secure_gateways.second"))) s_from_ns_arm8_api_flash_runtime_to_storage_addr(uintptr_t addr) {
    canary_entry(S_FROM_NS_ARM8_API_FLASH_RUNTIME_TO_STORAGE_ADDR);
    // Only allow SAU-NonSecure addresses to be translated via this gateway
    hx_bool addr_ok;
    addr = (uintptr_t)call_s_native_validate_ns_buffer((const void *) addr, 1, hx_false(), addr_ok);
    if (hx_is_false(addr_ok)) {
        // already the case
        // addr = (uintptr_t)BOOTROM_ERROR_INVALID_ADDRESS;
        goto runtime_to_storage_addr_done;
    }
    hx_assert_true(addr_ok);
    addr = s_varm_api_flash_runtime_to_storage_addr(addr);
    runtime_to_storage_addr_done:
    canary_exit_return(S_FROM_NS_ARM8_API_FLASH_RUNTIME_TO_STORAGE_ADDR, (int)addr);
}
#else
int __attribute__((naked)) __attribute__((noinline, section(".secure_gateways.second"))) s_from_ns_arm8_api_flash_runtime_to_storage_addr(__unused uintptr_t addr) {
    // slightly painful, but need to inline because GCC insists on using expensive 4 byte variant of pop {pc}
    pico_default_asm_volatile(
         // rcp_canary_get_nodelay r1, S_FROM_NS_ARM8_API_FLASH_RUNTIME_TO_STORAGE_ADDR
         "mrc2 p7, #0, r1, c%c[canary_hi], c%c[canary_lo], #1\n"
        "push {r1, r2, r3, lr}\n"
        "movs r1, #1\n"
        "mov.w r2, %[bit_pattern_false]\n"
        "bl s_native_validate_ns_buffer_internal\n"
        // r1 is addr_valid hx_bool; if hx_false() r0 already has error code, otherwise r0 is still addr
        "cmp r1, #0\n"
        "bge 1f\n"
        // rcp_btrue r1
        "mcr p7, #2, r1, c0, c0, #0\n"
        "bl s_varm_api_flash_runtime_to_storage_addr\n"
        "1:\n"
        "pop {r1}\n" // canary value
         // rcp_canary_check_nodelay r1, S_FROM_NS_ARM8_API_FLASH_RUNTIME_TO_STORAGE_ADDR
        "mcr2 p7, #0, r1, c%c[canary_hi], c%c[canary_lo], #1\n"
        // overwrite canary value in r1 with input r2
        "pop {r1, r3, pc}\n"
        :
        : [canary_hi] "i" (((CTAG_S_FROM_NS_ARM8_API_FLASH_RUNTIME_TO_STORAGE_ADDR) >> 4) & 0xf),
          [canary_lo] "i" ((CTAG_S_FROM_NS_ARM8_API_FLASH_RUNTIME_TO_STORAGE_ADDR) & 0xf),
          [bit_pattern_false] "i" (HX_BIT_PATTERN_FALSE)
        :"cc", "memory"
    );
}
#endif
