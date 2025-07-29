/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#pragma once

#include "bootrom.h"

// Validate a NonSecure buffer.
//
// Entire buffer must fit in range XIP_BASE -> SRAM_END, and must be
// accessible from NS caller according to SAU + NS MPU (privileged or not
// based on current processor IPSR and NS CONTROL flag). We also allow
// buffers in USB RAM if this is granted to NS via ACCESSCTRL -- note USB RAM
// is IDAU-Exempt so will fail tt* checks.
//
// Note this is a arm6-to-native call so that it can be stubbed on RISC-V,
// avoiding execution of Armv8-M instructions under emulation. This allows
// nsboot-to-secure shims (accessed via nsboot's service SG) to be shared.

#if 0 // not currently used
#define call_varm_to_s_native_validate_ns_buffer(buf, size, write, buffer_ok) \
({ \
    register uintptr_t r0 asm ("r0") = (uintptr_t)(buf); \
    register uint32_t r1 asm ("r1"); \
    register hx_bool r2 asm ("r2") = write; \
    pico_default_asm_volatile( \
        "movs r1, #0\n" /* dummy write */ \
        "mov r1, %[sz]\n" \
        "bl varm_to_s_native_validate_ns_buffer_internal\n" \
        : "+l" (r0), "=&l" (r1), "+l" (r2)\
        : [sz] "r" (size) \
        : "r3", "ip", "cc", "memory" \
    ); \
    (buffer_ok).v = r1; \
    (void *)r0; \
})

#define call_varm_to_s_native_validate_ns_buffer_const_size(buf, size, write, buffer_ok) \
({ \
    register uintptr_t r0 asm ("r0") = (uintptr_t)(buf); \
    register uint32_t r1 asm ("r1"); \
    register hx_bool r2 asm ("r2") = write; \
    if (size < 256) { \
        pico_default_asm_volatile( \
            "movs r1, %[sz]\n" /* dummy write but with correct value as <256 is cheap */ \
            "movs r1, %[sz]\n" \
            "bl varm_to_s_native_validate_ns_buffer_internal\n" \
            : "+l" (r0), "=&l" (r1), "+l" (r2)\
            : [sz] "i" (size) \
            : "r3", "ip", "cc", "memory" \
        ); \
    } else { \
        pico_default_asm_volatile( \
            "movs r1, #0\n" /* dummy write */ \
            "movw r1, %[sz]\n" \
            "bl varm_to_s_native_validate_ns_buffer_internal\n" \
            : "+l" (r0), "=&l" (r1), "+l" (r2)\
            : [sz] "i" (size) \
            : "r3", "ip", "cc", "memory" \
        ); \
    } \
    (buffer_ok).v = r1; \
    (void *)r0; \
})

// beware - parameters should be registers register variables in r0, r1, r2
#define call_s_native_validate_ns_buffer(buf, size, write, buffer_ok) \
({ \
    register uintptr_t r0 asm ("r0") = (uintptr_t)(buf); \
    register uint32_t r1 asm ("r1") = size; \
    register hx_bool r2 asm ("r2") = write; \
    pico_default_asm_volatile( \
        "bl s_native_validate_ns_buffer_internal\n" \
        : "+l" (r0), "+l" (r1), "+l" (r2)\
        : \
        : "r3", "ip", "cc", "memory" \
    ); \
    (buffer_ok).v = r1; \
    (void *)r0; \
})
#endif

uint64_t varm_callable(s_native_validate_ns_buffer_internal)(const void *addr, uint32_t size, hx_bool write);
