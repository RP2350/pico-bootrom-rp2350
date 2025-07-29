/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// These functions are shims between the nsboot service SG, and the secure
// varm-able functions which implement its services. They implement parameter
// checking, and/or conversion to the internal API
//
// These functions are compiled for varm compatiblity, so that they can be run under RISC-V
// emulation. The NS buffer validation is dispatched back to native
// code so that Armv8-M tt* instructions can be used in a native Arm context.
//
// Note funcitons of the form s_from_nsboot_ are only called via the NSBOOT SG, however
// there are some functions here of the form s_from_ns_ which (because their behavior
// is the same when calling from NS or NSBOOT) may be called from either

#include "bootrom.h"

#include "varm_checked_flash.h"
#include "nsboot_secure_calls.h"

int __noinline s_from_ns_varm_api_otp_access_internal(hx_xbool secure, uint32_t buf_len, otp_cmd_t cmd, aligned4_uint8_t *buf);

int s_from_nsboot_varm_otp_access(aligned4_uint8_t *buf, uint32_t buf_len, otp_cmd_t cmd) {
    // regalloc: use prolog-saved reg to avoid separate spill over call
    canary_entry_reg(r4, S_FROM_NSBOOT_VARM_OTP_ACCESS);
    // note: we assume interrupts are enabled at this point to save space, as there is no reason
    // the NS code would call us with IRQs disabled
    bootrom_assert(MISC, !save_and_disable_interrupts());
    disable_irqs(); // uint32_t save = save_and_disable_interrupts();
    // OTP locks are already advanced, so pass true for secure access
    int rc = s_from_ns_varm_api_otp_access_internal(hx_otp_secure_true(), buf_len, cmd, buf);
    enable_irqs(); // restore_interrupts(save);
    canary_exit_return(S_FROM_NSBOOT_VARM_OTP_ACCESS, rc);
}

#if FEATURE_EXEC2
int s_from_ns_varm_picoboot_exec2(struct picoboot_exec2_cmd *_cmd) {
    int rc;
    canary_entry(S_FROM_NS_VARM_PICOBOOT_EXEC2);
    struct picoboot_exec2_cmd *cmd = __builtin_assume_aligned(_cmd, 4);
    hx_bool addr_ok;
    cmd = varm_to_s_native_api_validate_ns_buffer(cmd, sizeof(*cmd), hx_false(), &addr_ok);
    struct picoboot_exec2_cmd cpy;
    bootram->always.nonce++;
    if (hx_is_xtrue(hx_step_safe_get_boot_flagx(OTP_DATA_BOOT_FLAGS0_DISABLE_BOOTSEL_EXEC2_LSB))) {
        rc = BOOTROM_ERROR_NOT_PERMITTED;
        goto exec2_done;
    }
    rc = BOOTROM_ERROR_INVALID_ADDRESS;
    if (hx_is_true(addr_ok)) {
        cpy = *cmd;
        varm_to_s_native_api_validate_ns_buffer((void *)cpy.image_base, cpy.image_size, hx_false(), &addr_ok);
        uint aligns = cpy.image_base | cpy.image_size | cpy.workarea_base | cpy.workarea_size;
        if (hx_is_true(addr_ok) && !(aligns & 0x1f)) {
            varm_to_s_native_api_validate_ns_buffer((void *)cpy.workarea_base, cpy.workarea_size, hx_true(), &addr_ok);
            if (hx_is_true(addr_ok)) {
                hx_assert_true(addr_ok);
                if (cpy.workarea_size < sizeof(scan_workarea_t)) {
                    rc = BOOTROM_ERROR_BUFFER_TOO_SMALL;
                    goto exec2_done;
                }
                scan_workarea_t *scan_workarea = (scan_workarea_t *) cpy.workarea_base;
                s_varm_crit_get_non_booting_boot_scan_context(scan_workarea, true, false);
                boot_scan_context_t *ctx = &scan_workarea->ctx_holder.ctx;
                ctx->exec2 = hx_true();
                ctx->current_search_window.base = cpy.image_base;
                ctx->current_search_window.size = cpy.image_size;

                branch_under_varmulet(varm);
                // make regions secure-only, so
                //   a) NS can't mess with them
                //   b) we can execute from the image
                // this will overlap existing regions thus making it secure again
                INIT_SAU_REGION_D(4, cpy.image_base, cpy.image_base + cpy.image_size, false, true);
                INIT_SAU_REGION_D(5, cpy.workarea_base, cpy.workarea_base + cpy.workarea_size, false, true);
                varm:

                rc = s_varm_crit_ram_trash_checked_ram_or_flash_window_launch(ctx);
                branch_under_varmulet(varm2);
                DISABLE_SAU_REGION(4);
                DISABLE_SAU_REGION(5);
                varm2: ;
            }
        }
    }
    exec2_done:
    canary_exit_return(S_FROM_NS_VARM_PICOBOOT_EXEC2, rc);
}
#endif

