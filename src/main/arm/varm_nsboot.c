/*
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "varm_boot_path.h"
#include "bootrom_otp.h"
#include "native_generic_flash.h"
#include "nsboot_config.h"
#include "varm_resets.h"
#include "boot/uf2.h"
#include "varm_flash_permissions.h"
#include "hardware/structs/accessctrl.h"
#include "hardware/structs/clocks.h"
#include "hardware/structs/ioqspi.h"
#include "hardware/structs/pll.h"
#include "hardware/structs/psm.h"
#include "hardware/structs/rosc.h"
#include "hardware/structs/ticks.h"
#include "hardware/structs/usb.h"
#include "hardware/structs/xosc.h"
#if USE_BOOTROM_GPIO
#include "hardware/structs/iobank0.h"
#include "hardware/structs/padsbank0.h"
#include "hardware/gpio.h"
#endif

#if defined(__ARM_ARCH_8M_MAIN__) || !defined(__ARM_ARCH_8M_BASE__)
#error this must be compiled with armv8m-base
#endif

#define varm_to_native_memcpy dont_use_this_here
#define varm_to_native_memset dont_use_this_here
#define varm_to_native_memset0 dont_use_this_here

static_assert(PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS == PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS << (RP2040_FAMILY_ID - RP2040_FAMILY_ID), "");
static_assert(PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_ABSOLUTE_BITS == PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS << (ABSOLUTE_FAMILY_ID - RP2040_FAMILY_ID), "");
static_assert(PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_DATA_BITS == PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS << (DATA_FAMILY_ID - RP2040_FAMILY_ID), "");
static_assert(PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2350_ARM_S_BITS == PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS << (RP2350_ARM_S_FAMILY_ID - RP2040_FAMILY_ID), "");
static_assert(PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2350_RISCV_BITS == PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS << (RP2350_RISCV_FAMILY_ID - RP2040_FAMILY_ID), "");
static_assert(PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2350_ARM_NS_BITS == PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS << (RP2350_ARM_NS_FAMILY_ID - RP2040_FAMILY_ID), "");

static int s_varm_crit_ram_trash_find_uf2_target_partition(uf2_target_workarea_t *uf_2_target_workarea, bool unowned_only, bool bootable_only);

// Get clk_sys and clk_usb at 48 MHz, and clk_ref at 24 MHz, before entering
// nsboot. This function is assumed to run after a full reset of the
// oscillators, PLLs and clock generators.
//
// This means that initially (as of silicon version A3):
//
// - ROSC is running, at approximately 48 MHz (12 at reset; increased 4x early in varm_boot_path)
// - XOSC is not running
// - PLLs are not running
// - clk_ref is running from ROSC with div=4 (1 at reset; increased 4x early in varm_boot_path)
// - clk_sys is running from ROSC with div=1 (*not* chained from clk_ref; hardware difference from A2 -> A3)
// - clk_usb is disabled, selects USB PLL, div=1

static __force_inline void s_varm_nsboot_clock_setup(uint32_t pll_reset_mask) {
    // The final clk_ref divisor for NSBOOT (=2) is less than the initial
    // divisor, so we don't touch it until the end.

    s_varm_step_safe_reset_unreset_block_wait_noinline(pll_reset_mask);

    hx_bool otp_osc_pll_setup_valid = hx_step_safe_get_boot_flag(
            OTP_DATA_BOOT_FLAGS0_ENABLE_BOOTSEL_NON_DEFAULT_PLL_XOSC_CFG_LSB
    );

    // Use XOSC and PLL to derive 48 MHz base frequency. This code attempts to not drop through
    // if no crystal is present, because this is where we end up on *any* board with a blank
    // flash, and we need to be able to attach the debugger. Ideally XI should be grounded if no
    // crystal is present, so the STABLE counter will never complete. Poor designs might leave
    // XI floating, in which case we just have to hope the PLL doesn't lock.
    if (hx_is_true(otp_osc_pll_setup_valid)) {
        // A specific XOSC configuration has been programmed in OTP, so use it.
        uint32_t xosc_config = inline_s_otp_read_ecc_guarded(OTP_DATA_BOOTSEL_XOSC_CFG_ROW);
        uint xosc_range = (xosc_config & OTP_DATA_BOOTSEL_XOSC_CFG_RANGE_BITS) >>
                                                                               OTP_DATA_BOOTSEL_XOSC_CFG_RANGE_LSB;
        uint xosc_startup = (xosc_config & OTP_DATA_BOOTSEL_XOSC_CFG_STARTUP_BITS) >>
                                                                                   OTP_DATA_BOOTSEL_XOSC_CFG_STARTUP_LSB;
        xosc_hw->startup = xosc_startup;
        xosc_hw->ctrl = (XOSC_CTRL_FREQ_RANGE_VALUE_1_15MHZ + xosc_range) << XOSC_CTRL_FREQ_RANGE_LSB |
                        (XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB);
    } else {
        // OTP has told us nothing about XOSC, so assume the defaults are
        // good, and just switch it on.
        xosc_hw->ctrl = XOSC_CTRL_ENABLE_VALUE_ENABLE << XOSC_CTRL_ENABLE_LSB;
    }
    while (!(xosc_hw->status & XOSC_STATUS_STABLE_BITS));

    // Default USB PLL setup for 12 MHz crystal:
    // - VCO freq 1200 MHz, so feedback divisor of 100. Range is 750 MHz to 1.6 GHz
    // - Postdiv1 of 5, down to 240 MHz (appnote recommends postdiv1 >= postdiv2)
    // - Postdiv2 of 5, down to 48 MHz
    //
    // Total postdiv of 25 means that too-fast xtal will push VCO out of
    // lockable range *before* clk_sys goes out of closure (factor of 1.88)
    uint pll_refdiv = 1;
    uint pll_fbdiv = 100;
    uint pll_postdiv1 = 5;
    uint pll_postdiv2 = 5;

    if (hx_is_true(otp_osc_pll_setup_valid)) {
        uint32_t pll_config = inline_s_otp_read_ecc_guarded(OTP_DATA_BOOTSEL_PLL_CFG_ROW);
        pll_refdiv = 1 + ((pll_config & OTP_DATA_BOOTSEL_PLL_CFG_REFDIV_BITS) >>
                                                                              OTP_DATA_BOOTSEL_PLL_CFG_REFDIV_LSB);
        pll_fbdiv = (pll_config & OTP_DATA_BOOTSEL_PLL_CFG_FBDIV_BITS) >>
                                                                       OTP_DATA_BOOTSEL_PLL_CFG_FBDIV_LSB;
        pll_postdiv1 = (pll_config & OTP_DATA_BOOTSEL_PLL_CFG_POSTDIV1_BITS) >>
                                                                             OTP_DATA_BOOTSEL_PLL_CFG_POSTDIV1_LSB;
        pll_postdiv2 = (pll_config & OTP_DATA_BOOTSEL_PLL_CFG_POSTDIV2_BITS) >>
                                                                             OTP_DATA_BOOTSEL_PLL_CFG_POSTDIV2_LSB;
    }

    pll_usb_hw->cs = pll_refdiv << PLL_CS_REFDIV_LSB;
    pll_usb_hw->fbdiv_int = pll_fbdiv;
    // (delay for output synchroniser on fbdiv_int -- should be fine if you
    // poll for LOCK, but trips an assert on the PLL model.)
    (void)pll_usb_hw->fbdiv_int;
    uint32_t prim_bits =
        (pll_postdiv1 << PLL_PRIM_POSTDIV1_LSB) |
        (pll_postdiv2 << PLL_PRIM_POSTDIV2_LSB);
    pll_usb_hw->prim = prim_bits;

    // Power up, wait for lock (note these are all power-*down* bits).
    // We want to write zero but we already have a close-enough value:
    static_assert((PLL_PRIM_BITS & PLL_PWR_BITS) == 0);
    pll_usb_hw->pwr = prim_bits;

    // PLL may intermittently report lock when XOSC is just floating. Make
    // sure lock stays high for a large consecutive number of reads
    const int consecutive_n_locks = 0xff;
    for (int lock_count = 0; lock_count < consecutive_n_locks; ++lock_count) {
//        if (!(pll_usb_hw->cs & PLL_CS_LOCK_BITS)) {
//            lock_count = 0;
//        }
        static_assert(PLL_CS_LOCK_BITS == 0x80000000, "");
        lock_count &= ((int32_t)pll_usb_hw->cs) >> 31;
    }

    // Glitchless switch of clk_sys to chained clk_ref, before we touch its
    // aux mux. (The write after this one provides delay for the aux side of
    // the glitchless mux to shut off, before we touch the clk_sys aux mux)
    clocks_hw->clk[clk_sys].ctrl = CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_ROSC_CLKSRC << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB;

    // Glitchy switch of clk_ref aux, then clk_sys aux to USB PLL output.
    // (Note clk_usb selects USB PLL by default.)
    clocks_hw->clk[clk_ref].ctrl = CLOCKS_CLK_REF_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_REF_CTRL_AUXSRC_LSB;
    clocks_hw->clk[clk_sys].ctrl =
            (CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB << CLOCKS_CLK_SYS_CTRL_AUXSRC_LSB);

    // (the set_clocks_hw instance here is to avoid pooling a pointer literal
    // for every invocation of e.g. hw_set_bits, since the function call loses
    // the original allocation so misses the struct member optimisations)
    clocks_hw_t *set_clocks_hw = hw_set_alias(clocks_hw);

    // Glitchlessly select clk_ref aux source -- no need to wait for the
    // switch, as ROSC isn't going anywhere.
    set_clocks_hw->clk[clk_ref].ctrl =
                CLOCKS_CLK_REF_CTRL_SRC_VALUE_CLKSRC_CLK_REF_AUX << CLOCKS_CLK_REF_CTRL_SRC_LSB;

    // Glitchless switch of clk_sys to aux source -- no need to wait for the
    // switch as clk_ref isn't going anywhere.
    set_clocks_hw->clk[clk_sys].ctrl = CLOCKS_CLK_SYS_CTRL_SRC_BITS;

    // Enable clk_peri, glitchy switch to clk_sys
    clocks_hw->clk[clk_peri].ctrl = CLOCKS_CLK_PERI_CTRL_ENABLE_BITS |
                                    (CLOCKS_CLK_PERI_CTRL_AUXSRC_VALUE_CLK_SYS << CLOCKS_CLK_PERI_CTRL_AUXSRC_LSB);

    // Enable clk_usb using whatever source was selected
    set_clocks_hw->clk[clk_usb].ctrl = CLOCKS_CLK_USB_CTRL_ENABLE_BITS;

    // Ensure timer and watchdog ticks are running at correct speed (Do this
    // now to give the resus logic a chance to kick in if necessary)
    ticks_hw->ticks[TICK_TIMER0].cycles = 24u;
    ticks_hw->ticks[TICK_WATCHDOG].cycles = 24u;
    static_assert(TICKS_TIMER0_CTRL_ENABLE_BITS == 1, "");
    static_assert(TICKS_WATCHDOG_CTRL_ENABLE_BITS == 1, "");;
    uint32_t one = __get_opaque_value(TICKS_TIMER0_CTRL_ENABLE_BITS);
    ticks_hw->ticks[TICK_TIMER0].ctrl = one; // TICKS_TIMER0_CTRL_ENABLE_BITS
    ticks_hw->ticks[TICK_WATCHDOG].ctrl = one; // TICKS_WATCHDOG_CTRL_ENABLE_BITS

    // Increase clk_ref to its final speed of 24 MHz (DIV=2, down from initial
    // value of 5). No need to wait, it will just run slow for a couple more
    // clk_ref cycles after we change the divisor.
    clocks_hw->clk[clk_ref].div = one << (CLOCKS_CLK_REF_DIV_INT_LSB + 1);
}

/**
 *
 * @param usb_activity_gpio_config pin number for activity LED or -1 for none
 * @param bootselFlags
 */
void s_varm_crit_nsboot(mpu_hw_t *mpu_on_arm, uint32_t usb_activity_pin, uint32_t bootselFlags, uint serial_mode) {
    printf("Entering _nsboot gpio=%08x flags=%02x mode=%d\n", usb_activity_pin, (int)bootselFlags, serial_mode);
//    hw_set_bits(&psm_hw->frce_off, PSM_FRCE_OFF_PROC1_BITS);
//    hw_clear_bits(&psm_hw->frce_off, PSM_FRCE_OFF_PROC1_BITS);
    // we have a lot of constants which are powers of 2, so we pass these thru quite a lot of the top of this function
    // in r0
    static_assert((1u << 12) == REG_ALIAS_CLR_BITS - REG_ALIAS_SET_BITS, "");
    static_assert((1u << 24) == PSM_FRCE_OFF_PROC1_BITS, "");
    io_rw_32 *psm_set = hw_set_alias(&psm_hw->frce_off);
    uint set_to_clr;
    register uint r0 asm ("r0");
    pico_default_asm_volatile(
        "movs %[r0], #1\n"
        "lsls %[set_to_clr], %[r0], #12\n"
        "lsls %[r0], #24\n"
        "str %[r0], [%[psm_set]]\n"
        "str %[r0], [%[psm_set], %[set_to_clr]]\n"
        : [set_to_clr] "=&l" (set_to_clr), [r0] "=&l" (r0)
        : [psm_set] "l" (psm_set)
        : "cc"
    );

    // Move the varm register file out of USB RAM before we toggle the USB reset. At this point it's
    // ok to trash main RAM, and we only need this register file until we get to native RISC-V code
    // in riscv_nsboot_vm.c, at which point we create a new varmulet instance.

    // r0 is currently PSM_FRCE_OFF_PROC1_BITS
    static_assert(SRAM_BASE == (PSM_FRCE_OFF_PROC1_BITS << 5), "");
    const uint varm_relocate_opcode = HINT_OPCODE_BASE + 16 * HINT_RELOCATE_VARM_REGISTERS;
    pico_default_asm_volatile (
        "lsls r0, #5\n"
        ".hword %c[op_code]\n"
        : [r0] "+l" (r0)
        : [op_code] "i" (varm_relocate_opcode)
        : "cc"
    );

    // r0 is now SRAM_BASE
    static_assert((SRAM_BASE >> (29 - RESETS_RESET_PLL_USB_LSB)) == RESETS_RESET_PLL_USB_BITS);
    r0 >>= 29 - RESETS_RESET_PLL_USB_LSB;
    s_varm_nsboot_clock_setup(r0);

    s_varm_step_safe_reset_unreset_block_wait_noinline(
        RESETS_RESET_USBCTRL_BITS |
#if !MINI_PRINTF
        // In production bootrom we can hoist the UART reset up here for size
        RESETS_RESET_UART0_BITS
#else
        // Try to avoid killing UART printf until we actually go down the UART
        // boot path
        0
#endif
    );


    // Swap USB DP/DM pins if requested via OTP. USB is not set up until after
    // entering nsboot proper, but it is not reset after this point either.
    if (s_varm_step_safe_otp_read_rbit3_guarded(OTP_DATA_USB_BOOT_FLAGS_ROW) & OTP_DATA_USB_BOOT_FLAGS_DP_DM_SWAP_BITS) {
        hw_set_bits(&usb_hw->muxing, USB_USB_MUXING_SWAP_DPDM_BITS);
    }

    if (serial_mode != BOOTSEL_MODE_UART) {
        // Setup flash once, before entering nsboot. The XIP modes used by the bootrom all have serial
        // prefixes, so can be freely mixed with serial programming commands.
        s_varm_api_crit_connect_internal_flash();
        s_varm_api_crit_flash_exit_xip();
        s_varm_api_crit_flash_select_xip_read_mode(BOOTROM_XIP_MODE_03H_SERIAL, BOOTROM_SPI_CLKDIV_NSBOOT);

        // The resident partition table must be initialised so that the permissioned flash API can be
        // used. Trying to use permissioned flash before the table is loaded will refuse all requests.
        //
        // note: this replaces the flash_ctx in the workspace with a non booting one
        s_varm_crit_load_resident_partition_table(&core0_boot_usbram_workspace, false);
    }

    // note nothing in nsboot_config can be set before this as nsboot_config is in USB RAM !!!
    s_varm_step_safe_crit_mem_erase_by_words_const_size(USBCTRL_DPRAM_BASE, USBCTRL_DPRAM_SIZE, true);
    nsboot_config->chip_id = *struct_member_ptr_shared_base(bootram, always, always.chip_id);
    printf("locking down otp for nsboot\n");
    uint32_t page = 0;
    uint32_t page2 = __get_opaque_value(0u);
    do {
        uint32_t sw_lock = call_s_otp_advance_bl_to_s_value(0xff, page); // note top bit must be set in param
        uint32_t sw_lock2 = call_s_otp_advance_bl_to_s_value(0xcf, page2); // note top bit must be set in param
        static_assert(OTP_SW_LOCK0_BITS == 0xf, ""); // we rely on the fact that other bits in sw_lock/sw_lock2 are ignored
#if !ASM_SIZE_HACKS
        // these are OR writes
        otp_hw->sw_lock[page] = sw_lock;
        otp_hw->sw_lock[page] = sw_lock2;
        hx_assert_equal2i(sw_lock, sw_lock2);
        uint32_t sw_lock_verify = otp_hw->sw_lock[page] & sw_lock;
#else
        uint32_t t0 = page * 4;
        uint32_t sw_lock_verify = (uintptr_t)&otp_hw->sw_lock[0];
        pico_default_asm_volatile(
            // this is an OR write
            "str %[sw_lock], [%[sw_lock_verify], %[t0]]\n"
            // this is an OR write
            "str %[sw_lock2], [%[sw_lock_verify], %[t0]]\n"
            ".cpu cortex-m33\n"
            "mcrr p7, #7, %[sw_lock], %[sw_lock2], c0\n" // rcp_iequal
            ".cpu cortex-m23\n"
            "ldr %[sw_lock_verify], [%[sw_lock_verify], %[t0]]\n"
            "ands %[sw_lock_verify], %[sw_lock]\n"
            : [sw_lock_verify] "+l" (sw_lock_verify), [sw_lock2] "+l" (sw_lock2)
            : [t0] "l" (t0), [sw_lock] "l" (sw_lock)
        );
#endif
//            printf(" swlock readback %08x\n", (int)sw_lock_verify);
//        hx_assert_equal2i(sw_lock_verify & sw_lock, sw_lock);
//        hx_assert_equal2i(sw_lock_verify & OTP_DATA_PAGE0_LOCK1_LOCK_NS_BITS, OTP_DATA_PAGE0_LOCK1_LOCK_NS_BITS);
        static_assert(OTP_SW_LOCK0_BITS == 0xf, "");
        // use get_opaque_value so that compiler does two separate shifts of 28 - it's clever otherwise
        pico_default_asm_volatile(
            "adds %[sw_lock2], #1\n"
            "lsls %[sw_lock2], #28\n"
            "lsls %[sw_lock_verify], #28\n" // this will clear C, so we subtract an extra 1 below
            "sbcs %[sw_lock2], %[sw_lock_verify]\n"
            "lsrs %[sw_lock2], #27\n" // if all is well this will be 1
            "adds %[page2], %[sw_lock2]\n" // so we can increment page2
            : [sw_lock_verify] "+l" (sw_lock_verify),
              [page2] "+l" (page2),
              [sw_lock2] "+l" (sw_lock2)
            : [sw_lock] "l" (sw_lock) // keep this alive so we don't share regs
            :"cc"
        );
        // note we don't need to reset this around the loop, since the check increments the value
        rcp_count_check_nodelay(STEPTAG_S_OTP_ADVANCE_BL_TO_S_VALUE + 1);
        page++;
        hx_assert_equal2i(page, page2);
    } while (page < NUM_OTP_PAGES);
    hx_set_step(STEPTAG_NSBOOT_OTP_ADVANCE);
    uint32_t num_pages;
    uint32_t num_pages2;
    pico_default_asm_volatile(
        "movs %0, %2\n"
        "movs %1, %2\n"
        : "=&l" (num_pages), "=&l" (num_pages2)
        : "i" (NUM_OTP_PAGES)
        : "cc");
    hx_assert_equal2i(page2, num_pages);
    hx_assert_equal2i(page, num_pages2);

    hx_bool disable_usb_msd      = hx_step_safe_get_boot_flag(OTP_DATA_BOOT_FLAGS0_DISABLE_BOOTSEL_USB_MSD_IFC_LSB);
    hx_bool disable_usb_picoboot = hx_step_safe_get_boot_flag(OTP_DATA_BOOT_FLAGS0_DISABLE_BOOTSEL_USB_PICOBOOT_IFC_LSB);
    hx_bool disable_uart_boot    = hx_step_safe_get_boot_flag(OTP_DATA_BOOT_FLAGS0_DISABLE_BOOTSEL_UART_BOOT_LSB);

    // Set up ACCESSCTRL, pin muxing, etc that is specific to each serial mode
    // We only support UART instance 0. Anything else can be implemented
    // using an OTP bootloader.
    uint serial_inst = 0;
    if (serial_mode == BOOTSEL_MODE_USB) {
        hx_assert_or(hx_not(disable_usb_msd), hx_not(disable_usb_picoboot));
    } else if (serial_mode == BOOTSEL_MODE_UART) {
        debug_label(stepx_nsboot_prep_uart_boot);
        // shared set alias object, to avoid unnecessary pointer literals getting pooled:
        accessctrl_hw_t *set_accessctrl_hw = hw_set_alias(accessctrl_hw);
#if MINI_PRINTF
        // Note granting RESETS is safe, since it still only allows NS to reset NS peripherals.
        // However it's only required if the UART is being used for printf (so we defer its reset
        // as late as possible), otherwise UART gets reset already during this secure preamble.
        set_accessctrl_hw->resets = ACCESSCTRL_PASSWORD_BITS | ACCESSCTRL_RESETS_NSP_BITS;
#endif
        // Need to grant UART + pins to NS *before* attempting to select the UART on the pin, as
        // it's impossible to have a Secure function selected on a NonSecure pin.
        set_accessctrl_hw->uart[serial_inst] = ACCESSCTRL_PASSWORD_BITS | ACCESSCTRL_UART0_NSP_BITS;
        // Set up GPIOs now to avoid hassle of passing GPIO numbers to NS code
        set_accessctrl_hw->gpio_nsmask[1] = 0xcu << ACCESSCTRL_GPIO_NSMASK1_QSPI_SD_LSB;
        ioqspi_hw->io[4].ctrl = IO_QSPI_GPIO_QSPI_SD2_CTRL_FUNCSEL_VALUE_UART0_TX <<
            IO_QSPI_GPIO_QSPI_SD2_CTRL_FUNCSEL_LSB;
        ioqspi_hw->io[5].ctrl = IO_QSPI_GPIO_QSPI_SD3_CTRL_FUNCSEL_VALUE_UART0_RX <<
            IO_QSPI_GPIO_QSPI_SD3_CTRL_FUNCSEL_LSB;
        hx_assert_false(disable_uart_boot);
    } else {
        // the caller already sets serial_mode to one of the possible modes above
        // and the compiler knows this because of LTO, so this code is elided.
        //
        // if that were tog change, we'd want to handle this case with some sort
        // of halt.
        pico_default_asm_volatile("hello, I am a bad instruction");
    }
    // need write for cache flush
    inline_s_set_flash_rw_xn(mpu_on_arm);

#if FEATURE_UART_BOOT_SELECTABLE_INSTANCE
    nsboot_config->serial_mode_and_inst = (uint8_t)(serial_mode | (serial_inst << 4u));
#else
    nsboot_config->serial_mode_and_inst = (uint8_t)serial_mode;
#endif
    debug_label(stepx_nsboot_xip_setup);

    // Flush cache before pinning it. This is not required for functional
    // correctness, but avoids X propagation in ASIC sim (with non-behavioural
    // memories) caused by the cache implicitly doing an invalidate-by-tag
    // before the actual pinning, which drives chip enable to X because the
    // tag memory is initially all Xs. No, Graham, we can't remove this :-)
    s_varm_api_crit_flash_flush_cache();

    // Pin entire XIP cache at top of cached address space, in case a RAM-only
    // binary wants to write to it. This will require a cache flush before
    // using the cache as a cache again. (note it is also used for
    // flash writing bitmaps)
    s_varm_crit_pin_xip_ram();

    debug_label(stepx_nsboot_gpio_config);
    // if the BOOTSEL_FLAG_GPIO_PIN_SPECIFIED is set the usb_activity_pin_config is an explicitly chosen pin
    // (or -1 for none)
    if (!(bootselFlags & BOOTSEL_FLAG_GPIO_PIN_SPECIFIED)) {
        usb_activity_pin = (uint32_t)-1; // default to disabled
        if (s_varm_step_safe_otp_read_rbit3_guarded(OTP_DATA_BOOT_FLAGS0_ROW) &
                                    (1u << OTP_DATA_BOOT_FLAGS0_ENABLE_BOOTSEL_LED_LSB)) {
            uint32_t usbboot_cfg_word = inline_s_otp_read_ecc_guarded(OTP_DATA_BOOTSEL_LED_CFG_ROW);
            static_assert(OTP_DATA_BOOTSEL_LED_CFG_PIN_LSB == 0, "");
            static_assert(OTP_DATA_BOOTSEL_LED_CFG_PIN_MSB < 8, "");
            static_assert((OTP_DATA_BOOTSEL_LED_CFG_BITS & 0xff & ~OTP_DATA_BOOTSEL_LED_CFG_PIN_BITS) == 0, "");
            usb_activity_pin = usbboot_cfg_word;
#if 0 && GENERAL_SIZE_HACKS // currently doesn't save space
            static_assert((OTP_DATA_BOOTSEL_LED_CFG_ACTIVELOW_BITS >> 4) == BOOTSEL_FLAG_GPIO_PIN_ACTIVE_LOW, "");
            bootselFlags |= (usbboot_cfg_word >> 4) & BOOTSEL_FLAG_GPIO_PIN_ACTIVE_LOW;
#else
            if (usbboot_cfg_word & OTP_DATA_BOOTSEL_LED_CFG_ACTIVELOW_BITS) {
                bootselFlags |= BOOTSEL_FLAG_GPIO_PIN_ACTIVE_LOW;
            }
#endif
        }
    }
    // note that nsboot does not check LED related bootselFlags other
    // than BOOTSEL_FLAG_GPIO_PIN_ACTIVE_LOW; usb_activity_pin == -1 is used
    // to disable activity pin
    nsboot_config->usb_activity_pin = (int8_t)usb_activity_pin;
    nsboot_config->bootsel_flags = (uint8_t)bootselFlags;
#if USE_BOOTROM_GPIO
    if (nsboot_config->usb_activity_pin >= 0) {
        uint gpio = (uint)nsboot_config->usb_activity_pin;

        uint32_t mask = 1u << (gpio & 0x1fu);
#if ASM_SIZE_HACKS
        sio_hw_t *sio = sio_hw;
        uint tmp;
        pico_default_asm_volatile(
                "lsrs %[tmp], %[gpio], #6\n"
                "bcc 1f\n"
                "adds %[sio], #%c[_SIO_GPIO_HI_OE_SET_OFFSET] - %c[_SIO_GPIO_OE_SET_OFFSET]\n"
                "1:\n"
                "str %[mask], [%[sio], %[_SIO_GPIO_OE_SET_OFFSET]]\n"
                : [sio] "+l" (sio), [tmp] "=&l" (tmp)
                : [mask] "l" (mask), [gpio] "l" (gpio),
                  [_SIO_GPIO_OE_SET_OFFSET] "i" (SIO_GPIO_OE_SET_OFFSET),
                  [_SIO_GPIO_HI_OE_SET_OFFSET] "i" (SIO_GPIO_HI_OE_SET_OFFSET)
                : "cc"
                );
#else
        if (gpio < 32) {
            sio_hw->gpio_oe_set = mask;
        } else {
            sio_hw->gpio_hi_oe_set = mask;
        }
#endif
        // Set input enable off, output disable off
        pads_bank0_hw->io[gpio] = PADS_BANK0_GPIO0_RESET & ~(PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS);
        // Zero all fields apart from fsel; we want this IO to do what the peripheral tells it.
        // This doesn't affect e.g. pullup/pulldown, as these are in pad controls.
        uint ctrl;
#if ASM_SIZE_HACKS
        // GCC has lost its mind below - you'd think loading the value 5 would be simple
        pico_default_asm_volatile(
                "movs %0, %1"
                : "=l" (ctrl)
                : "i" (GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB)
                : "cc"
                );
#else
        ctrl = GPIO_FUNC_SIO << IO_BANK0_GPIO0_CTRL_FUNCSEL_LSB;
#endif
#if GENERAL_SIZE_HACKS
        static_assert(((IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_INVERT << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB) >> 8) ==
                              BOOTSEL_FLAG_GPIO_PIN_ACTIVE_LOW, "");
        ctrl |= (bootselFlags >> 8u) & BOOTSEL_FLAG_GPIO_PIN_ACTIVE_LOW;
#else
        if (bootselFlags & BOOTSEL_FLAG_GPIO_PIN_ACTIVE_LOW) {
            ctrl |= IO_BANK0_GPIO0_CTRL_OUTOVER_VALUE_INVERT << IO_BANK0_GPIO0_CTRL_OUTOVER_LSB;
        }
#endif
        iobank0_hw->io[gpio].ctrl = ctrl;
        // Remove pad isolation now that the correct peripheral is in control of the pad
        pads_bank0_hw->io[gpio] = PADS_BANK0_GPIO0_RESET & ~(PADS_BANK0_GPIO0_IE_BITS | PADS_BANK0_GPIO0_OD_BITS | PADS_BANK0_GPIO0_ISO_BITS);
    }
#endif

    debug_label(stepx_nsboot_mem_erase);
    // Everything which will become NS-accessible must be cleared:
    // - Main SRAM (SAU NS)
    // - XIP SRAM (SAU NS)
    // - USB RAM (IDAU Exempt, ACCESSCTRL NS)
    // USB RAM needs to be cleared anyway, since that's where .bss goes.
    inline_s_set_ram_rw_xn(mpu_on_arm);
    s_varm_step_safe_crit_mem_erase_by_words_const_size(XIP_SRAM_BASE, XIP_SRAM_END - XIP_SRAM_BASE, true);

    // Launch USB boot client either in ARM non-secure mode, or via varmulet

    printf("entering NS boot\n");
    // note this never returns (and is marked as such)
    varm_to_s_native_crit_launch_nsboot();
    __builtin_unreachable();
}

// returns BOOTROM_ERROR_NOT_FOUND if no suitable partition is found, otherwise returns
// partition info in partition_out, and a value of the partition index, or PARTITION_TABLE_NO_PARTITION_INDEX
// if the download is to unpartitioned space
int s_varm_ram_trash_get_uf2_target_partition_workarea(uint32_t family_id, resident_partition_t *partition_out, uf2_target_workarea_t *uf2_target_workarea) {
    canary_entry(S_VARM_RAM_TRASH_GET_UF2_TARGET_PARTITION_WORKAREA);
    // this will load the partition table if not already loaded (note flash should be set up before calling this),
    // and initialize the ctx used below...
    s_varm_crit_load_resident_partition_table(&uf2_target_workarea->scan_workarea, false);

    uint32_t family_index = family_id - RP2040_FAMILY_ID;
    uint32_t family_bit = 0;
    if (family_index <= FAMILY_ID_MAX - RP2040_FAMILY_ID) {
        static_assert((1u << PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILIES_LSB) == PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS, "");
        family_bit = PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_RP2040_BITS << family_index;
    }
#if MINI_PRINTF
    if (family_bit == 0) {
        printf("Family id %08x is not one of our defaults");
    } else {
        static const char *default_family_names[] = {
                "rp2040",
                "absolute",
                "data",
                "rp2350_arm_s",
                "rp2350_riscv",
                "rp2350_arm_ns",
        };
        static_assert(count_of(default_family_names) == FAMILY_ID_MAX - RP2040_FAMILY_ID + 1, "");
        printf("Family id %08x is %s\n", family_id, default_family_names[family_index]);
    }
#endif
    // This gets redundant literals, but making this pointer opaque screws up register allocation:
    resident_partition_table_t *pt = &bootram->always.partition_table;
    int rc;
    if (!pt->partition_count || family_bit == PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_ABSOLUTE_BITS) {
        *partition_out = s_varm_flashperm_get_default_partition();
        if (!(family_bit & partition_out->permissions_and_flags)) {
            if (!pt->secure_item_address) {
                printf("With no partition table, only ABSOLUTE, ARM_S, and RISCV are accepted\n");
            } else if (partition_out->permissions_and_flags & PICOBIN_PARTITION_FLAGS_ACCEPTS_DEFAULT_FAMILY_ABSOLUTE_BITS) {
                printf("With this partition table, only ABSOLUTE, ARM_S, and RISCV are accepted\n");
            } else {
                printf("With this partition table, only ARM_S, and RISCV are accepted\n");
            }
            rc = BOOTROM_ERROR_NOT_FOUND;
            goto get_uf2_target_partition_workarea_done;
        }
        rc = PARTITION_TABLE_NO_PARTITION_INDEX;
        goto get_uf2_target_partition_workarea_done;
    }
    boot_scan_context_t *ctx = &uf2_target_workarea->scan_workarea.ctx_holder.ctx;
    uf2_target_workarea->accepting_partition_mask = 0;
    // read all the family IDs
    __unused int words = s_varm_api_get_partition_table_info(uf2_target_workarea->family_id_buffer, count_of(uf2_target_workarea->family_id_buffer), PT_INFO_PARTITION_FAMILY_IDS);
    // note this can certainly fail in general, if you overwrite the flash where the PT was loaded from. in that case we just wont respect the family ids
//    bootrom_assert(NSBOOT, words >= 1);
    if (words >= 1) {
        bootrom_assert(NSBOOT, uf2_target_workarea->family_id_buffer[0] == PT_INFO_PARTITION_FAMILY_IDS);
        const uint32_t *extra_family_ids = uf2_target_workarea->family_id_buffer + 1;
        for (int pi = 0; pi < pt->partition_count; pi++) {
            if (pt->partitions[pi].permissions_and_flags & family_bit) {
                uf2_target_workarea->accepting_partition_mask |= 1u << pi;
            }
            for (uint i = 0; i < inline_s_partition_accepts_num_extra_families(&pt->partitions[pi]); i++) {
                if (family_id == *extra_family_ids++) {
                    uf2_target_workarea->accepting_partition_mask |= 1u << pi;
                }
            }
            bootrom_assert(NSBOOT, extra_family_ids <= uf2_target_workarea->family_id_buffer +
                                                       count_of(uf2_target_workarea->family_id_buffer));
        }
        bootrom_assert(NSBOOT, extra_family_ids == uf2_target_workarea->family_id_buffer + words);
    }

    printf("Find UF2 target partition for family %08x\n", family_id);
    // The following search order is used based on the family ID being downloaded, and the attributes (and accepted families of the partition)

    // 1. look for target bootable with current CPU
    // rationale: you could keep some partitions un-bootable (or switch boot flags over time)... makes most sense to apply data to bootable
    // note: find_target_partition will find a partition which is NSBOOT writable and matches the constraints
    printf("1. Looking for bootable, unowned partition for current CPU (%s)\n", ctx->boot_cpu == PICOBIN_IMAGE_TYPE_EXE_CPU_RISCV ? "RISC-V" : "ARM");
    rc = s_varm_crit_ram_trash_find_uf2_target_partition(uf2_target_workarea, /*unowned_only=*/true, /*bootable_only=*/true);
    // 2. look for target bootable (or owned by bootable) with other CPU (if allowed)
    // rational: if cpu swap is supported, then this makes things more consistent when nsboot may be using either CPU
    if (rc < 0) {
        static_assert(OTP_CRITICAL_ARM_DISABLE_BITS << PICOBIN_IMAGE_TYPE_EXE_CPU_ARM == OTP_CRITICAL_ARM_DISABLE_BITS, "");
        static_assert(OTP_CRITICAL_ARM_DISABLE_BITS << PICOBIN_IMAGE_TYPE_EXE_CPU_RISCV == OTP_CRITICAL_RISCV_DISABLE_BITS, "");
        if (!(otp_hw->critical & (OTP_CRITICAL_ARM_DISABLE_BITS << ctx->boot_cpu))) {
            printf("2. Looking for bootable, unowned partition for other CPU\n");
            ctx->boot_cpu ^= (PICOBIN_IMAGE_TYPE_EXE_CPU_ARM ^ PICOBIN_IMAGE_TYPE_EXE_CPU_RISCV);
            rc = s_varm_crit_ram_trash_find_uf2_target_partition(uf2_target_workarea, /*unowned_only=*/true, /*bootable_only=*/true);
        } else {
            printf("2. Not looking for bootable, unowned partition for other CPU as switching is disabled\n");
        }
    }
    // 3. look for unowned partitions
    // rational: top level partitions over owned partitions of non-bootable
    if (rc < 0) {
        printf("3. Looking for any unowned partition\n");
        rc = s_varm_crit_ram_trash_find_uf2_target_partition(uf2_target_workarea, /*unowned_only=*/true, /*bootable_only=*/false);
    }
    if (rc < 0) {
        printf("4. Looking for any partition\n");
        rc = s_varm_crit_ram_trash_find_uf2_target_partition(uf2_target_workarea, /*unowned_only=*/false, /*bootable_only=*/false);
    }
    if (rc >= 0) {
        bootrom_assert(NSBOOT, rc != PARTITION_TABLE_NO_PARTITION_INDEX);
        *partition_out = pt->partitions[rc];
    }
    get_uf2_target_partition_workarea_done:
    canary_exit_return(S_VARM_RAM_TRASH_GET_UF2_TARGET_PARTITION_WORKAREA, rc);
}

int s_varm_ram_trash_get_uf2_target_partition(uint32_t family_id, resident_partition_t *partition_out) {
    // we are starting a UF2 download, so happy to trash any part of RAM
    uf2_target_workarea_t *uf2_target_workarea = (uf2_target_workarea_t *)SRAM0_BASE;
    return s_varm_ram_trash_get_uf2_target_partition_workarea(family_id, partition_out, uf2_target_workarea);
}

static int s_varm_crit_ram_trash_find_uf2_target_partition(uf2_target_workarea_t *uf2_target_workarea, bool unowned_only,
                                                           bool bootable_only) {
    canary_entry(S_VARM_CRIT_RAM_TRASH_FIND_UF2_TARGET_PARTITION);
    int rc;
    boot_scan_context_t *ctx = &uf2_target_workarea->scan_workarea.ctx_holder.ctx;
    ctx->executable_image_def_only = bootable_only;
    resident_partition_table_t *pt = __get_opaque_ptr(&bootram->always.partition_table);
    for (int pi = 0; pi < pt->partition_count; pi++) {
        resident_partition_t *partition_a = pt->partitions + pi;
        if (!inline_s_is_b_partition(partition_a) && inline_s_partition_is_nsboot_writable(partition_a)) {
            printf("  considering ns-boot writable A partition %d\n", pi);
            if (uf2_target_workarea->accepting_partition_mask & (1u << pi)) {
                // we have found an A partition which is writable, and accepts the family.

                // "top_pi_a" is the partition we'll use for A/B comparisons (i.e.
                // it is the A partition's owner if there is one, otherwise the A partition itself)
                int top_pi_a = pi;
                if (bootable_only && !inline_s_partition_is_marked_bootable(partition_a, ctx->boot_cpu)) {
                    printf("  ignoring partition %d which is marked non bootable for this CPU\n");
                    continue;
                }
                if (inline_s_is_owned_partition(partition_a)) {
                    // filter on whether owned partitions are allowed
                    if (unowned_only) {
                        printf("    ignoring partition %d as it is owned\n");
                        continue;
                    }
                    int owner_pi = (int) inline_s_partition_link_value(partition_a);
                    if (owner_pi < pt->partition_count) {
                        top_pi_a = owner_pi;
                    } else {
                        printf("    (silently) ignoring invalid owner partition %d in partition %d\n", owner_pi, pi);
                    }
                }
                // we only do a check if there is a b partition
                int bpi = s_varm_api_crit_get_b_partition((uint) pi);
                if (bpi >= 0) {
                    bootrom_assert(NSBOOT, bpi < PARTITION_TABLE_MAX_PARTITIONS);
                    if (inline_s_partition_is_nsboot_writable(&pt->partitions[bpi])) {
                        bootrom_assert(NSBOOT, ctx->flash_update_boot_offset ==
                                               INVALID_FLASH_UPDATE_BOOT_OFFSET); // tbyb should be disabled in a non booting context
                        int which = s_varm_crit_ram_trash_pick_ab_image(ctx, (uint) top_pi_a);
                        // pick the partition which matches which of the A/B owner partitions are booted
                        if (top_pi_a == pi) {
                            printf("    Switching sense since we want the non-bootable partition\n");
                            which ^= 1;
                        } else {
                            printf("    A/B partition bootiness of owner %d = %s\n", top_pi_a, which ? "B" : "A");
                            if (partition_a->permissions_and_flags & PICOBIN_PARTITION_FLAGS_UF2_DOWNLOAD_AB_NON_BOOTABLE_OWNER_AFFINITY) {
                                printf("    Switching sense because partition has non-bootable owner affinity\n");
                                which ^= 1;
                            }
                        }
                        if (which) {
                            pi = bpi;
                        }
                        printf("> so selecting partition %s (%d)\n", which ? "B" : "A", pi);
                    } else {
                        printf("    ignoring partition %d as its b partition (%d) isn't nsboot writable\n", pi, bpi);
                        continue;
                    }
                } else {
                    printf("> select partition %d as it is has no B partition\n", pi);
                }
                bootrom_assert(NSBOOT, pi < pt->partition_count);
                rc = pi;
                goto find_uf2_target_partition_done;
            } else {
                printf("    ignoring partition %d as it is not a family match\n", pi);
            }
        }
    }
    rc = BOOTROM_ERROR_NOT_FOUND;
    find_uf2_target_partition_done:
    canary_exit_return(S_VARM_CRIT_RAM_TRASH_FIND_UF2_TARGET_PARTITION, rc);
}

