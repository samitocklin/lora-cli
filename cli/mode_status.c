// mode_status.c — dump chip identity, status registers and calibration state
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"
#include "lr11xx_system_types.h"

static const char *cmd_status_str(lr11xx_system_command_status_t s)
{
    switch (s) {
    case LR11XX_SYSTEM_CMD_STATUS_FAIL: return "FAIL";
    case LR11XX_SYSTEM_CMD_STATUS_PERR: return "PERR";
    case LR11XX_SYSTEM_CMD_STATUS_OK:   return "OK";
    case LR11XX_SYSTEM_CMD_STATUS_DATA: return "DATA";
    default: return "UNKNOWN";
    }
}

static const char *reset_status_str(lr11xx_system_reset_status_t s)
{
    switch (s) {
    case LR11XX_SYSTEM_RESET_STATUS_CLEARED:      return "CLEARED";
    case LR11XX_SYSTEM_RESET_STATUS_ANALOG:       return "ANALOG";
    case LR11XX_SYSTEM_RESET_STATUS_EXTERNAL:     return "EXTERNAL";
    case LR11XX_SYSTEM_RESET_STATUS_SYSTEM:       return "SYSTEM";
    case LR11XX_SYSTEM_RESET_STATUS_WATCHDOG:     return "WATCHDOG";
    case LR11XX_SYSTEM_RESET_STATUS_IOCD_RESTART: return "IOCD_RESTART";
    case LR11XX_SYSTEM_RESET_STATUS_RTC_RESTART:  return "RTC_RESTART";
    default: return "UNKNOWN";
    }
}

static const char *chip_mode_str(lr11xx_system_chip_modes_t m)
{
    switch (m) {
    case LR11XX_SYSTEM_CHIP_MODE_SLEEP:     return "SLEEP";
    case LR11XX_SYSTEM_CHIP_MODE_STBY_RC:   return "STBY_RC";
    case LR11XX_SYSTEM_CHIP_MODE_STBY_XOSC: return "STBY_XOSC";
    case LR11XX_SYSTEM_CHIP_MODE_FS:        return "FS";
    case LR11XX_SYSTEM_CHIP_MODE_RX:        return "RX";
    case LR11XX_SYSTEM_CHIP_MODE_TX:        return "TX";
    case LR11XX_SYSTEM_CHIP_MODE_LOC:       return "LOC";
    default: return "UNKNOWN";
    }
}

static const char *chip_type_str(lr11xx_system_version_type_t t)
{
    switch (t) {
    case LR11XX_SYSTEM_VERSION_TYPE_LR1110: return "LR1110";
    case LR11XX_SYSTEM_VERSION_TYPE_LR1120: return "LR1120";
    case LR11XX_SYSTEM_VERSION_TYPE_LR1121: return "LR1121";
    default: return "UNKNOWN";
    }
}

static void print_irqs(lr11xx_system_irq_mask_t irq)
{
    printf("  irq_mask               : 0x%08lX", (unsigned long)irq);
    if (irq == 0) { printf(" (none)\n"); return; }
    printf("\n");
    if (irq & LR11XX_SYSTEM_IRQ_TX_DONE)                printf("    TX_DONE\n");
    if (irq & LR11XX_SYSTEM_IRQ_RX_DONE)                printf("    RX_DONE\n");
    if (irq & LR11XX_SYSTEM_IRQ_PREAMBLE_DETECTED)      printf("    PREAMBLE_DETECTED\n");
    if (irq & LR11XX_SYSTEM_IRQ_SYNC_WORD_HEADER_VALID) printf("    SYNC_WORD_HEADER_VALID\n");
    if (irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR)           printf("    HEADER_ERROR\n");
    if (irq & LR11XX_SYSTEM_IRQ_CRC_ERROR)              printf("    CRC_ERROR\n");
    if (irq & LR11XX_SYSTEM_IRQ_CAD_DONE)               printf("    CAD_DONE\n");
    if (irq & LR11XX_SYSTEM_IRQ_CAD_DETECTED)           printf("    CAD_DETECTED\n");
    if (irq & LR11XX_SYSTEM_IRQ_TIMEOUT)                printf("    TIMEOUT\n");
    if (irq & LR11XX_SYSTEM_IRQ_CMD_ERROR)              printf("    CMD_ERROR\n");
    if (irq & LR11XX_SYSTEM_IRQ_ERROR)                  printf("    ERROR\n");
}

static void print_errors(lr11xx_system_errors_t e)
{
    printf("  error_flags            : 0x%04X", e);
    if (e == 0) { printf(" (none)\n"); return; }
    printf("\n");
    if (e & LR11XX_SYSTEM_ERRORS_LF_RC_CALIB_MASK)    printf("    LF_RC_CALIB_FAIL\n");
    if (e & LR11XX_SYSTEM_ERRORS_HF_RC_CALIB_MASK)    printf("    HF_RC_CALIB_FAIL\n");
    if (e & LR11XX_SYSTEM_ERRORS_ADC_CALIB_MASK)      printf("    ADC_CALIB_FAIL\n");
    if (e & LR11XX_SYSTEM_ERRORS_PLL_CALIB_MASK)      printf("    PLL_CALIB_FAIL\n");
    if (e & LR11XX_SYSTEM_ERRORS_IMG_CALIB_MASK)      printf("    IMG_CALIB_FAIL\n");
    if (e & LR11XX_SYSTEM_ERRORS_HF_XOSC_START_MASK)  printf("    HF_XOSC_START_FAIL\n");
    if (e & LR11XX_SYSTEM_ERRORS_LF_XOSC_START_MASK)  printf("    LF_XOSC_START_FAIL\n");
    if (e & LR11XX_SYSTEM_ERRORS_PLL_LOCK_MASK)       printf("    PLL_LOCK_FAIL\n");
}

int mode_status(lora_ctx_t *ctx)
{
    lr11xx_status_t st;

    printf("=== LR1121 Status ===\n");
    printf("spi                    : %s\n", ctx->pins.spidev_path);
    printf("gpio  reset=%-2u busy=%-2u dio8=%-2u irq=%-2u\n",
           ctx->pins.gpio_reset, ctx->pins.gpio_busy,
           ctx->pins.gpio_dio8,  ctx->pins.gpio_irq);
    printf("gpio  led_tx=%-2u led_rx=%-2u led_hb=%-2u\n\n",
           ctx->pins.gpio_led_tx, ctx->pins.gpio_led_rx, ctx->pins.gpio_led_hb);

    st = lr11xx_system_reset(ctx);
    printf("reset                  : %s\n", st == LR11XX_STATUS_OK ? "OK" : "ERROR");
    if (st != LR11XX_STATUS_OK) return -1;

    st = lr11xx_system_wakeup(ctx);
    printf("wakeup                 : %s\n", st == LR11XX_STATUS_OK ? "OK" : "ERROR");
    if (st != LR11XX_STATUS_OK) return -1;

    // -- system status
    {
        lr11xx_system_stat1_t    stat1;
        lr11xx_system_stat2_t    stat2;
        lr11xx_system_irq_mask_t irq = 0;
        memset(&stat1, 0, sizeof(stat1));
        memset(&stat2, 0, sizeof(stat2));

        st = lr11xx_system_get_status(ctx, &stat1, &stat2, &irq);
        if (st == LR11XX_STATUS_OK) {
            printf("\n--- system status ---\n");
            printf("  cmd_status             : %s\n", cmd_status_str(stat1.command_status));
            printf("  irq_active             : %s\n", stat1.is_interrupt_active ? "YES" : "NO");
            printf("  reset_status           : %s\n", reset_status_str(stat2.reset_status));
            printf("  chip_mode              : %s\n", chip_mode_str(stat2.chip_mode));
            printf("  running_from_flash     : %s\n", stat2.is_running_from_flash ? "YES" : "NO");
            print_irqs(irq);
        }
    }

    // -- version
    {
        lr11xx_system_version_t ver;
        memset(&ver, 0, sizeof(ver));
        st = lr11xx_system_get_version(ctx, &ver);
        if (st == LR11XX_STATUS_OK) {
            printf("\n--- chip version ---\n");
            printf("  hw                     : 0x%02X\n", ver.hw);
            printf("  type                   : %s\n", chip_type_str(ver.type));
            printf("  fw                     : 0x%04X\n", ver.fw);
        }
    }

    // -- errors
    {
        lr11xx_system_errors_t errors = 0;
        st = lr11xx_system_get_errors(ctx, &errors);
        if (st == LR11XX_STATUS_OK) {
            printf("\n--- error flags ---\n");
            print_errors(errors);
        }
    }

    // -- vbat / temp
    {
        uint8_t vbat = 0;
        st = lr11xx_system_get_vbat(ctx, &vbat);
        if (st == LR11XX_STATUS_OK) {
            printf("\n--- power / temp ---\n");
            printf("  vbat_raw               : %u (0x%02X)\n", vbat, vbat);
        }
        uint16_t temp = 0;
        st = lr11xx_system_get_temp(ctx, &temp);
        if (st == LR11XX_STATUS_OK)
            printf("  temp_raw               : %u (0x%04X)\n", temp, temp);
    }

    pulse_led(&ctx->led_tx, 120);
    return 0;
}
