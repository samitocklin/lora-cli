// mode_tx.c — LoRa TX with configurable modulation, PA, and payload
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "board.h"
#include "radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"

int mode_tx(lora_ctx_t *ctx, const lora_tx_params_t *p,
            const uint8_t *payload, uint16_t payload_len)
{
    bool is_24g = freq_is_24g(p->freq_hz);

    const char *modem_str = p->fsk ? "FSK" : "LoRa";
    printf("=== TX mode (%s) ===\n", modem_str);
    printf("freq    : %.6f MHz (%s)\n", p->freq_hz / 1e6, is_24g ? "2.4G" : "sub-GHz");
    if (p->fsk) {
        uint32_t br = p->fsk_bitrate ? p->fsk_bitrate : 50000;
        uint32_t fd = p->fsk_fdev_hz ? p->fsk_fdev_hz : 25000;
        printf("modem   : FSK  %u bps  fdev %u Hz\n", br, fd);
    } else {
        printf("modem   : SF%u  BW%u kHz  CR 4/%u  preamble %u\n",
               p->sf, p->bw_khz, p->cr, p->preamble);
        printf("packet  : CRC=%s  header=%s  LDRO=%s\n",
               p->crc ? "on" : "off",
               p->implicit ? "implicit" : "explicit",
               p->ldro ? "on" : "off");
    }
    printf("power   : %d dBm  PA=%s\n", p->power_dbm,
           is_24g ? "HF" : (p->hp_pa || p->power_dbm > 15) ? "HP" : "LP");
    printf("payload : %u bytes\n", payload_len);
    printf("count   : %s\n", p->count == 0 ? "infinite" : "");
    if (p->count) printf("          %u packet(s), %u ms interval\n", p->count, p->interval_ms);
    printf("\n");

    if (radio_common_init(ctx, is_24g) < 0) return -1;

    if (p->fsk) {
        if (radio_set_fsk_modem(ctx, p->freq_hz,
                                p->fsk_bitrate, p->fsk_fdev_hz,
                                (uint8_t)payload_len, is_24g) < 0)
            return -1;
    } else {
        if (radio_set_lora_modem(ctx, p->freq_hz,
                                 sf_to_enum(p->sf),
                                 bw_to_enum(p->bw_khz, is_24g),
                                 cr_to_enum(p->cr),
                                 p->preamble,
                                 p->crc, p->implicit, payload_len,
                                 p->ldro, p->invert_iq,
                                 p->sync_word, is_24g) < 0)
            return -1;
    }

    if (radio_configure_pa(ctx, is_24g, p->hp_pa, p->power_dbm) < 0) return -1;

    lr11xx_status_t st = lr11xx_system_set_dio_irq_params(
        ctx,
        LR11XX_SYSTEM_IRQ_TX_DONE | LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_ERROR,
        0);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_dio_irq failed\n"); return -1; }

    uint32_t sent = 0;

    while (!g_stop && (p->count == 0 || sent < p->count)) {
        st = lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
        if (st != LR11XX_STATUS_OK) { fprintf(stderr, "clear_irq failed\n"); return -1; }

        st = lr11xx_regmem_write_buffer8(ctx, payload, payload_len);
        if (st != LR11XX_STATUS_OK) { fprintf(stderr, "write_buffer failed\n"); return -1; }

        // Start TX; 0 = no timeout (wait for TX_DONE IRQ)
        st = lr11xx_radio_set_tx(ctx, 0);
        if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_tx failed\n"); return -1; }

        // Wait for TX_DONE (up to 10 s)
        gpio_set(&ctx->led_tx, 1);
        int rc = wait_irq_rising(ctx, 10000);
        gpio_set(&ctx->led_tx, 0);

        if (rc <= 0) {
            fprintf(stderr, "TX IRQ timeout\n");
            break;
        }

        lr11xx_system_irq_mask_t irq = 0;
        (void)lr11xx_system_get_and_clear_irq_status(ctx, &irq);

        if (irq & LR11XX_SYSTEM_IRQ_TX_DONE) {
            sent++;
            printf("[%4u] TX_DONE  %.6f MHz  %u bytes\n", sent, p->freq_hz / 1e6, payload_len);
            fflush(stdout);
        } else if (irq & LR11XX_SYSTEM_IRQ_ERROR) {
            lr11xx_system_errors_t errs = 0;
            (void)lr11xx_system_get_errors(ctx, &errs);
            fprintf(stderr, "TX ERROR (flags=0x%04X)\n", errs);
            break;
        } else if (irq & LR11XX_SYSTEM_IRQ_TIMEOUT) {
            fprintf(stderr, "TX TIMEOUT\n");
            break;
        }

        if (p->interval_ms > 0 && (p->count == 0 || sent < p->count))
            sleep_ms(p->interval_ms);
    }

    (void)lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
    printf("\nTX done. Sent %u packet(s).\n", sent);
    return 0;
}
