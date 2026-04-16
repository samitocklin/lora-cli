// mode_rx.c — continuous LoRa RX with configurable modulation parameters
#include <stdio.h>
#include <string.h>

#include "board.h"
#include "radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"

static void print_hex_ascii(const uint8_t *buf, size_t len)
{
    for (size_t i = 0; i < len; i += 16) {
        printf("  %04zx  ", i);
        for (size_t j = 0; j < 16; j++) {
            if (i + j < len) printf("%02X ", buf[i + j]);
            else              printf("   ");
        }
        printf(" ");
        for (size_t j = 0; j < 16 && (i + j) < len; j++) {
            uint8_t c = buf[i + j];
            printf("%c", (c >= 32 && c <= 126) ? c : '.');
        }
        printf("\n");
    }
}

static int start_rx_continuous(lora_ctx_t *ctx, bool is_24g)
{
    lr11xx_status_t st;
    if (is_24g) {
        st = lr11xx_radio_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);
    } else {
        st = lr11xx_radio_set_rx_with_timeout_in_rtc_step_and_lna_mode(
            ctx, 0xFFFFFF, LR11XX_RADIO_LNA_MODE_DIFFERENTIAL_LF0);
    }
    return (st == LR11XX_STATUS_OK) ? 0 : -1;
}

int mode_rx(lora_ctx_t *ctx, const lora_rx_params_t *p)
{
    bool is_24g = freq_is_24g(p->freq_hz);

    printf("=== RX mode (%s) ===\n", p->fsk ? "FSK" : "LoRa");
    printf("freq  : %.6f MHz (%s)\n", p->freq_hz / 1e6, is_24g ? "2.4G" : "sub-GHz");
    if (p->fsk) {
        uint32_t br = p->fsk_bitrate ? p->fsk_bitrate : 50000;
        uint32_t fd = p->fsk_fdev_hz ? p->fsk_fdev_hz : 25000;
        printf("modem : FSK  %u bps  fdev %u Hz\n", br, fd);
    } else {
        printf("modem : SF%u  BW%u kHz  CR 4/%u  preamble %u\n",
               p->sf, p->bw_khz, p->cr, p->preamble);
        printf("sync  : 0x%02X%s\n", p->sync_word ? p->sync_word : 0x12,
               p->sync_word == 0x2B ? " (Meshtastic)" :
               p->sync_word == 0x34 ? " (LoRaWAN public)" : " (private)");
        printf("packet: CRC=%s  header=%s  LDRO=%s  invertIQ=%s\n",
               p->crc ? "on" : "off",
               p->implicit ? "implicit" : "explicit",
               p->ldro ? "on" : "off",
               p->invert_iq ? "on" : "off");
    }
    if (p->boosted_rx) printf("        boosted LNA enabled\n");
    printf("\n");

    if (radio_common_init(ctx, is_24g) < 0) return -1;

    if (p->fsk) {
        if (radio_set_fsk_modem(ctx, p->freq_hz,
                                p->fsk_bitrate, p->fsk_fdev_hz,
                                p->payload_len, is_24g) < 0)
            return -1;
    } else {
        if (radio_set_lora_modem(ctx, p->freq_hz,
                                 sf_to_enum(p->sf),
                                 bw_to_enum(p->bw_khz, is_24g),
                                 cr_to_enum(p->cr),
                                 p->preamble,
                                 p->crc, p->implicit, p->payload_len,
                                 p->ldro, p->invert_iq,
                                 p->sync_word, is_24g) < 0)
            return -1;
    }

    if (p->boosted_rx) {
        lr11xx_status_t st = lr11xx_radio_cfg_rx_boosted(ctx, true);
        if (st != LR11XX_STATUS_OK) {
            fprintf(stderr, "boosted_rx failed\n"); return -1;
        }
    }

    lr11xx_status_t st = lr11xx_system_set_dio_irq_params(
        ctx,
        LR11XX_SYSTEM_IRQ_RX_DONE   |
        LR11XX_SYSTEM_IRQ_TIMEOUT   |
        LR11XX_SYSTEM_IRQ_CRC_ERROR |
        LR11XX_SYSTEM_IRQ_HEADER_ERROR |
        LR11XX_SYSTEM_IRQ_ERROR,
        0);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_dio_irq failed\n"); return -1; }

    st = lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "clear_irq failed\n"); return -1; }

    if (start_rx_continuous(ctx, is_24g) < 0) {
        fprintf(stderr, "start RX failed\n"); return -1;
    }

    printf("Listening on %.6f MHz  (Ctrl+C to stop)\n\n", p->freq_hz / 1e6);

    while (!g_stop) {
        lr11xx_system_irq_mask_t irq = 0;

        gpio_set(&ctx->led_hb, 1);
        int rc = wait_irq_rising(ctx, 500);
        gpio_set(&ctx->led_hb, 0);

        if (rc < 0) { sleep_ms(50); continue; }
        if (rc == 0) continue; // timeout, keep waiting

        st = lr11xx_system_get_and_clear_irq_status(ctx, &irq);
        if (st != LR11XX_STATUS_OK || irq == 0) { sleep_ms(50); continue; }

        if (irq & LR11XX_SYSTEM_IRQ_RX_DONE) {
            uint8_t                         payload[256];
            lr11xx_radio_rx_buffer_status_t rxbuf;
            memset(&rxbuf, 0, sizeof(rxbuf));

            st = lr11xx_radio_get_rx_buffer_status(ctx, &rxbuf);
            if (st == LR11XX_STATUS_OK &&
                rxbuf.pld_len_in_bytes > 0 &&
                rxbuf.pld_len_in_bytes <= sizeof(payload))
            {
                st = lr11xx_regmem_read_buffer8(ctx, payload,
                                                rxbuf.buffer_start_pointer,
                                                rxbuf.pld_len_in_bytes);
            }

            if (st == LR11XX_STATUS_OK) {
                printf("=== RX packet ===\n");
                printf("  len    : %u bytes\n", rxbuf.pld_len_in_bytes);
                if (p->fsk) {
                    lr11xx_radio_pkt_status_gfsk_t gpkt;
                    memset(&gpkt, 0, sizeof(gpkt));
                    (void)lr11xx_radio_get_gfsk_pkt_status(ctx, &gpkt);
                    printf("  RSSI   : %d dBm (sync)  %d dBm (avg)\n",
                           gpkt.rssi_sync_in_dbm, gpkt.rssi_avg_in_dbm);
                    if (gpkt.is_crc_err) printf("  [CRC error]\n");
                } else {
                    lr11xx_radio_pkt_status_lora_t pkt;
                    memset(&pkt, 0, sizeof(pkt));
                    (void)lr11xx_radio_get_lora_pkt_status(ctx, &pkt);
                    printf("  RSSI   : %d dBm\n",   pkt.rssi_pkt_in_dbm);
                    printf("  SNR    : %d dB\n",    pkt.snr_pkt_in_db);
                    printf("  sigRSSI: %d dBm\n",   pkt.signal_rssi_pkt_in_dbm);
                }
                print_hex_ascii(payload, rxbuf.pld_len_in_bytes);
                printf("\n");
                pulse_led(&ctx->led_rx, 100);
            } else {
                fprintf(stderr, "  [error reading payload]\n");
            }
        }

        if (irq & LR11XX_SYSTEM_IRQ_CRC_ERROR)    printf("[irq] CRC_ERROR\n");
        if (irq & LR11XX_SYSTEM_IRQ_HEADER_ERROR)  printf("[irq] HEADER_ERROR\n");
        if (irq & LR11XX_SYSTEM_IRQ_TIMEOUT)       printf("[irq] TIMEOUT\n");
        if (irq & LR11XX_SYSTEM_IRQ_ERROR) {
            lr11xx_system_errors_t errs = 0;
            (void)lr11xx_system_get_errors(ctx, &errs);
            printf("[irq] ERROR (flags=0x%04X)\n", errs);
        }
    }

    (void)lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
    gpio_set(&ctx->led_hb, 0);
    printf("\nRX stopped.\n");
    return 0;
}
