// mode_txtest.c — RF TX test modes for spectrum analyser / PA validation
//
// Two sub-modes:
//
//   CW  (--cw)   : unmodulated carrier at the exact configured frequency.
//                  Best for measuring output power, harmonic content, and
//                  frequency accuracy with a spectrum analyser.
//
//   Preamble     : continuous modulated preamble (LoRa chirp or FSK 0xAA...).
//   (default)      Saturates the transmitter at full duty cycle.
//                  Best for measuring occupied bandwidth and spectral mask.
//
// In both cases the radio transmits continuously until Ctrl+C.
//
// Examples:
//   Sub-GHz CW, 14 dBm LP PA:
//     sudo lora-cli --spi spi0 --txtest --cw --freq 868000000 --power 14
//
//   Sub-GHz CW, 22 dBm HP PA:
//     sudo lora-cli --spi spi0 --txtest --cw --freq 868000000 --power 22 --hp-pa
//
//   2.4 GHz CW, 13 dBm HF PA:
//     sudo lora-cli --spi spi0 --txtest --cw --freq 2440000000 --power 13
//
//   Sub-GHz LoRa preamble, SF7 BW125:
//     sudo lora-cli --spi spi0 --txtest --freq 868000000 --sf 7 --bw 125
//
//   Sub-GHz FSK preamble, 50 kbps:
//     sudo lora-cli --spi spi0 --txtest --fsk --freq 868000000 --bitrate 50000
//
//   2.4 GHz FSK preamble, 100 kbps:
//     sudo lora-cli --spi spi0 --txtest --fsk --freq 2440000000 --bitrate 100000

#include <stdio.h>
#include <string.h>

#include "board.h"
#include "radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"

int mode_txtest(lora_ctx_t *ctx, const lora_txtest_params_t *p)
{
    bool is_24g = freq_is_24g(p->freq_hz);
    const char *pa_str = is_24g ? "HF" : (p->hp_pa || p->power_dbm > 15) ? "HP" : "LP";

    printf("=== TX test mode ===\n");
    if (p->cw) {
        printf("type  : CW (unmodulated carrier)\n");
    } else if (p->fsk) {
        uint32_t br = p->fsk_bitrate ? p->fsk_bitrate : 50000;
        uint32_t fd = p->fsk_fdev_hz ? p->fsk_fdev_hz : 25000;
        printf("type  : FSK infinite preamble  %u bps  fdev %u Hz\n", br, fd);
    } else {
        printf("type  : LoRa infinite preamble  SF%u  BW%u kHz\n", p->sf, p->bw_khz);
    }
    printf("freq  : %.6f MHz (%s)\n", p->freq_hz / 1e6, is_24g ? "2.4G" : "sub-GHz");
    printf("power : %d dBm  PA=%s\n", p->power_dbm, pa_str);
    printf("\n");
    printf("WARNING: continuous RF transmission — confirm you are allowed on this frequency.\n");
    printf("Press Ctrl+C to stop.\n\n");
    fflush(stdout);

    if (radio_common_init(ctx, is_24g) < 0) return -1;

    if (p->cw) {
        // CW only needs frequency and PA — no modem params required
        lr11xx_status_t st = lr11xx_radio_set_rf_freq(ctx, p->freq_hz);
        if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_rf_freq failed\n"); return -1; }
        // No image calibration needed for CW (no demodulation path)
    } else if (p->fsk) {
        if (radio_set_fsk_modem(ctx, p->freq_hz,
                                p->fsk_bitrate, p->fsk_fdev_hz,
                                0, is_24g) < 0)
            return -1;
    } else {
        // LoRa preamble: set modem params (only mod params needed — no pkt params for preamble)
        lr11xx_status_t st = lr11xx_radio_set_rf_freq(ctx, p->freq_hz);
        if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_rf_freq failed\n"); return -1; }

        // Image cal so the TX path is properly calibrated
        if (!is_24g) {
            const lr11xx_radio_mod_params_lora_t mod = {
                .sf   = sf_to_enum(p->sf),
                .bw   = bw_to_enum(p->bw_khz, is_24g),
                .cr   = LR11XX_RADIO_LORA_CR_4_5,
                .ldro = 0,
            };
            // radio_common_init already set PKT_TYPE_LORA
            st = lr11xx_radio_set_lora_mod_params(ctx, &mod);
            if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_mod_params failed\n"); return -1; }
        } else {
            const lr11xx_radio_mod_params_lora_t mod = {
                .sf   = sf_to_enum(p->sf),
                .bw   = bw_to_enum(p->bw_khz, is_24g),
                .cr   = LR11XX_RADIO_LORA_CR_4_5,
                .ldro = 0,
            };
            st = lr11xx_radio_set_lora_mod_params(ctx, &mod);
            if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_mod_params failed\n"); return -1; }
        }
    }

    if (radio_configure_pa(ctx, is_24g, p->hp_pa, p->power_dbm) < 0) return -1;

    // Arm transmitter
    gpio_set(&ctx->led_tx, 1);
    lr11xx_status_t st;
    if (p->cw)
        st = lr11xx_radio_set_tx_cw(ctx);
    else
        st = lr11xx_radio_set_tx_infinite_preamble(ctx);

    if (st != LR11XX_STATUS_OK) {
        fprintf(stderr, "TX test start failed\n");
        gpio_set(&ctx->led_tx, 0);
        return -1;
    }

    printf("Transmitting %.6f MHz  %d dBm ... ", p->freq_hz / 1e6, p->power_dbm);
    fflush(stdout);

    // Heartbeat blink while transmitting
    while (!g_stop) {
        sleep_ms(400);
        gpio_set(&ctx->led_hb, 1);
        sleep_ms(100);
        gpio_set(&ctx->led_hb, 0);
    }

    // Stop: standby puts the PA off immediately
    (void)lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
    gpio_set(&ctx->led_tx, 0);
    gpio_set(&ctx->led_hb, 0);
    printf("\nTX test stopped.\n");
    return 0;
}
