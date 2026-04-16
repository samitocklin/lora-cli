// radio.c — enum converters and shared LR11xx radio setup
#include <stdio.h>
#include <string.h>

#include "radio.h"
#include "board.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"

// FSK defaults
#define FSK_DEFAULT_BITRATE  50000u   // 50 kbps
#define FSK_DEFAULT_FDEV     25000u   // 25 kHz deviation

// ------------------------------------------------------------ converters ----

bool freq_is_24g(uint32_t freq_hz)
{
    return freq_hz >= 2400000000u;
}

lr11xx_radio_lora_sf_t sf_to_enum(uint8_t sf)
{
    switch (sf) {
    case 5:  return LR11XX_RADIO_LORA_SF5;
    case 6:  return LR11XX_RADIO_LORA_SF6;
    case 7:  return LR11XX_RADIO_LORA_SF7;
    case 8:  return LR11XX_RADIO_LORA_SF8;
    case 9:  return LR11XX_RADIO_LORA_SF9;
    case 10: return LR11XX_RADIO_LORA_SF10;
    case 11: return LR11XX_RADIO_LORA_SF11;
    case 12: return LR11XX_RADIO_LORA_SF12;
    default:
        fprintf(stderr, "Invalid SF %u, using SF7\n", sf);
        return LR11XX_RADIO_LORA_SF7;
    }
}

lr11xx_radio_lora_bw_t bw_to_enum(uint16_t bw_khz, bool is_24g)
{
    if (is_24g) {
        switch (bw_khz) {
        case 200:  return LR11XX_RADIO_LORA_BW_200;
        case 400:  return LR11XX_RADIO_LORA_BW_400;
        case 800:  return LR11XX_RADIO_LORA_BW_800;
        default:
            fprintf(stderr, "Invalid 2.4G BW %u kHz, using 400 kHz\n", bw_khz);
            return LR11XX_RADIO_LORA_BW_400;
        }
    } else {
        switch (bw_khz) {
        case 125: return LR11XX_RADIO_LORA_BW_125;
        case 250: return LR11XX_RADIO_LORA_BW_250;
        case 500: return LR11XX_RADIO_LORA_BW_500;
        default:
            fprintf(stderr, "Invalid sub-GHz BW %u kHz, using 125 kHz\n", bw_khz);
            return LR11XX_RADIO_LORA_BW_125;
        }
    }
}

lr11xx_radio_lora_cr_t cr_to_enum(uint8_t cr)
{
    switch (cr) {
    case 5: return LR11XX_RADIO_LORA_CR_4_5;
    case 6: return LR11XX_RADIO_LORA_CR_4_6;
    case 7: return LR11XX_RADIO_LORA_CR_4_7;
    case 8: return LR11XX_RADIO_LORA_CR_4_8;
    default:
        fprintf(stderr, "Invalid CR 4/%u, using 4/5\n", cr);
        return LR11XX_RADIO_LORA_CR_4_5;
    }
}

// cad_symb_nb is a plain uint8_t in lr11xx_radio_cad_params_t; valid values: 1,2,4,8,16
uint8_t cad_symb_to_enum(uint8_t symb)
{
    switch (symb) {
    case 1: case 2: case 4: case 8: case 16:
        return symb;
    default:
        fprintf(stderr, "Invalid CAD symbols %u, using 1\n", symb);
        return 1;
    }
}

void cad_thresholds(uint8_t sf, uint8_t *peak, uint8_t *min)
{
    // Conservative defaults — high enough to suppress noise on short CAD sweeps.
    // Lower values are more sensitive but give more false positives.
    // Use --cad-peak / --cad-min to override.
    if (sf <= 6)       { *peak = 30; *min = 14; }
    else if (sf <= 8)  { *peak = 28; *min = 12; }
    else               { *peak = 24; *min = 10; }
}

// ----------------------------------------------- RF switch configuration ---

// SKY13588 switch: DIO5=V1=RFSW0, DIO6=V2=RFSW1
// RX path: RFSW0 high, TX path: RFSW1 high
static lr11xx_status_t rf_switch_subghz(lora_ctx_t *ctx)
{
    lr11xx_system_rfswitch_cfg_t cfg = {
        .enable  = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
        .standby = 0,
        .rx      = LR11XX_SYSTEM_RFSW0_HIGH,
        .tx      = LR11XX_SYSTEM_RFSW1_HIGH,
        .tx_hp   = LR11XX_SYSTEM_RFSW1_HIGH,
        .tx_hf   = 0,
        .gnss    = 0,
        .wifi    = 0,
    };
    return lr11xx_system_set_dio_as_rf_switch(ctx, &cfg);
}

static lr11xx_status_t rf_switch_24g(lora_ctx_t *ctx)
{
    // 2.4 GHz uses internal path; switch disabled
    lr11xx_system_rfswitch_cfg_t cfg = {
        .enable  = LR11XX_SYSTEM_RFSW0_HIGH | LR11XX_SYSTEM_RFSW1_HIGH,
        .standby = 0,
        .rx = 0, .tx = 0, .tx_hp = 0, .tx_hf = 0, .gnss = 0, .wifi = 0,
    };
    return lr11xx_system_set_dio_as_rf_switch(ctx, &cfg);
}

static lr11xx_status_t enable_tcxo(lora_ctx_t *ctx)
{
    lr11xx_status_t st;
    st = lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
    if (st != LR11XX_STATUS_OK) return st;
    // 3.3 V TCXO, 10 ms startup: 10 ms / 30.52 µs ≈ 328
    st = lr11xx_system_set_tcxo_mode(ctx, LR11XX_SYSTEM_TCXO_CTRL_3_3V, 328);
    if (st != LR11XX_STATUS_OK) return st;
    sleep_ms(12);
    return LR11XX_STATUS_OK;
}

// --------------------------------------------------- shared init sequence ---

int radio_common_init(lora_ctx_t *ctx, bool is_24g)
{
    lr11xx_status_t st;

    st = lr11xx_system_reset(ctx);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "reset failed\n"); return -1; }

    st = lr11xx_system_wakeup(ctx);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "wakeup failed\n"); return -1; }

    st = lr11xx_system_clear_errors(ctx);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "clear_errors failed\n"); return -1; }

    st = enable_tcxo(ctx);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "tcxo failed\n"); return -1; }

    st = lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_XOSC);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "standby_xosc failed\n"); return -1; }

    st = lr11xx_system_clear_errors(ctx);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "clear_errors(2) failed\n"); return -1; }

    st = is_24g ? rf_switch_24g(ctx) : rf_switch_subghz(ctx);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "rf_switch failed\n"); return -1; }

    st = lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_LORA);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_pkt_type failed\n"); return -1; }

    return 0;
}

// ----------------------------------------------- PA / TX power config ------

int radio_configure_pa(lora_ctx_t *ctx, bool is_24g, bool hp_pa, int8_t power_dbm)
{
    lr11xx_radio_pa_cfg_t pa_cfg;
    memset(&pa_cfg, 0, sizeof(pa_cfg));

    if (is_24g) {
        pa_cfg.pa_sel        = LR11XX_RADIO_PA_SEL_HF;
        pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        pa_cfg.pa_duty_cycle = 0;
        pa_cfg.pa_hp_sel     = 0;
    } else if (hp_pa || power_dbm > 15) {
        pa_cfg.pa_sel        = LR11XX_RADIO_PA_SEL_HP;
        pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VBAT;
        pa_cfg.pa_duty_cycle = 7;
        pa_cfg.pa_hp_sel     = 7;
    } else {
        pa_cfg.pa_sel        = LR11XX_RADIO_PA_SEL_LP;
        pa_cfg.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        pa_cfg.pa_duty_cycle = 4;
        pa_cfg.pa_hp_sel     = 0;
    }

    lr11xx_status_t st = lr11xx_radio_set_pa_cfg(ctx, &pa_cfg);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_pa_cfg failed\n"); return -1; }
    st = lr11xx_radio_set_tx_params(ctx, power_dbm, LR11XX_RADIO_RAMP_48_US);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_tx_params failed\n"); return -1; }
    return 0;
}

// ---------------------------------------------- modem parameter program ----

// Image calibration for sub-GHz — extracted so both LoRa and FSK can share it.
static int image_cal(lora_ctx_t *ctx, uint32_t freq_hz)
{
    uint8_t f1, f2;
    if      (freq_hz < 446000000)  { f1 = 0x6B; f2 = 0x6F; }
    else if (freq_hz < 734000000)  { f1 = 0xD7; f2 = 0xDB; }
    else if (freq_hz < 828000000)  { f1 = 0xD7; f2 = 0xDB; }
    else if (freq_hz < 877000000)  { f1 = 0xD7; f2 = 0xDB; }
    else if (freq_hz < 930000000)  { f1 = 0xE1; f2 = 0xE9; }
    else                           { f1 = 0xE1; f2 = 0xE9; }
    lr11xx_status_t st = lr11xx_system_calibrate_image_in_mhz(ctx, f1, f2);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "image_cal failed\n"); return -1; }
    return 0;
}

int radio_set_lora_modem(lora_ctx_t *ctx,
                         uint32_t freq_hz,
                         lr11xx_radio_lora_sf_t sf,
                         lr11xx_radio_lora_bw_t bw,
                         lr11xx_radio_lora_cr_t cr,
                         uint16_t preamble,
                         bool crc, bool implicit, uint8_t payload_len,
                         bool ldro, bool invert_iq,
                         uint8_t sync_word,
                         bool is_24g)
{
    lr11xx_status_t st;

    const lr11xx_radio_mod_params_lora_t mod = {
        .sf   = sf,
        .bw   = bw,
        .cr   = cr,
        .ldro = ldro ? 1 : 0,
    };

    const lr11xx_radio_pkt_params_lora_t pkt = {
        .preamble_len_in_symb = preamble,
        .header_type          = implicit ? LR11XX_RADIO_LORA_PKT_IMPLICIT
                                         : LR11XX_RADIO_LORA_PKT_EXPLICIT,
        .pld_len_in_bytes     = payload_len ? payload_len : 255,
        .crc                  = crc ? LR11XX_RADIO_LORA_CRC_ON : LR11XX_RADIO_LORA_CRC_OFF,
        .iq                   = invert_iq ? LR11XX_RADIO_LORA_IQ_INVERTED
                                          : LR11XX_RADIO_LORA_IQ_STANDARD,
    };

    st = lr11xx_radio_set_rf_freq(ctx, freq_hz);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_rf_freq failed\n"); return -1; }

    if (!is_24g && image_cal(ctx, freq_hz) < 0) return -1;

    st = lr11xx_radio_set_lora_mod_params(ctx, &mod);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_mod_params failed\n"); return -1; }

    st = lr11xx_radio_set_lora_pkt_params(ctx, &pkt);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_pkt_params failed\n"); return -1; }

    // set_lora_public_network is deprecated; use set_lora_sync_word directly.
    // 0x12 = private LoRa, 0x34 = LoRaWAN public, 0x2B = Meshtastic, etc.
    st = lr11xx_radio_set_lora_sync_word(ctx, sync_word ? sync_word : 0x12);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_lora_sync_word failed\n"); return -1; }

    return 0;
}

// ------------------------------------------------ GFSK modem setup ----------

int radio_set_fsk_modem(lora_ctx_t *ctx,
                        uint32_t freq_hz,
                        uint32_t bitrate_bps,
                        uint32_t fdev_hz,
                        uint8_t  payload_len,
                        bool     is_24g)
{
    if (!bitrate_bps) bitrate_bps = FSK_DEFAULT_BITRATE;
    if (!fdev_hz)     fdev_hz     = FSK_DEFAULT_FDEV;

    lr11xx_status_t st;

    // radio_common_init sets LORA; override to GFSK
    st = lr11xx_radio_set_pkt_type(ctx, LR11XX_RADIO_PKT_TYPE_GFSK);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_pkt_type(GFSK) failed\n"); return -1; }

    st = lr11xx_radio_set_rf_freq(ctx, freq_hz);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_rf_freq failed\n"); return -1; }

    if (!is_24g && image_cal(ctx, freq_hz) < 0) return -1;

    // RX bandwidth: Carson's rule BW = 2*(fdev + bitrate/2), then 50% margin
    uint32_t carson_bw = 2 * (fdev_hz + bitrate_bps / 2);
    lr11xx_radio_gfsk_bw_t bw_enum;
    lr11xx_radio_get_gfsk_rx_bandwidth(carson_bw + carson_bw / 2, &bw_enum);

    const lr11xx_radio_mod_params_gfsk_t mod = {
        .br_in_bps    = bitrate_bps,
        .pulse_shape  = LR11XX_RADIO_GFSK_PULSE_SHAPE_BT_05,  // standard GFSK
        .bw_dsb_param = bw_enum,
        .fdev_in_hz   = fdev_hz,
    };

    const lr11xx_radio_pkt_params_gfsk_t pkt = {
        .preamble_len_in_bits  = 40,  // 5 bytes of 0xAA
        .preamble_detector     = LR11XX_RADIO_GFSK_PREAMBLE_DETECTOR_MIN_16BITS,
        .sync_word_len_in_bits = 32,  // 4 bytes
        .address_filtering     = LR11XX_RADIO_GFSK_ADDRESS_FILTERING_DISABLE,
        .header_type           = LR11XX_RADIO_GFSK_PKT_VAR_LEN,
        .pld_len_in_bytes      = payload_len ? payload_len : 255,
        .crc_type              = LR11XX_RADIO_GFSK_CRC_2_BYTES,
        .dc_free               = LR11XX_RADIO_GFSK_DC_FREE_WHITENING,
    };

    // Standard 4-byte sync word (padded to 8 bytes as required by driver)
    // 0x2DD4 is the classic GFSK sync used by SX127x/SX126x devices
    const uint8_t sync[LR11XX_RADIO_GFSK_SYNC_WORD_LENGTH] = {
        0x2D, 0xD4, 0x2D, 0xD4, 0x00, 0x00, 0x00, 0x00
    };

    st = lr11xx_radio_set_gfsk_mod_params(ctx, &mod);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_gfsk_mod_params failed\n"); return -1; }

    st = lr11xx_radio_set_gfsk_pkt_params(ctx, &pkt);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_gfsk_pkt_params failed\n"); return -1; }

    st = lr11xx_radio_set_gfsk_sync_word(ctx, sync);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_gfsk_sync_word failed\n"); return -1; }

    return 0;
}
