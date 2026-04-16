// radio.h — LoRa parameter structs and shared radio setup API
#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "board.h"
#include "lr11xx_radio_types.h"

// ---------------------------------------------------------- param structs ---

typedef struct {
    uint32_t freq_hz;
    uint8_t  sf;          // spreading factor: 5-12
    uint16_t bw_khz;      // bandwidth kHz: 125/250/500 (sub-GHz), 200/400/800 (2.4G)
    uint8_t  cr;          // coding rate denominator: 5=4/5, 6=4/6, 7=4/7, 8=4/8
    uint16_t preamble;    // preamble length in symbols
    bool     crc;         // CRC on/off
    bool     implicit;    // true = implicit header
    uint8_t  payload_len; // expected payload length (implicit header only)
    bool     ldro;        // low data rate optimisation
    bool     invert_iq;   // invert IQ
    bool     boosted_rx;  // boosted LNA
    // Sync word — filters packets at hardware level before the CPU sees them.
    // 0x12 = private LoRa (satellite, default)
    // 0x34 = LoRaWAN public
    // 0x2B = Meshtastic
    // 0x00 here means "use default (0x12)"
    uint8_t  sync_word;
    // GFSK modem (--fsk flag)
    bool     fsk;           // use GFSK instead of LoRa
    uint32_t fsk_bitrate;   // bps  (0 = default 50000)
    uint32_t fsk_fdev_hz;   // Hz   (0 = default 25000)
} lora_rx_params_t;

typedef struct {
    uint32_t freq_hz;
    uint8_t  sf;
    uint16_t bw_khz;
    uint8_t  cr;
    uint16_t preamble;
    bool     crc;
    bool     implicit;
    bool     ldro;
    bool     invert_iq;
    int8_t   power_dbm;   // TX power in dBm
    bool     hp_pa;       // use HP PA (sub-GHz, >15 dBm capable)
    uint32_t count;       // packets to send, 0 = infinite
    uint32_t interval_ms; // delay between packets (ms)
    uint8_t  sync_word;   // same values as lora_rx_params_t
    // GFSK modem (--fsk flag)
    bool     fsk;
    uint32_t fsk_bitrate;
    uint32_t fsk_fdev_hz;
} lora_tx_params_t;

// TX test mode (CW carrier or infinite preamble for spectrum analyser work)
typedef struct {
    uint32_t freq_hz;
    int8_t   power_dbm;
    bool     hp_pa;
    bool     cw;           // true = unmodulated CW carrier; false = infinite preamble
    // For preamble mode only:
    bool     fsk;          // FSK preamble (default: LoRa)
    uint8_t  sf;           // LoRa SF
    uint16_t bw_khz;       // LoRa BW
    uint32_t fsk_bitrate;
    uint32_t fsk_fdev_hz;
} lora_txtest_params_t;

typedef struct {
    uint32_t freq_start;  // Hz
    uint32_t freq_end;    // Hz
    uint32_t freq_step;   // Hz
    uint8_t  sf;
    uint16_t bw_khz;
    uint8_t  cad_symb;    // CAD symbol count: 1, 2, 4, 8, or 16
    uint8_t  cad_peak;   // CAD detect_peak threshold (0 = use default per SF)
    uint8_t  cad_min;    // CAD detect_min threshold  (0 = use default per SF)
    bool     continuous;  // loop scan until Ctrl+C
} lora_scan_params_t;

typedef struct {
    uint32_t freq_hz;       // centre frequency in Hz (default: 2440000000)
    uint8_t  sf;            // spreading factor (default: 7)
    uint16_t bw_khz;        // bandwidth kHz   (default: 800)
    uint8_t  cr;            // coding rate denominator (default: 5)
    uint16_t preamble;      // preamble symbols (default: 8)
    int8_t   power_dbm;     // TX power in dBm  (default: 13)
    // TDMA timing
    bool     air_side;      // true = drone/AIR (TDMA master), false = ground (slave)
    uint32_t slot_ms;       // full cycle interval in ms (default: 200)
    uint32_t guard_ms;      // guard time GND waits before responding (default: 10)
    // Transport — exactly one of the three must be non-zero/non-empty:
    char     uart_path[64]; // e.g. /dev/serial0  (air side)
    uint32_t uart_baud;     // UART baud rate, e.g. 57600
    int      tcp_port;      // TCP server port  (ground side)
    char     pty_path[64];  // PTY symlink path  (testing / laptop)
} lora_mavlink_params_t;

// ------------------------------------------------------------ converters ----

bool                       freq_is_24g(uint32_t freq_hz);
lr11xx_radio_lora_sf_t     sf_to_enum(uint8_t sf);
lr11xx_radio_lora_bw_t     bw_to_enum(uint16_t bw_khz, bool is_24g);
lr11xx_radio_lora_cr_t     cr_to_enum(uint8_t cr);
uint8_t cad_symb_to_enum(uint8_t symb);
void                       cad_thresholds(uint8_t sf, uint8_t *peak, uint8_t *min);

// --------------------------------------------------- shared radio helpers ---

// Reset, wakeup, TCXO, RF switch, packet type (LORA).
// Call once before configuring the modem.
int radio_common_init(lora_ctx_t *ctx, bool is_24g);

// Configure PA and TX power.  Shared by TX, txtest, and mavlink modes.
int radio_configure_pa(lora_ctx_t *ctx, bool is_24g, bool hp_pa, int8_t power_dbm);

// Program LoRa modulation + packet parameters on a pre-initialised radio.
int radio_set_lora_modem(lora_ctx_t *ctx,
                         uint32_t freq_hz,
                         lr11xx_radio_lora_sf_t sf,
                         lr11xx_radio_lora_bw_t bw,
                         lr11xx_radio_lora_cr_t cr,
                         uint16_t preamble,
                         bool crc, bool implicit, uint8_t payload_len,
                         bool ldro, bool invert_iq,
                         uint8_t sync_word,   // 0 = use 0x12 default
                         bool is_24g);

// Program GFSK modulation + packet parameters.
// Overrides pkt type to GFSK (radio_common_init sets it to LoRa).
// bitrate_bps/fdev_hz: 0 uses defaults (50000 / 25000).
// RX bandwidth is auto-calculated from Carson's rule.
int radio_set_fsk_modem(lora_ctx_t *ctx,
                        uint32_t freq_hz,
                        uint32_t bitrate_bps,
                        uint32_t fdev_hz,
                        uint8_t  payload_len,
                        bool     is_24g);
