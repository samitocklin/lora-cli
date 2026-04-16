// main.c — lora-cli: LR1121 diagnostic and radio CLI for Raspberry Pi
//
// Usage examples:
//   sudo lora-cli --spi spi0 --status
//   sudo lora-cli --spi spi0 --rx --freq 868300000 --sf 7 --bw 125 --cr 5
//   sudo lora-cli --spi spi0 --rx --freq 2445000000 --sf 7 --bw 400 --bw-boosted
//   sudo lora-cli --spi spi0 --tx --freq 868300000 --sf 7 --bw 125 --power 14 \
//                             --payload "hello world" --count 10 --interval 1000
//   sudo lora-cli --spi spi0 --scan --freq-start 863000000 --freq-end 870000000
//   sudo lora-cli --spi spi0 --scan --freq-start 863000000 --freq-end 870000000 \
//                             --sf 7 --bw 125 --cad-symb 1 --continuous
//
// MAVLink bridge (two Pi Zeros, one at each end):
//   Air side  (FC UART → LoRa):
//     sudo lora-cli --spi spi0 --mavlink --freq 2440000000 --sf 7 --bw 800 \
//                   --uart /dev/serial0 --baud 57600
//   Ground side (LoRa → TCP for mavproxy on laptop):
//     sudo lora-cli --spi spi0 --mavlink --freq 2440000000 --sf 7 --bw 800 \
//                   --tcp-port 5760
//     mavproxy.py --master tcp:<ground-pi-ip>:5760

#define _GNU_SOURCE

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "board.h"
#include "radio.h"

// ------------------------------------------------- forward declarations ----

int mode_status(lora_ctx_t *ctx);
int mode_rx(lora_ctx_t *ctx, const lora_rx_params_t *p);
int mode_tx(lora_ctx_t *ctx, const lora_tx_params_t *p,
            const uint8_t *payload, uint16_t payload_len);
int mode_scan(lora_ctx_t *ctx, const lora_scan_params_t *p);
int mode_txtest(lora_ctx_t *ctx, const lora_txtest_params_t *p);
int mode_mavlink(lora_ctx_t *ctx, const lora_mavlink_params_t *p);

// ---------------------------------------------------------------- usage ----

static void print_usage(const char *prog)
{
    printf(
        "Usage: %s --spi <spi0|spi1> <mode> [options]\n"
        "\n"
        "Modes:\n"
        "  --status          read chip version, status and error registers\n"
        "  --rx              continuous receive (LoRa or GFSK)\n"
        "  --tx              transmit one or more packets (LoRa or GFSK)\n"
        "  --scan            CAD-based frequency scan with live spectrum display\n"
        "  --txtest          RF test: CW carrier or infinite preamble\n"
        "  --mavlink         MAVLink telemetry bridge over 2.4 GHz LoRa (TDMA)\n"
        "\n"
        "SPI:\n"
        "  --spi <spi0|spi1>  SPI bus                              [default: spi0]\n"
        "\n"
        "Shared  [rx · tx · scan · txtest · mavlink]\n"
        "  --freq <Hz>        centre frequency                      [default: 868100000]\n"
        "\n"
        "LoRa modem  [rx · tx · scan · txtest · mavlink]\n"
        "  --sf <5-12>        spreading factor                      [default: 7]\n"
        "  --bw <kHz>         bandwidth: 125/250/500 sub-GHz\n"
        "                                200/400/800 2.4 GHz        [default: 125]\n"
        "  --cr <5-8>         coding rate 4/x                       [default: 5]\n"
        "  --preamble <n>     preamble length in symbols            [default: 8]\n"
        "  --sync-word <hex>  sync byte                             [default: 12]\n"
        "                       12 = private LoRa / satellite\n"
        "                       34 = LoRaWAN public\n"
        "                       2B = Meshtastic\n"
        "  --no-crc           disable CRC\n"
        "  --implicit         implicit header mode\n"
        "  --payload-len <n>  expected payload bytes (implicit)     [default: 255]\n"
        "  --ldro             low data-rate optimisation\n"
        "  --invert-iq        invert IQ polarity\n"
        "\n"
        "GFSK modem  [rx · tx · txtest]  (add --fsk to any of these modes)\n"
        "  --fsk              use GFSK instead of LoRa\n"
        "  --bitrate <bps>    bit rate                              [default: 50000]\n"
        "  --fdev <Hz>        frequency deviation                   [default: 25000]\n"
        "  (fixed: BT=0.5, 5-byte preamble, sync 0x2DD42DD4, 2-byte CRC, whitening)\n"
        "\n"
        "──── --rx ────────────────────────────────────────────────────────────────\n"
        "  --bw-boosted       boosted LNA for better sensitivity (sub-GHz only)\n"
        "\n"
        "──── --tx ────────────────────────────────────────────────────────────────\n"
        "  --power <dBm>      TX power                              [default: 14]\n"
        "  --hp-pa            high-power PA (sub-GHz, required for >15 dBm)\n"
        "  --payload <str>    ASCII payload                         [default: \"lora-cli test\"]\n"
        "  --payload-hex <hex>  hex payload (e.g. deadbeef)\n"
        "  --count <n>        packets to send; 0 = infinite         [default: 1]\n"
        "  --interval <ms>    inter-packet delay                    [default: 1000]\n"
        "\n"
        "──── --scan ──────────────────────────────────────────────────────────────\n"
        "  --freq-start <Hz>  scan start frequency                  [default: 863000000]\n"
        "  --freq-end <Hz>    scan end frequency                    [default: 870000000]\n"
        "  --freq-step <Hz>   step size                             [default: 200000]\n"
        "  --cad-symb <n>     CAD symbol count: 1/2/4/8/16          [default: 1]\n"
        "  --cad-peak <0-255> CAD detect_peak threshold             [default: auto per SF]\n"
        "  --cad-min <0-255>  CAD detect_min threshold              [default: auto per SF]\n"
        "  --continuous       repeat scan until Ctrl+C\n"
        "\n"
        "──── --txtest ────────────────────────────────────────────────────────────\n"
        "  --cw               unmodulated CW carrier (default: infinite preamble)\n"
        "  --power <dBm>      TX power                              [default: 14]\n"
        "  --hp-pa            high-power PA (sub-GHz, required for >15 dBm)\n"
        "  (LoRa preamble: use --sf and --bw)\n"
        "  (GFSK preamble: add --fsk; use --bitrate and --fdev)\n"
        "\n"
        "──── --mavlink ───────────────────────────────────────────────────────────\n"
        "  --air              this unit is on the drone (TDMA master)\n"
        "                       omit for ground side (TDMA slave)\n"
        "  --uart <path>      UART device for flight controller     [e.g. /dev/serial0]\n"
        "  --baud <rate>      UART baud rate                        [default: 57600]\n"
        "  --tcp-port <port>  TCP server port for mavproxy          [e.g. 5760]\n"
        "  --pty <path>       create PTY symlink for local testing  [e.g. /tmp/ttyLORA0]\n"
        "  --slot-ms <ms>     TDMA slot interval                    [default: 200]\n"
        "                       must be > air TX + guard + GND TX\n"
        "                       at SF7/BW800 each side needs ~62 ms\n"
        "  --guard-ms <ms>    guard time ground waits before TX     [default: 10]\n"
        "  (defaults: freq=2440000000, SF7, BW800, CR4/5, power=13 dBm)\n"
        "\n"
        "Examples:\n"
        "  %s --spi spi0 --status\n"
        "  %s --spi spi0 --rx --freq 868300000 --sf 9 --bw 125\n"
        "  %s --spi spi0 --rx --freq 869525000 --sf 11 --bw 250 --sync-word 2B\n"
        "  %s --spi spi0 --rx --fsk --freq 868000000\n"
        "  %s --spi spi0 --tx --freq 868300000 --payload \"hello\" --count 5\n"
        "  %s --spi spi0 --tx --fsk --freq 868000000 --payload \"gfsk\"\n"
        "  %s --spi spi0 --scan --continuous\n"
        "  %s --spi spi0 --txtest --cw --freq 868000000 --power 22 --hp-pa\n"
        "  %s --spi spi0 --txtest --freq 2440000000 --sf 7 --bw 800\n"
        "  %s --spi spi0 --mavlink --air --freq 2440000000 --uart /dev/serial0\n"
        "  %s --spi spi0 --mavlink --freq 2440000000 --tcp-port 5760\n",
        prog,
        prog, prog, prog, prog, prog, prog, prog, prog, prog, prog, prog);
}

// ---------------------------------------------- hex string → bytes --------

static int parse_hex(const char *hex, uint8_t *out, size_t max_len, size_t *out_len)
{
    size_t n = strlen(hex);
    if (n % 2 != 0) { fprintf(stderr, "Hex string must have even length\n"); return -1; }
    if (n / 2 > max_len) { fprintf(stderr, "Hex payload too long (max %zu bytes)\n", max_len); return -1; }
    *out_len = n / 2;
    for (size_t i = 0; i < *out_len; i++) {
        unsigned byte;
        if (sscanf(hex + 2 * i, "%02x", &byte) != 1) {
            fprintf(stderr, "Invalid hex at position %zu\n", 2 * i);
            return -1;
        }
        out[i] = (uint8_t)byte;
    }
    return 0;
}

// ----------------------------------------------------- signal handler ------

static void on_sigint(int sig)
{
    (void)sig;
    g_stop = 1;
}

// -------------------------------------------------------- arg parsing ------

typedef enum {
    MODE_NONE,
    MODE_STATUS,
    MODE_RX,
    MODE_TX,
    MODE_SCAN,
    MODE_TXTEST,
    MODE_MAVLINK,
} app_mode_t;

typedef struct {
    app_mode_t            mode;
    const board_pins_t   *pins;
    lora_rx_params_t      rx;
    lora_tx_params_t      tx;
    lora_scan_params_t    scan;
    lora_txtest_params_t  txtest;
    lora_mavlink_params_t mav;
    uint8_t               payload[255];
    uint16_t              payload_len;
} app_cfg_t;

static void cfg_defaults(app_cfg_t *c)
{
    memset(c, 0, sizeof(*c));
    c->pins = &BOARD_SPI0;

    // RX defaults
    c->rx.freq_hz    = 868100000;
    c->rx.sf         = 7;
    c->rx.bw_khz     = 125;
    c->rx.cr         = 5;
    c->rx.preamble   = 8;
    c->rx.crc        = true;
    c->rx.payload_len = 255;

    // TX defaults (mirrors RX)
    c->tx.freq_hz    = 868100000;
    c->tx.sf         = 7;
    c->tx.bw_khz     = 125;
    c->tx.cr         = 5;
    c->tx.preamble   = 8;
    c->tx.crc        = true;
    c->tx.power_dbm  = 14;
    c->tx.count      = 1;
    c->tx.interval_ms = 1000;

    // Scan defaults
    c->scan.freq_start = 863000000;
    c->scan.freq_end   = 870000000;
    c->scan.freq_step  = 200000;
    c->scan.sf         = 7;
    c->scan.bw_khz     = 125;
    c->scan.cad_symb   = 1;

    // TX test defaults
    c->txtest.freq_hz    = 868000000;
    c->txtest.power_dbm  = 14;
    c->txtest.sf         = 7;
    c->txtest.bw_khz     = 125;

    // MAVLink bridge defaults (2.44 GHz, SF7 BW800 for max throughput)
    c->mav.freq_hz    = 2440000000u;
    c->mav.sf         = 7;
    c->mav.bw_khz     = 800;
    c->mav.cr         = 5;
    c->mav.preamble   = 8;
    c->mav.power_dbm  = 13;
    c->mav.uart_baud  = 57600;
    c->mav.slot_ms    = 200;
    c->mav.guard_ms   = 10;
}

// Check that flag at argv[i] has a following argument; fatal if not.
static int need_arg(const char *flag, int i, int argc)
{
    if (i + 1 >= argc) {
        fprintf(stderr, "%s requires an argument\n", flag);
        return -1;
    }
    return 0;
}

static int parse_args(int argc, char **argv, app_cfg_t *c)
{
    cfg_defaults(c);

    if (argc < 2) return -1;

    bool payload_set = false;

    for (int i = 1; i < argc; i++) {
        const char *a = argv[i];

        // ---- mode flags
        if (strcmp(a, "--status")  == 0) { c->mode = MODE_STATUS;  continue; }
        if (strcmp(a, "--rx")      == 0) { c->mode = MODE_RX;      continue; }
        if (strcmp(a, "--tx")      == 0) { c->mode = MODE_TX;      continue; }
        if (strcmp(a, "--scan")    == 0) { c->mode = MODE_SCAN;    continue; }
        if (strcmp(a, "--txtest")  == 0) { c->mode = MODE_TXTEST;  continue; }
        if (strcmp(a, "--mavlink") == 0) { c->mode = MODE_MAVLINK; continue; }

        // ---- boolean flags (no argument)
        if (strcmp(a, "--no-crc")     == 0) { c->rx.crc = c->tx.crc = false; continue; }
        if (strcmp(a, "--implicit")   == 0) { c->rx.implicit = c->tx.implicit = true; continue; }
        if (strcmp(a, "--ldro")       == 0) { c->rx.ldro = c->tx.ldro = true; continue; }
        if (strcmp(a, "--invert-iq")  == 0) { c->rx.invert_iq = c->tx.invert_iq = true; continue; }
        if (strcmp(a, "--fsk")        == 0) {
            c->rx.fsk = c->tx.fsk = c->txtest.fsk = true; continue;
        }
        if (strcmp(a, "--cw")         == 0) { c->txtest.cw = true; continue; }
        if (strcmp(a, "--sync-word") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            unsigned long v = strtoul(argv[++i], NULL, 16);
            c->rx.sync_word = c->tx.sync_word = (uint8_t)v;
            continue;
        }
        if (strcmp(a, "--bw-boosted") == 0) { c->rx.boosted_rx = true; continue; }
        if (strcmp(a, "--hp-pa")      == 0) { c->tx.hp_pa = c->txtest.hp_pa = true; continue; }
        if (strcmp(a, "--continuous") == 0) { c->scan.continuous = true; continue; }
        if (strcmp(a, "--help") == 0 || strcmp(a, "-h") == 0) return -1;

        // ---- flags that take an argument
        if (strcmp(a, "--spi") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            if      (strcmp(argv[++i], "spi0") == 0) c->pins = &BOARD_SPI0;
            else if (strcmp(argv[i],   "spi1") == 0) c->pins = &BOARD_SPI1;
            else { fprintf(stderr, "Unknown SPI bus: %s\n", argv[i]); return -1; }
            continue;
        }
        if (strcmp(a, "--freq") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            uint32_t f = (uint32_t)strtoul(argv[++i], NULL, 10);
            c->rx.freq_hz = c->tx.freq_hz = c->mav.freq_hz = c->txtest.freq_hz = f;
            continue;
        }
        if (strcmp(a, "--sf") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            uint8_t sf = (uint8_t)atoi(argv[++i]);
            if (sf < 5 || sf > 12) { fprintf(stderr, "--sf must be 5-12\n"); return -1; }
            c->rx.sf = c->tx.sf = c->scan.sf = c->mav.sf = c->txtest.sf = sf;
            continue;
        }
        if (strcmp(a, "--bw") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            uint16_t bw = (uint16_t)atoi(argv[++i]);
            c->rx.bw_khz = c->tx.bw_khz = c->scan.bw_khz = c->mav.bw_khz = c->txtest.bw_khz = bw;
            continue;
        }
        if (strcmp(a, "--cr") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            uint8_t cr = (uint8_t)atoi(argv[++i]);
            if (cr < 5 || cr > 8) { fprintf(stderr, "--cr must be 5-8\n"); return -1; }
            c->rx.cr = c->tx.cr = c->mav.cr = cr;
            continue;
        }
        if (strcmp(a, "--preamble") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            uint16_t pre = (uint16_t)atoi(argv[++i]);
            c->rx.preamble = c->tx.preamble = c->mav.preamble = pre;
            continue;
        }
        if (strcmp(a, "--payload-len") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->rx.payload_len = (uint8_t)atoi(argv[++i]);
            continue;
        }
        if (strcmp(a, "--power") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            int8_t pwr = (int8_t)atoi(argv[++i]);
            c->tx.power_dbm = c->mav.power_dbm = c->txtest.power_dbm = pwr;
            continue;
        }
        if (strcmp(a, "--bitrate") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            uint32_t br = (uint32_t)strtoul(argv[++i], NULL, 10);
            c->rx.fsk_bitrate = c->tx.fsk_bitrate = c->txtest.fsk_bitrate = br;
            continue;
        }
        if (strcmp(a, "--fdev") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            uint32_t fd = (uint32_t)strtoul(argv[++i], NULL, 10);
            c->rx.fsk_fdev_hz = c->tx.fsk_fdev_hz = c->txtest.fsk_fdev_hz = fd;
            continue;
        }
        if (strcmp(a, "--payload") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            size_t len = strlen(argv[++i]);
            if (len > sizeof(c->payload)) {
                fprintf(stderr, "--payload too long (max %zu bytes)\n", sizeof(c->payload));
                return -1;
            }
            memcpy(c->payload, argv[i], len);
            c->payload_len = (uint16_t)len;
            payload_set = true;
            continue;
        }
        if (strcmp(a, "--payload-hex") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            size_t len = 0;
            if (parse_hex(argv[++i], c->payload, sizeof(c->payload), &len) < 0) return -1;
            c->payload_len = (uint16_t)len;
            payload_set = true;
            continue;
        }
        if (strcmp(a, "--count") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->tx.count = (uint32_t)strtoul(argv[++i], NULL, 10);
            continue;
        }
        if (strcmp(a, "--interval") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->tx.interval_ms = (uint32_t)strtoul(argv[++i], NULL, 10);
            continue;
        }
        if (strcmp(a, "--freq-start") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->scan.freq_start = (uint32_t)strtoul(argv[++i], NULL, 10);
            continue;
        }
        if (strcmp(a, "--freq-end") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->scan.freq_end = (uint32_t)strtoul(argv[++i], NULL, 10);
            continue;
        }
        if (strcmp(a, "--freq-step") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->scan.freq_step = (uint32_t)strtoul(argv[++i], NULL, 10);
            continue;
        }
        if (strcmp(a, "--cad-symb") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->scan.cad_symb = (uint8_t)atoi(argv[++i]);
            continue;
        }
        if (strcmp(a, "--cad-peak") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->scan.cad_peak = (uint8_t)atoi(argv[++i]);
            continue;
        }
        if (strcmp(a, "--cad-min") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->scan.cad_min = (uint8_t)atoi(argv[++i]);
            continue;
        }

        // ---- MAVLink bridge options
        if (strcmp(a, "--air")  == 0) { c->mav.air_side = true;  continue; }
        if (strcmp(a, "--uart") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            snprintf(c->mav.uart_path, sizeof(c->mav.uart_path), "%s", argv[++i]);
            continue;
        }
        if (strcmp(a, "--baud") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->mav.uart_baud = (uint32_t)strtoul(argv[++i], NULL, 10);
            continue;
        }
        if (strcmp(a, "--tcp-port") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->mav.tcp_port = atoi(argv[++i]);
            continue;
        }
        if (strcmp(a, "--pty") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            snprintf(c->mav.pty_path, sizeof(c->mav.pty_path), "%s", argv[++i]);
            continue;
        }
        if (strcmp(a, "--slot-ms") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->mav.slot_ms = (uint32_t)strtoul(argv[++i], NULL, 10);
            continue;
        }
        if (strcmp(a, "--guard-ms") == 0) {
            if (need_arg(a, i, argc) < 0) return -1;
            c->mav.guard_ms = (uint32_t)strtoul(argv[++i], NULL, 10);
            continue;
        }

        fprintf(stderr, "Unknown option: %s\n", a);
        return -1;
    }

    if (c->mode == MODE_NONE) {
        fprintf(stderr, "No mode specified.\n");
        return -1;
    }

    if (c->mode == MODE_TX && !payload_set) {
        // Default TX payload: "lora-cli test"
        const char *def = "lora-cli test";
        memcpy(c->payload, def, strlen(def));
        c->payload_len = (uint16_t)strlen(def);
    }

    return 0;
}

// ----------------------------------------------------------------- main ----

int main(int argc, char **argv)
{
    signal(SIGINT, on_sigint);

    app_cfg_t app;
    if (parse_args(argc, argv, &app) < 0) {
        print_usage(argv[0]);
        return 1;
    }

    lora_ctx_t ctx;
    if (ctx_init(&ctx, app.pins) < 0) {
        fprintf(stderr, "Failed to initialise platform\n");
        return 1;
    }

    int rc = 0;
    switch (app.mode) {
    case MODE_STATUS:
        rc = mode_status(&ctx);
        break;
    case MODE_RX:
        rc = mode_rx(&ctx, &app.rx);
        break;
    case MODE_TX:
        rc = mode_tx(&ctx, &app.tx, app.payload, app.payload_len);
        break;
    case MODE_SCAN:
        rc = mode_scan(&ctx, &app.scan);
        break;
    case MODE_TXTEST:
        rc = mode_txtest(&ctx, &app.txtest);
        break;
    case MODE_MAVLINK:
        rc = mode_mavlink(&ctx, &app.mav);
        break;
    default:
        print_usage(argv[0]);
        rc = 1;
    }

    if (rc < 0) pulse_led(&ctx.led_rx, 250);

    ctx_deinit(&ctx);
    return (rc == 0) ? 0 : 1;
}
