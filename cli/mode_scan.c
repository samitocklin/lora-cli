// mode_scan.c — LoRa CAD frequency scanner with in-place terminal spectrum display
//
// Redraws a spectrum view after each full sweep using ANSI escape codes.
// When channels fit in the terminal: one line per channel.
// When there are more channels than rows: adjacent channels are bucketed
// and the bucket's peak hit count is displayed.

#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "board.h"
#include "radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_system.h"

// ---------------------------------------------------------------- ANSI ----

#define A_RESET  "\033[0m"
#define A_BOLD   "\033[1m"
#define A_DIM    "\033[2m"
#define A_RED    "\033[91m"
#define A_YELLOW "\033[93m"
#define A_GREEN  "\033[92m"
#define A_ORANGE "\033[38;5;208m"
#define A_CYAN   "\033[96m"
#define A_GRAY   "\033[90m"
#define A_WHITE  "\033[97m"
#define A_HOME   "\033[H"
#define A_CLR    "\033[2J\033[H"
#define A_EL     "\033[K"   // erase to end of line

// --------------------------------------------------------- terminal size ---

static void term_size(int *w, int *h)
{
    struct winsize ws;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0) {
        *w = (int)ws.ws_col;
        *h = (int)ws.ws_row;
    } else {
        *w = 80;
        *h = 24;
    }
}

// ---------------------------------------------------------- bar drawing ----

// Colour is based on detection RATE (hits / pass), not relative to max.
// This means the colours are absolute and stable regardless of how many
// channels are active — a channel at 100% rate is always red, 0% always grey.
//
// Bar LENGTH is still proportional to max_hits across all channels so you
// can see relative differences even when colours are all the same.
static void draw_bar(uint32_t hits, uint32_t max_hits, uint32_t pass, int bar_w)
{
    if (bar_w < 1) return;

    float rate = (pass > 0) ? (float)hits / (float)pass : 0.0f;

    // Absolute rate → colour
    const char *col;
    if      (rate == 0.0f) col = A_GRAY;
    else if (rate < 0.05f) col = "\033[32m";  // dark green  (very rare)
    else if (rate < 0.20f) col = A_GREEN;      // green       (<20% of passes)
    else if (rate < 0.50f) col = A_YELLOW;     // yellow      (<50%)
    else if (rate < 0.80f) col = A_ORANGE;     // orange      (<80%)
    else                   col = A_RED;         // red         (≥80% — very active)

    // Bar length relative to the busiest channel (comparison within scan)
    int filled = 0;
    if (max_hits > 0 && hits > 0) {
        filled = (int)(((unsigned long long)hits * (unsigned)bar_w + max_hits - 1) / max_hits);
        if (filled > bar_w) filled = bar_w;
    }

    if (filled > 0) {
        printf("%s%s", A_BOLD, col);
        for (int j = 0; j < filled; j++) printf("█");
        printf(A_RESET);
    }
    if (filled < bar_w) {
        printf(A_DIM A_GRAY);
        for (int j = filled; j < bar_w; j++) printf("·");
        printf(A_RESET);
    }
}

// --------------------------------------------------------- full redraw -----

// Layout constants
// " 868.100 ▕<bar>▏ 100% nnn\n"
//  ^9       ^1  ^1 ^5   ^4    = 20 chars overhead for single-ch rows
//  " 863.0─869.9 ▕<bar>▏ 100% nnn\n"  = 28 chars overhead for bucket rows
#define SINGLE_OVERHEAD  20
#define BUCKET_OVERHEAD  28

// Colour legend shown in header
static void print_legend(void)
{
    printf("  "
           "\033[32m" "▐▌" A_RESET "<5%%  "
           A_GREEN    "▐▌" A_RESET "<20%%  "
           A_YELLOW   "▐▌" A_RESET "<50%%  "
           A_ORANGE   "▐▌" A_RESET "<80%%  "
           A_RED      "▐▌" A_RESET "≥80%%  "
           A_GRAY     "(detection rate / pass)" A_RESET);
}

static void redraw(const uint32_t *freqs, const uint32_t *hits, int n_ch,
                   uint32_t max_hits, uint32_t pass, uint32_t total_hits,
                   const lora_scan_params_t *p, int tw, int th)
{
    printf(A_HOME);

    // ── header ──────────────────────────────────────────────────────────────
    printf(A_BOLD A_WHITE " LoRa CAD Spectrum" A_RESET
           "  SF%u  BW%u kHz  %u-sym"
           "  pass " A_CYAN "%-4u" A_RESET
           "  total hits " A_YELLOW "%u" A_RESET
           A_EL "\n",
           p->sf, p->bw_khz, p->cad_symb, pass, total_hits);

    printf(" ");
    print_legend();
    printf(A_EL "\n");

    // separator
    printf(A_DIM A_GRAY);
    for (int i = 0; i < tw; i++) printf("─");
    printf(A_RESET A_EL "\n");

    // ── channel rows ─────────────────────────────────────────────────────────
    int avail_rows = th - 4 - 1;  // hdr(2) + sep(1) + sep(1) + footer(1)
    if (avail_rows < 1) avail_rows = 1;

    if (n_ch <= avail_rows) {
        // ── one line per channel ───────────────────────────────────────────
        int bar_w = tw - SINGLE_OVERHEAD;
        if (bar_w < 4) bar_w = 4;

        for (int i = 0; i < n_ch; i++) {
            float rate = pass > 0 ? (float)hits[i] / (float)pass : 0.0f;
            printf(A_BOLD A_CYAN " %7.3f" A_RESET A_DIM A_GRAY "▕" A_RESET,
                   freqs[i] / 1e6);
            draw_bar(hits[i], max_hits, pass, bar_w);
            printf(A_DIM A_GRAY "▏" A_RESET);
            if (hits[i] > 0)
                printf(" " A_BOLD "%3u%%" A_RESET A_DIM " %3u" A_RESET,
                       (unsigned)(rate * 100 + 0.5f), hits[i]);
            else
                printf(A_DIM A_GRAY "   ·    " A_RESET);
            printf(A_EL "\n");
        }
    } else {
        // ── bucket: multiple channels per row ──────────────────────────────
        int bucket = (n_ch + avail_rows - 1) / avail_rows;
        int bar_w  = tw - BUCKET_OVERHEAD;
        if (bar_w < 4) bar_w = 4;

        for (int row = 0; row < avail_rows; row++) {
            int start = row * bucket;
            if (start >= n_ch) { printf(A_EL "\n"); continue; }
            int end = start + bucket;
            if (end > n_ch) end = n_ch;

            uint32_t bmax = 0;
            for (int k = start; k < end; k++)
                if (hits[k] > bmax) bmax = hits[k];

            float rate = pass > 0 ? (float)bmax / (float)pass : 0.0f;

            printf(A_BOLD A_CYAN " %7.3f" A_RESET A_DIM A_GRAY "─%7.3f▕" A_RESET,
                   freqs[start] / 1e6, freqs[end - 1] / 1e6);
            draw_bar(bmax, max_hits, pass, bar_w);
            printf(A_DIM A_GRAY "▏" A_RESET);
            if (bmax > 0)
                printf(" " A_BOLD "%3u%%" A_RESET A_DIM " %3u" A_RESET,
                       (unsigned)(rate * 100 + 0.5f), bmax);
            else
                printf(A_DIM A_GRAY "   ·    " A_RESET);
            printf(A_EL "\n");
        }
    }

    // ── footer ──────────────────────────────────────────────────────────────
    printf(A_DIM A_GRAY);
    for (int i = 0; i < tw; i++) printf("─");
    printf(A_RESET A_EL "\n");
    printf(A_DIM " %d ch  %.3f─%.3f MHz  step %.0f kHz"
                 "  if all red: try --cad-symb 2 or --cad-peak 28"
                 "  Ctrl+C to stop" A_RESET A_EL "\n",
           n_ch, freqs[0] / 1e6, freqs[n_ch - 1] / 1e6, p->freq_step / 1e3);

    fflush(stdout);
}

// ---------------------------------------------------------- scan logic -----

#define MAX_SCAN_CHANNELS 4096

// Approximate CAD duration (ms) for timeout sizing.
static unsigned cad_timeout_ms(uint8_t sf, uint16_t bw_khz, uint8_t symb)
{
    float sym_ms = (float)(1u << sf) / (float)bw_khz;
    unsigned t   = (unsigned)(sym_ms * symb * 1.5f) + 10;
    return t < 20 ? 20 : t;
}

int mode_scan(lora_ctx_t *ctx, const lora_scan_params_t *p)
{
    if (p->freq_start >= p->freq_end || p->freq_step == 0) {
        fprintf(stderr, "Invalid scan range\n");
        return -1;
    }

    bool is_24g = freq_is_24g(p->freq_start);

    // Build channel table
    static uint32_t freqs[MAX_SCAN_CHANNELS];
    static uint32_t hits[MAX_SCAN_CHANNELS];
    int n_ch = 0;
    for (uint32_t f = p->freq_start;
         f <= p->freq_end && n_ch < MAX_SCAN_CHANNELS;
         f += p->freq_step)
    {
        freqs[n_ch] = f;
        hits[n_ch]  = 0;
        n_ch++;
    }
    if (n_ch == 0) { fprintf(stderr, "No channels in range\n"); return -1; }

    unsigned timeout_ms = cad_timeout_ms(p->sf, p->bw_khz, p->cad_symb);

    // ── radio init ──────────────────────────────────────────────────────────
    if (radio_common_init(ctx, is_24g) < 0) return -1;

    const lr11xx_radio_mod_params_lora_t mod = {
        .sf   = sf_to_enum(p->sf),
        .bw   = bw_to_enum(p->bw_khz, is_24g),
        .cr   = LR11XX_RADIO_LORA_CR_4_5,
        .ldro = 0,
    };
    lr11xx_status_t st = lr11xx_radio_set_lora_mod_params(ctx, &mod);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_mod_params failed\n"); return -1; }

    uint8_t peak, cad_min;
    cad_thresholds(p->sf, &peak, &cad_min);
    if (p->cad_peak) peak    = p->cad_peak;
    if (p->cad_min)  cad_min = p->cad_min;
    const lr11xx_radio_cad_params_t cad_cfg = {
        .cad_symb_nb     = cad_symb_to_enum(p->cad_symb),
        .cad_detect_peak = peak,
        .cad_detect_min  = cad_min,
        .cad_exit_mode   = LR11XX_RADIO_CAD_EXIT_MODE_STANDBYRC,
        .cad_timeout     = 0,
    };
    st = lr11xx_radio_set_cad_params(ctx, &cad_cfg);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_cad_params failed\n"); return -1; }

    st = lr11xx_system_set_dio_irq_params(ctx,
             LR11XX_SYSTEM_IRQ_CAD_DONE | LR11XX_SYSTEM_IRQ_CAD_DETECTED, 0);
    if (st != LR11XX_STATUS_OK) { fprintf(stderr, "set_dio_irq failed\n"); return -1; }

    // Clear screen once before we start drawing in-place
    printf(A_CLR);
    fflush(stdout);

    uint32_t pass        = 0;
    uint32_t total_hits  = 0;
    uint32_t max_hits    = 1;

    do {
        pass++;

        for (int ci = 0; ci < n_ch && !g_stop; ci++) {
            st = lr11xx_radio_set_rf_freq(ctx, freqs[ci]);
            if (st != LR11XX_STATUS_OK) goto done;

            st = lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
            if (st != LR11XX_STATUS_OK) goto done;

            st = lr11xx_radio_set_cad(ctx);
            if (st != LR11XX_STATUS_OK) goto done;

            wait_irq_rising(ctx, (int)timeout_ms);

            lr11xx_system_irq_mask_t irq = 0;
            lr11xx_system_get_and_clear_irq_status(ctx, &irq);

            if (irq & LR11XX_SYSTEM_IRQ_CAD_DETECTED) {
                hits[ci]++;
                total_hits++;
                if (hits[ci] > max_hits) max_hits = hits[ci];
                pulse_led(&ctx->led_rx, 20);
            }
        }

        // Redraw after every complete sweep
        int tw, th;
        term_size(&tw, &th);
        redraw(freqs, hits, n_ch, max_hits, pass, total_hits, p, tw, th);

    } while (p->continuous && !g_stop);

done:
    lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
    // Move cursor below the display before printing the summary
    printf("\033[%dH\n", 999);
    printf("Scan done. %u pass(es), %u detection(s).\n", pass, total_hits);
    return 0;
}
