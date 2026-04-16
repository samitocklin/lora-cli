// mode_mavlink.c — MAVLink telemetry bridge, TDMA half-duplex over 2.4 GHz LoRa
//
// Protocol: fixed ping-pong slots driven by the AIR (drone) side.
//
//   Cycle (default slot_ms = 200 ms):
//   ┌─────────────────────────────────────────────────────────────┐
//   │ ← slot_ms (200 ms) →                                        │
//   │ [AIR TX ~62 ms][guard 10 ms][GND TX ~62 ms][idle]           │
//   └─────────────────────────────────────────────────────────────┘
//
//   AIR side (drone, TDMA master):
//     • Sends one LoRa packet at the start of every slot_ms window.
//     • Switches to RX immediately after TX_DONE.
//     • Waits for a GND response within the slot window while
//       draining the UART (reads any new FC data into the TX queue).
//     • When slot_ms elapses, repeats regardless of GND response.
//
//   GND side (ground station, TDMA slave):
//     • Stays in continuous RX, draining the TCP/PTY transport.
//     • On each AIR packet: writes payload to transport, then
//       waits guard_ms and transmits one GND packet.
//     • Returns to RX.
//
// On-air frame (5-byte header):
//   [0xAA][0x55][seq][flags][len][MAVLink bytes ...]
//   flags: FLAG_ROLE_AIR (0x01) or FLAG_ROLE_GND (0x02) | FLAG_BEACON (0x80)
//   max data per packet: 250 bytes
//
// Usage:
//   Air:    sudo lora-cli --spi spi0 --mavlink --air  --freq 2440000000 \
//                         --sf 7 --bw 800 --uart /dev/serial0 --baud 57600
//   Ground: sudo lora-cli --spi spi0 --mavlink --freq 2440000000 \
//                         --sf 7 --bw 800 --tcp-port 5760

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#ifdef __linux__
#  include <pty.h>
#endif

#include "board.h"
#include "radio.h"
#include "lr11xx_radio.h"
#include "lr11xx_regmem.h"
#include "lr11xx_system.h"

// ----------------------------------------------------------------- framing ---

#define FRAME_MAGIC0    0xAAu
#define FRAME_MAGIC1    0x55u
#define FRAME_HDR_LEN   5          // magic0, magic1, seq, flags, datalen
#define MAX_PKT_DATA    (255 - FRAME_HDR_LEN)   // 250 bytes

#define FLAG_ROLE_AIR   0x01u  // packet originated on air side
#define FLAG_ROLE_GND   0x02u  // packet originated on ground side
#define FLAG_BEACON     0x80u  // no payload (slot-keeper / timing-only)

// ------------------------------------------------------------ TX ring buffer --

#define TXBUF_SIZE  8192

typedef struct {
    uint8_t data[TXBUF_SIZE];
    int     head, tail, used;
} ringbuf_t;

static void rb_reset(ringbuf_t *rb) { rb->head = rb->tail = rb->used = 0; }

static int rb_push(ringbuf_t *rb, const uint8_t *src, int n)
{
    if (rb->used + n > TXBUF_SIZE) return -1;
    for (int i = 0; i < n; i++) {
        rb->data[rb->tail] = src[i];
        rb->tail = (rb->tail + 1) % TXBUF_SIZE;
    }
    rb->used += n;
    return 0;
}

static int rb_pop(ringbuf_t *rb, uint8_t *dst, int n)
{
    if (n > rb->used) n = rb->used;
    for (int i = 0; i < n; i++) {
        dst[i] = rb->data[rb->head];
        rb->head = (rb->head + 1) % TXBUF_SIZE;
    }
    rb->used -= n;
    return n;
}

// ----------------------------------------------------------------- helpers ---

static int64_t now_ms(void)
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000 + ts.tv_nsec / 1000000;
}

static void drain_irq_events(lora_ctx_t *ctx)
{
    struct gpiod_edge_event_buffer *buf = gpiod_edge_event_buffer_new(16);
    if (buf) {
        gpiod_line_request_read_edge_events(ctx->irq.req, buf, 16);
        gpiod_edge_event_buffer_free(buf);
    }
}

// Read any available data from transport into ring buffer.
static void drain_transport(int fd, ringbuf_t *txbuf)
{
    uint8_t tmp[512];
    ssize_t n = read(fd, tmp, sizeof(tmp));
    if (n > 0) {
        if (rb_push(txbuf, tmp, (int)n) < 0)
            fprintf(stderr, "[mav] TX overflow, dropped %zd B\n", n);
    }
}

// Build one framed LoRa packet: pop up to MAX_PKT_DATA bytes from txbuf.
// If txbuf is empty, produces a beacon frame (no data).
// Returns total packet length.
static uint8_t build_packet(ringbuf_t *txbuf, uint8_t *pkt, uint8_t seq, uint8_t role)
{
    uint8_t data[MAX_PKT_DATA];
    int n = rb_pop(txbuf, data, MAX_PKT_DATA);

    uint8_t flags = role | (n == 0 ? FLAG_BEACON : 0);
    pkt[0] = FRAME_MAGIC0;
    pkt[1] = FRAME_MAGIC1;
    pkt[2] = seq;
    pkt[3] = flags;
    pkt[4] = (uint8_t)n;
    if (n > 0) memcpy(pkt + FRAME_HDR_LEN, data, n);
    return (uint8_t)(FRAME_HDR_LEN + n);
}

// Send a prepared LoRa packet.  Puts radio in standby first.
static int radio_send(lora_ctx_t *ctx, const uint8_t *pkt, uint8_t pkt_len)
{
    lr11xx_status_t st;
    st = lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_XOSC);
    if (st != LR11XX_STATUS_OK) return -1;
    st = lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
    if (st != LR11XX_STATUS_OK) return -1;
    st = lr11xx_regmem_write_buffer8(ctx, pkt, pkt_len);
    if (st != LR11XX_STATUS_OK) return -1;
    st = lr11xx_radio_set_tx(ctx, 0);
    return (st == LR11XX_STATUS_OK) ? 0 : -1;
}

// Arm continuous RX.
static int radio_rx(lora_ctx_t *ctx)
{
    lr11xx_status_t st;
    st = lr11xx_system_clear_irq_status(ctx, LR11XX_SYSTEM_IRQ_ALL_MASK);
    if (st != LR11XX_STATUS_OK) return -1;
    st = lr11xx_radio_set_rx_with_timeout_in_rtc_step(ctx, 0xFFFFFF);
    return (st == LR11XX_STATUS_OK) ? 0 : -1;
}

// Wait for a LoRa IRQ (TX_DONE or RX_DONE) by polling the gpiod fd.
// Also drains transport_fd into txbuf while waiting.
// Returns the LR11xx IRQ mask, or 0 on timeout/error.
static lr11xx_system_irq_mask_t wait_irq(lora_ctx_t *ctx, int irq_fd,
                                          int transport_fd, ringbuf_t *txbuf,
                                          int timeout_ms)
{
    int64_t deadline = now_ms() + timeout_ms;

    struct pollfd fds[2];
    fds[0].fd = irq_fd;     fds[0].events = POLLIN;
    fds[1].fd = transport_fd; fds[1].events = POLLIN;

    while (!g_stop) {
        int64_t remain = deadline - now_ms();
        if (remain <= 0) return 0;
        int t = (remain > 10) ? 10 : (int)remain;

        int pr = poll(fds, 2, t);
        if (pr < 0) { if (errno == EINTR) continue; return 0; }

        if (fds[1].revents & POLLIN)
            drain_transport(transport_fd, txbuf);

        if (fds[0].revents & POLLIN) {
            drain_irq_events(ctx);
            lr11xx_system_irq_mask_t irq = 0;
            if (lr11xx_system_get_and_clear_irq_status(ctx, &irq) == LR11XX_STATUS_OK)
                return irq;
        }
    }
    return 0;
}

// Decode a received LoRa packet.  Returns bytes written to transport, or -1.
static int process_rx_packet(lora_ctx_t *ctx, int transport_fd,
                              uint8_t expected_role, uint32_t *rx_pkts)
{
    uint8_t                         payload[255];
    lr11xx_radio_rx_buffer_status_t rxbuf;
    memset(&rxbuf, 0, sizeof(rxbuf));

    lr11xx_status_t st = lr11xx_radio_get_rx_buffer_status(ctx, &rxbuf);
    if (st != LR11XX_STATUS_OK || rxbuf.pld_len_in_bytes < FRAME_HDR_LEN) return -1;

    st = lr11xx_regmem_read_buffer8(ctx, payload,
                                    rxbuf.buffer_start_pointer,
                                    rxbuf.pld_len_in_bytes);
    if (st != LR11XX_STATUS_OK) return -1;

    if (payload[0] != FRAME_MAGIC0 || payload[1] != FRAME_MAGIC1) return -1;

    uint8_t flags   = payload[3];
    uint8_t datalen = payload[4];

    // Validate role (reject own echoes)
    if ((flags & 0x03u) != expected_role) return -1;

    if (datalen > 0 && (uint16_t)FRAME_HDR_LEN + datalen <= rxbuf.pld_len_in_bytes
        && !(flags & FLAG_BEACON))
    {
        ssize_t wr = write(transport_fd, payload + FRAME_HDR_LEN, datalen);
        if (wr > 0) {
            (*rx_pkts)++;
            return (int)wr;
        }
    } else if (flags & FLAG_BEACON) {
        (*rx_pkts)++;
        return 0;   // beacon: timing counted but no data
    }
    return -1;
}

// ---------------------------------------------------- transport open helpers --

static int transport_uart(const char *path, uint32_t baud)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { fprintf(stderr, "open(%s): %s\n", path, strerror(errno)); return -1; }

    struct termios tty;
    tcgetattr(fd, &tty);
    cfmakeraw(&tty);
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 0;

    speed_t spd;
    switch (baud) {
    case 9600:   spd = B9600;   break;
    case 19200:  spd = B19200;  break;
    case 38400:  spd = B38400;  break;
    case 57600:  spd = B57600;  break;
    case 115200: spd = B115200; break;
    case 230400: spd = B230400; break;
    case 460800: spd = B460800; break;
    case 921600: spd = B921600; break;
    default:
        fprintf(stderr, "Unsupported baud %u, using 115200\n", baud);
        spd = B115200;
    }
    cfsetispeed(&tty, spd); cfsetospeed(&tty, spd);
    tcsetattr(fd, TCSANOW, &tty);

    printf("UART  : %s  %u baud\n", path, baud);
    return fd;
}

static int transport_tcp_accept(int port)
{
    int srv = socket(AF_INET, SOCK_STREAM, 0);
    if (srv < 0) { perror("socket"); return -1; }
    int one = 1;
    setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    struct sockaddr_in addr = {
        .sin_family      = AF_INET,
        .sin_addr.s_addr = INADDR_ANY,
        .sin_port        = htons((uint16_t)port),
    };
    if (bind(srv, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind"); close(srv); return -1;
    }
    listen(srv, 1);
    printf("TCP   : waiting on port %d\n", port);
    printf("  mavproxy.py --master tcp:<this-ip>:%d\n\n", port);
    fflush(stdout);
    int cli = accept(srv, NULL, NULL);
    close(srv);
    if (cli < 0) { perror("accept"); return -1; }
    setsockopt(cli, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
    int flags = fcntl(cli, F_GETFL, 0);
    if (flags >= 0) fcntl(cli, F_SETFL, flags | O_NONBLOCK);
    printf("TCP   : client connected\n");
    return cli;
}

static int transport_pty(const char *pty_path)
{
    int master = posix_openpt(O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (master < 0) { perror("posix_openpt"); return -1; }
    grantpt(master); unlockpt(master);
    const char *sname = ptsname(master);
    unlink(pty_path);
    symlink(sname, pty_path);
    printf("PTY   : %s -> %s\n", pty_path, sname);
    printf("  mavproxy.py --master %s\n\n", pty_path);
    return master;
}

// --------------------------------------------------------- AIR side (master) --

static int run_air(lora_ctx_t *ctx, const lora_mavlink_params_t *p,
                   int transport_fd, int irq_fd)
{
    ringbuf_t txbuf;
    rb_reset(&txbuf);

    uint8_t  tx_seq  = 0;
    uint32_t tx_pkts = 0;
    uint32_t rx_pkts = 0;

    // Pace: t_next_tx advances by slot_ms each cycle regardless of jitter.
    int64_t t_next_tx = now_ms();

    printf("Role  : AIR (master)  slot=%u ms  guard=%u ms\n\n",
           p->slot_ms, p->guard_ms);
    fflush(stdout);

    while (!g_stop) {
        // ----- PHASE 1: transmit one AIR packet -----
        // Snapshot any FC data that arrived since last poll
        drain_transport(transport_fd, &txbuf);

        uint8_t pkt[255];
        uint8_t pkt_len = build_packet(&txbuf, pkt, tx_seq++, FLAG_ROLE_AIR);

        gpio_set(&ctx->led_tx, 1);
        if (radio_send(ctx, pkt, pkt_len) < 0) {
            fprintf(stderr, "[air] TX start failed\n");
            sleep_ms(50); continue;
        }

        // Wait for TX_DONE (up to 500 ms); drain transport while waiting
        lr11xx_system_irq_mask_t irq = wait_irq(ctx, irq_fd, transport_fd, &txbuf, 500);
        gpio_set(&ctx->led_tx, 0);

        if (!(irq & LR11XX_SYSTEM_IRQ_TX_DONE)) {
            fprintf(stderr, "[air] TX_DONE not received (irq=0x%08X)\n", (unsigned)irq);
        } else {
            tx_pkts++;
            printf("[air] TX #%u  %u B  (queue %d B remaining)\n",
                   tx_pkts, pkt_len - FRAME_HDR_LEN, txbuf.used);
            fflush(stdout);
        }

        // ----- PHASE 2: listen for GND response until slot expires -----
        radio_rx(ctx);
        t_next_tx += p->slot_ms;

        bool got_gnd = false;
        while (!g_stop && now_ms() < t_next_tx) {
            int64_t remain = t_next_tx - now_ms();
            int     t      = (remain > 10) ? 10 : (int)remain;
            if (t <= 0) break;

            struct pollfd fds[2];
            fds[0].fd = irq_fd;       fds[0].events = POLLIN;
            fds[1].fd = transport_fd; fds[1].events = POLLIN;

            int pr = poll(fds, 2, t);
            if (pr < 0) { if (errno == EINTR) break; break; }

            if (fds[1].revents & POLLIN)
                drain_transport(transport_fd, &txbuf);

            if (!got_gnd && (fds[0].revents & POLLIN)) {
                drain_irq_events(ctx);
                lr11xx_system_irq_mask_t rx_irq = 0;
                lr11xx_system_get_and_clear_irq_status(ctx, &rx_irq);

                if (rx_irq & LR11XX_SYSTEM_IRQ_RX_DONE) {
                    int n = process_rx_packet(ctx, transport_fd, FLAG_ROLE_GND, &rx_pkts);
                    if (n >= 0) {
                        got_gnd = true;
                        gpio_set(&ctx->led_rx, 1);
                        printf("[air] RX from GND  %d B\n", n);
                        fflush(stdout);
                        sleep_ms(3);
                        gpio_set(&ctx->led_rx, 0);
                    }
                    // Re-arm RX to catch any late packets within the slot
                    radio_rx(ctx);
                }
            }
        }

        // Snap slot start to now if we've drifted ahead
        if (t_next_tx < now_ms()) t_next_tx = now_ms();
    }

    printf("\n[air] stopped. TX=%u RX=%u\n", tx_pkts, rx_pkts);
    return 0;
}

// ------------------------------------------------------ GND side (slave) -----

static int run_gnd(lora_ctx_t *ctx, const lora_mavlink_params_t *p,
                   int transport_fd, int irq_fd, int tcp_port)
{
    ringbuf_t txbuf;
    rb_reset(&txbuf);

    uint8_t  tx_seq  = 0;
    uint32_t tx_pkts = 0;
    uint32_t rx_pkts = 0;

    printf("Role  : GND (slave)  guard=%u ms\n\n", p->guard_ms);
    fflush(stdout);

    // Start in RX
    radio_rx(ctx);

    while (!g_stop) {
        // ----- PHASE 1: wait for AIR packet -----
        // Timeout = 3x slot_ms so we notice if air side goes silent
        int64_t deadline = now_ms() + (int64_t)p->slot_ms * 3;
        bool got_air = false;

        while (!g_stop && now_ms() < deadline) {
            int64_t remain = deadline - now_ms();
            int     t      = (remain > 10) ? 10 : (int)remain;

            struct pollfd fds[2];
            fds[0].fd = irq_fd;       fds[0].events = POLLIN;
            fds[1].fd = transport_fd; fds[1].events = POLLIN;

            int pr = poll(fds, 2, t);
            if (pr < 0) { if (errno == EINTR) break; break; }

            if (fds[1].revents & POLLIN)
                drain_transport(transport_fd, &txbuf);

            if (fds[0].revents & POLLIN) {
                drain_irq_events(ctx);
                lr11xx_system_irq_mask_t rx_irq = 0;
                lr11xx_system_get_and_clear_irq_status(ctx, &rx_irq);

                if (rx_irq & LR11XX_SYSTEM_IRQ_RX_DONE) {
                    int n = process_rx_packet(ctx, transport_fd, FLAG_ROLE_AIR, &rx_pkts);
                    if (n >= 0) {
                        got_air = true;
                        gpio_set(&ctx->led_rx, 1);
                        printf("[gnd] RX from AIR  %d B\n", n);
                        fflush(stdout);
                        sleep_ms(3);
                        gpio_set(&ctx->led_rx, 0);
                        break;
                    }
                    // CRC error or non-AIR frame; stay in RX
                    radio_rx(ctx);
                }

                if (rx_irq & (LR11XX_SYSTEM_IRQ_CRC_ERROR | LR11XX_SYSTEM_IRQ_HEADER_ERROR |
                               LR11XX_SYSTEM_IRQ_TIMEOUT   | LR11XX_SYSTEM_IRQ_ERROR)) {
                    radio_rx(ctx);   // re-arm after error
                }
            }
        }

        if (!got_air && !g_stop)
            printf("[gnd] timeout waiting for AIR — is air side running?\n");

        // ----- PHASE 2: guard time, then TX GND packet -----
        // Drain transport one more time during guard
        if (p->guard_ms > 0) {
            struct pollfd fds[1];
            fds[0].fd = transport_fd; fds[0].events = POLLIN;
            poll(fds, 1, (int)p->guard_ms);
            if (fds[0].revents & POLLIN)
                drain_transport(transport_fd, &txbuf);
        }

        // Handle TCP reconnect
        if (tcp_port > 0) {
            uint8_t probe[1];
            ssize_t r = recv(transport_fd, probe, 1, MSG_PEEK | MSG_DONTWAIT);
            if (r == 0) {
                printf("[gnd] TCP client disconnected, re-listening...\n");
                close(transport_fd);
                transport_fd = transport_tcp_accept(tcp_port);
                if (transport_fd < 0) break;
                rb_reset(&txbuf);
            }
        }

        uint8_t pkt[255];
        uint8_t pkt_len = build_packet(&txbuf, pkt, tx_seq++, FLAG_ROLE_GND);

        gpio_set(&ctx->led_tx, 1);
        if (radio_send(ctx, pkt, pkt_len) < 0) {
            fprintf(stderr, "[gnd] TX start failed\n");
            radio_rx(ctx); continue;
        }

        // Wait for TX_DONE (drain transport while waiting)
        lr11xx_system_irq_mask_t irq = wait_irq(ctx, irq_fd, transport_fd, &txbuf, 500);
        gpio_set(&ctx->led_tx, 0);

        if (!(irq & LR11XX_SYSTEM_IRQ_TX_DONE)) {
            fprintf(stderr, "[gnd] TX_DONE not received (irq=0x%08X)\n", (unsigned)irq);
        } else {
            tx_pkts++;
            printf("[gnd] TX #%u  %u B\n", tx_pkts, pkt_len - FRAME_HDR_LEN);
            fflush(stdout);
        }

        // Back to RX for next AIR packet
        radio_rx(ctx);
    }

    printf("\n[gnd] stopped. TX=%u RX=%u\n", tx_pkts, rx_pkts);
    return 0;
}

// ------------------------------------------------------------------- mode ----

int mode_mavlink(lora_ctx_t *ctx, const lora_mavlink_params_t *p)
{
    bool is_24g = freq_is_24g(p->freq_hz);

    printf("=== MAVLink bridge (%s) ===\n", p->air_side ? "AIR" : "GROUND");
    printf("freq  : %.6f MHz (%s)\n", p->freq_hz / 1e6, is_24g ? "2.4G" : "sub-GHz");
    printf("modem : SF%u  BW%u kHz  CR 4/%u  preamble %u  power %d dBm\n",
           p->sf, p->bw_khz, p->cr, p->preamble, p->power_dbm);

    // ---- Open transport ----
    int transport_fd = -1;
    bool is_pty = false;
    int  tcp_port_for_reconnect = 0;

    if (p->uart_path[0]) {
        transport_fd = transport_uart(p->uart_path, p->uart_baud);
    } else if (p->tcp_port > 0) {
        tcp_port_for_reconnect = p->tcp_port;
        transport_fd = transport_tcp_accept(p->tcp_port);
    } else if (p->pty_path[0]) {
        is_pty = true;
        transport_fd = transport_pty(p->pty_path);
    } else {
        fprintf(stderr, "No transport: specify --uart, --tcp-port, or --pty\n");
        return -1;
    }
    if (transport_fd < 0) return -1;

    // ---- Init radio ----
    if (radio_common_init(ctx, is_24g) < 0) goto fail;

    if (radio_set_lora_modem(ctx, p->freq_hz,
                             sf_to_enum(p->sf),
                             bw_to_enum(p->bw_khz, is_24g),
                             cr_to_enum(p->cr),
                             p->preamble,
                             true, false, 0,    // CRC on, explicit header, variable len
                             false, false,       // LDRO off, standard IQ
                             0x12, is_24g) < 0) goto fail;

    // PA config
    {
        lr11xx_radio_pa_cfg_t pa = { 0 };
        if (is_24g) {
            pa.pa_sel        = LR11XX_RADIO_PA_SEL_HF;
            pa.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
        } else {
            pa.pa_sel        = LR11XX_RADIO_PA_SEL_LP;
            pa.pa_reg_supply = LR11XX_RADIO_PA_REG_SUPPLY_VREG;
            pa.pa_duty_cycle = 4;
        }
        if (lr11xx_radio_set_pa_cfg(ctx, &pa)                                  != LR11XX_STATUS_OK ||
            lr11xx_radio_set_tx_params(ctx, p->power_dbm, LR11XX_RADIO_RAMP_48_US) != LR11XX_STATUS_OK)
        {
            fprintf(stderr, "PA/TX params failed\n"); goto fail;
        }
    }

    // IRQ mask: everything
    if (lr11xx_system_set_dio_irq_params(
            ctx,
            LR11XX_SYSTEM_IRQ_RX_DONE | LR11XX_SYSTEM_IRQ_TX_DONE |
            LR11XX_SYSTEM_IRQ_TIMEOUT | LR11XX_SYSTEM_IRQ_CRC_ERROR |
            LR11XX_SYSTEM_IRQ_HEADER_ERROR | LR11XX_SYSTEM_IRQ_ERROR,
            0) != LR11XX_STATUS_OK) {
        fprintf(stderr, "set_dio_irq failed\n"); goto fail;
    }

    // ---- Run ----
    int irq_fd = gpiod_line_request_get_fd(ctx->irq.req);
    int rc;

    if (p->air_side)
        rc = run_air(ctx, p, transport_fd, irq_fd);
    else
        rc = run_gnd(ctx, p, transport_fd, irq_fd, tcp_port_for_reconnect);

    lr11xx_system_set_standby(ctx, LR11XX_SYSTEM_STANDBY_CFG_RC);
    gpio_set(&ctx->led_tx, 0);
    gpio_set(&ctx->led_rx, 0);
    if (is_pty && p->pty_path[0]) unlink(p->pty_path);
    close(transport_fd);
    return rc;

fail:
    if (is_pty && p->pty_path[0]) unlink(p->pty_path);
    close(transport_fd);
    return -1;
}
