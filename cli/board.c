// board.c — platform HAL: GPIO (libgpiod v2), SPI (spidev), LR11xx HAL callbacks
#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "board.h"
#include "lr11xx_hal.h"

// --------------------------------------------------------- global stop flag -

volatile sig_atomic_t g_stop = 0;

// --------------------------------------------------------------- board defs -

const board_pins_t BOARD_SPI0 = {
    .spidev_path  = "/dev/spidev0.0",
    .spi_speed_hz = 1000000,
    .spi_mode     = SPI_MODE_0,
    .spi_bits     = 8,
    .gpio_reset   = 25,
    .gpio_busy    = 5,
    .gpio_dio8    = 26,
    .gpio_irq     = 23,
    .gpio_led_tx  = 4,
    .gpio_led_rx  = 17,
    .gpio_led_hb  = 27,
};

const board_pins_t BOARD_SPI1 = {
    .spidev_path  = "/dev/spidev1.0",
    .spi_speed_hz = 1000000,
    .spi_mode     = SPI_MODE_0,
    .spi_bits     = 8,
    .gpio_reset   = 24,
    .gpio_busy    = 7,
    .gpio_dio8    = 16,
    .gpio_irq     = 22,
    .gpio_led_tx  = 4,
    .gpio_led_rx  = 17,
    .gpio_led_hb  = 27,
};

// --------------------------------------------------------------- utilities --

void sleep_ms(unsigned ms)
{
    struct timespec ts = {
        .tv_sec  = ms / 1000,
        .tv_nsec = (long)(ms % 1000) * 1000000L,
    };
    nanosleep(&ts, NULL);
}

// --------------------------------------------------------------- GPIO helpers

static int gpio_request_output(struct gpiod_chip *chip, unsigned int offset,
                               int initial, gpio_line_t *line)
{
    struct gpiod_line_settings  *settings = NULL;
    struct gpiod_line_config    *line_cfg = NULL;
    struct gpiod_request_config *req_cfg  = NULL;

    memset(line, 0, sizeof(*line));
    line->offset = offset;

    settings = gpiod_line_settings_new();
    line_cfg  = gpiod_line_config_new();
    req_cfg   = gpiod_request_config_new();
    if (!settings || !line_cfg || !req_cfg) goto fail;

    if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_OUTPUT) < 0)
        goto fail;
    if (gpiod_line_settings_set_output_value(
            settings, initial ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE) < 0)
        goto fail;

    unsigned int off[1] = { offset };
    if (gpiod_line_config_add_line_settings(line_cfg, off, 1, settings) < 0) goto fail;
    gpiod_request_config_set_consumer(req_cfg, CONSUMER);

    line->req = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!line->req) goto fail;

    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    return 0;

fail:
    if (settings) gpiod_line_settings_free(settings);
    if (line_cfg)  gpiod_line_config_free(line_cfg);
    if (req_cfg)   gpiod_request_config_free(req_cfg);
    return -1;
}

static int gpio_request_input(struct gpiod_chip *chip, unsigned int offset,
                              gpio_line_t *line)
{
    struct gpiod_line_settings  *settings = NULL;
    struct gpiod_line_config    *line_cfg = NULL;
    struct gpiod_request_config *req_cfg  = NULL;

    memset(line, 0, sizeof(*line));
    line->offset = offset;

    settings = gpiod_line_settings_new();
    line_cfg  = gpiod_line_config_new();
    req_cfg   = gpiod_request_config_new();
    if (!settings || !line_cfg || !req_cfg) goto fail;

    if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT) < 0)
        goto fail;

    unsigned int off[1] = { offset };
    if (gpiod_line_config_add_line_settings(line_cfg, off, 1, settings) < 0) goto fail;
    gpiod_request_config_set_consumer(req_cfg, CONSUMER);

    line->req = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!line->req) goto fail;

    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    return 0;

fail:
    if (settings) gpiod_line_settings_free(settings);
    if (line_cfg)  gpiod_line_config_free(line_cfg);
    if (req_cfg)   gpiod_request_config_free(req_cfg);
    return -1;
}

static int gpio_request_irq_rising(struct gpiod_chip *chip, unsigned int offset,
                                   gpio_line_t *line)
{
    struct gpiod_line_settings  *settings = NULL;
    struct gpiod_line_config    *line_cfg = NULL;
    struct gpiod_request_config *req_cfg  = NULL;

    memset(line, 0, sizeof(*line));
    line->offset = offset;

    settings = gpiod_line_settings_new();
    line_cfg  = gpiod_line_config_new();
    req_cfg   = gpiod_request_config_new();
    if (!settings || !line_cfg || !req_cfg) goto fail;

    if (gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT) < 0)
        goto fail;
    if (gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING) < 0)
        goto fail;

    unsigned int off[1] = { offset };
    if (gpiod_line_config_add_line_settings(line_cfg, off, 1, settings) < 0) goto fail;
    gpiod_request_config_set_consumer(req_cfg, CONSUMER);

    line->req = gpiod_chip_request_lines(chip, req_cfg, line_cfg);
    if (!line->req) goto fail;

    gpiod_line_settings_free(settings);
    gpiod_line_config_free(line_cfg);
    gpiod_request_config_free(req_cfg);
    return 0;

fail:
    if (settings) gpiod_line_settings_free(settings);
    if (line_cfg)  gpiod_line_config_free(line_cfg);
    if (req_cfg)   gpiod_request_config_free(req_cfg);
    return -1;
}

int gpio_get(gpio_line_t *line)
{
    return gpiod_line_request_get_value(line->req, line->offset);
}

int gpio_set(gpio_line_t *line, int value)
{
    return gpiod_line_request_set_value(
        line->req, line->offset,
        value ? GPIOD_LINE_VALUE_ACTIVE : GPIOD_LINE_VALUE_INACTIVE);
}

void gpio_release(gpio_line_t *line)
{
    if (line->req) {
        gpiod_line_request_release(line->req);
        line->req = NULL;
    }
}

void pulse_led(gpio_line_t *line, unsigned ms)
{
    gpio_set(line, 1);
    sleep_ms(ms);
    gpio_set(line, 0);
}

// --------------------------------------------------------------- SPI --------

static int spi_open(lora_ctx_t *ctx)
{
    ctx->spi_fd = open(ctx->pins.spidev_path, O_RDWR);
    if (ctx->spi_fd < 0) {
        fprintf(stderr, "open(%s): %s\n", ctx->pins.spidev_path, strerror(errno));
        return -1;
    }

    if (ioctl(ctx->spi_fd, SPI_IOC_WR_MODE,          &ctx->pins.spi_mode)     < 0 ||
        ioctl(ctx->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &ctx->pins.spi_bits)     < 0 ||
        ioctl(ctx->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ,  &ctx->pins.spi_speed_hz) < 0)
    {
        fprintf(stderr, "SPI setup: %s\n", strerror(errno));
        close(ctx->spi_fd);
        ctx->spi_fd = -1;
        return -1;
    }
    return 0;
}

static int spi_transfer(int fd, const uint8_t *tx, uint8_t *rx, size_t len,
                        uint32_t speed_hz)
{
    struct spi_ioc_transfer tr = {
        .tx_buf        = (uintptr_t)tx,
        .rx_buf        = (uintptr_t)rx,
        .len           = (uint32_t)len,
        .speed_hz      = speed_hz,
        .bits_per_word = 8,
    };
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
        fprintf(stderr, "SPI transfer: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

// ------------------------------------------------------------- ctx lifecycle -

int ctx_init(lora_ctx_t *ctx, const board_pins_t *pins)
{
    memset(ctx, 0, sizeof(*ctx));
    ctx->spi_fd = -1;
    ctx->pins   = *pins;

    ctx->chip = gpiod_chip_open("/dev/gpiochip0");
    if (!ctx->chip) {
        fprintf(stderr, "gpiod_chip_open: %s\n", strerror(errno));
        return -1;
    }

    if (gpio_request_output(ctx->chip, pins->gpio_reset, 1, &ctx->reset) < 0) return -1;
    if (gpio_request_input (ctx->chip, pins->gpio_busy,     &ctx->busy)  < 0) return -1;
    if (gpio_request_input (ctx->chip, pins->gpio_dio8,     &ctx->dio8)  < 0) return -1;
    if (gpio_request_irq_rising(ctx->chip, pins->gpio_irq,  &ctx->irq)   < 0) return -1;
    if (gpio_request_output(ctx->chip, pins->gpio_led_tx, 0, &ctx->led_tx) < 0) return -1;
    if (gpio_request_output(ctx->chip, pins->gpio_led_rx, 0, &ctx->led_rx) < 0) return -1;
    if (gpio_request_output(ctx->chip, pins->gpio_led_hb, 0, &ctx->led_hb) < 0) return -1;

    if (spi_open(ctx) < 0) return -1;

    return 0;
}

void ctx_deinit(lora_ctx_t *ctx)
{
    if (ctx->spi_fd >= 0) { close(ctx->spi_fd); ctx->spi_fd = -1; }
    gpio_release(&ctx->reset);
    gpio_release(&ctx->busy);
    gpio_release(&ctx->dio8);
    gpio_release(&ctx->irq);
    gpio_release(&ctx->led_tx);
    gpio_release(&ctx->led_rx);
    gpio_release(&ctx->led_hb);
    if (ctx->chip) { gpiod_chip_close(ctx->chip); ctx->chip = NULL; }
}

// -------------------------------------------------------- wait helpers ------

int wait_busy_low(lora_ctx_t *ctx, unsigned timeout_ms)
{
    for (unsigned waited = 0; waited < timeout_ms; waited++) {
        int v = gpio_get(&ctx->busy);
        if (v < 0) { fprintf(stderr, "BUSY GPIO read failed\n"); return -1; }
        if (v == 0) return 0;
        sleep_ms(1);
    }
    fprintf(stderr, "Timeout waiting BUSY low\n");
    return -1;
}

int wait_irq_rising(lora_ctx_t *ctx, int timeout_ms)
{
    int64_t timeout_ns = (int64_t)timeout_ms * 1000000LL;
    int rc = gpiod_line_request_wait_edge_events(ctx->irq.req, timeout_ns);
    if (rc < 0) { fprintf(stderr, "IRQ wait failed\n"); return -1; }
    if (rc == 0) return 0; // timeout

    struct gpiod_edge_event_buffer *buf = gpiod_edge_event_buffer_new(4);
    if (!buf) { fprintf(stderr, "IRQ event buffer alloc failed\n"); return -1; }
    rc = gpiod_line_request_read_edge_events(ctx->irq.req, buf, 4);
    gpiod_edge_event_buffer_free(buf);
    if (rc < 0) { fprintf(stderr, "IRQ event read failed\n"); return -1; }

    return 1;
}

// ------------------------------------------------------- LR11xx HAL impl ---

lr11xx_hal_status_t lr11xx_hal_reset(const void *context)
{
    lora_ctx_t *ctx = (lora_ctx_t *)context;
    gpio_set(&ctx->reset, 0);
    sleep_ms(5);
    gpio_set(&ctx->reset, 1);
    sleep_ms(20);
    return wait_busy_low(ctx, 500) < 0 ? LR11XX_HAL_STATUS_ERROR : LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_wakeup(const void *context)
{
    lora_ctx_t *ctx = (lora_ctx_t *)context;
    uint8_t tx[1] = { 0x00 }, rx[1] = { 0x00 };
    if (spi_transfer(ctx->spi_fd, tx, rx, 1, ctx->pins.spi_speed_hz) < 0)
        return LR11XX_HAL_STATUS_ERROR;
    return wait_busy_low(ctx, 500) < 0 ? LR11XX_HAL_STATUS_ERROR : LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_abort_blocking_cmd(const void *context)
{
    (void)context;
    return LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_write(const void *context, const uint8_t *command,
                                     const uint16_t command_length, const uint8_t *data,
                                     const uint16_t data_length)
{
    lora_ctx_t *ctx   = (lora_ctx_t *)context;
    size_t      total = (size_t)command_length + (size_t)data_length;

    if (wait_busy_low(ctx, 500) < 0) return LR11XX_HAL_STATUS_ERROR;

    uint8_t *tx = calloc(total, 1);
    uint8_t *rx = calloc(total, 1);
    if (!tx || !rx) { free(tx); free(rx); return LR11XX_HAL_STATUS_ERROR; }

    memcpy(tx, command, command_length);
    if (data && data_length) memcpy(tx + command_length, data, data_length);

    int rc = spi_transfer(ctx->spi_fd, tx, rx, total, ctx->pins.spi_speed_hz);
    free(tx); free(rx);

    if (rc < 0) return LR11XX_HAL_STATUS_ERROR;
    return wait_busy_low(ctx, 500) < 0 ? LR11XX_HAL_STATUS_ERROR : LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_read(const void *context, const uint8_t *command,
                                    const uint16_t command_length, uint8_t *data,
                                    const uint16_t data_length)
{
    lora_ctx_t *ctx = (lora_ctx_t *)context;

    if (wait_busy_low(ctx, 500) < 0) return LR11XX_HAL_STATUS_ERROR;

    uint8_t *rx_dummy = calloc(command_length, 1);
    if (!rx_dummy) return LR11XX_HAL_STATUS_ERROR;
    int rc = spi_transfer(ctx->spi_fd, command, rx_dummy, command_length, ctx->pins.spi_speed_hz);
    free(rx_dummy);
    if (rc < 0) return LR11XX_HAL_STATUS_ERROR;

    if (wait_busy_low(ctx, 500) < 0) return LR11XX_HAL_STATUS_ERROR;

    size_t  total = (size_t)data_length + 1;
    uint8_t *tx   = calloc(total, 1);
    uint8_t *rx   = calloc(total, 1);
    if (!tx || !rx) { free(tx); free(rx); return LR11XX_HAL_STATUS_ERROR; }

    rc = spi_transfer(ctx->spi_fd, tx, rx, total, ctx->pins.spi_speed_hz);
    if (rc == 0) memcpy(data, rx + 1, data_length);
    free(tx); free(rx);

    return rc < 0 ? LR11XX_HAL_STATUS_ERROR : LR11XX_HAL_STATUS_OK;
}

lr11xx_hal_status_t lr11xx_hal_direct_read(const void *context, uint8_t *data,
                                           const uint16_t data_length)
{
    lora_ctx_t *ctx = (lora_ctx_t *)context;
    if (wait_busy_low(ctx, 500) < 0) return LR11XX_HAL_STATUS_ERROR;

    uint8_t *tx = calloc(data_length, 1);
    if (!tx) return LR11XX_HAL_STATUS_ERROR;
    int rc = spi_transfer(ctx->spi_fd, tx, data, data_length, ctx->pins.spi_speed_hz);
    free(tx);

    return rc < 0 ? LR11XX_HAL_STATUS_ERROR : LR11XX_HAL_STATUS_OK;
}
