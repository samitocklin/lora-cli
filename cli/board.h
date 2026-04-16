// board.h — hardware context, GPIO/SPI types, platform declarations
#pragma once

#define _GNU_SOURCE

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <gpiod.h>

#define CONSUMER "lora-cli"

extern volatile sig_atomic_t g_stop;

// ------------------------------------------------------------------ types ---

typedef struct {
    const char  *spidev_path;
    uint32_t     spi_speed_hz;
    uint8_t      spi_mode;
    uint8_t      spi_bits;
    unsigned int gpio_reset;
    unsigned int gpio_busy;
    unsigned int gpio_dio8;
    unsigned int gpio_irq;
    unsigned int gpio_led_tx;
    unsigned int gpio_led_rx;
    unsigned int gpio_led_hb;
} board_pins_t;

typedef struct {
    unsigned int               offset;
    struct gpiod_line_request *req;
} gpio_line_t;

typedef struct {
    int                spi_fd;
    struct gpiod_chip *chip;
    gpio_line_t        reset;
    gpio_line_t        busy;
    gpio_line_t        dio8;
    gpio_line_t        irq;
    gpio_line_t        led_tx;
    gpio_line_t        led_rx;
    gpio_line_t        led_hb;
    board_pins_t       pins;
} lora_ctx_t;

// --------------------------------------------------------------- boards ----

extern const board_pins_t BOARD_SPI0;
extern const board_pins_t BOARD_SPI1;

// ---------------------------------------------------------- public API -----

void sleep_ms(unsigned ms);

int  ctx_init(lora_ctx_t *ctx, const board_pins_t *pins);
void ctx_deinit(lora_ctx_t *ctx);

int  wait_busy_low(lora_ctx_t *ctx, unsigned timeout_ms);
int  wait_irq_rising(lora_ctx_t *ctx, int timeout_ms);

int  gpio_get(gpio_line_t *line);
int  gpio_set(gpio_line_t *line, int value);
void gpio_release(gpio_line_t *line);
void pulse_led(gpio_line_t *line, unsigned ms);
