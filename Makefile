# Makefile — lora-cli
#
# LR11xx driver sources (Semtech SWDR001, BSD-licensed) are vendored in src/.
#
# Dependencies (Raspberry Pi OS):
#   sudo apt install gcc libgpiod-dev pkg-config

TARGET  := lora-cli
CC      := gcc
CFLAGS  := -O2 -Wall -Wextra -std=c11 -DLR11XX_DISABLE_WARNINGS
CFLAGS  += $(shell pkg-config --cflags libgpiod)
LDFLAGS := $(shell pkg-config --libs libgpiod)

INC := -I./src -I./cli

# Semtech LR11xx driver sources (from SWDR001)
DRIVER_SRCS := \
	src/lr11xx_system.c \
	src/lr11xx_radio.c \
	src/lr11xx_regmem.c \
	src/lr11xx_bootloader.c \
	src/lr11xx_driver_version.c

# CLI application sources
CLI_SRCS := \
	cli/board.c \
	cli/radio.c \
	cli/mode_status.c \
	cli/mode_rx.c \
	cli/mode_tx.c \
	cli/mode_scan.c \
	cli/mode_txtest.c \
	cli/mode_mavlink.c \
	cli/main.c

ALL_SRCS := $(DRIVER_SRCS) $(CLI_SRCS)
OBJS     := $(ALL_SRCS:.c=.o)

# ----------------------------------------------------------------- targets --

.PHONY: all clean install uninstall

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) $(INC) -c -o $@ $<

clean:
	rm -f $(OBJS) $(TARGET)

install: $(TARGET)
	install -m 755 $(TARGET) /usr/local/bin/$(TARGET)

uninstall:
	rm -f /usr/local/bin/$(TARGET)

# ---------------------------------------------------------------- cross build

# Cross-compile for Raspberry Pi from x86 host:
#   make CROSS=1
ifeq ($(CROSS),1)
CC       := arm-linux-gnueabihf-gcc
CFLAGS   += --sysroot=$(SYSROOT)
LDFLAGS  += --sysroot=$(SYSROOT)
INC      += -I$(SYSROOT)/usr/include
endif
