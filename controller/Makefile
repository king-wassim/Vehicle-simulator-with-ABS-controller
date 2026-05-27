# Minimal Makefile for week-1 milestone — POSIX, tested under WSL Ubuntu
# and MSYS2 UCRT64. CMake will replace this once we add unit tests in week 2.

CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Wpedantic -O2 -g
LDFLAGS ?=
INC     := -Isrc/drivers -Isrc/hal -Isrc/app -Isrc/utils

BUILD   := build
SRC_DRV := src/drivers/crc.c src/drivers/socket_drv.c

.PHONY: all clean test_crc ping_client

all: test_crc ping_client

$(BUILD):
	mkdir -p $(BUILD)

test_crc: $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_crc.c src/drivers/crc.c \
	    -o $(BUILD)/test_crc $(LDFLAGS)

ping_client: $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/ping_client.c $(SRC_DRV) \
	    -o $(BUILD)/ping_client $(LDFLAGS)

clean:
	rm -rf $(BUILD)
