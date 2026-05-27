# Build the ABS SIL controller. POSIX-only (Linux / WSL Ubuntu).

CC      ?= gcc
CFLAGS  ?= -std=c11 -Wall -Wextra -Wpedantic -O2 -g
LDFLAGS ?=
LDLIBS  := -lm
INC     := -Isrc/drivers -Isrc/hal -Isrc/app -Isrc/utils

BUILD   := build

SRC_DRV := src/drivers/crc.c src/drivers/socket_drv.c
SRC_HAL := src/hal/hal_sensors.c src/hal/hal_actuators.c src/hal/hal_link.c
SRC_APP := src/app/abs_state.c src/app/abs_controller.c src/app/diagnostic.c
SRC_ALL := $(SRC_DRV) $(SRC_HAL) $(SRC_APP) src/main.c

.PHONY: all clean test tests ping_client abs_ecu test_crc test_abs_state test_abs_controller test_diagnostic

all: abs_ecu test_crc test_abs_state test_abs_controller test_diagnostic ping_client

$(BUILD):
	mkdir -p $(BUILD)

abs_ecu: $(BUILD)
	$(CC) $(CFLAGS) $(INC) $(SRC_ALL) -o $(BUILD)/abs_ecu $(LDFLAGS) $(LDLIBS)

test_crc: $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_crc.c src/drivers/crc.c \
	    -o $(BUILD)/test_crc $(LDFLAGS)

test_abs_state: $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_abs_state.c src/app/abs_state.c \
	    -o $(BUILD)/test_abs_state $(LDFLAGS)

test_abs_controller: $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_abs_controller.c \
	    src/app/abs_controller.c src/app/abs_state.c \
	    -o $(BUILD)/test_abs_controller $(LDFLAGS) $(LDLIBS)

test_diagnostic: $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/test_diagnostic.c src/app/diagnostic.c \
	    -o $(BUILD)/test_diagnostic $(LDFLAGS) $(LDLIBS)

ping_client: $(BUILD)
	$(CC) $(CFLAGS) $(INC) tests/ping_client.c $(SRC_DRV) \
	    -o $(BUILD)/ping_client $(LDFLAGS)

# Run every unit test binary built above. Fails fast on the first non-zero.
test: test_crc test_abs_state test_abs_controller test_diagnostic
	@echo "=== test_crc ===" &&            $(BUILD)/test_crc
	@echo "=== test_abs_state ===" &&      $(BUILD)/test_abs_state
	@echo "=== test_abs_controller ===" && $(BUILD)/test_abs_controller
	@echo "=== test_diagnostic ===" &&     $(BUILD)/test_diagnostic

tests: test

clean:
	rm -rf $(BUILD)
