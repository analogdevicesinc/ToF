# Project config
BINARY_NAME := adcam
TARGET_DIR := target/release

# Default target: build the release binary
all: build

build:
	cargo build --release

run:
	$(TARGET_DIR)/$(BINARY_NAME)

clean:
	cargo clean

install:
	cp $(TARGET_DIR)/$(BINARY_NAME) /usr/local/bin/$(BINARY_NAME)

uninstall:
	rm -f /usr/local/bin/$(BINARY_NAME)
