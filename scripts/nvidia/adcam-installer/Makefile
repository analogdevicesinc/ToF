APP_NAME=adcam-installer
BUILD_DIR=target/release
INSTALLER=$(BUILD_DIR)/$(APP_NAME)

.PHONY: all build run clean

all: build

build:
	cargo build --release

run: build
	$(INSTALLER)

clean:
	cargo clean
