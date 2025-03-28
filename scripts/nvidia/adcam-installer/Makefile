PACKAGE_NAME=$(shell grep '^name =' Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
VERSION=$(shell jq -r '.version' ../config.json)
JETPACKVERSION ?=
JETPACKTAG := $(JETPACKVERSION)
ifneq ($(strip $(JETPACKVERSION)),)
JETPACKTAG := $(JETPACKVERSION)-
endif
APP_NAME=$(PACKAGE_NAME)-$(JETPACKTAG)$(VERSION).bin
BUILD_DIR=target/release
INSTALLER=$(BUILD_DIR)/$(PACKAGE_NAME)
OUTPUT_BIN=$(BUILD_DIR)/$(APP_NAME)
HASH_FILE=$(OUTPUT_BIN).sha256
JSON_FILE=$(OUTPUT_BIN:.bin=.json)

.PHONY: all build run clean package

all: build package

build:
	echo "Building $(JETPACKVERSION)"
	cargo build --release -j 1
	mv $(INSTALLER) $(OUTPUT_BIN)
	@echo "BuiltXYZ: $(OUTPUT_BIN)"

run: build
	$(INSTALLER)

clean:
	cargo clean
