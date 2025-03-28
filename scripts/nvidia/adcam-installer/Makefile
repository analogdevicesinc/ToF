PACKAGE_NAME=$(shell grep '^name =' Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
VERSION=$(shell grep '^version =' Cargo.toml | sed 's/.*"\(.*\)".*/\1/')
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

package:
	cp $(INSTALLER) $(OUTPUT_BIN)
	@echo "Built: $(OUTPUT_BIN)"
	shasum -a 256 $(OUTPUT_BIN) > $(HASH_FILE)
	@echo "SHA256 saved to: $(HASH_FILE)"
	@HASH=$$(cut -d ' ' -f1 $(HASH_FILE)); \
	echo '{' > $(JSON_FILE); \
	echo '  "version": "$(VERSION)",' >> $(JSON_FILE); \
	echo '  "filename": "$(APP_NAME)",' >> $(JSON_FILE); \
	echo "  \"sa256hash\": \"$$HASH\"" >> $(JSON_FILE); \
	echo '}' >> $(JSON_FILE)
	@echo "JSON saved to: $(JSON_FILE)"

run: build
	$(INSTALLER)

clean:
	cargo clean
