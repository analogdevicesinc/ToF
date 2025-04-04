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

BUNDLE_ARCHIVE=$(BUNDLE)

.PHONY: build run clean package

build:
	@echo "🔨 Building $(JETPACKVERSION)"
	cargo build --release
	@cp $(INSTALLER) $(OUTPUT_BIN)

run: build
	@$(OUTPUT_BIN)

package: build
	@echo "📦 Bundling archive into $(OUTPUT_BIN)"
	@printf 'ADCAM_TGZ_START' >> $(OUTPUT_BIN)
	@echo "$(MARKER)" >> $(OUTPUT_BIN)
	@cat $(BUNDLE_ARCHIVE) >> $(OUTPUT_BIN)
	@chmod +x $(OUTPUT_BIN)

	@echo "🔐 Computing SHA256..."
	@sha256sum $(OUTPUT_BIN) | cut -d' ' -f1 > $(HASH_FILE)

	@echo "📝 Generating JSON: $(JSON_FILE)"
	@HASH=$$(cat $(HASH_FILE)); \
	echo "{"                          > $(JSON_FILE); \
	echo "  \"version\": \"$(VERSION)\"," >> $(JSON_FILE); \
	echo "  \"filename\": \"$(APP_NAME)\"," >> $(JSON_FILE); \
	echo "  \"sa256hash\": \"$$HASH\""    >> $(JSON_FILE); \
	echo "}"                          >> $(JSON_FILE)

	@echo "BuiltXYZ: $(OUTPUT_BIN)"

clean:
	@cargo clean
	@rm -f $(OUTPUT_BIN) $(HASH_FILE) $(JSON_FILE)
