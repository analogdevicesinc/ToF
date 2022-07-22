CONFIG?=debug

# For generate_crypto_packets.py
export CONFIG

ifneq ($(CONFIG), debug)
ifneq ($(CONFIG), debug-fpga)
ifneq ($(CONFIG), release)
$(error Expected command: make CONFIG=<debug,debug-fpga,release> [target])
endif
endif
endif

include riscv_config.mk
include makeutils.mk

HSPAPILIB = libhspapi.$(ARCH).$(CONFIG).a

RETAIL_1SP_C = retail1sp.c startup.c common_1sp.c apbmailbox_1sp.c postcode_1sp.c

HSPLIB_INC_DIR = ../hspapi/inc
HSPLIB_BIN_DIR = ../hspapi/lib

BUILD_DIR = ../build/adi_retail1sp/$(CONFIG)
OUT_DIR = ../out/adi_retail1sp/$(CONFIG)

HSPAPILIB = $(HSPLIB_BIN_DIR)/libhspapi.$(ARCH).$(CONFIG).a

RETAIL_1SP = riscv-hspfw-analog-retail1sp-$(CONFIG)

RETAIL_1SP_INC := -I$(HSPLIB_INC_DIR) -I$(HSPLIB_INC_DIR)/utils/uart

RETAIL_1SP_OBJS=$(addprefix $(BUILD_DIR)/, $(RETAIL_1SP_C:.c=.o))

# Set the arch, ABI, and code model
COMMON_CFLAGS := $(RETAIL_1SP_INC) 
COMMON_CFLAGS += -Os
COMMON_CFLAGS += -Wall -Wstrict-prototypes -Wc++-compat -Wconversion -Wno-sign-conversion -Wno-unused-function
COMMON_CFLAGS += -ffunction-sections -fdata-sections

ifeq ($(CONFIG), debug)
COMMON_CFLAGS += -g -DDEBUG=1
endif

ifeq ($(CONFIG), debug-fpga)
COMMON_CFLAGS += -g -DDEBUG=1 -DBUILD_FOR_FPGA=1
endif

RISCV_CCASFLAGS += $(COMMON_CFLAGS) $(RISCV_COMMON_FLAGS)
RISCV_CFLAGS    += $(COMMON_CFLAGS) $(RISCV_COMMON_FLAGS)

RISCV_LDFLAGS += -Wl,--gc-sections -nostartfiles -nostdlib
RISCV_LDLIBS += -L. -l:$(HSPAPILIB)
RISCV_LDLIBS += -Wl,--start-group -lc -lgcc -Wl,--end-group

RETAIL_1SP_LINKERSCRIPT := retail_1sp.lds
RETAIL_1SP_ELF 	:= $(OUT_DIR)/$(RETAIL_1SP).elf
RETAIL_1SP_LST 	:= $(OUT_DIR)/$(RETAIL_1SP).lst
RETAIL_1SP_BIN	:= $(OUT_DIR)/$(RETAIL_1SP).bin
# Unsigned Newton cmd text file
RETAIL_1SP_NEWTON_CMD_FILE := $(OUT_DIR)/$(RETAIL_1SP)_cmd_file.txt
RETAIL_1SP_NEWTON_CMD_FILE_TEST_SIGNED := $(OUT_DIR)/$(RETAIL_1SP)_test_signed_cmd_file.txt
RETAIL_1SP_NEWTON_CMD_FILE_NOT_ENC_SIGNED := $(OUT_DIR)/$(RETAIL_1SP)_not_encrypted_signed_cmd_file.txt

1SP_INPUT_FILES_DIR := ../adi_1sp_input_files

GEN_CMD_1SP_FILE := python ../tools/gen_cmd_1sp_file.py

.PHONY: all retail_1sp
all: retail_1sp

retail_1sp: $(RETAIL_1SP_ELF) $(RETAIL_1SP_NEWTON_CMD_FILE) $(RETAIL_1SP_NEWTON_CMD_FILE_TEST_SIGNED) $(RETAIL_1SP_NEWTON_CMD_FILE_NOT_ENC_SIGNED)

$(BUILD_DIR)/%.o: %.c
	@echo "Compile: $(notdir $<)"
	$(call makedir,$(dir $@))
	$(RISCV_GCC) $(RISCV_CFLAGS) -c -o $@ $<

$(RETAIL_1SP_ELF): $(RETAIL_1SP_OBJS) $(RETAIL_1SP_LINKERSCRIPT)
	@echo "Linking $(RETAIL_1SP)"
	$(call makedir,$(dir $@))
	$(RISCV_GCC) $(RISCV_CFLAGS) $(RISCV_LDFLAGS) -T$(RETAIL_1SP_LINKERSCRIPT) \
		-Wl,-Map,$(dir $@)$(RETAIL_1SP).map $(RETAIL_1SP_OBJS) $(RISCV_LDLIBS) -o $@
	$(RISCV_OBJDUMP) --source --all-headers --demangle --line-numbers --wide $@ > $(RETAIL_1SP_LST)
	$(RISCV_SIZE) $@

$(RETAIL_1SP_BIN): $(RETAIL_1SP_ELF)
	$(RISCV_OBJCOPY) -O binary --gap-fill 0xff $(RETAIL_1SP_ELF) $@

$(RETAIL_1SP_NEWTON_CMD_FILE): $(RETAIL_1SP_BIN)
	$(GEN_CMD_1SP_FILE) $(RETAIL_1SP_BIN) $(RETAIL_1SP_NEWTON_CMD_FILE)

$(RETAIL_1SP_NEWTON_CMD_FILE_TEST_SIGNED): $(RETAIL_1SP_BIN)
	python ../tools/generic_1sp_encryption.py $(RETAIL_1SP_BIN) $(1SP_INPUT_FILES_DIR)/release_svn.txt $(1SP_INPUT_FILES_DIR)/encryptionkey.txt $(1SP_INPUT_FILES_DIR)/adi_encryption_kdf_value_zero.txt $(1SP_INPUT_FILES_DIR)/initial_vector.bin $(OUT_DIR)/retail_adi_test_encrypted.bin
	python ../tools/gen_txt_1sp.py $(OUT_DIR)/retail_adi_test_encrypted.bin $(OUT_DIR)/retail_adi_test_encrypted.txt
	python ../tools/memory_file_splitter.py $(OUT_DIR)/retail_adi_test_encrypted.txt IMAGE_1SP 2048
	python ../tools/generate_crypto_packets.py $(1SP_INPUT_FILES_DIR)/adi_signed_group_packet_correct_order_with_retail_1sp_test_key_enc.xml $(OUT_DIR)/adi_signed_group_packet_correct_order_with_retail_1sp_test_key_enc.txt

$(RETAIL_1SP_NEWTON_CMD_FILE_NOT_ENC_SIGNED): $(RETAIL_1SP_BIN)
	python ../tools/gen_txt_1sp.py $(RETAIL_1SP_BIN) $(OUT_DIR)/retail_adi_not_encrypted.txt
	python ../tools/memory_file_splitter.py $(OUT_DIR)/retail_adi_not_encrypted.txt IMAGE_1SP 2048
	python ../tools/generate_crypto_packets.py $(1SP_INPUT_FILES_DIR)/adi_signed_group_packet_correct_order_with_retail_1sp_not_enc.xml $(OUT_DIR)/adi_signed_group_packet_correct_order_with_retail_1sp_not_enc.txt

.PHONY: clean
clean:
	@-$(REMOVE_FOLDER) "$(BUILD_DIR)"
	@-$(REMOVE_FOLDER) "$(OUT_DIR)"
