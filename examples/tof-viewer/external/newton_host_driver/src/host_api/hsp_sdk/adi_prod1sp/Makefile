CONFIG?=debug

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

PROD_1SP_C = prod1sp_adi.c startup.c common_1sp.c apbmailbox_1sp.c postcode_1sp.c

HSPLIB_INC_DIR = ../hspapi/inc
HSPLIB_BIN_DIR = ../hspapi/lib

BUILD_DIR = ../build/adi_prod1sp/$(CONFIG)
OUT_DIR = ../out/adi_prod1sp/$(CONFIG)

HSPAPILIB = $(HSPLIB_BIN_DIR)/libhspapi.$(ARCH).$(CONFIG).a

PROD_1SP = riscv-hspfw-analog-prod1sp-$(CONFIG)

PROD_1SP_INC :=	-I$(HSPLIB_INC_DIR) -I$(HSPLIB_INC_DIR)/utils/uart

PROD_1SP_OBJS=$(addprefix $(BUILD_DIR)/, $(PROD_1SP_C:.c=.o))

# Set the arch, ABI, and code model
COMMON_CFLAGS := $(PROD_1SP_INC) 
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

PROD_1SP_LINKERSCRIPT := prod_1sp.lds
PROD_1SP_ELF 	:= $(OUT_DIR)/$(PROD_1SP).elf
PROD_1SP_LST 	:= $(OUT_DIR)/$(PROD_1SP).lst
PROD_1SP_BIN	:= $(OUT_DIR)/$(PROD_1SP).bin
PROD_1SP_BIN_FUSEDATA	:= $(OUT_DIR)/$(PROD_1SP)-fusedata.bin

PROD_1SP_NEWTON_CMD_FILE := $(OUT_DIR)/$(PROD_1SP)_cmd_file.txt
PROD_1SP_NEWTON_CMD_FILE_ZERO_SIGNED := $(OUT_DIR)/$(PROD_1SP)_zero_encrypted_signed_cmd_file.txt
PROD_1SP_NEWTON_CMD_FILE_TEST_SIGNED := $(OUT_DIR)/$(PROD_1SP)_test_encrypted_signed_cmd_file.txt
PROD_1SP_NEWTON_CMD_FILE_NOT_ENC_SIGNED := $(OUT_DIR)/$(PROD_1SP)_not_encrypted_signed_cmd_file.txt

1SP_INPUT_FILES_DIR := ../adi_1sp_input_files

GEN_CMD_1SP_FILE := python ../tools/gen_cmd_1sp_file.py

.PHONY: all prod_1sp
all: prod_1sp

prod_1sp: $(PROD_1SP_ELF) $(PROD_1SP_NEWTON_CMD_FILE) $(PROD_1SP_NEWTON_CMD_FILE_ZERO_SIGNED) $(PROD_1SP_NEWTON_CMD_FILE_TEST_SIGNED) $(PROD_1SP_NEWTON_CMD_FILE_NOT_ENC_SIGNED)

$(BUILD_DIR)/%.o: %.c
	@echo "Operating system is " $(OS)
	$(call makedir,$(dir $@))
	$(RISCV_GCC) $(RISCV_CFLAGS) -c -o $@ $<

$(PROD_1SP_ELF): $(PROD_1SP_OBJS) $(PROD_1SP_LINKERSCRIPT)
	@echo "Linking $(PROD_1SP)"
	$(call makedir,$(dir $@))
	$(RISCV_GCC) $(RISCV_CFLAGS) $(RISCV_LDFLAGS) -T$(PROD_1SP_LINKERSCRIPT) \
		-Wl,-Map,$(dir $@)$(PROD_1SP).map $(PROD_1SP_OBJS) $(RISCV_LDLIBS) -o $@
	$(RISCV_OBJDUMP) --source --all-headers --demangle --line-numbers --wide $@ > $(PROD_1SP_LST)
	$(RISCV_SIZE) $@

$(PROD_1SP_BIN): $(PROD_1SP_ELF)
	$(RISCV_OBJCOPY) -O binary --gap-fill 0xff $(PROD_1SP_ELF) $@

FUSE_DIR=fuse_files
$(PROD_1SP_NEWTON_CMD_FILE) $(PROD_1SP_BIN_FUSEDATA) &: $(PROD_1SP_BIN)
	$(GEN_CMD_1SP_FILE) $(PROD_1SP_BIN) $(PROD_1SP_NEWTON_CMD_FILE) \
		--production-fuses $(FUSE_DIR)/svn.txt $(FUSE_DIR)/publichash.txt $(FUSE_DIR)/encryptionkey.txt $(FUSE_DIR)/globalprivatekey.txt $(FUSE_DIR)/blockedoperationmodes.txt

$(PROD_1SP_NEWTON_CMD_FILE_ZERO_SIGNED): $(PROD_1SP_BIN_FUSEDATA)
	python ../tools/generic_1sp_encryption.py $(PROD_1SP_BIN_FUSEDATA) $(1SP_INPUT_FILES_DIR)/release_svn.txt $(1SP_INPUT_FILES_DIR)/zeroencryptionkey.txt $(1SP_INPUT_FILES_DIR)/adi_encryption_kdf_value_zero.txt $(1SP_INPUT_FILES_DIR)/initial_vector.bin $(OUT_DIR)/prod1sp_adi_zero_encrypted.bin
	python ../tools/gen_txt_1sp.py $(OUT_DIR)/prod1sp_adi_zero_encrypted.bin $(OUT_DIR)/prod1sp_adi_zero_encrypted.txt
	python ../tools/memory_file_splitter.py $(OUT_DIR)/prod1sp_adi_zero_encrypted.txt IMAGE_1SP 2048
	python ../tools/generate_crypto_packets.py $(1SP_INPUT_FILES_DIR)/adi_signed_group_prod_1sp_zero_key_enc.xml $(OUT_DIR)/adi_signed_group_prod_1sp_zero_key_enc.txt

$(PROD_1SP_NEWTON_CMD_FILE_TEST_SIGNED): $(PROD_1SP_BIN_FUSEDATA)
	python ../tools/generic_1sp_encryption.py $(PROD_1SP_BIN_FUSEDATA) $(1SP_INPUT_FILES_DIR)/release_svn.txt $(1SP_INPUT_FILES_DIR)/encryptionkey.txt $(1SP_INPUT_FILES_DIR)/adi_encryption_kdf_value_zero.txt $(1SP_INPUT_FILES_DIR)/initial_vector.bin $(OUT_DIR)/prod1sp_adi_test_encrypted.bin
	python ../tools/gen_txt_1sp.py $(OUT_DIR)/prod1sp_adi_test_encrypted.bin $(OUT_DIR)/prod1sp_adi_test_encrypted.txt
	python ../tools/memory_file_splitter.py $(OUT_DIR)/prod1sp_adi_test_encrypted.txt IMAGE_1SP 2048
	python ../tools/generate_crypto_packets.py $(1SP_INPUT_FILES_DIR)/adi_signed_group_prod_1sp_test_key_enc.xml $(OUT_DIR)/adi_signed_group_prod_1sp_test_key_enc.txt

$(PROD_1SP_NEWTON_CMD_FILE_NOT_ENC_SIGNED): $(PROD_1SP_BIN_FUSEDATA)
	python ../tools/gen_txt_1sp.py $(PROD_1SP_BIN_FUSEDATA) $(OUT_DIR)/prod1sp_adi_not_encrypted.txt
	python ../tools/memory_file_splitter.py $(OUT_DIR)/prod1sp_adi_not_encrypted.txt IMAGE_1SP 2048
	python ../tools/generate_crypto_packets.py $(1SP_INPUT_FILES_DIR)/adi_signed_group_prod_1sp_not_encrypted.xml $(OUT_DIR)/adi_signed_group_prod_1sp_not_encrypted.txt

.PHONY: clean
clean:
	@-$(REMOVE_FOLDER) "$(BUILD_DIR)"
	@-$(REMOVE_FOLDER) "$(OUT_DIR)"
