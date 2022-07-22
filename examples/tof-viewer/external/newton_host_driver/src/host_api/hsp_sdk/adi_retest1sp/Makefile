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

RETEST_1SP_C = release1spToRetest.c startup.c common_1sp.c apbmailbox_1sp.c postcode_1sp.c

HSPLIB_INC_DIR = ../hspapi/inc
HSPLIB_BIN_DIR = ../hspapi/lib

BUILD_DIR = ../build/adi_retest1sp/$(CONFIG)
OUT_DIR = ../out/adi_retest1sp/$(CONFIG)

HSPAPILIB = $(HSPLIB_BIN_DIR)/libhspapi.$(ARCH).$(CONFIG).a

RETEST_1SP = riscv-hspfw-analog-retest1sp-$(CONFIG)

RETEST_1SP_INC := -I$(HSPLIB_INC_DIR) -I$(HSPLIB_INC_DIR)/utils/uart

RETEST_1SP_OBJS=$(addprefix $(BUILD_DIR)/, $(RETEST_1SP_C:.c=.o))

# Set the arch, ABI, and code model
COMMON_CFLAGS := $(RETEST_1SP_INC) 
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

# Share same lds for RETAIL 1SP
RETEST_1SP_LINKERSCRIPT := retail_1sp.lds
RETEST_1SP_ELF 	:= $(OUT_DIR)/$(RETEST_1SP).elf
RETEST_1SP_LST 	:= $(OUT_DIR)/$(RETEST_1SP).lst
RETEST_1SP_BIN	:= $(OUT_DIR)/$(RETEST_1SP).bin
# Unsigned Newton cmd text file
RETEST_1SP_NEWTON_CMD_FILE := $(OUT_DIR)/$(RETEST_1SP)_cmd_file.txt
RETEST_1SP_NEWTON_CMD_FILE_TEST_SIGNED := $(OUT_DIR)/$(RETEST_1SP)_test_signed_cmd_file.txt

1SP_INPUT_FILES_DIR := ../adi_1sp_input_files

GEN_CMD_1SP_FILE := python ../tools/gen_cmd_1sp_file.py

.PHONY: all retest_1sp
all: retest_1sp

retest_1sp: $(RETEST_1SP_ELF) $(RETEST_1SP_NEWTON_CMD_FILE) $(RETEST_1SP_NEWTON_CMD_FILE_TEST_SIGNED)

$(BUILD_DIR)/%.o: %.c
	@echo "Compile: $(notdir $<)"
	$(call makedir,$(dir $@))
	$(RISCV_GCC) $(RISCV_CFLAGS) -c -o $@ $<

$(RETEST_1SP_ELF): $(RETEST_1SP_OBJS) $(RETEST_1SP_LINKERSCRIPT)
	@echo "Linking $(RETEST_1SP)"
	$(call makedir,$(dir $@))
	$(RISCV_GCC) $(RISCV_CFLAGS) $(RISCV_LDFLAGS) -T$(RETEST_1SP_LINKERSCRIPT) \
		-Wl,-Map,$(dir $@)$(RETEST_1SP).map $(RETEST_1SP_OBJS) $(RISCV_LDLIBS) -o $@
	$(RISCV_OBJDUMP) --source --all-headers --demangle --line-numbers --wide $@ > $(RETEST_1SP_LST)
	$(RISCV_SIZE) $@

$(RETEST_1SP_BIN): $(RETEST_1SP_ELF)
	$(RISCV_OBJCOPY) -O binary --gap-fill 0xff $(RETEST_1SP_ELF) $@

$(RETEST_1SP_NEWTON_CMD_FILE): $(RETEST_1SP_BIN)
	$(GEN_CMD_1SP_FILE) $(RETEST_1SP_BIN) $(RETEST_1SP_NEWTON_CMD_FILE)

$(RETEST_1SP_NEWTON_CMD_FILE_TEST_SIGNED): $(RETEST_1SP_BIN)
	python ../tools/generic_1sp_encryption.py $(RETEST_1SP_BIN) $(1SP_INPUT_FILES_DIR)/release_svn.txt $(1SP_INPUT_FILES_DIR)/encryptionkey.txt $(1SP_INPUT_FILES_DIR)/adi_encryption_kdf_value_zero.txt $(1SP_INPUT_FILES_DIR)/initial_vector.bin $(OUT_DIR)/retest_adi_test_encrypted.bin
	python ../tools/gen_txt_1sp.py $(OUT_DIR)/retest_adi_test_encrypted.bin $(OUT_DIR)/retest_adi_test_encrypted.txt
	python ../tools/memory_file_splitter.py $(OUT_DIR)/retest_adi_test_encrypted.txt IMAGE_1SP 2048
	python ../tools/generate_crypto_packets.py $(1SP_INPUT_FILES_DIR)/adi_signed_group_prod_1sp_retest_test_key_enc.xml $(OUT_DIR)/adi_signed_group_prod_1sp_retest_test_key_enc.txt

.PHONY: clean
clean:
	@-$(REMOVE_FOLDER) "$(BUILD_DIR)"
	@-$(REMOVE_FOLDER) "$(OUT_DIR)"
