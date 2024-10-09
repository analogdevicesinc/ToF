obj-m += src/adsd3500.o
CURRENT_DIR := $(shell pwd)
BUILD_DIR:= $(abspath /usr/src/linux-imx/) # Path to the linux-imx kernel source
OUTPUT_DIR := $(CURRENT_DIR)/out

all: clean adsd3500

adsd3500:
	make -C $(BUILD_DIR) M=${PWD} modules
	mkdir -p $(OUTPUT_DIR)
	mv src/.*.ko.cmd \
	 	src/.*.mod.cmd \
	 	src/.*.o.cmd \
	 	src/*.ko \
	 	src/*.o \
	 	src/*.mod \
	 	src/*.mod.c \
	 	$(OUTPUT_DIR)
	mv 	$(CURRENT_DIR)/Module.symvers \
	 	$(CURRENT_DIR)/modules.order \
		$(CURRENT_DIR)/.Module.symvers.cmd \
	 	$(CURRENT_DIR)/.modules.order.cmd \
		$(OUTPUT_DIR)

clean:
	make -C $(BUILD_DIR) M=${PWD} clean
	rm -rf $(OUTPUT_DIR)
