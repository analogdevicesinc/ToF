###########################################################
## Makefile: Makefile
###########################################################

WIRINGPI  := 1
DEBUG     := 0
FPGA      := 0
OLIMEX    := 1

DEPENDS_INSTALL_DIR := $(PRJ_USER)/sw

PROJECT_NAME := $(PRJ_NAME)

CC = g++
GCC = gcc
ObjectDir = linux

ifeq ($(WIRINGPI), 1)
    DEPENDS_INSTALL_DIR := /home/$(USER)
    WIRINGPI_HOME = $(DEPENDS_INSTALL_DIR)/wiringPi
endif

ifeq ($(PROJECT_NAME), newton)
    PYTHON_INCLUDE_DIR := /cad/adi/apps/gnu/linux/x86_64/6/local/python/3.7.3/include/python3.7m
else
    PYTHON_INCLUDE_DIR := /usr/include/python2.7
endif

ifeq ($(DEBUG), 1)
    CFLAGS += -DENABLE_DEBUG_LOW
endif

ifeq ($(DEBUG), 2)
    CFLAGS += -DENABLE_DEBUG_LOW
    CFLAGS += -DENABLE_DEBUG_MEDIUM
endif

ifeq ($(DEBUG), 3)
    CFLAGS += -DENABLE_DEBUG_LOW
    CFLAGS += -DENABLE_DEBUG_MEDIUM
    CFLAGS += -DENABLE_DEBUG_HIGH
endif

ifeq ($(FPGA), 1)
    CFLAGS += -DFPGA
endif

ifeq ($(WIRINGPI), 1)
    CFLAGS += -DWIRINGPI
endif

ifeq ($(OLIMEX), 1)
    CFLAGS += -DOLIMEX
endif

CFLAGS += -DPYTHON
CFLAGS += -fPIC

SOURCE_FILES := ../../src/newton_control.c
SOURCE_FILES := ./newton_ctrl_swig_wrap.c

OBJ_FILES := $(ObjectDir)/newton_control.o
OBJ_FILES += $(ObjectDir)/newton_ctrl_swig_wrap.o

INCDIR  := -I.
INCDIR  += -I../../common
INCDIR  += -I../../include
INCDIR  += -I$(PYTHON_INCLUDE_DIR)

ifeq ($(WIRINGPI), 1)
    INCDIR  += -I$(WIRINGPI_HOME)/wiringPi
endif

ifeq ($(WIRINGPI), 1)
    LIBRARIES += /usr/lib/libwiringPi.so
endif

SO_FILE  := _newton_control.so

.PHONY: default
default: main

help:
	@echo ""
	@echo "     ################################################################"
	@echo "     #"
	@echo "     # make help            -- prints this message"
	@echo "     # make                 -- builds python module"
	@echo "     # make clean           -- cleans generated files"
	@echo "     #"
	@echo "     ################################################################"

###########################################################
## main and clean
###########################################################
.PHONY: main
main: \
	$(ObjectDir) \
	gen_swig \
	gen_objs \
	gen_so

.PHONY: clean
clean: clean_all

.PHONY: clean_all
clean_all: \
	clean_swig \
	clean_objs

$(ObjectDir):
	mkdir -p $(ObjectDir)

.PHONY: clean_objdir
clean_objdir:
	rm -fr $(ObjectDir)/

###########################################################
## swig targets
###########################################################
.PHONY: python
python: \
	$(ObjectDir) \
	gen_swig \
	gen_objs \
	gen_so

.PHONY: gen_swig
gen_swig: ../../include/newton_control.h
	python swig_pp.py ../../include/newton_control.h newton_ctrl_swig.h
	swig -python $(INCDIR) -DPYTHON -module newton_control newton_ctrl_swig.h

.PHONY: clean_swig
clean_swig:
	rm -fr ./newton_control_wrap.c

###########################################################
## object targets
###########################################################

.PHONY: gen_objs
gen_objs: $(OBJ_FILES)

.PHONY: clean_objs
clean_objs:
	rm -fr $(ObjectDir)/*.o

###########################################################
## shared object targets
###########################################################

.PHONY: gen_so
gen_so: gen_swig $(SO_FILE)

.PHONY: clean_so
clean_so:
	rm -fr *.so

$(SO_FILE): $(OBJ_FILES)
	ld -shared $(OBJ_FILES) $(LIBRARIES) -o $@

###########################################################
## object files
###########################################################
$(ObjectDir)/newton_control.o: ../../src/newton_control.c ../../include/newton_control.h
	$(GCC) $(CFLAGS) $(INCDIR) -o $@ -c ../../src/newton_control.c

$(ObjectDir)/newton_ctrl_swig_wrap.o: ./newton_ctrl_swig_wrap.c
	$(GCC) $(CFLAGS) $(INCDIR) -o $@ -c ./newton_ctrl_swig_wrap.c



