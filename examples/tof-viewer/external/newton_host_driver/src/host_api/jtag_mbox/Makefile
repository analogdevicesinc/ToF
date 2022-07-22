
CC = g++

BUILD_DIR = $(PWD)

MAKE_PATH = $(dir $(realpath $(firstword $(MAKEFILE_LIST))))

CFLAGS += -I$(MAKE_PATH)/inc \
					-I. \
					-I /usr/include/libusb-1.0 \
					-g 

LIB_PATH = -L.

all: clean example

example: main.c
	$(CC) -o $@ $^ $(LIB_PATH) -lusb-1.0 -ltdr

clean:
	rm -f *.o example
