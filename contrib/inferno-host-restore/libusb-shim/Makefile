# Build inferno-usbd (the broker) and libinferno-usb (the libusb-1.0 shim).
#
# The shim is installed as libusb-1.0.{dylib,so} + a libusb-1.0.pc so the
# libimobiledevice restore stack links against it instead of the real libusb.

UNAME := $(shell uname -s)
CC     ?= cc
CFLAGS ?= -std=c11 -Wall -Wextra -O2 -g -fPIC
PREFIX ?= $(HOME)/InfernoData/host-tools

ifeq ($(UNAME),Darwin)
  SHLIB   := libusb-1.0.0.dylib
  SHLINK  := libusb-1.0.dylib
  SONAME  := -install_name $(PREFIX)/lib/$(SHLIB)
else
  SHLIB   := libusb-1.0.so.0
  SHLINK  := libusb-1.0.so
  SONAME  := -Wl,-soname,$(SHLIB)
endif

all: inferno-usbd $(SHLIB)

inferno-usbd: inferno_usbd.c broker_proto.h tcp_usb_proto.h
	$(CC) $(CFLAGS) -o $@ inferno_usbd.c

$(SHLIB): libusb_shim.c libusb.h broker_proto.h tcp_usb_proto.h
	$(CC) $(CFLAGS) -shared $(SONAME) -o $@ libusb_shim.c

install: all
	mkdir -p $(PREFIX)/lib/pkgconfig $(PREFIX)/include/libusb-1.0 $(PREFIX)/bin
	cp inferno-usbd $(PREFIX)/bin/
	cp $(SHLIB) $(PREFIX)/lib/
	ln -sf $(SHLIB) $(PREFIX)/lib/$(SHLINK)
	cp libusb.h $(PREFIX)/include/libusb-1.0/
	sed -e 's|@PREFIX@|$(PREFIX)|g' libusb-1.0.pc.in > $(PREFIX)/lib/pkgconfig/libusb-1.0.pc

selftest: selftest.c libusb_shim.c libusb.h broker_proto.h tcp_usb_proto.h inferno-usbd
	$(CC) $(CFLAGS) -I. -o $@ selftest.c libusb_shim.c

test: selftest
	./selftest

clean:
	rm -f inferno-usbd $(SHLIB) $(SHLINK) selftest *.o

.PHONY: all install clean test
