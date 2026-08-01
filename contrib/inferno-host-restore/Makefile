# Top-level Makefile for inferno-host-restore contrib tools.
#
# inferno-netd — userspace reverse-tethering for the emulated iPhone: claims the
# device's CDC-NCM interface via the inferno-usbd broker and NATs it to the host
# network with libslirp.
#
# The broker + libusb shim have their own Makefile under libusb-shim/.

CC       ?= cc
CFLAGS   ?= -O2 -Wall
SLIRP_CFLAGS := $(shell pkg-config --cflags slirp)
SLIRP_LIBS   := $(shell pkg-config --libs slirp)

BROKER_INC := -Ilibusb-shim

all: inferno-netd

inferno-netd: inferno-netd.c libusb-shim/broker_proto.h
	$(CC) $(CFLAGS) $(BROKER_INC) $(SLIRP_CFLAGS) inferno-netd.c -o $@ $(SLIRP_LIBS)

clean:
	rm -f inferno-netd

.PHONY: all clean
