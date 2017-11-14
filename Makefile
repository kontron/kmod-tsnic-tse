#
# Makefile for the Altera device drivers.
#

obj-m += tsnic-tse.o
tsnic-tse-objs := altera_tse_main.o altera_tse_ethtool.o \
altera_msgdma.o altera_sgdma.o altera_utils.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build

all: modules

clean: modules

clean modules:
	$(MAKE) -C $(KERNEL_SRC) M=$$PWD $@
