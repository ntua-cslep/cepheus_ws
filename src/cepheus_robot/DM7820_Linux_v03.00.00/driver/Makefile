#
#	FILE NAME: Makefile
#
#	FILE DESCRIPTION: Make description file for building driver
#
#	PROJECT NAME: Linux Software (DM7820)
#
#	PROJECT VERSION: (Defined in README.TXT)
#
#	This file and its contents are copyright (C) RTD Embedded Technologies,
#	Inc.  All Rights Reserved.
#
#	This software is licensed as described in the RTD End-User Software
#	License Agreement.  For a copy of this agreement, refer to the file
#	LICENSE.TXT (which should be included with this software) or contact RTD
#	Embedded Technologies, Inc.
#


MY_CFLAGS := -I$(shell pwd)/../include

#
# Kernel build environment directory.  Supposedly it is safer to use this
# method of referring to it than using /usr/src.
#

KERNEL_DIR := /lib/modules/$(shell uname -r)/build

obj-m := rtd-dm7820.o

DRIVER_MODULE=rtd-dm7820.ko
FILES_TO_CLEAN= \
	*.o \
	rtd-dm7820.ko \
	.*.cmd \
	*.ko \
	.tmp_versions \
	rtd-dm7820.mod.c \
	.*.d \
	*.symvers \
	*.order \
    *.markers \
	*~

driver:		rtd-dm7820.c
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(MY_CFLAGS)" modules

.PHONY: debug
debug:
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(MY_CFLAGS) -DDM7820_DEBUG_INTERRUPTS" modules

.PHONY: debug_dma
debug_dma:
	make -C $(KERNEL_DIR) SUBDIRS=`pwd` EXTRA_CFLAGS="$(MY_CFLAGS) -DDM7820_DEBUG_DMA" modules


clean:
	rm -rf $(FILES_TO_CLEAN)

load:
	insmod ./$(DRIVER_MODULE)
	chmod 666 /dev/rtd-dm7820-*
	
unload:
	rmmod rtd-dm7820
	@echo "Removing old device files ..."; \
	rm -f /dev/rtd-dm7820-* \
	echo ""; \
	
