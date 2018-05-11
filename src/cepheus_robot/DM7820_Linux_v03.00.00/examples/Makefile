#
#	FILE NAME: Makefile
#
#	FILE DESCRIPTION: Make description file for building example programs
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

CC=gcc
DEBUG_FLAGS=-g
INCLUDE_FLAGS=-I../include
LIBRARY_FLAGS=-L../lib -lrtd-dm7820 -lpthread
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

EXAMPLES = \
	advint_event_interrupt \
	advint_match_interrupt \
	advint_status \
	advint_strobe_interrupt \
	brdctl_device_info \
	dma_capture_buffers \
	dma_capture_file \
	fifo_cascaded \
	fifo_interrupt \
	fifo_pci_access \
	incenc_encoders \
	incenc_interrupt \
	incenc_status \
	loopback \
	prgclk_clocks \
	pwm_interrupt \
	pwm_measure \
	pwm_modulators \
	stdio_digital_io \
	stdio_peripheral_output \
	stdio_strobe_signal \
	tmrctr_interrupt \
	tmrctr_status \
	tmrctr_timers

TESTS = \
	basic_test \
	library_test

all:	$(EXAMPLES) $(TESTS)

advint_event_interrupt:	advint_event_interrupt.c
	$(CC) $(CFLAGS) -o advint_event_interrupt advint_event_interrupt.c \
		$(LIBRARY_FLAGS)

advint_match_interrupt:	advint_match_interrupt.c
	$(CC) $(CFLAGS) -o advint_match_interrupt advint_match_interrupt.c \
		$(LIBRARY_FLAGS)

advint_status:	advint_status.c
	$(CC) $(CFLAGS) -o advint_status advint_status.c $(LIBRARY_FLAGS)

advint_strobe_interrupt:	advint_strobe_interrupt.c
	$(CC) $(CFLAGS) -o advint_strobe_interrupt advint_strobe_interrupt.c \
		$(LIBRARY_FLAGS)

basic_test:	basic_test.c
	$(CC) $(CFLAGS) -o basic_test basic_test.c

brdctl_device_info:	brdctl_device_info.c
	$(CC) $(CFLAGS) -o brdctl_device_info brdctl_device_info.c \
		$(LIBRARY_FLAGS)

dma_capture_buffers:	dma_capture_buffers.c
	$(CC) $(CFLAGS) -o dma_capture_buffers dma_capture_buffers.c \
	$(LIBRARY_FLAGS)

dma_capture_file:	dma_capture_file.c
	$(CC) $(CFLAGS) -o dma_capture_file dma_capture_file.c \
	$(LIBRARY_FLAGS)

fifo_cascaded:	fifo_cascaded.c
	$(CC) $(CFLAGS) -o fifo_cascaded fifo_cascaded.c $(LIBRARY_FLAGS)

fifo_interrupt:	fifo_interrupt.c
	$(CC) $(CFLAGS) -o fifo_interrupt fifo_interrupt.c $(LIBRARY_FLAGS)

fifo_pci_access:	fifo_pci_access.c
	$(CC) $(CFLAGS) -o fifo_pci_access fifo_pci_access.c $(LIBRARY_FLAGS)

incenc_encoders:	incenc_encoders.c
	$(CC) $(CFLAGS) -o incenc_encoders incenc_encoders.c $(LIBRARY_FLAGS)

incenc_interrupt:	incenc_interrupt.c
	$(CC) $(CFLAGS) -o incenc_interrupt incenc_interrupt.c $(LIBRARY_FLAGS)

incenc_status:	incenc_status.c
	$(CC) $(CFLAGS) -o incenc_status incenc_status.c $(LIBRARY_FLAGS)

loopback:	loopback.c
	$(CC) $(CFLAGS) -o loopback loopback.c $(LIBRARY_FLAGS)

library_test:	library_test.c
	$(CC) $(CFLAGS) -o library_test library_test.c $(LIBRARY_FLAGS)

prgclk_clocks:	prgclk_clocks.c
	$(CC) $(CFLAGS) -o prgclk_clocks prgclk_clocks.c $(LIBRARY_FLAGS)

pwm_interrupt:	pwm_interrupt.c
	$(CC) $(CFLAGS) -o pwm_interrupt pwm_interrupt.c $(LIBRARY_FLAGS)

pwm_measure:	pwm_measure.c
	$(CC) $(CFLAGS) -o pwm_measure pwm_measure.c $(LIBRARY_FLAGS)

pwm_modulators:	pwm_modulators.c
	$(CC) $(CFLAGS) -o pwm_modulators pwm_modulators.c $(LIBRARY_FLAGS)

stdio_digital_io:	stdio_digital_io.c
	$(CC) $(CFLAGS) -o stdio_digital_io stdio_digital_io.c $(LIBRARY_FLAGS)

stdio_peripheral_output:	stdio_peripheral_output.c
	$(CC) $(CFLAGS) -o stdio_peripheral_output stdio_peripheral_output.c \
		$(LIBRARY_FLAGS)

stdio_strobe_signal:	stdio_strobe_signal.c
	$(CC) $(CFLAGS) -o stdio_strobe_signal stdio_strobe_signal.c \
		$(LIBRARY_FLAGS)

tmrctr_interrupt:	tmrctr_interrupt.c
	$(CC) $(CFLAGS) -o tmrctr_interrupt tmrctr_interrupt.c $(LIBRARY_FLAGS)

tmrctr_status:	tmrctr_status.c
	$(CC) $(CFLAGS) -o tmrctr_status tmrctr_status.c $(LIBRARY_FLAGS)

tmrctr_timers:	tmrctr_timers.c
	$(CC) $(CFLAGS) -o tmrctr_timers tmrctr_timers.c $(LIBRARY_FLAGS)

clean:
	rm -f *.o *.dat *~ $(EXAMPLES) $(TESTS)
