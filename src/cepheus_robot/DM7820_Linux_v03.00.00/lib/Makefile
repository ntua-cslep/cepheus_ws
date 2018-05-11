#
#	FILE NAME: Makefile
#
#	FILE DESCRIPTION: Make description file for building user library
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
DEBUG_FLAGS=
INCLUDE_FLAGS=-I../include
OPTIMIZE_FLAGS=-O6
WARNING_FLAGS=-Wall
CFLAGS:=$(DEBUG_FLAGS) $(INCLUDE_FLAGS) $(OPTIMIZE_FLAGS) $(WARNING_FLAGS)

all:			librtd-dm7820.a

librtd-dm7820.a:	librtd-dm7820.o
	ar -cru librtd-dm7820.a librtd-dm7820.o

librtd-dm7820.o:	librtd-dm7820.c
	$(CC) $(CFLAGS) -c librtd-dm7820.c

clean:
	rm -f *.o *.a *~
