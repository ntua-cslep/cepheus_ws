/**
    @file

    @brief
        Program which tests the basic functionality of the driver

    @verbatim
    --------------------------------------------------------------------------
    This file and its contents are copyright (C) RTD Embedded Technologies,
    Inc.  All Rights Reserved.

    This software is licensed as described in the RTD End-User Software License
    Agreement.  For a copy of this agreement, refer to the file LICENSE.TXT
    (which should be included with this software) or contact RTD Embedded
    Technologies, Inc.
    --------------------------------------------------------------------------
    @endverbatim

    @warning
        This program ABSOLUTELY IS NOT INTENDED to be an example of how to
        program a board.  Some of the techniques appearing herein can lead to
        erratic program or system behavior and are used only to cause specific
        error conditions.

    @warning
        This program uses sbrk() to determine what should be an invalid address
        for causing certain errors in the driver.  The address returned by
        sbrk() may not cause failure in some circumstances but there seems to
        be no reliable and easy way to determine whether an arbitrary user
        address is actually mapped into a process' address space.

        $Id: basic_test.c 89872 2015-07-08 16:05:17Z rgroner $
*/

#include <errno.h>
#include <error.h>
#include <fcntl.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/sysmacros.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/utsname.h>
#include <unistd.h>

#include "dm7820_ioctl.h"
#include "dm7820_registers.h"

/*=============================================================================
Macros
 =============================================================================*/

/**
 * Default driver limit on number of DMA buffers that can be allocated per
 * DMA/FIFO channel
 */

#define DM7820_MAX_DMA_BUFFER_COUNT 32

/**
 * Default driver limit in bytes on size of DMA buffers that can be allocated
 * per DMA/FIFO channel
 */

#define DM7820_MAX_DMA_BUFFER_SIZE  0x40000

/*=============================================================================
Enumerations
 =============================================================================*/

/**
 * Status of program initialization.  Used to indication which operations need
 * to be undone when cleaning up after an error.  This is a bit mask, so only
 * one bit should be set for each enumeration value.
 */

enum initialization_state {

    /**
     * No initialization performed
     */

	INIT_NO_INITIALIZATION = 0x00000000,

    /**
     * File descriptor array allocated
     */

	INIT_BOARD_ARRAY_ALLOCATED = 0x00000004,

    /**
     * /dev entry with invalid minor number created
     */

	INIT_BAD_DEVICE_FILE_CREATED = 0x00000008,

    /**
     * File descriptor array contains at least one opened file
     */

	INIT_BOARD_ARRAY_OPENED = 0x00000010
};

/*=============================================================================
Structures
 =============================================================================*/

/**
 * Register read test information
 */

struct register_read {

    /**
     * Size of read in bits
     */

	dm7820_pci_region_access_size_t size;

    /**
     * The PCI region to read
     */

	dm7820_pci_region_num_t region;

    /**
     * Offset within region to read
     */

	uint16_t offset;

    /**
     * Expected value of read; recast based upon size of read
     */

	uint32_t expected;
};

/*=============================================================================
Type definitions
 =============================================================================*/

/**
 * Register read test case type
 */

typedef struct register_read register_read_t;

/**
 * Program initialization status type
 */

typedef enum initialization_state initialization_state_t;

/*=============================================================================
Global variables
 =============================================================================*/

/**
 * Name of the program as invoked on the command line
 */

static char *program_name;

/**
 * Size in bits for each of the access sizes
 */

static uint8_t access_sizes[] = {

    /**
     * DM7820_PCI_REGION_ACCESS_8
     */

	8,

    /**
     * DM7820_PCI_REGION_ACCESS_16
     */

	16,

    /**
     * DM7820_PCI_REGION_ACCESS_32
     */

	32
};

/**
 * Length in bytes for each of the PCI regions
 */

static uint16_t region_sizes[] = {

    /**
     * DM7820_PCI_REGION_PLX_MEM
     */

	DM7820_BAR0_LENGTH,

    /**
     * DM7820_PCI_REGION_PLX_IO
     */

	DM7820_BAR1_LENGTH,

    /**
     * DM7820_PCI_REGION_FPGA_MEM
     */

	DM7820_BAR2_LENGTH
};

/**
 * PCI region names
 */

static char *region_names[] = {

    /**
     * DM7820_PCI_REGION_PLX_MEM
     */

	"BAR0",

    /**
     * DM7820_PCI_REGION_PLX_IO
     */

	"BAR1",

    /**
     * DM7820_PCI_REGION_FPGA_MEM
     */

	"BAR2"
};

/**
 * Data for register read tests.  The board module ID registers are used because
 * they have known values.
 *
 * @note
 *      These registers are 16 bits wide, therefore a 32-bit read of them must
 *      mask off the most significant 16 bits.
 */

register_read_t register_reads[] = {
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_TC_ID,
	 0x1001},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_FIFO0_ID,
	 0x2011},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_FIFO1_ID,
	 0x2011},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK0_ID,
	 0x1000},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK1_ID,
	 0x1000},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK2_ID,
	 0x1000},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK3_ID,
	 0x1000},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_ADVINT0_ID,
	 0x0001},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_ADVINT1_ID,
	 0x0001},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_INCENC0_ID,
	 0x0002},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_INCENC1_ID,
	 0x0002},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PWM0_ID,
	 0x0003},
	{
	 DM7820_PCI_REGION_ACCESS_16,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PWM1_ID,
	 0x0003},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_TC_ID,
	 0x01},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_TC_ID + 1),
	 0x10},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_FIFO0_ID,
	 0x11},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_FIFO0_ID + 1),
	 0x20},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_FIFO1_ID,
	 0x11},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_FIFO1_ID + 1),
	 0x20},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK0_ID,
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_PRGCLK0_ID + 1),
	 0x10},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK1_ID,
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_PRGCLK1_ID + 1),
	 0x10},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK2_ID,
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_PRGCLK2_ID + 1),
	 0x10},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK3_ID,
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_PRGCLK3_ID + 1),
	 0x10},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_ADVINT0_ID,
	 0x01},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_ADVINT0_ID + 1),
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_ADVINT1_ID,
	 0x01},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_ADVINT1_ID + 1),
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_INCENC0_ID,
	 0x02},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_INCENC0_ID + 1),
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_INCENC1_ID,
	 0x02},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_INCENC1_ID + 1),
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PWM0_ID,
	 0x03},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_PWM0_ID + 1),
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PWM1_ID,
	 0x03},
	{
	 DM7820_PCI_REGION_ACCESS_8,
	 DM7820_PCI_REGION_FPGA_MEM,
	 (DM7820_BAR2_PWM1_ID + 1),
	 0x00},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_TC_ID,
	 0x00001001},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_FIFO0_ID,
	 0x00002011},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_FIFO1_ID,
	 0x00002011},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK0_ID,
	 0x00001000},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK1_ID,
	 0x00001000},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK2_ID,
	 0x00001000},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PRGCLK3_ID,
	 0x00001000},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_ADVINT0_ID,
	 0x00000001},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_ADVINT1_ID,
	 0x00000001},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_INCENC0_ID,
	 0x00000002},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_INCENC1_ID,
	 0x00000002},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PWM0_ID,
	 0x00000003},
	{
	 DM7820_PCI_REGION_ACCESS_32,
	 DM7820_PCI_REGION_FPGA_MEM,
	 DM7820_BAR2_PWM1_ID,
	 0x00000003}
};

/**
 * Program initialization state
 */

static volatile initialization_state_t init_state = INIT_NO_INITIALIZATION;

/**
 * Array of DM7820 device file descriptors
 */

static volatile int *descriptors;

/**
 * Path name of DM7820 device file with invalid minor number
 */

static volatile char bad_device_name[30];

/**
 * Number of devices found when driver was loaded
 */

static volatile unsigned long device_count = 1;

/*=============================================================================
Program code
 =============================================================================*/

/**
*******************************************************************************
@brief
    Perform actions necessary to clean up after the program encounters a fatal
    error.
 *******************************************************************************
 */

static void cleanup(void)
{
	fprintf(stdout, ">> Performing program clean up ...\n");

	if (init_state & INIT_BOARD_ARRAY_OPENED) {
		unsigned long minor_number;

		fprintf(stdout,
			"    Closing opened files in device file descriptor array ...\n");

		for (minor_number = 0; minor_number < device_count;
		     minor_number++) {
			if (descriptors[minor_number] != -1) {
				fprintf(stdout, "        Minor number %lu\n",
					minor_number);
				(void)close(descriptors[minor_number]);
			}
		}

		init_state &= ~INIT_BOARD_ARRAY_OPENED;
	}

	if (init_state & INIT_BAD_DEVICE_FILE_CREATED) {
		fprintf(stdout,
			"    Removing device file with invalid minor number ...\n");
		(void)unlink((char *)&(bad_device_name[0]));
		init_state &= ~INIT_BAD_DEVICE_FILE_CREATED;
	}

	if (init_state & INIT_BOARD_ARRAY_ALLOCATED) {
		fprintf(stdout,
			"    Freeing board file descriptor array ...\n");
		init_state &= ~INIT_BOARD_ARRAY_ALLOCATED;
	}

}

/**
*******************************************************************************
@brief
    Print information on stderr about how the program is to be used.  After
    doing so, the program is exited.
 *******************************************************************************
*/

static void usage(void)
{
	fprintf(stderr, "\n");
	fprintf(stderr, "%s\n", program_name);
	fprintf(stderr, "\n");
	fprintf(stderr, "Usage: %s [--help]\n", program_name);
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --help:     Display usage information and exit.\n");
	fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

/**
*******************************************************************************
@brief
    Verify that a function being tested succeeded.

@param
    status

    Return code from function being tested.
 *******************************************************************************
 */

static void expect_success(int status)
{
	if (status == -1) {
		error(EXIT_FAILURE,
		      errno,
		      "FAILED: Expected success but failure occurred with errno %d",
		      errno);
	}
}

/**
*******************************************************************************
@brief
    Verify that a function being tested failed with the given errno.

@param
    status

    Return code from function being tested.

@param
    expected_errno

    Expected errno that function should have set.
 *******************************************************************************
 */

static void expect_failure_and_check(int status, int expected_errno)
{
	if (status != -1) {
		error(EXIT_FAILURE, 0,
		      "FAILED: Expected failure but success occurred");
	}

	if (errno != expected_errno) {
		error(EXIT_FAILURE,
		      0,
		      "FAILED: Expected errno %d, got %d",
		      expected_errno, errno);
	}
}

/**
*******************************************************************************
@brief
    Main program code.

@param
    argument_count

    Number of command line arguments passed to executable.

@param
    arguments

    Address of array containing command line arguments.

@retval
    EXIT_SUCCESS

    Success.

@retval
    EXIT_FAILURE

    Failure.
 ******************************************************************************/

int main(int argument_count, char **arguments)
{
	char device_name[30];
	dm7820_ioctl_argument_t ioctl_request;
	int descriptor;
	int request_code;
	int status;
	struct option options[] = {
		{"help", 0, 0, 1},
		{0, 0, 0, 0}
	};
	struct stat file_stat;
	struct timeval current_time;
	uint16_t offset;
	uint8_t help_option = 0x00;
	uint8_t test_index;
	uint8_t region;
	uint8_t size;
	unsigned long minor_number;
	void *bad_address;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 Basic Functionality Test\n");
	fprintf(stdout, "\n");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Process command line options
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	while (1) {

		/*
		 * Parse the next command line option and any arguments it may require
		 */

		status = getopt_long(argument_count,
				     arguments, "", options, NULL);

		/*
		 * If getopt_long() returned -1, then all options have been processed
		 */

		if (status == -1) {
			break;
		}

		/*
		 * Figure out what getopt_long() found
		 */

		switch (status) {

			/*#################################################################
			   User entered '--help'
			   ################################################################# */

		case 1:

			/*
			 * Refuse to accept duplicate '--help' options
			 */

			if (help_option) {
				error(0, 0, "ERROR: Duplicate option '--help'");
				usage();
			}

			/*
			 * '--help' option has been seen
			 */

			help_option = 0xFF;
			break;

			/*#################################################################
			   User entered unsupported option
			   ################################################################# */

		case '?':
			usage();
			break;

			/*#################################################################
			   getopt_long() returned unexpected value
			   ################################################################# */

		default:
			error(EXIT_FAILURE,
			      0,
			      "ERROR: getopt_long() returned unexpected value %#x",
			      status);
			break;
		}
	}

	/*
	 * Recognize '--help' option before any others
	 */

	if (help_option) {
		usage();
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*#########################################################################
	   Seed the random number generator with the number of microseconds in the
	   current time to introduce more randomness
	   ######################################################################### */

	fprintf(stdout, "## Setting random number generator seed ...\n");

	if (gettimeofday(&current_time, NULL) == -1) {
		error(EXIT_FAILURE, errno, "ERROR: gettimeofday() FAILED");
	}

	srand((unsigned int)current_time.tv_usec);

	fprintf(stdout, "    Allocating file descriptor array ...\n");

	descriptors = (int *)calloc(device_count, sizeof(int));
	if (descriptors == NULL) {
		cleanup();
		error(EXIT_FAILURE, errno, "ERROR: calloc() FAILED");
	}

	init_state |= INIT_BOARD_ARRAY_ALLOCATED;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Perform functionality checks
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*#########################################################################
	   Verify that a device file with an invalid minor number cannot be opened
	   ######################################################################### */

	fprintf(stdout, "## Testing open() with invalid minor number ...\n");

	/*
	 * Get the DM7820 character major number from a valid device file
	 */

	fprintf(stdout,
		"    Getting DM7820 driver character major number ...\n");

	if (stat("/dev/rtd-dm7820-0", &file_stat) == -1) {
		cleanup();
		error(EXIT_FAILURE, errno, "ERROR: stat() FAILED");
	}

	fprintf(stdout, "        %u\n", (unsigned int)major(file_stat.st_rdev));

	/*
	 * Create the device file name for the invalid minor number
	 */

	(void)snprintf((char *)&(bad_device_name[0]),
		       sizeof(device_name),
		       "/dev/rtd-dm7820-%lu", device_count);

	fprintf(stdout, "    Creating device file %s ...\n",
		&(bad_device_name[0]));

	if (mknod((char *)&(bad_device_name[0]),
		  (S_IFCHR | S_IRUSR | S_IWUSR),
		  makedev(major(file_stat.st_rdev), device_count)
	    )
	    == -1) {
		cleanup();
		error(EXIT_FAILURE, errno, "ERROR: mknod() FAILED");
	}

	init_state |= INIT_BAD_DEVICE_FILE_CREATED;

	/*
	 * Open the invalid device file.
	 */

	fprintf(stdout, "    Opening device file ...\n");

	status = open((char *)&(bad_device_name[0]), O_RDWR);
	if (status != -1) {
		(void)close(status);
		cleanup();
		error(EXIT_FAILURE,
		      0, "ERROR: Expected open() failure but success occurred");
	}

	if (errno != ENXIO) {
		cleanup();
		error(EXIT_FAILURE,
		      0,
		      "ERROR: Expected open() errno %d, got %d", ENXIO, errno);
	}

	fprintf(stdout, "    Removing device file ...\n");

	if (unlink((char *)&(bad_device_name[0])) == -1) {
		cleanup();
		error(EXIT_FAILURE, errno, "ERROR: unlink() FAILED");
	}

	init_state &= ~INIT_BAD_DEVICE_FILE_CREATED;

	/*#########################################################################
	   Verify that each device file can be opened once but not twice
	   ######################################################################### */

	fprintf(stdout, "## Testing open() ...\n");

	for (minor_number = 0; minor_number < device_count; minor_number++) {
		fprintf(stdout, "    Minor number %lu\n", minor_number);

		(void)snprintf(&(device_name[0]),
			       sizeof(device_name),
			       "/dev/rtd-dm7820-%lu", minor_number);

		descriptors[minor_number] = -1;

		fprintf(stdout, "        First device open ...");

		descriptors[minor_number] = open(&(device_name[0]), O_RDWR);
		if (descriptors[minor_number] == -1) {
			cleanup();
			error(EXIT_FAILURE, errno, "ERROR: open() FAILED");
		} else {
			fprintf(stdout, "success.\n");
		}

		init_state |= INIT_BOARD_ARRAY_OPENED;

		fprintf(stdout, "        Second device open ...");

		descriptor = open(&(device_name[0]), O_RDWR);

		/* Expect -1, as the open should fail because the device is already open */
		if (descriptor != -1) {
			(void)close(descriptor);
			cleanup();
			error(EXIT_FAILURE, 0,
			      "ERROR: open() single access violated");
		} else {
			fprintf(stdout, "success.\n");
		}

		if (errno != EBUSY) {
			cleanup();
			error(EXIT_FAILURE, 0,
			      "ERROR: open() set errno incorrectly");
		}
	}

	/*#########################################################################
	   Verify that each device file can be closed
	   ######################################################################### */

	fprintf(stdout, "## Testing close() ...\n");

	for (minor_number = 0; minor_number < device_count; minor_number++) {
		fprintf(stdout, "    Minor number %lu...", minor_number);

		if (close(descriptors[minor_number]) == -1) {
			cleanup();
			error(EXIT_FAILURE, errno, "ERROR: close() FAILED");
		} else {
			fprintf(stdout, "success.\n");
		}

		descriptors[minor_number] = -1;
	}

	init_state &= ~INIT_BOARD_ARRAY_OPENED;

	fprintf(stdout, "    Freeing file descriptor array ...\n");

	free((void *)descriptors);

	init_state &= ~INIT_BOARD_ARRAY_ALLOCATED;

	/*#########################################################################
	   Verify that each device file can be opened again
	   ######################################################################### */

	fprintf(stdout, "## Testing open() and close() ...\n");

	for (minor_number = 0; minor_number < device_count; minor_number++) {
		fprintf(stdout, "    Minor number %lu\n", minor_number);

		(void)snprintf(&(device_name[0]),
			       sizeof(device_name),
			       "/dev/rtd-dm7820-%lu", minor_number);

		fprintf(stdout, "        Open device ...\n");

		descriptor = open(&(device_name[0]), O_RDWR);
		if (descriptor == -1) {
			cleanup();
			error(EXIT_FAILURE, errno, "ERROR: open() FAILED");
		}

		fprintf(stdout, "        Close device ...\n");

		if (close(descriptor) == -1) {
			cleanup();
			error(EXIT_FAILURE, errno, "ERROR: close() FAILED");
		}
	}

	/*#########################################################################
	   Set device file name for all remaining tests and open it
	   ######################################################################### */

	fprintf(stdout, "## Using /dev/rtd-dm7820-0 for remaining tests ...\n");

	(void)snprintf(&(device_name[0]), sizeof(device_name),
		       "/dev/rtd-dm7820-0");

	descriptor = open(&(device_name[0]), O_RDWR);
	if (descriptor == -1) {
		cleanup();
		error(EXIT_FAILURE, errno, "ERROR: open() FAILED");
	}

	/*#########################################################################
	   Determine an invalid address to use for tests that check for invalid user
	   addresses
	   ######################################################################### */

	fprintf(stdout,
		"## Determining EFAULT errno tests invalid address ...\n");

	bad_address = sbrk(0);

	fprintf(stdout, "    %#lx\n", (unsigned long)bad_address);

	/*#########################################################################
	   Verify invalid ioctl() request code error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing invalid ioctl() request code ...\n");

	status = ioctl(descriptor, TCFLSH, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	/*#########################################################################
	   Verify ioctl(DM7820_IOCTL_REGION_READ) request code error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing ioctl(DM7820_IOCTL_REGION_READ) ...\n");

	request_code = DM7820_IOCTL_REGION_READ;

	fprintf(stdout, "    On bad descriptor address ...\n");

	status = ioctl(descriptor, request_code, bad_address);
	expect_failure_and_check(status, EFAULT);

	fprintf(stdout, "    On invalid access size ...\n");

	ioctl_request.readwrite.access.size = (DM7820_PCI_REGION_ACCESS_32 + 1);
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.readwrite.access.offset = 0;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EMSGSIZE);

	fprintf(stdout, "    On invalid PCI region ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;
	ioctl_request.readwrite.access.region =
	    (DM7820_PCI_REGION_FPGA_MEM + 1);
	ioctl_request.readwrite.access.offset = 0;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid PCI region offset ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR0_LENGTH;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, ERANGE);

	fprintf(stdout, "    On misaligned PCI region offset ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_32;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.readwrite.access.offset = 3;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EOPNOTSUPP);

	fprintf(stdout, "    On all regions ...\n");

	for (region = DM7820_PCI_REGION_PLX_MEM;
	     region <= DM7820_PCI_REGION_FPGA_MEM; region++) {
		fprintf(stdout, "        Region %s\n", region_names[region]);

		for (size = DM7820_PCI_REGION_ACCESS_8;
		     size <= DM7820_PCI_REGION_ACCESS_32; size++) {
			fprintf(stdout, "            %2u bits\n",
				access_sizes[size]);

			for (offset = 0;
			     offset < region_sizes[region];
			     offset += (access_sizes[size] / 8)
			    ) {
				fprintf(stdout, "                Offset %u\r",
					offset);

				ioctl_request.readwrite.access.size = size;
				ioctl_request.readwrite.access.region = region;
				ioctl_request.readwrite.access.offset = offset;

				status =
				    ioctl(descriptor, request_code,
					  &ioctl_request);
				expect_success(status);
			}
		}
	}

	/*#########################################################################
	   Verify ioctl(DM7820_IOCTL_REGION_WRITE) request code error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing ioctl(DM7820_IOCTL_REGION_WRITE) ...\n");

	request_code = DM7820_IOCTL_REGION_WRITE;

	fprintf(stdout, "    On bad descriptor address ...\n");

	status = ioctl(descriptor, request_code, bad_address);
	expect_failure_and_check(status, EFAULT);

	fprintf(stdout, "    On invalid access size ...\n");

	ioctl_request.readwrite.access.size = (DM7820_PCI_REGION_ACCESS_32 + 1);
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = 0;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EMSGSIZE);

	fprintf(stdout, "    On invalid PCI region ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region =
	    (DM7820_PCI_REGION_FPGA_MEM + 1);
	ioctl_request.readwrite.access.offset = 0;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid PCI region offset ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_32;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_LENGTH;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, ERANGE);

	fprintf(stdout, "    On misaligned PCI region offset ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.readwrite.access.offset = 1;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EOPNOTSUPP);

	fprintf(stdout, "    On all regions ...\n");

	/*
	 * We increment the loop by 2 so that we skip over the I/O PCI region,
	 * which isn't used on this board, and causes an exception when
	 * accessed on an ARM processor.
	 */
	for (region = DM7820_PCI_REGION_PLX_MEM;
	     region <= DM7820_PCI_REGION_FPGA_MEM; region += 2) {
		fprintf(stdout, "        Region %s\n", region_names[region]);

		for (size = DM7820_PCI_REGION_ACCESS_8;
		     size <= DM7820_PCI_REGION_ACCESS_32; size++) {
			fprintf(stdout, "            %2u bits\n",
				access_sizes[size]);

			for (offset = 0;
			     offset < region_sizes[region];
			     offset += (access_sizes[size] / 8)
			    ) {
				fprintf(stdout, "                Offset %u\r",
					offset);

				/*
				 * We want to read the offset to get the current value so that
				 * the register contents do not change when we are unprepared
				 * for the changes
				 */

				ioctl_request.readwrite.access.size = size;
				ioctl_request.readwrite.access.region = region;
				ioctl_request.readwrite.access.offset = offset;

				status =
				    ioctl(descriptor, DM7820_IOCTL_REGION_READ,
					  &ioctl_request);
				expect_success(status);

				status =
				    ioctl(descriptor, request_code,
					  &ioctl_request);
				expect_success(status);
			}
		}
	}

	/*#########################################################################
	   Verify ioctl(DM7820_IOCTL_REGION_MODIFY) request code error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing ioctl(DM7820_IOCTL_REGION_MODIFY) ...\n");

	request_code = DM7820_IOCTL_REGION_MODIFY;

	fprintf(stdout, "    On bad descriptor address ...\n");

	status = ioctl(descriptor, request_code, bad_address);
	expect_failure_and_check(status, EFAULT);

	fprintf(stdout, "    On invalid access size ...\n");

	ioctl_request.modify.access.size = (DM7820_PCI_REGION_ACCESS_32 + 1);
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.offset = 0;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EMSGSIZE);

	fprintf(stdout, "    On invalid PCI region ...\n");

	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_8;
	ioctl_request.modify.access.region = (DM7820_PCI_REGION_FPGA_MEM + 1);
	ioctl_request.modify.access.offset = 0;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid PCI region offset ...\n");

	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_32;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.offset = DM7820_BAR2_LENGTH;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, ERANGE);

	fprintf(stdout, "    On misaligned PCI region offset ...\n");

	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_32;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.modify.access.offset = 2;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EOPNOTSUPP);

	fprintf(stdout, "    On all regions ...\n");

	/*
	 * We increment the loop by 2 so that we skip over the I/O PCI region,
	 * which isn't used on this board, and causes an exception when
	 * accessed on an ARM processor.
	 */
	for (region = DM7820_PCI_REGION_PLX_MEM;
	     region <= DM7820_PCI_REGION_FPGA_MEM; region += 2) {
		fprintf(stdout, "        Region %s\n", region_names[region]);

		for (size = DM7820_PCI_REGION_ACCESS_8;
		     size <= DM7820_PCI_REGION_ACCESS_32; size++) {
			fprintf(stdout, "            %2u bits\n",
				access_sizes[size]);

			for (offset = 0;
			     offset < region_sizes[region];
			     offset += (access_sizes[size] / 8)
			    ) {
				fprintf(stdout, "                Offset %u\r",
					offset);

				ioctl_request.modify.access.size = size;
				ioctl_request.modify.access.region = region;
				ioctl_request.modify.access.offset = offset;

				/*
				 * Set the new value and mask so that the register contents do
				 * not change when we are unprepared for the changes
				 */

				switch (size) {
				case DM7820_PCI_REGION_ACCESS_8:
					ioctl_request.modify.access.data.data8 =
					    0x00;
					ioctl_request.modify.mask.mask8 = 0x00;
					break;

				case DM7820_PCI_REGION_ACCESS_16:
					ioctl_request.modify.access.data.
					    data16 = 0x0000;
					ioctl_request.modify.mask.mask16 =
					    0x0000;
					break;

				case DM7820_PCI_REGION_ACCESS_32:
					ioctl_request.modify.access.data.
					    data32 = 0x00000000;
					ioctl_request.modify.mask.mask32 =
					    0x00000000;
					break;
				}

				status =
				    ioctl(descriptor, request_code,
					  &ioctl_request);
				expect_success(status);
			}
		}
	}

	/*#########################################################################
	   Verify that ioctl(DM7820_IOCTL_REGION_READ) reads the correct register
	   ######################################################################### */

	fprintf(stdout,
		"## Testing ioctl(DM7820_IOCTL_REGION_READ) access ...\n");

	for (test_index = 0;
	     test_index < (sizeof(register_reads) / sizeof(register_read_t));
	     test_index++) {
		fprintf(stdout,
			"    %s: offset %#x, %u bits\r",
			region_names[register_reads[test_index].region],
			register_reads[test_index].offset,
			access_sizes[register_reads[test_index].size]
		    );

		ioctl_request.readwrite.access.size =
		    register_reads[test_index].size;
		ioctl_request.readwrite.access.region =
		    register_reads[test_index].region;
		ioctl_request.readwrite.access.offset =
		    register_reads[test_index].offset;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
		expect_success(status);

		switch (register_reads[test_index].size) {
		case DM7820_PCI_REGION_ACCESS_8:
			if (ioctl_request.readwrite.access.data.data8
			    != (uint8_t) register_reads[test_index].expected) {
				error(EXIT_FAILURE, 0,
				      "ERROR: Invalid value read");
			}
			break;

		case DM7820_PCI_REGION_ACCESS_16:
			if (ioctl_request.readwrite.access.data.data16
			    != (uint16_t) register_reads[test_index].expected) {
				error(EXIT_FAILURE, 0,
				      "ERROR: Invalid value read");
			}
			break;

		case DM7820_PCI_REGION_ACCESS_32:

			/*
			 * Mask off most significant 16 bits of value returned because
			 * the module ID registers are only 16 bits wide
			 */

			if ((ioctl_request.readwrite.access.data.
			     data32 & 0xFFFF)
			    != (uint32_t) register_reads[test_index].expected) {
				error(EXIT_FAILURE, 0,
				      "ERROR: Invalid value read");
			}
			break;
		}
	}

	/*#########################################################################
	   Verify that ioctl(DM7820_IOCTL_REGION_WRITE) writes the correct register
	   ######################################################################### */

	fprintf(stdout,
		"## Testing ioctl(DM7820_IOCTL_REGION_WRITE) access ...\n");

	fprintf(stdout, "    16 bits ...\n");

	/*
	 * Write a bit pattern to Port 0 Output Register
	 */

	fprintf(stdout, "        Write to Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0xF1A7;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request);
	expect_success(status);

	/*
	 * Verify value written to Port 0 Output Register
	 */

	fprintf(stdout, "        Read from Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0x0000;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
	expect_success(status);

	if (ioctl_request.readwrite.access.data.data16 != 0xF1A7) {
		error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
	}

	/*
	 * Write reset command to FPGA Board Reset Register
	 */

	fprintf(stdout, "        Write to FPGA Board Reset Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_BOARD_RESET;
	ioctl_request.readwrite.access.data.data16 =
	    DM7820_BOARD_RESET_DO_RESET;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request);
	expect_success(status);

	/*
	 * Verify that board reset occurred
	 */

	fprintf(stdout, "        Read from Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0xFFFF;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
	expect_success(status);

	if (ioctl_request.readwrite.access.data.data16 != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
	}

	fprintf(stdout, "    8 bits ...\n");

	/*
	 * Write a bit pattern to Port 0 Output Register
	 */

	fprintf(stdout, "        Write to Port 0 Output Register low ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data8 = 0x95;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request);
	expect_success(status);

	fprintf(stdout, "        Write to Port 0 Output Register high ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = (DM7820_BAR2_PORT0_OUTPUT + 1);
	ioctl_request.readwrite.access.data.data8 = 0xA6;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request);
	expect_success(status);

	/*
	 * Verify value written to Port 0 Output Register
	 */

	fprintf(stdout, "        Read from Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0x0000;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
	expect_success(status);

	if (ioctl_request.readwrite.access.data.data16 != 0xA695) {
		error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
	}

	/*
	 * Write reset command to FPGA Board Reset Register
	 */

	fprintf(stdout, "        Write to FPGA Board Reset Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_BOARD_RESET;
	ioctl_request.readwrite.access.data.data16 =
	    DM7820_BOARD_RESET_DO_RESET;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request);
	expect_success(status);

	/*
	 * Verify that board reset occurred
	 */

	fprintf(stdout, "        Read from Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0xFFFF;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
	expect_success(status);

	if (ioctl_request.readwrite.access.data.data16 != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
	}

	/*#########################################################################
	   Verify that ioctl(DM7820_IOCTL_REGION_MODIFY) writes the correct register
	   ######################################################################### */

	fprintf(stdout,
		"## Testing ioctl(DM7820_IOCTL_REGION_MODIFY) access ...\n");

	/*
	 * Write a bit pattern to Port 0 Output Register
	 */

	fprintf(stdout, "    Write to Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0x1234;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request);
	expect_success(status);

	/*
	 * Verify value written to Port 0 Output Register
	 */

	fprintf(stdout, "    Read from Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0x0000;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
	expect_success(status);

	if (ioctl_request.readwrite.access.data.data16 != 0x1234) {
		error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
	}

	/*
	 * Modify FPGA Board Reset Register contents to issue reset command
	 */

	fprintf(stdout, "    Modify FPGA Board Reset Register ...\n");

	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.offset = DM7820_BAR2_BOARD_RESET;
	ioctl_request.modify.access.data.data16 = DM7820_BOARD_RESET_DO_RESET;
	ioctl_request.modify.mask.mask16 = 0xFFFF;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_MODIFY, &ioctl_request);
	expect_success(status);

	/*
	 * Verify that board reset occurred
	 */

	fprintf(stdout, "    Read from Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0xFFFF;

	status = ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
	expect_success(status);

	if (ioctl_request.readwrite.access.data.data16 != 0x0000) {
		error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
	}

	/*#########################################################################
	   Verify that ioctl(DM7820_IOCTL_REGION_MODIFY) handles masks correctly
	   ######################################################################### */

	fprintf(stdout,
		"## Testing ioctl(DM7820_IOCTL_REGION_MODIFY) masks ...\n");

	fprintf(stdout, "    8 bits ...\n");

	/*
	 * We increment the loop by 2 so that we skip over the I/O PCI region,
	 * which isn't used on this board, and causes an exception when
	 * accessed on an ARM processor.
	 */
	for (region = DM7820_PCI_REGION_PLX_MEM;
	     region <= DM7820_PCI_REGION_FPGA_MEM; region += 2) {
		uint16_t offset = 0x0000;
		uint8_t expected;
		uint8_t initial;
		uint8_t mask;
		uint8_t new;

		fprintf(stdout, "        Region %s\n", region_names[region]);

		/*
		 * Determine a suitable offset in each region for the test.
		 */

		switch (region) {
		case DM7820_PCI_REGION_PLX_MEM:
			offset = DM7820_BAR0_DMAPADR0;
			break;

		case DM7820_PCI_REGION_FPGA_MEM:
			offset = DM7820_BAR2_PORT0_PERIPH_SEL_L;
			break;
		}

		fprintf(stdout, "            Offset %#x\r", offset);

		/*
		 * Generate random values for the initial register value, modification
		 * mask, and new value
		 */

		initial = (uint8_t) rand();

		fprintf(stdout, "                Initial value: %#x\n",
			initial);

		mask = (uint8_t) rand();

		fprintf(stdout, "                Mask: %#x\n", mask);

		new = (uint8_t) rand();

		fprintf(stdout, "                New value: %#x\n", new);

		/*
		 * Figure out expected result of the modification
		 */

		expected = initial;
		expected &= ~mask;
		expected |= (new & mask);

		fprintf(stdout, "                Expected result: %#x\n",
			expected);

		/*
		 * Write initial value to register
		 */

		fprintf(stdout, "                Write initial value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_8;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;
		ioctl_request.readwrite.access.data.data8 = initial;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		expect_success(status);

		/*
		 * Modify the register value based given the mask and new value
		 */

		fprintf(stdout, "                Modify register value ...\n");

		ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_8;
		ioctl_request.modify.access.region = region;
		ioctl_request.modify.access.offset = offset;
		ioctl_request.modify.access.data.data8 = new;
		ioctl_request.modify.mask.mask8 = mask;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_MODIFY,
			  &ioctl_request);
		expect_success(status);

		/*
		 * Read new register value and compare to expected result
		 */

		fprintf(stdout, "                Read register value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_8;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
		expect_success(status);

		fprintf(stdout,
			"                Compare to expected value ...\n");

		if (ioctl_request.readwrite.access.data.data8 != expected) {
			error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
		}

		/*
		 * Restore default register value.  The registers were chosen such that
		 * all of them have default values of zero.
		 */

		fprintf(stdout, "                Restore default value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_8;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;
		ioctl_request.readwrite.access.data.data8 = 0x00;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		expect_success(status);
	}

	fprintf(stdout, "    16 bits ...\n");

	/*
	 * We increment the loop by 2 so that we skip over the I/O PCI region,
	 * which isn't used on this board, and causes an exception when
	 * accessed on an ARM processor.
	 */
	for (region = DM7820_PCI_REGION_PLX_MEM;
	     region <= DM7820_PCI_REGION_FPGA_MEM; region += 2) {
		uint16_t expected;
		uint16_t initial;
		uint16_t mask;
		uint16_t new;
		uint16_t offset = 0x0000;

		fprintf(stdout, "        Region %s\n", region_names[region]);

		/*
		 * Determine a suitable offset in each region for the test.
		 */

		switch (region) {
		case DM7820_PCI_REGION_PLX_MEM:
			offset = DM7820_BAR0_DMAPADR0;
			break;

		case DM7820_PCI_REGION_FPGA_MEM:
			offset = DM7820_BAR2_PORT0_PERIPH_SEL_L;
			break;
		}

		fprintf(stdout, "            Offset %#x\r", offset);

		/*
		 * Generate random values for the initial register value, modification
		 * mask, and new value
		 */

		initial = (uint16_t) rand();

		fprintf(stdout, "                Initial value: %#x\n",
			initial);

		mask = (uint16_t) rand();

		fprintf(stdout, "                Mask: %#x\n", mask);

		new = (uint16_t) rand();

		fprintf(stdout, "                New value: %#x\n", new);

		/*
		 * Figure out expected result of the modification
		 */

		expected = initial;
		expected &= ~mask;
		expected |= (new & mask);

		fprintf(stdout, "                Expected result: %#x\n",
			expected);

		/*
		 * Write initial value to register
		 */

		fprintf(stdout, "                Write initial value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;
		ioctl_request.readwrite.access.data.data16 = initial;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		expect_success(status);

		/*
		 * Modify the register value based given the mask and new value
		 */

		fprintf(stdout, "                Modify register value ...\n");

		ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
		ioctl_request.modify.access.region = region;
		ioctl_request.modify.access.offset = offset;
		ioctl_request.modify.access.data.data16 = new;
		ioctl_request.modify.mask.mask16 = mask;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_MODIFY,
			  &ioctl_request);
		expect_success(status);

		/*
		 * Read new register value and compare to expected result
		 */

		fprintf(stdout, "                Read register value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
		expect_success(status);

		fprintf(stdout,
			"                Compare to expected value ...\n");

		if (ioctl_request.readwrite.access.data.data16 != expected) {
			error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
		}

		/*
		 * Restore default register value.  The registers were chosen such that
		 * all of them have default values of zero.
		 */

		fprintf(stdout, "                Restore default value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;
		ioctl_request.readwrite.access.data.data16 = 0x0000;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		expect_success(status);
	}

	fprintf(stdout, "    32 bits ...\n");

	/*
	 * We increment the loop by 2 so that we skip over the I/O PCI region,
	 * which isn't used on this board, and causes an exception when
	 * accessed on an ARM processor.
	 */
	for (region = DM7820_PCI_REGION_PLX_MEM;
	     region <= DM7820_PCI_REGION_FPGA_MEM; region += 2) {
		uint16_t offset = 0x0000;
		uint32_t expected;
		uint32_t initial;
		uint32_t mask;
		uint32_t new;

		fprintf(stdout, "        Region %s\n", region_names[region]);

		/*
		 * Determine a suitable offset in each region for the test.
		 */

		switch (region) {
		case DM7820_PCI_REGION_PLX_MEM:
			offset = DM7820_BAR0_DMAPADR0;
			break;

		case DM7820_PCI_REGION_FPGA_MEM:
			offset = DM7820_BAR2_PORT0_PERIPH_SEL_L;
			break;
		}

		fprintf(stdout, "            Offset %#x\r", offset);

		/*
		 * Generate random values for the initial register value, modification
		 * mask, and new value
		 */

		initial = (uint32_t) rand();

		fprintf(stdout, "                Initial value: %#x\n",
			initial);

		mask = (uint32_t) rand();

		fprintf(stdout, "                Mask: %#x\n", mask);

		new = (uint32_t) rand();

		fprintf(stdout, "                New value: %#x\n", new);

		/*
		 * Figure out expected result of the modification
		 */

		expected = initial;
		expected &= ~mask;
		expected |= (new & mask);

		fprintf(stdout, "                Expected result: %#x\n",
			expected);

		/*
		 * Write initial value to register
		 */

		fprintf(stdout, "                Write initial value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_32;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;
		ioctl_request.readwrite.access.data.data32 = initial;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		expect_success(status);

		/*
		 * Modify the register value based given the mask and new value
		 */

		fprintf(stdout, "                Modify register value ...\n");

		ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_32;
		ioctl_request.modify.access.region = region;
		ioctl_request.modify.access.offset = offset;
		ioctl_request.modify.access.data.data32 = new;
		ioctl_request.modify.mask.mask32 = mask;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_MODIFY,
			  &ioctl_request);
		expect_success(status);

		/*
		 * Read new register value and compare to expected result
		 */

		fprintf(stdout, "                Read register value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_32;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request);
		expect_success(status);

		fprintf(stdout,
			"                Compare to expected value ...\n");

		if (ioctl_request.readwrite.access.data.data32 != expected) {
			error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
		}

		/*
		 * Restore default register value.  The registers were chosen such that
		 * all of them have default values of zero.
		 */

		fprintf(stdout, "                Restore default value ...\n");

		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_32;
		ioctl_request.readwrite.access.region = region;
		ioctl_request.readwrite.access.offset = offset;
		ioctl_request.readwrite.access.data.data32 = 0x00000000;

		status =
		    ioctl(descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		expect_success(status);
	}

	/*#########################################################################
	   Verify ioctl(DM7820_IOCTL_GET_INTERRUPT_STATUS) request code error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing ioctl(DM7820_IOCTL_GET_INTERRUPT_STATUS) ...\n");

	request_code = DM7820_IOCTL_GET_INTERRUPT_STATUS;

	fprintf(stdout, "    On bad descriptor address ...\n");

	status = ioctl(descriptor, request_code, bad_address);
	expect_failure_and_check(status, EFAULT);

	fprintf(stdout, "    When not waiting for interrupts ...\n");

	ioctl_request.int_status.wait_for_interrupt = 0x00;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_success(status);

	/*#########################################################################
	   Verify ioctl(DM7820_IOCTL_DMA_FUNCTION) request code error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing ioctl(DM7820_IOCTL_DMA_FUNCTION) ...\n");

	request_code = DM7820_IOCTL_DMA_FUNCTION;

	fprintf(stdout, "    On bad descriptor address ...\n");

	status = ioctl(descriptor, request_code, bad_address);
	expect_failure_and_check(status, EFAULT);

	fprintf(stdout, "    On invalid function ...\n");

	ioctl_request.dma_function.function = (DM7820_DMA_FUNCTION_WRITE + 7);
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid FIFO ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_INITIALIZE;
	ioctl_request.dma_function.fifo = (DM7820_FIFO_QUEUE_1 + 1);
	ioctl_request.dma_function.arguments.dma_init.buffer_count = 1;
	ioctl_request.dma_function.arguments.dma_init.buffer_size = 1024;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout,
		"    With function DM7820_DMA_FUNCTION_INITIALIZE ...\n");

	fprintf(stdout, "        On buffer count of zero ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_INITIALIZE;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;
	ioctl_request.dma_function.arguments.dma_init.buffer_count = 0;
	ioctl_request.dma_function.arguments.dma_init.buffer_size = 1024;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "        On buffer count exceeding maximum ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_INITIALIZE;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;
	ioctl_request.dma_function.arguments.dma_init.buffer_count =
	    (DM7820_MAX_DMA_BUFFER_COUNT + 1);
	ioctl_request.dma_function.arguments.dma_init.buffer_size = 1024;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "        On buffer size of zero ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_INITIALIZE;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;
	ioctl_request.dma_function.arguments.dma_init.buffer_count = 1;
	ioctl_request.dma_function.arguments.dma_init.buffer_size = 0;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "        On odd buffer size ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_INITIALIZE;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;
	ioctl_request.dma_function.arguments.dma_init.buffer_count = 1;
	ioctl_request.dma_function.arguments.dma_init.buffer_size = 1;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "        On buffer size exceeding maximum ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_INITIALIZE;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;
	ioctl_request.dma_function.arguments.dma_init.buffer_count = 1;
	ioctl_request.dma_function.arguments.dma_init.buffer_size =
	    (DM7820_MAX_DMA_BUFFER_SIZE + 2);

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    With function DM7820_DMA_FUNCTION_STOP ...\n");

	fprintf(stdout, "        On bad address ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_STOP;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;

	status = ioctl(descriptor, request_code, bad_address);
	expect_failure_and_check(status, EFAULT);

	fprintf(stdout, "        On invalid fifo ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_STOP;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_1 + 1;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    With function DM7820_DMA_FUNCTION_READ ...\n");

	fprintf(stdout, "        On bad address ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_READ;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;
	ioctl_request.dma_function.transfer_size = 2;

	status = ioctl(descriptor, request_code, bad_address);
	expect_failure_and_check(status, EFAULT);

	fprintf(stdout, "        On invalid fifo ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_READ;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_1 + 1;
	ioctl_request.dma_function.transfer_size = 2;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "        On invalid transfer size ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_READ;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_1 + 1;
	ioctl_request.dma_function.transfer_size = -1;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    With function DM7820_DMA_FUNCTION_WRITE ...\n");

	fprintf(stdout, "        On bad address ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_WRITE;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_0;
	ioctl_request.dma_function.transfer_size = 2;

	status = ioctl(descriptor, request_code, bad_address);
	expect_failure_and_check(status, EFAULT);

	fprintf(stdout, "        On invalid fifo ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_WRITE;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_1 + 1;
	ioctl_request.dma_function.transfer_size = 2;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "        On invalid transfer size ...\n");

	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_WRITE;
	ioctl_request.dma_function.fifo = DM7820_FIFO_QUEUE_1 + 1;
	ioctl_request.dma_function.transfer_size = -1;

	status = ioctl(descriptor, request_code, &ioctl_request);
	expect_failure_and_check(status, EINVAL);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "## Closing device ...\n");

	if (close(descriptor) == -1) {
		error(EXIT_FAILURE, errno, "ERROR: close() FAILED");
	}

	fprintf(stdout, "\n");
	fprintf(stdout, "SUCCESS.  All tests passed.\n");

	exit(EXIT_SUCCESS);

}
