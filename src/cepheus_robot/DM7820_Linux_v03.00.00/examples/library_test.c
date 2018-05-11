/**
    @file

    @brief
        Program which tests the basic functionality of the user library

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

    $Id: library_test.c 86306 2015-03-05 15:27:53Z rgroner $
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
#include "dm7820_library.h"
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

#define DM7820_MAX_DMA_BUFFER_SIZE  0x20000

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
     * Board descriptor array allocated
     */

	INIT_BOARD_ARRAY_ALLOCATED = 0x00000004,

    /**
     * /dev entry with invalid minor number created
     */

	INIT_BAD_DEVICE_FILE_CREATED = 0x00000008,

    /**
     * Board descriptor array contains at least one opened file
     */

	INIT_BOARD_ARRAY_OPENED = 0x00000010
};

/*=============================================================================
Type definitions
 =============================================================================*/

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
 * Program initialization state
 */

static volatile initialization_state_t init_state = INIT_NO_INITIALIZATION;

/**
 * Array of DM7820 library device descriptors
 */

static volatile DM7820_Board_Descriptor **boards;

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
    Verify that a function being tested failed with the given errno.

@param
    status

    Return code from function being tested.

@param
    expected_errno

    Expected errno that function should have set.
 *******************************************************************************
 */

static void expect_failure_and_check(DM7820_Error status, int expected_errno)
{
	if (status != -1) {
		cleanup();
		error(EXIT_FAILURE, 0,
		      "FAILED: Expected failure but success occurred.");
	}
	if (errno != expected_errno) {
		cleanup();
		error(EXIT_FAILURE,
		      0,
		      "FAILED: Expected errno %d, got %d.\n",
		      expected_errno, errno);
	}
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

static void expect_success(DM7820_Error status)
{
	if (status == -1) {
		cleanup();
		error(EXIT_FAILURE,
		      0,
		      "FAILED: Expected success but failure occurred with errno %d.\n",
		      errno);
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
	DM7820_Board_Descriptor *board;
	DM7820_Error status;
	DM7820_StdIO_IO_Mode stdio_mode;
	DM7820_StdIO_Periph_Mode peripheral_mode;
	DM7820_StdIO_Port stdio_port;
	DM7820_StdIO_Strobe strobe_signal;
	dm7820_advint_interrupt interrupt;
	dm7820_fifo_queue fifo;
	dm7820_incenc_encoder encoder;
	dm7820_interrupt_info int_status;
	dm7820_interrupt_source source;
	dm7820_ioctl_argument_t ioctl_request;
	dm7820_prgclk_clock clock;
	dm7820_tmrctr_timer timer;
	dm7820_pwm_modulator pwm;
	struct option options[] = {
		{"help", 0, 0, 1},
		{0, 0, 0, 0}
	};

	struct timeval current_time;
	struct utsname kernel_info;
	uint16_t capture_value;
	uint16_t fifo_data;
	uint16_t independent_value;
	uint16_t stdio_value;
	uint16_t svn_version;
	uint16_t timer_value;
	uint32_t joined_value;
	uint8_t encoder_status;
	uint8_t fifo_status;
	uint8_t fpga_type_id;
	uint8_t fpga_version;
	uint8_t help_option = 0x00;
	uint8_t interrupt_status;
	uint8_t pci_master;
	uint8_t strobe_state;
	uint8_t timer_status;
	unsigned long minor_number;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 User Library Functionality Test\n");
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

	/*#########################################################################
	   Determine operating system release.  The open() system call behaves
	   differently on 2.4 and 2.6 kernels when given a device file with a valid
	   major number but an invalid minor number.
	   ######################################################################### */

	fprintf(stdout, "## Determining operating system release ...\n");

	if (uname(&kernel_info) == -1) {
		error(EXIT_FAILURE, errno, "ERROR: uname() FAILED");
	}

	fprintf(stdout, "    Allocating board descriptor array ...\n");

	boards =
	    (volatile DM7820_Board_Descriptor **)calloc(device_count,
							sizeof
							(DM7820_Board_Descriptor
							 *)
	    );
	if (boards == NULL) {
		cleanup();
		error(EXIT_FAILURE, errno, "ERROR: calloc() FAILED");
	}

	init_state |= INIT_BOARD_ARRAY_ALLOCATED;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Perform functionality checks
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*#########################################################################
	   Verify DM7820_General_Close_Board() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_General_Close_Board() error checking ...\n");

	if (DM7820_General_Close_Board(NULL) != -1) {
		cleanup();
		error(EXIT_FAILURE,
		      0,
		      "ERROR: Expected DM7820_General_Close_Board() failure but success"
		      " occurred");
	}

	if (errno != ENODATA) {
		cleanup();
		error(EXIT_FAILURE,
		      0,
		      "ERROR: Expected DM7820_General_Close_Board() errno %d, got %d",
		      ENODATA, errno);
	}

	/*#########################################################################
	   Verify that each device file can be opened once but not twice
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_General_Open_Board() ...\n");

	for (minor_number = 0; minor_number < device_count; minor_number++) {
		fprintf(stdout, "    Minor number %lu\n", minor_number);

		fprintf(stdout, "        First device open ...\n");

		boards[minor_number] = NULL;

		status =
		    DM7820_General_Open_Board(minor_number,
					      (DM7820_Board_Descriptor **) &
					      (boards[minor_number])
		    );
		if (status == -1) {
			cleanup();
			error(EXIT_FAILURE, errno,
			      "ERROR: DM7820_General_Open_Board() FAILED");
		}

		init_state |= INIT_BOARD_ARRAY_OPENED;

		fprintf(stdout, "        Second device open ...\n");

		status =
		    DM7820_General_Open_Board(minor_number,
					      (DM7820_Board_Descriptor **) &
					      (boards[minor_number])
		    );
		if (status != -1) {
			cleanup();
			error(EXIT_FAILURE,
			      0,
			      "ERROR: DM7820_General_Open_Board() single access violated");
		}

		if (errno != EBUSY) {
			cleanup();
			error(EXIT_FAILURE,
			      0,
			      "ERROR: DM7820_General_Open_Board() set errno incorrectly");
		}
	}

	/*#########################################################################
	   Verify that each device file can be closed
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_General_Close_Board() ...\n");

	for (minor_number = 0; minor_number < device_count; minor_number++) {
		fprintf(stdout, "    Minor number %lu\n", minor_number);

		if (DM7820_General_Close_Board((DM7820_Board_Descriptor
						*) (boards[minor_number])
		    )
		    == -1) {
			cleanup();
			error(EXIT_FAILURE,
			      errno,
			      "ERROR: DM7820_General_Close_Board() FAILED");
		}

		boards[minor_number] = NULL;
	}

	init_state &= ~INIT_BOARD_ARRAY_OPENED;

	fprintf(stdout, "    Freeing file descriptor array ...\n");

	free(boards);

	init_state &= ~INIT_BOARD_ARRAY_ALLOCATED;

	/*#########################################################################
	   Test DM7820_AdvInt_Get_Status() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_AdvInt_Get_Status() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid interrupt ...\n");

	status =
	    DM7820_AdvInt_Get_Status(board, (DM7820_ADVINT_INTERRUPT_1 + 1),
				     &interrupt_status);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid interrupts ...\n");

	for (interrupt = DM7820_ADVINT_INTERRUPT_0;
	     interrupt <= DM7820_ADVINT_INTERRUPT_1; interrupt++) {
		fprintf(stdout, "        Interrupt %u\n", interrupt);

		status =
		    DM7820_AdvInt_Get_Status(board, interrupt,
					     &interrupt_status);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_AdvInt_Read_Capture() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_AdvInt_Read_Capture() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid interrupt ...\n");

	status = DM7820_AdvInt_Read_Capture(board,
					    (DM7820_ADVINT_INTERRUPT_1 + 1),
					    DM7820_STDIO_PORT_0,
					    &capture_value);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid port ...\n");

	status = DM7820_AdvInt_Read_Capture(board,
					    DM7820_ADVINT_INTERRUPT_0,
					    (DM7820_STDIO_PORT_2 + 1),
					    &capture_value);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid interrupts and ports ...\n");

	for (interrupt = DM7820_ADVINT_INTERRUPT_0;
	     interrupt <= DM7820_ADVINT_INTERRUPT_1; interrupt++) {
		fprintf(stdout, "        Interrupt %u\n", interrupt);

		for (stdio_port = DM7820_STDIO_PORT_0;
		     stdio_port <= DM7820_STDIO_PORT_2; stdio_port++) {
			fprintf(stdout, "            Port %u\n", stdio_port);

			status =
			    DM7820_AdvInt_Read_Capture(board, interrupt,
						       stdio_port,
						       &capture_value);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_AdvInt_Set_Compare() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_AdvInt_Set_Compare() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid interrupt ...\n");

	status = DM7820_AdvInt_Set_Compare(board,
					   (DM7820_ADVINT_INTERRUPT_1 + 1),
					   DM7820_STDIO_PORT_0, 0x0000);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid port ...\n");

	status = DM7820_AdvInt_Set_Compare(board,
					   DM7820_ADVINT_INTERRUPT_0,
					   (DM7820_STDIO_PORT_2 + 1), 0x0000);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid interrupts and ports ...\n");

	for (interrupt = DM7820_ADVINT_INTERRUPT_0;
	     interrupt <= DM7820_ADVINT_INTERRUPT_1; interrupt++) {
		fprintf(stdout, "        Interrupt %u\n", interrupt);

		for (stdio_port = DM7820_STDIO_PORT_0;
		     stdio_port <= DM7820_STDIO_PORT_2; stdio_port++) {
			fprintf(stdout, "            Port %u\n", stdio_port);

			status =
			    DM7820_AdvInt_Set_Compare(board, interrupt,
						      stdio_port, 0x0000);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_AdvInt_Set_Mask() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_AdvInt_Set_Mask() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid interrupt ...\n");

	status = DM7820_AdvInt_Set_Mask(board,
					(DM7820_ADVINT_INTERRUPT_1 + 1),
					DM7820_STDIO_PORT_0, 0xFFFF);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid port ...\n");

	status = DM7820_AdvInt_Set_Mask(board,
					DM7820_ADVINT_INTERRUPT_0,
					(DM7820_STDIO_PORT_2 + 1), 0xFFFF);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid interrupts and ports ...\n");

	for (interrupt = DM7820_ADVINT_INTERRUPT_0;
	     interrupt <= DM7820_ADVINT_INTERRUPT_1; interrupt++) {
		fprintf(stdout, "        Interrupt %u\n", interrupt);

		for (stdio_port = DM7820_STDIO_PORT_0;
		     stdio_port <= DM7820_STDIO_PORT_2; stdio_port++) {
			fprintf(stdout, "            Port %u\n", stdio_port);

			status =
			    DM7820_AdvInt_Set_Mask(board, interrupt, stdio_port,
						   0xFFFF);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_AdvInt_Set_Master() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_AdvInt_Set_Master() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid interrupt ...\n");

	status = DM7820_AdvInt_Set_Master(board,
					  (DM7820_ADVINT_INTERRUPT_1 + 1),
					  DM7820_ADVINT_MASTER_25_MHZ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid master clock ...\n");

	status = DM7820_AdvInt_Set_Master(board,
					  DM7820_ADVINT_INTERRUPT_0,
					  (DM7820_ADVINT_MASTER_INV_STROBE_2 +
					   1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid interrupts and master clocks ...\n");

	for (interrupt = DM7820_ADVINT_INTERRUPT_0;
	     interrupt <= DM7820_ADVINT_INTERRUPT_1; interrupt++) {
		dm7820_advint_master_clock master;

		fprintf(stdout, "        Interrupt %u\n", interrupt);

		for (master = DM7820_ADVINT_MASTER_25_MHZ;
		     master <= DM7820_ADVINT_MASTER_INV_STROBE_2; master++) {
			fprintf(stdout, "            Master clock %u\n",
				master);

			status =
			    DM7820_AdvInt_Set_Master(board, interrupt, master);

			if (master != DM7820_ADVINT_MASTER_RESERVED) {
				expect_success(status);
			} else {
				expect_failure_and_check(status, EOPNOTSUPP);
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_AdvInt_Set_Mode() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_AdvInt_Set_Mode() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid interrupt ...\n");

	status = DM7820_AdvInt_Set_Mode(board,
					(DM7820_ADVINT_INTERRUPT_1 + 1),
					DM7820_ADVINT_MODE_DISABLED);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid mode ...\n");

	status = DM7820_AdvInt_Set_Mode(board,
					DM7820_ADVINT_INTERRUPT_0,
					(DM7820_ADVINT_MODE_EVENT + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid interrupts and modes ...\n");

	for (interrupt = DM7820_ADVINT_INTERRUPT_0;
	     interrupt <= DM7820_ADVINT_INTERRUPT_1; interrupt++) {
		dm7820_advint_mode mode;

		fprintf(stdout, "        Interrupt %u\n", interrupt);

		for (mode = DM7820_ADVINT_MODE_DISABLED;
		     mode <= DM7820_ADVINT_MODE_EVENT; mode++) {
			fprintf(stdout, "            Mode %u\n", mode);

			status = DM7820_AdvInt_Set_Mode(board, interrupt, mode);
			expect_success(status);
		}

		/*
		 * Set mode back to disabled because we are not ready for any interrupts
		 */

		status =
		    DM7820_AdvInt_Set_Mode(board, interrupt,
					   DM7820_ADVINT_MODE_DISABLED);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Verify that DM7820_General_Get_Version_Info() succeeds
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_General_Get_Version_Info() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    Reading version information ...\n");

	status =
	    DM7820_General_Get_Version_Info(board, &fpga_type_id, &fpga_version,
					    &svn_version);
	expect_success(status);

	fprintf(stdout, "        FPGA type ID:    0x%02x", fpga_type_id);
	if (fpga_type_id == 0x10) {
		fprintf(stdout, " (Standard FPGA)");
	}
	fprintf(stdout, "\n");
	fprintf(stdout, "        FPGA version ID: 0x%02x\n", fpga_version);
	fprintf(stdout, "        SVN version ID:  0x%04x\n", svn_version);

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Verify that DM7820_General_Is_PCI_Master() succeeds
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_General_Is_PCI_Master() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    Reading PCI master capable status ...\n");

	status = DM7820_General_Is_PCI_Master(board, &pci_master);
	expect_success(status);

	fprintf(stdout, "        ");

	if (!pci_master) {
		fprintf(stdout, "Not ");
	}

	fprintf(stdout, "PCI master capable\n");

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_BRD_STAT;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(board->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	expect_success(status);

	/*
	 * Verify flag returned against Board Status Register
	 */

	fprintf(stdout, "    Verify PCI master capable status ...\n");

	if (!(ioctl_request.readwrite.access.data.data16 & 0x0001)) {
		if (!pci_master) {
			error(EXIT_FAILURE, 0,
			      "ERROR: PCI master status disagrees");
		}
	} else {
		if (pci_master) {
			error(EXIT_FAILURE, 0,
			      "ERROR: PCI master status disagrees");
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Verify that DM7820_General_Reset() actually resets the board
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_General_Reset() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	/*
	 * Write a bit pattern to Port 0 Output Register
	 */

	fprintf(stdout, "    Write to Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0xF1A7;

	status =
	    ioctl(board->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		  &ioctl_request);
	expect_success(status);

	/*
	 * Verify value written to Port 0 Output Register
	 */

	fprintf(stdout, "    Read from Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0x0000;

	status =
	    ioctl(board->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	expect_success(status);

	if (ioctl_request.readwrite.access.data.data16 != 0xF1A7) {
		error(EXIT_FAILURE, 0, "ERROR: Invalid value read");
	}

	/*
	 * Reset the board
	 */

	fprintf(stdout, "    Reset board ...\n");

	status = DM7820_General_Reset(board);
	expect_success(status);

	/*
	 * Verify that board reset occurred
	 */

	fprintf(stdout, "    Read from Port 0 Output Register ...\n");

	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_PORT0_OUTPUT;
	ioctl_request.readwrite.access.data.data16 = 0xFFFF;

	status =
	    ioctl(board->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	expect_success(status);

	fprintf(stdout, "    Verify board reset ...\n");

	if (ioctl_request.readwrite.access.data.data16 != 0x0000) {
		error(EXIT_FAILURE, 0,
		      "ERROR: Board appears not to have been reset");
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_Enable() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_FIFO_Enable() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status = DM7820_FIFO_Enable(board, (DM7820_FIFO_QUEUE_1 + 1), 0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid FIFOs ...\n");

	for (fifo = DM7820_FIFO_QUEUE_0; fifo <= DM7820_FIFO_QUEUE_1; fifo++) {
		fprintf(stdout, "        FIFO %u\n", fifo);

		status = DM7820_FIFO_Enable(board, fifo, 0x00);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_Get_Status() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_FIFO_Get_Status() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status =
	    DM7820_FIFO_Get_Status(board, (DM7820_FIFO_QUEUE_1 + 1),
				   DM7820_FIFO_STATUS_EMPTY, &fifo_status);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid status condition ...\n");

	status = DM7820_FIFO_Get_Status(board,
					DM7820_FIFO_QUEUE_0,
					(DM7820_FIFO_STATUS_UNDERFLOW + 1),
					&fifo_status);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid FIFOs and status conditions ...\n");

	for (fifo = DM7820_FIFO_QUEUE_0; fifo <= DM7820_FIFO_QUEUE_1; fifo++) {
		dm7820_fifo_status_condition condition;

		fprintf(stdout, "        FIFO %u\n", fifo);

		for (condition = DM7820_FIFO_STATUS_READ_REQUEST;
		     condition <= DM7820_FIFO_STATUS_UNDERFLOW; condition++) {
			fprintf(stdout, "            Status condition %u\n",
				condition);

			status =
			    DM7820_FIFO_Get_Status(board, fifo, condition,
						   &fifo_status);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_DMA_Initialize() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_FIFO_DMA_Initialize() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status =
	    DM7820_FIFO_DMA_Initialize(board, (DM7820_FIFO_QUEUE_1 + 1), 1, 2);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On zero buffer count ...\n");

	status = DM7820_FIFO_DMA_Initialize(board, DM7820_FIFO_QUEUE_0, 0, 2);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On buffer count exceeding maximum ...\n");

	status =
	    DM7820_FIFO_DMA_Initialize(board, DM7820_FIFO_QUEUE_0,
				       (DM7820_MAX_DMA_BUFFER_COUNT + 1), 2);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On zero number of values ...\n");

	status = DM7820_FIFO_DMA_Initialize(board, DM7820_FIFO_QUEUE_0, 1, 0);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On number of values exceeding maximum ...\n");

	status =
	    DM7820_FIFO_DMA_Initialize(board, DM7820_FIFO_QUEUE_0, 1,
				       ((DM7820_MAX_DMA_BUFFER_SIZE / 2) + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_DMA_Configure() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_FIFO_DMA_Configure() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status =
	    DM7820_FIFO_DMA_Configure(board,
				      (DM7820_FIFO_QUEUE_1 + 1),
				      DM7820_DMA_DEMAND_ON_PCI_TO_DM7820,
				      DM7820_MAX_DMA_BUFFER_SIZE);
	fprintf(stdout, "\n\n%d\n\n", status);

	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid direction ...\n");

	status =
	    DM7820_FIFO_DMA_Configure(board,
				      DM7820_FIFO_QUEUE_0,
				      -1, DM7820_MAX_DMA_BUFFER_SIZE);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid buffer size ...\n");

	status =
	    DM7820_FIFO_DMA_Configure(board,
				      DM7820_FIFO_QUEUE_0,
				      -1, DM7820_MAX_DMA_BUFFER_SIZE - 1);
	expect_failure_and_check(status, EINVAL);

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_Read() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_FIFO_Read() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status = DM7820_FIFO_Read(board, (DM7820_FIFO_QUEUE_1 + 1), &fifo_data);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid FIFOs ...\n");

	for (fifo = DM7820_FIFO_QUEUE_0; fifo <= DM7820_FIFO_QUEUE_1; fifo++) {
		fprintf(stdout, "        FIFO %u\n", fifo);

		status = DM7820_FIFO_Read(board, fifo, &fifo_data);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_Set_DMA_Request() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_FIFO_Set_DMA_Request() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status =
	    DM7820_FIFO_Set_DMA_Request(board, (DM7820_FIFO_QUEUE_1 + 1),
					DM7820_FIFO_DMA_REQUEST_READ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid DMA request source ...\n");

	status =
	    DM7820_FIFO_Set_DMA_Request(board, DM7820_FIFO_QUEUE_0,
					(DM7820_FIFO_DMA_REQUEST_NOT_FULL + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid FIFOs and DMA request sources ...\n");

	for (fifo = DM7820_FIFO_QUEUE_0; fifo <= DM7820_FIFO_QUEUE_1; fifo++) {
		dm7820_fifo_dma_request source;

		fprintf(stdout, "        FIFO %u\n", fifo);

		for (source = DM7820_FIFO_DMA_REQUEST_READ;
		     source <= DM7820_FIFO_DMA_REQUEST_NOT_FULL; source++) {
			fprintf(stdout, "            Source %u\n", source);

			status =
			    DM7820_FIFO_Set_DMA_Request(board, fifo, source);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_Set_Data_Input() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_FIFO_Set_Data_Input() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status =
	    DM7820_FIFO_Set_Data_Input(board, (DM7820_FIFO_QUEUE_1 + 1),
				       DM7820_FIFO_0_DATA_INPUT_PCI_DATA);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid data input ...\n");

	status = DM7820_FIFO_Set_Data_Input(board,
					    DM7820_FIFO_QUEUE_0,
					    (DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_B
					     + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid FIFOs and data inputs ...\n");

	for (fifo = DM7820_FIFO_QUEUE_0; fifo <= DM7820_FIFO_QUEUE_1; fifo++) {
		dm7820_fifo_data_input input;

		fprintf(stdout, "        FIFO %u\n", fifo);

		for (input = DM7820_FIFO_0_DATA_INPUT_PCI_DATA;
		     input <= DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_B;
		     input++) {
			fprintf(stdout, "            Data input %u\n", input);

			status = DM7820_FIFO_Set_Data_Input(board, fifo, input);

			switch (fifo) {
			case DM7820_FIFO_QUEUE_0:
				if (input < DM7820_FIFO_1_DATA_INPUT_PCI_DATA) {
					expect_success(status);
				} else {
					expect_failure_and_check(status,
								 EOPNOTSUPP);
				}

				break;

			case DM7820_FIFO_QUEUE_1:
				if (input < DM7820_FIFO_1_DATA_INPUT_PCI_DATA) {
					expect_failure_and_check(status,
								 EOPNOTSUPP);
				} else {
					expect_success(status);
				}

				break;
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_Set_Input_Clock() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_FIFO_Set_Input_Clock() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status =
	    DM7820_FIFO_Set_Input_Clock(board, (DM7820_FIFO_QUEUE_1 + 1),
					DM7820_FIFO_INPUT_CLOCK_25_MHZ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid input clock ...\n");

	status =
	    DM7820_FIFO_Set_Input_Clock(board, DM7820_FIFO_QUEUE_0,
					(DM7820_FIFO_INPUT_CLOCK_PCI_WRITE + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid FIFOs and input clocks ...\n");

	for (fifo = DM7820_FIFO_QUEUE_0; fifo <= DM7820_FIFO_QUEUE_1; fifo++) {
		dm7820_fifo_input_clock clock;

		fprintf(stdout, "        FIFO %u\n", fifo);

		for (clock = DM7820_FIFO_INPUT_CLOCK_25_MHZ;
		     clock <= DM7820_FIFO_INPUT_CLOCK_PCI_WRITE; clock++) {
			fprintf(stdout, "            Input clock %u\n", clock);

			status =
			    DM7820_FIFO_Set_Input_Clock(board, fifo, clock);

			switch (clock) {
			case DM7820_FIFO_INPUT_CLOCK_RESERVED_1:
			case DM7820_FIFO_INPUT_CLOCK_RESERVED_2:
			case DM7820_FIFO_INPUT_CLOCK_RESERVED_3:
			case DM7820_FIFO_INPUT_CLOCK_RESERVED_4:
				expect_failure_and_check(status, EOPNOTSUPP);
				break;

			default:
				expect_success(status);
				break;
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_Set_Output_Clock() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_FIFO_Set_Output_Clock() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status =
	    DM7820_FIFO_Set_Output_Clock(board, (DM7820_FIFO_QUEUE_1 + 1),
					 DM7820_FIFO_OUTPUT_CLOCK_25_MHZ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid output clock ...\n");

	status =
	    DM7820_FIFO_Set_Output_Clock(board, DM7820_FIFO_QUEUE_0,
					 (DM7820_FIFO_OUTPUT_CLOCK_PCI_WRITE +
					  1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid FIFOs and output clocks ...\n");

	for (fifo = DM7820_FIFO_QUEUE_0; fifo <= DM7820_FIFO_QUEUE_1; fifo++) {
		dm7820_fifo_output_clock clock;

		fprintf(stdout, "        FIFO %u\n", fifo);

		for (clock = DM7820_FIFO_OUTPUT_CLOCK_25_MHZ;
		     clock <= DM7820_FIFO_OUTPUT_CLOCK_PCI_WRITE; clock++) {
			fprintf(stdout, "            Output clock %u\n", clock);

			status =
			    DM7820_FIFO_Set_Output_Clock(board, fifo, clock);

			switch (clock) {
			case DM7820_FIFO_OUTPUT_CLOCK_RESERVED_1:
			case DM7820_FIFO_OUTPUT_CLOCK_RESERVED_2:
			case DM7820_FIFO_OUTPUT_CLOCK_RESERVED_3:
			case DM7820_FIFO_OUTPUT_CLOCK_RESERVED_4:
				expect_failure_and_check(status, EOPNOTSUPP);
				break;

			default:
				expect_success(status);
				break;
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_FIFO_Write() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_FIFO_Write() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid FIFO ...\n");

	status = DM7820_FIFO_Write(board, (DM7820_FIFO_QUEUE_1 + 1), 0x1234);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid FIFOs ...\n");

	for (fifo = DM7820_FIFO_QUEUE_0; fifo <= DM7820_FIFO_QUEUE_1; fifo++) {
		fprintf(stdout, "        FIFO %u\n", fifo);

		status = DM7820_FIFO_Write(board, fifo, 0x8000);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_General_Enable_Interrupt() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_General_Enable_Interrupt() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid interrupt source ...\n");

	status =
	    DM7820_General_Enable_Interrupt(board,
					    (DM7820_INTERRUPT_FIFO_1_DMA_DONE +
					     1), 0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid interrupt sources ...\n");

	for (source = DM7820_INTERRUPT_ADVINT_0;
	     source <= DM7820_INTERRUPT_FIFO_1_DMA_DONE; source++) {
		fprintf(stdout, "        Interrupt source %u\n", source);

		status = DM7820_General_Enable_Interrupt(board, source, 0x00);

		if ((source != DM7820_INTERRUPT_FIFO_0_DMA_DONE)
		    && (source != DM7820_INTERRUPT_FIFO_1_DMA_DONE)
		    ) {
			expect_success(status);
		} else {
			expect_failure_and_check(status, EOPNOTSUPP);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Verify that DM7820_General_Get_Interrupt_Status() succeeds
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_General_Get_Interrupt_Status() ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	status = DM7820_General_Get_Interrupt_Status(board, &int_status, 0x00);
	expect_success(status);

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Configure() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Configure() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status = DM7820_IncEnc_Configure(board,
					 (DM7820_INCENC_ENCODER_1 + 1),
					 0x0000,
					 DM7820_INCENC_INPUT_SINGLE_ENDED,
					 0x00,
					 DM7820_INCENC_CHANNEL_INDEPENDENT,
					 0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid input mode ...\n");

	status = DM7820_IncEnc_Configure(board,
					 DM7820_INCENC_ENCODER_0,
					 0x0000,
					 (DM7820_INCENC_INPUT_DIFFERENTIAL + 1),
					 0x00,
					 DM7820_INCENC_CHANNEL_INDEPENDENT,
					 0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid channel mode ...\n");

	status = DM7820_IncEnc_Configure(board,
					 DM7820_INCENC_ENCODER_0,
					 0x0000,
					 DM7820_INCENC_INPUT_SINGLE_ENDED,
					 0x00,
					 (DM7820_INCENC_CHANNEL_JOINED + 1),
					 0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout,
		"    On all valid encoders, input modes, and channel modes ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		dm7820_incenc_input_mode input_mode;

		fprintf(stdout, "        Encoder %u\n", encoder);

		for (input_mode = DM7820_INCENC_INPUT_SINGLE_ENDED;
		     input_mode <= DM7820_INCENC_INPUT_DIFFERENTIAL;
		     input_mode++) {
			dm7820_incenc_channel_mode channel_mode;

			fprintf(stdout, "            Input mode %u\n",
				input_mode);

			for (channel_mode = DM7820_INCENC_CHANNEL_INDEPENDENT;
			     channel_mode <= DM7820_INCENC_CHANNEL_JOINED;
			     channel_mode++) {
				fprintf(stdout,
					"                Channel mode %u\n",
					channel_mode);

				status =
				    DM7820_IncEnc_Configure(board, encoder,
							    0x0000, input_mode,
							    0x00, channel_mode,
							    0x00);
				expect_success(status);
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Enable() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Enable() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status =
	    DM7820_IncEnc_Enable(board, (DM7820_INCENC_ENCODER_1 + 1), 0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid encoders ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		fprintf(stdout, "        Encoder %u\n", encoder);

		status = DM7820_IncEnc_Enable(board, encoder, 0x00);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Enable_Hold() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Enable_Hold() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status =
	    DM7820_IncEnc_Enable_Hold(board, (DM7820_INCENC_ENCODER_1 + 1),
				      0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid encoders ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		fprintf(stdout, "        Encoder %u\n", encoder);

		status = DM7820_IncEnc_Enable_Hold(board, encoder, 0x00);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Get_Independent_Value() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Get_Independent_Value() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status = DM7820_IncEnc_Get_Independent_Value(board,
						     (DM7820_INCENC_ENCODER_1 +
						      1),
						     DM7820_INCENC_CHANNEL_A,
						     &independent_value);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid channel ...\n");

	status = DM7820_IncEnc_Get_Independent_Value(board,
						     DM7820_INCENC_ENCODER_0,
						     (DM7820_INCENC_CHANNEL_B +
						      1), &independent_value);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On joined channels ...\n");

	status = DM7820_IncEnc_Configure(board,
					 DM7820_INCENC_ENCODER_0,
					 0x0000,
					 DM7820_INCENC_INPUT_SINGLE_ENDED,
					 0x00,
					 DM7820_INCENC_CHANNEL_JOINED, 0x00);
	expect_success(status);

	status = DM7820_IncEnc_Get_Independent_Value(board,
						     DM7820_INCENC_ENCODER_0,
						     DM7820_INCENC_CHANNEL_A,
						     &independent_value);
	expect_failure_and_check(status, EOPNOTSUPP);

	fprintf(stdout,
		"    On all valid encoders and independent channels ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		dm7820_incenc_channel channel;

		fprintf(stdout, "        Encoder %u\n", encoder);

		status = DM7820_IncEnc_Configure(board,
						 encoder,
						 0x0000,
						 DM7820_INCENC_INPUT_SINGLE_ENDED,
						 0x00,
						 DM7820_INCENC_CHANNEL_INDEPENDENT,
						 0x00);
		expect_success(status);

		for (channel = DM7820_INCENC_CHANNEL_A;
		     channel <= DM7820_INCENC_CHANNEL_B; channel++) {
			fprintf(stdout, "            Channel %u\n", channel);

			status =
			    DM7820_IncEnc_Get_Independent_Value(board, encoder,
								channel,
								&independent_value);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Get_Joined_Value() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Get_Joined_Value() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status = DM7820_IncEnc_Get_Joined_Value(board,
						(DM7820_INCENC_ENCODER_1 + 1),
						&joined_value);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On independent channels ...\n");

	status = DM7820_IncEnc_Configure(board,
					 DM7820_INCENC_ENCODER_0,
					 0x0000,
					 DM7820_INCENC_INPUT_SINGLE_ENDED,
					 0x00,
					 DM7820_INCENC_CHANNEL_INDEPENDENT,
					 0x00);
	expect_success(status);

	status = DM7820_IncEnc_Get_Joined_Value(board,
						DM7820_INCENC_ENCODER_0,
						&joined_value);
	expect_failure_and_check(status, EOPNOTSUPP);

	fprintf(stdout, "    On all valid encoders with joined channels ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		fprintf(stdout, "        Encoder %u\n", encoder);

		status = DM7820_IncEnc_Configure(board,
						 encoder,
						 0x0000,
						 DM7820_INCENC_INPUT_SINGLE_ENDED,
						 0x00,
						 DM7820_INCENC_CHANNEL_JOINED,
						 0x00);
		expect_success(status);

		status =
		    DM7820_IncEnc_Get_Joined_Value(board, encoder,
						   &joined_value);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Get_Status() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Get_Status() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status = DM7820_IncEnc_Get_Status(board,
					  (DM7820_INCENC_ENCODER_1 + 1),
					  DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER,
					  &encoder_status);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid status condition ...\n");

	status = DM7820_IncEnc_Get_Status(board,
					  DM7820_INCENC_ENCODER_0,
					  (DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER
					   + 1), &encoder_status);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout,
		"    On all valid encoders and status conditions ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		dm7820_incenc_status_condition condition;

		fprintf(stdout, "        Encoder %u\n", encoder);

		for (condition =
		     DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER;
		     condition <=
		     DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER;
		     condition++) {
			fprintf(stdout, "            Status condition %u\n",
				condition);

			status =
			    DM7820_IncEnc_Get_Status(board, encoder, condition,
						     &encoder_status);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Set_Independent_Value() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Set_Independent_Value() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status = DM7820_IncEnc_Set_Independent_Value(board,
						     (DM7820_INCENC_ENCODER_1 +
						      1),
						     DM7820_INCENC_CHANNEL_A,
						     0x0000);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid channel ...\n");

	status = DM7820_IncEnc_Set_Independent_Value(board,
						     DM7820_INCENC_ENCODER_0,
						     (DM7820_INCENC_CHANNEL_B +
						      1), 0x0000);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On joined channels ...\n");

	status = DM7820_IncEnc_Configure(board,
					 DM7820_INCENC_ENCODER_0,
					 0x0000,
					 DM7820_INCENC_INPUT_SINGLE_ENDED,
					 0x00,
					 DM7820_INCENC_CHANNEL_JOINED, 0x00);
	expect_success(status);

	status = DM7820_IncEnc_Set_Independent_Value(board,
						     DM7820_INCENC_ENCODER_0,
						     DM7820_INCENC_CHANNEL_A,
						     0x0000);
	expect_failure_and_check(status, EOPNOTSUPP);

	fprintf(stdout,
		"    On all valid encoders and independent channels ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		dm7820_incenc_channel channel;

		fprintf(stdout, "        Encoder %u\n", encoder);

		status = DM7820_IncEnc_Configure(board,
						 encoder,
						 0x0000,
						 DM7820_INCENC_INPUT_SINGLE_ENDED,
						 0x00,
						 DM7820_INCENC_CHANNEL_INDEPENDENT,
						 0x00);
		expect_success(status);

		for (channel = DM7820_INCENC_CHANNEL_A;
		     channel <= DM7820_INCENC_CHANNEL_B; channel++) {
			fprintf(stdout, "            Channel %u\n", channel);

			status =
			    DM7820_IncEnc_Set_Independent_Value(board, encoder,
								channel,
								0x0000);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Set_Joined_Value() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Set_Joined_Value() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status = DM7820_IncEnc_Set_Joined_Value(board,
						(DM7820_INCENC_ENCODER_1 + 1),
						0x00000000);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On independent channels ...\n");

	status = DM7820_IncEnc_Configure(board,
					 DM7820_INCENC_ENCODER_0,
					 0x0000,
					 DM7820_INCENC_INPUT_SINGLE_ENDED,
					 0x00,
					 DM7820_INCENC_CHANNEL_INDEPENDENT,
					 0x00);
	expect_success(status);

	status = DM7820_IncEnc_Set_Joined_Value(board,
						DM7820_INCENC_ENCODER_0,
						0x00000000);
	expect_failure_and_check(status, EOPNOTSUPP);

	fprintf(stdout, "    On all valid encoders with joined channels ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		fprintf(stdout, "        Encoder %u\n", encoder);

		status = DM7820_IncEnc_Configure(board,
						 encoder,
						 0x0000,
						 DM7820_INCENC_INPUT_SINGLE_ENDED,
						 0x00,
						 DM7820_INCENC_CHANNEL_JOINED,
						 0x00);
		expect_success(status);

		status =
		    DM7820_IncEnc_Set_Joined_Value(board, encoder, 0x00000000);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_IncEnc_Set_Master() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_IncEnc_Set_Master() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid encoder ...\n");

	status =
	    DM7820_IncEnc_Set_Master(board, (DM7820_INCENC_ENCODER_1 + 1),
				     DM7820_INCENC_MASTER_25_MHZ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid master clock ...\n");

	status =
	    DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_0,
				     (DM7820_INCENC_MASTER_INV_STROBE_2 + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid encoders and master clocks ...\n");

	for (encoder = DM7820_INCENC_ENCODER_0;
	     encoder <= DM7820_INCENC_ENCODER_1; encoder++) {
		dm7820_incenc_master_clock master;

		fprintf(stdout, "        Encoder %u\n", encoder);

		for (master = DM7820_INCENC_MASTER_25_MHZ;
		     master <= DM7820_INCENC_MASTER_INV_STROBE_2; master++) {
			fprintf(stdout, "            Master clock %u\n",
				master);

			status =
			    DM7820_IncEnc_Set_Master(board, encoder, master);

			if (master != DM7820_INCENC_MASTER_RESERVED) {
				expect_success(status);
			} else {
				expect_failure_and_check(status, EOPNOTSUPP);
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PWM_Enable() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_PWM_Enable() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid PWM ...\n");

	status = DM7820_PWM_Enable(board, (DM7820_PWM_MODULATOR_1 + 1), 0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid PWMs ...\n");

	for (pwm = DM7820_PWM_MODULATOR_0; pwm <= DM7820_PWM_MODULATOR_1; pwm++) {
		fprintf(stdout, "        PWM %u\n", pwm);

		status = DM7820_PWM_Enable(board, pwm, 0x00);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PWM_Set_Period() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PWM_Set_Period() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid PWM ...\n");

	status = DM7820_PWM_Set_Period(board, (DM7820_PWM_MODULATOR_1 + 1), 1);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On period of zero ...\n");

	status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, 0);
	expect_failure_and_check(status, ERANGE);

	fprintf(stdout, "    On period larger than 0x10000...\n");

	status = DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, 0x10001);
	expect_failure_and_check(status, ERANGE);

	fprintf(stdout, "    On all valid PWMs ...\n");

	for (pwm = DM7820_PWM_MODULATOR_0; pwm <= DM7820_PWM_MODULATOR_1; pwm++) {
		fprintf(stdout, "        PWM %u\n", pwm);

		status = DM7820_PWM_Set_Period(board, pwm, 0x10000);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PWM_Set_Period_Master() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PWM_Set_Period_Master() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid PWM ...\n");

	status =
	    DM7820_PWM_Set_Period_Master(board, (DM7820_PWM_MODULATOR_1 + 1),
					 DM7820_PWM_PERIOD_MASTER_25_MHZ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid period master clock ...\n");

	status = DM7820_PWM_Set_Period_Master(board,
					      DM7820_PWM_MODULATOR_0,
					      (DM7820_PWM_PERIOD_MASTER_INV_STROBE_2
					       + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid PWMs and period master clocks ...\n");

	for (pwm = DM7820_PWM_MODULATOR_0; pwm <= DM7820_PWM_MODULATOR_1; pwm++) {
		dm7820_pwm_period_master_clock master;

		fprintf(stdout, "        PWM %u\n", pwm);

		for (master = DM7820_PWM_PERIOD_MASTER_25_MHZ;
		     master <= DM7820_PWM_PERIOD_MASTER_INV_STROBE_2;
		     master++) {
			fprintf(stdout, "            Period master clock %u\n",
				master);

			status =
			    DM7820_PWM_Set_Period_Master(board, pwm, master);

			if (master != DM7820_PWM_PERIOD_MASTER_RESERVED) {
				expect_success(status);
			} else {
				expect_failure_and_check(status, EOPNOTSUPP);
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PWM_Set_Width() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PWM_Set_Width() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid PWM ...\n");

	status =
	    DM7820_PWM_Set_Width(board, (DM7820_PWM_MODULATOR_1 + 1),
				 DM7820_PWM_OUTPUT_A, 0x1);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid output ...\n");

	status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0,
				 (DM7820_PWM_OUTPUT_D + 1), 1);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid PWMs and outputs ...\n");

	for (pwm = DM7820_PWM_MODULATOR_0; pwm <= DM7820_PWM_MODULATOR_1; pwm++) {
		dm7820_pwm_output output;

		fprintf(stdout, "        PWM %u\n", pwm);

		for (output = DM7820_PWM_OUTPUT_A;
		     output <= DM7820_PWM_OUTPUT_D; output++) {
			fprintf(stdout, "            Output %u\n", output);

			status = DM7820_PWM_Set_Width(board, pwm, output, 1);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PWM_Set_Width_Master() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PWM_Set_Width_Master() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid PWM ...\n");

	status =
	    DM7820_PWM_Set_Width_Master(board, (DM7820_PWM_MODULATOR_1 + 1),
					DM7820_PWM_WIDTH_MASTER_25_MHZ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid width master clock ...\n");

	status = DM7820_PWM_Set_Width_Master(board,
					     DM7820_PWM_MODULATOR_0,
					     (DM7820_PWM_WIDTH_MASTER_INV_STROBE_2
					      + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid PWMs and width master clocks ...\n");

	for (pwm = DM7820_PWM_MODULATOR_0; pwm <= DM7820_PWM_MODULATOR_1; pwm++) {
		dm7820_pwm_width_master_clock master;

		fprintf(stdout, "        PWM %u\n", pwm);

		for (master = DM7820_PWM_WIDTH_MASTER_25_MHZ;
		     master <= DM7820_PWM_WIDTH_MASTER_INV_STROBE_2; master++) {
			fprintf(stdout, "            Width master clock %u\n",
				master);

			status =
			    DM7820_PWM_Set_Width_Master(board, pwm, master);

			if (master != DM7820_PWM_WIDTH_MASTER_RESERVED) {
				expect_success(status);
			} else {
				expect_failure_and_check(status, EOPNOTSUPP);
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PrgClk_Set_Master() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PrgClk_Set_Master() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid clock ...\n");

	status =
	    DM7820_PrgClk_Set_Master(board, (DM7820_PRGCLK_CLOCK_3 + 1),
				     DM7820_PRGCLK_MASTER_25_MHZ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid master clock ...\n");

	status =
	    DM7820_PrgClk_Set_Master(board, DM7820_PRGCLK_CLOCK_0,
				     (DM7820_PRGCLK_MASTER_INV_STROBE_2 + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid clocks and master clocks ...\n");

	for (clock = DM7820_PRGCLK_CLOCK_0;
	     clock <= DM7820_PRGCLK_CLOCK_3; clock++) {
		dm7820_prgclk_master_clock master;

		fprintf(stdout, "        Clock %u\n", clock);

		for (master = DM7820_PRGCLK_MASTER_25_MHZ;
		     master <= DM7820_PRGCLK_MASTER_INV_STROBE_2; master++) {
			fprintf(stdout, "             Master clock %u\n",
				master);

			status = DM7820_PrgClk_Set_Master(board, clock, master);

		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PrgClk_Set_Mode() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PrgClk_Set_Mode() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid clock ...\n");

	status =
	    DM7820_PrgClk_Set_Mode(board, (DM7820_PRGCLK_CLOCK_3 + 1),
				   DM7820_PRGCLK_MODE_CONTINUOUS);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid mode ...\n");

	status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
				   (DM7820_PRGCLK_MODE_ONE_SHOT + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid clocks and modes ...\n");

	for (clock = DM7820_PRGCLK_CLOCK_0;
	     clock <= DM7820_PRGCLK_CLOCK_3; clock++) {
		dm7820_prgclk_mode mode;

		fprintf(stdout, "        Clock %u\n", clock);

		for (mode = DM7820_PRGCLK_MODE_DISABLED;
		     mode <= DM7820_PRGCLK_MODE_ONE_SHOT; mode++) {
			fprintf(stdout, "            Mode %u\n", mode);

			status = DM7820_PrgClk_Set_Mode(board, clock, mode);

			if (mode != DM7820_PRGCLK_MODE_RESERVED) {
				expect_success(status);
			} else {
				expect_failure_and_check(status, EOPNOTSUPP);
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PrgClk_Set_Period() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PrgClk_Set_Period() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid clock ...\n");

	status =
	    DM7820_PrgClk_Set_Period(board, (DM7820_PRGCLK_CLOCK_3 + 1), 10);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On period of zero ...\n");

	status = DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 0);
	expect_failure_and_check(status, ERANGE);

	fprintf(stdout, "    On period larger than 0x10000...\n");

	status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 0x10001);
	expect_failure_and_check(status, ERANGE);

	fprintf(stdout, "    On all valid clocks ...\n");

	for (clock = DM7820_PRGCLK_CLOCK_0;
	     clock <= DM7820_PRGCLK_CLOCK_3; clock++) {
		fprintf(stdout, "        Clock %u\n", clock);

		status = DM7820_PrgClk_Set_Period(board, clock, 0x10000);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PrgClk_Set_Start_Trigger() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PrgClk_Set_Start_Trigger() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid clock ...\n");

	status =
	    DM7820_PrgClk_Set_Start_Trigger(board, (DM7820_PRGCLK_CLOCK_3 + 1),
					    DM7820_PRGCLK_START_IMMEDIATE);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid start trigger ...\n");

	status =
	    DM7820_PrgClk_Set_Start_Trigger(board, DM7820_PRGCLK_CLOCK_0,
					    (DM7820_PRGCLK_START_FIFO_1_INT + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid clocks and start triggers ...\n");

	for (clock = DM7820_PRGCLK_CLOCK_0;
	     clock <= DM7820_PRGCLK_CLOCK_3; clock++) {
		dm7820_prgclk_start_trigger start;

		fprintf(stdout, "        Clock %u\n", clock);

		for (start = DM7820_PRGCLK_START_IMMEDIATE;
		     start <= DM7820_PRGCLK_START_FIFO_1_INT; start++) {
			fprintf(stdout, "            Start trigger %u\n",
				start);

			status =
			    DM7820_PrgClk_Set_Start_Trigger(board, clock,
							    start);

			switch (start) {
			case DM7820_PRGCLK_START_RESERVED_1:
			case DM7820_PRGCLK_START_RESERVED_2:
			case DM7820_PRGCLK_START_RESERVED_3:
			case DM7820_PRGCLK_START_RESERVED_4:
				expect_failure_and_check(status, EOPNOTSUPP);
				break;

			default:
				expect_success(status);
				break;
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_PrgClk_Set_Stop_Trigger() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_PrgClk_Set_Stop_Trigger() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid clock ...\n");

	status =
	    DM7820_PrgClk_Set_Stop_Trigger(board, (DM7820_PRGCLK_CLOCK_3 + 1),
					   DM7820_PRGCLK_STOP_NONE);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid start trigger ...\n");

	status =
	    DM7820_PrgClk_Set_Stop_Trigger(board, DM7820_PRGCLK_CLOCK_0,
					   (DM7820_PRGCLK_STOP_FIFO_1_INT + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid clocks and stop triggers ...\n");

	for (clock = DM7820_PRGCLK_CLOCK_0;
	     clock <= DM7820_PRGCLK_CLOCK_3; clock++) {
		dm7820_prgclk_stop_trigger stop;

		fprintf(stdout, "        Clock %u\n", clock);

		for (stop = DM7820_PRGCLK_STOP_NONE;
		     stop <= DM7820_PRGCLK_STOP_FIFO_1_INT; stop++) {
			fprintf(stdout, "            Stop trigger %u\n", stop);

			status =
			    DM7820_PrgClk_Set_Stop_Trigger(board, clock, stop);

			switch (stop) {
			case DM7820_PRGCLK_STOP_RESERVED_1:
			case DM7820_PRGCLK_STOP_RESERVED_2:
			case DM7820_PRGCLK_STOP_RESERVED_3:
			case DM7820_PRGCLK_STOP_RESERVED_4:
				expect_failure_and_check(status, EOPNOTSUPP);
				break;

			default:
				expect_success(status);
				break;
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_StdIO_Get_Input() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_StdIO_Get_Input() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid I/O port ...\n");

	status =
	    DM7820_StdIO_Get_Input(board, (DM7820_STDIO_PORT_2 + 1),
				   &stdio_value);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid I/O ports ...\n");

	for (stdio_port = DM7820_STDIO_PORT_0;
	     stdio_port <= DM7820_STDIO_PORT_2; stdio_port++) {
		fprintf(stdout, "        Port %u\n", stdio_port);

		status =
		    DM7820_StdIO_Get_Input(board, stdio_port, &stdio_value);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_StdIO_Set_IO_Mode() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_StdIO_Set_IO_Mode() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid I/O port ...\n");

	status =
	    DM7820_StdIO_Set_IO_Mode(board, (DM7820_STDIO_PORT_2 + 1), 0xFFFF,
				     DM7820_STDIO_MODE_INPUT);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid mode ...\n");

	status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF,
				     (DM7820_STDIO_MODE_PER_OUT + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid I/O ports and modes ...\n");

	for (stdio_port = DM7820_STDIO_PORT_0;
	     stdio_port <= DM7820_STDIO_PORT_2; stdio_port++) {
		fprintf(stdout, "        Port %u\n", stdio_port);

		for (stdio_mode = DM7820_STDIO_MODE_INPUT;
		     stdio_mode <= DM7820_STDIO_MODE_PER_OUT; stdio_mode++) {
			fprintf(stdout, "            Mode %u\n", stdio_mode);

			status =
			    DM7820_StdIO_Set_IO_Mode(board, stdio_port, 0xFFFF,
						     DM7820_STDIO_MODE_INPUT);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_StdIO_Set_Output() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_StdIO_Set_Output() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid I/O port ...\n");

	status =
	    DM7820_StdIO_Set_Output(board, (DM7820_STDIO_PORT_2 + 1),
				    stdio_value);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid I/O ports ...\n");

	for (stdio_port = DM7820_STDIO_PORT_0;
	     stdio_port <= DM7820_STDIO_PORT_2; stdio_port++) {
		fprintf(stdout, "        Port %u\n", stdio_port);

		status =
		    DM7820_StdIO_Set_Output(board, stdio_port, stdio_value);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_StdIO_Set_Periph_Mode() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_StdIO_Set_Periph_Mode() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid I/O port ...\n");

	status =
	    DM7820_StdIO_Set_Periph_Mode(board, (DM7820_STDIO_PORT_2 + 1),
					 0xFFFF, DM7820_STDIO_PERIPH_FIFO_0);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid mode ...\n");

	status =
	    DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF,
					 (DM7820_STDIO_PERIPH_FIFO_1 + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout,
		"    On all valid I/O ports and peripheral modes ...\n");

	for (stdio_port = DM7820_STDIO_PORT_0;
	     stdio_port <= DM7820_STDIO_PORT_2; stdio_port++) {
		fprintf(stdout, "        Port %u\n", stdio_port);

		for (peripheral_mode = DM7820_STDIO_PERIPH_PWM;
		     peripheral_mode <= DM7820_STDIO_PERIPH_FIFO_1;
		     peripheral_mode++) {
			fprintf(stdout, "            Peripheral mode %u\n",
				peripheral_mode);

			status =
			    DM7820_StdIO_Set_Periph_Mode(board, stdio_port,
							 0xFFFF,
							 peripheral_mode);

			switch (stdio_port) {
			case DM7820_STDIO_PORT_0:
			case DM7820_STDIO_PORT_1:
				switch (peripheral_mode) {
				case DM7820_STDIO_PERIPH_PWM:
				case DM7820_STDIO_PERIPH_CLK_OTHER:
					expect_failure_and_check(status,
								 EOPNOTSUPP);
					break;

				case DM7820_STDIO_PERIPH_FIFO_0:
				case DM7820_STDIO_PERIPH_FIFO_1:
					expect_success(status);
					break;
				}
				break;

			case DM7820_STDIO_PORT_2:
				expect_success(status);
				break;
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_StdIO_Strobe_Input() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_StdIO_Strobe_Input() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid strobe signal ...\n");

	status =
	    DM7820_StdIO_Strobe_Input(board, (DM7820_STDIO_STROBE_2 + 1),
				      &strobe_state);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid strobe signals ...\n");

	for (strobe_signal = DM7820_STDIO_STROBE_1;
	     strobe_signal <= DM7820_STDIO_STROBE_2; strobe_signal++) {
		fprintf(stdout, "        Strobe signal %u\n", strobe_signal);

		status =
		    DM7820_StdIO_Strobe_Input(board, strobe_signal,
					      &strobe_state);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_StdIO_Strobe_Mode() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_StdIO_Strobe_Mode() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid strobe signal ...\n");

	status =
	    DM7820_StdIO_Strobe_Mode(board, (DM7820_STDIO_STROBE_2 + 1), 0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid strobe signals ...\n");

	for (strobe_signal = DM7820_STDIO_STROBE_1;
	     strobe_signal <= DM7820_STDIO_STROBE_2; strobe_signal++) {
		fprintf(stdout, "        Strobe signal %u\n", strobe_signal);

		status = DM7820_StdIO_Strobe_Mode(board, strobe_signal, 0x00);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_StdIO_Strobe_Output() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_StdIO_Strobe_Output() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid strobe signal ...\n");

	status =
	    DM7820_StdIO_Strobe_Output(board, (DM7820_STDIO_STROBE_2 + 1),
				       0x00);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid strobe signals ...\n");

	for (strobe_signal = DM7820_STDIO_STROBE_1;
	     strobe_signal <= DM7820_STDIO_STROBE_2; strobe_signal++) {
		fprintf(stdout, "        Strobe signal %u\n", strobe_signal);

		status = DM7820_StdIO_Strobe_Output(board, strobe_signal, 0x00);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_TmrCtr_Get_Status() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_TmrCtr_Get_Status() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid timer ...\n");

	status =
	    DM7820_TmrCtr_Get_Status(board, (DM7820_TMRCTR_TIMER_B_2 + 1),
				     &timer_status);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid timers ...\n");

	for (timer = DM7820_TMRCTR_TIMER_A_0;
	     timer <= DM7820_TMRCTR_TIMER_B_2; timer++) {
		fprintf(stdout, "        Timer %u\n", timer);

		status = DM7820_TmrCtr_Get_Status(board, timer, &timer_status);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_TmrCtr_Program() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_TmrCtr_Program() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid timer ...\n");

	status = DM7820_TmrCtr_Program(board,
				       (DM7820_TMRCTR_TIMER_B_2 + 1),
				       DM7820_TMRCTR_WAVEFORM_EVENT_CTR,
				       DM7820_TMRCTR_COUNT_MODE_BINARY, 0x1000);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid waveform mode ...\n");

	status = DM7820_TmrCtr_Program(board,
				       DM7820_TMRCTR_TIMER_A_0,
				       (DM7820_TMRCTR_WAVEFORM_HARDWARE_STROBE +
					1), DM7820_TMRCTR_COUNT_MODE_BINARY,
				       0x1000);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid count mode ...\n");

	status = DM7820_TmrCtr_Program(board,
				       DM7820_TMRCTR_TIMER_A_0,
				       DM7820_TMRCTR_WAVEFORM_EVENT_CTR,
				       (DM7820_TMRCTR_COUNT_MODE_BCD + 1),
				       0x1000);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout,
		"    On all valid timers, waveforms, and count modes ...\n");

	for (timer = DM7820_TMRCTR_TIMER_A_0;
	     timer <= DM7820_TMRCTR_TIMER_B_2; timer++) {
		dm7820_tmrctr_waveform waveform;

		fprintf(stdout, "        Timer %u\n", timer);

		for (waveform = DM7820_TMRCTR_WAVEFORM_EVENT_CTR;
		     waveform <= DM7820_TMRCTR_WAVEFORM_HARDWARE_STROBE;
		     waveform++) {
			dm7820_tmrctr_count_mode count_mode;

			fprintf(stdout, "            Waveform %u\n", waveform);

			for (count_mode = DM7820_TMRCTR_COUNT_MODE_BINARY;
			     count_mode <= DM7820_TMRCTR_COUNT_MODE_BCD;
			     count_mode++) {
				fprintf(stdout,
					"                Count mode %u\n",
					count_mode);

				status =
				    DM7820_TmrCtr_Program(board, timer,
							  waveform, count_mode,
							  0x2000);
				expect_success(status);
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_TmrCtr_Read() error checking
	   ######################################################################### */

	fprintf(stdout, "## Testing DM7820_TmrCtr_Read() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid timer ...\n");

	status =
	    DM7820_TmrCtr_Read(board, (DM7820_TMRCTR_TIMER_B_2 + 1),
			       &timer_value);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid timers ...\n");

	for (timer = DM7820_TMRCTR_TIMER_A_0;
	     timer <= DM7820_TMRCTR_TIMER_B_2; timer++) {
		fprintf(stdout, "        Timer %u\n", timer);

		status = DM7820_TmrCtr_Read(board, timer, &timer_value);
		expect_success(status);
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_TmrCtr_Select_Clock() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_TmrCtr_Select_Clock() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid timer ...\n");

	status =
	    DM7820_TmrCtr_Select_Clock(board, (DM7820_TMRCTR_TIMER_B_2 + 1),
				       DM7820_TMRCTR_CLOCK_5_MHZ);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid clock input ...\n");

	status =
	    DM7820_TmrCtr_Select_Clock(board, DM7820_TMRCTR_TIMER_A_0,
				       (DM7820_TMRCTR_CLOCK_INV_STROBE_2 + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid timers and clock inputs ...\n");

	for (timer = DM7820_TMRCTR_TIMER_A_0;
	     timer <= DM7820_TMRCTR_TIMER_B_2; timer++) {
		dm7820_tmrctr_clock clock;

		fprintf(stdout, "        Timer %u\n", timer);

		for (clock = DM7820_TMRCTR_CLOCK_5_MHZ;
		     clock <= DM7820_TMRCTR_CLOCK_INV_STROBE_2; clock++) {
			fprintf(stdout, "            Clock input %u\n", clock);

			status =
			    DM7820_TmrCtr_Select_Clock(board, timer, clock);

			if (clock != DM7820_TMRCTR_CLOCK_RESERVED) {
				expect_success(status);
			} else {
				expect_failure_and_check(status, EOPNOTSUPP);
			}
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*#########################################################################
	   Test DM7820_TmrCtr_Select_Gate() error checking
	   ######################################################################### */

	fprintf(stdout,
		"## Testing DM7820_TmrCtr_Select_Gate() error checking ...\n");

	status = DM7820_General_Open_Board(0, &board);
	expect_success(status);

	fprintf(stdout, "    On invalid timer ...\n");

	status =
	    DM7820_TmrCtr_Select_Gate(board, (DM7820_TMRCTR_TIMER_B_2 + 1),
				      DM7820_TMRCTR_GATE_LOGIC_0);
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On invalid gate input ...\n");

	status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_0,
				      (DM7820_TMRCTR_GATE_PORT_2_BIT_15 + 1)
	    );
	expect_failure_and_check(status, EINVAL);

	fprintf(stdout, "    On all valid timers and gate inputs ...\n");

	for (timer = DM7820_TMRCTR_TIMER_A_0;
	     timer <= DM7820_TMRCTR_TIMER_B_2; timer++) {
		dm7820_tmrctr_gate gate;

		fprintf(stdout, "        Timer %u\n", timer);

		for (gate = DM7820_TMRCTR_GATE_LOGIC_0;
		     gate <= DM7820_TMRCTR_GATE_PORT_2_BIT_15; gate++) {
			fprintf(stdout, "            Gate input %u\n", gate);

			status = DM7820_TmrCtr_Select_Gate(board, timer, gate);
			expect_success(status);
		}
	}

	status = DM7820_General_Close_Board(board);
	expect_success(status);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "\n");
	fprintf(stdout, "SUCCESS.  All tests passed.\n");

	exit(EXIT_SUCCESS);
}
