/**
    @file

    @brief
        Example program which demonstrates how to use the FIFO block FIFOs with
        PCI read requests clocking data out of the FIFO and PCI write requests
        clocking data into the FIFO.

    @note
        This program shows how to access FIFOs at the most basic level.

    @note
        Because this program does not use interrupts, it also demonstrates how
        to check FIFO status when interrupts are not used.

    @warning
        This program takes a long time to complete.

    @verbatim
        This program uses FIFO 0, which is configured as follows: 1) input clock
        set to PCI write request, 2) output clock set to PCI read request, and
        3) data input set to PCI data.

        Values are written to FIFO 0 until the FIFO is full.  Then, values are
        read from FIFO 0 until the FIFO is empty.  While reading values, a check
        is made to see that the values are read back in the order they were
        written.
    @endverbatim

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

	$Id: fifo_pci_access.c 60252 2012-06-04 19:39:05Z rgroner $
*/

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "dm7820_library.h"

/*=============================================================================
Macros
 =============================================================================*/

/**
 * 2 megabyte 16-bit elements in SDRAM FIFO; one element is lost for hardware to
 * determine full status
 */

#define SDRAM_FIFO_ELEMENTS	((1024 * 1024 * 2) - 1)

/**
 * 256 16-bit elements in input FIFO;  one element is lost for hardware to
 * determine full status

 */

#define INPUT_FIFO_ELEMENTS	(256 - 1)

/**
 * 256 16-bit elements in output FIFO; one element is lost for hardware to
 * determine full status
 */

#define OUTPUT_FIFO_ELEMENTS	(256 - 1)

/**
 * Expected number of 16-bit FIFO elements
 */

#define EXPECTED_FIFO_ELEMENTS	( \
    SDRAM_FIFO_ELEMENTS + INPUT_FIFO_ELEMENTS + OUTPUT_FIFO_ELEMENTS \
)

/**
 * Divisor for modulus operator; used when writing data to FIFO that can be
 * easily recalculated when reading data from FIFO
 */

#define MODULUS_DIVISOR		4321

/*=============================================================================
Global variables
 =============================================================================*/

/**
 * Name of the program as invoked on the command line
 */

static char *program_name;

/*=============================================================================
Program code
 =============================================================================*/

/**
*******************************************************************************
@brief
    Determine whether or not the specified status condition has occurred for the
    given FIFO.

@param
    board

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to determine status of.

@param
    condition

    The status condition to check for.

@param
    status

    Address where occurrence flag should be stored.
 *******************************************************************************
 */

static void
get_fifo_status(DM7820_Board_Descriptor * board,
		dm7820_fifo_queue fifo,
		dm7820_fifo_status_condition condition, uint8_t * status)
{
	if (DM7820_FIFO_Get_Status(board, fifo, condition, status) == -1) {
		error(EXIT_FAILURE, errno,
		      "ERROR: DM7820_FIFO_Get_Status() FAILED");
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
	fprintf(stderr, "Usage: %s [--help] --minor MINOR\n", program_name);
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --help:      Display usage information and exit.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    --minor:     Use specified DM7820 device.\n");
	fprintf(stderr, "        MINOR:        Device minor number (>= 0).\n");
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
	DM7820_Error dm7820_status;
	int status;
	struct option options[] = {
		{"help", 0, 0, 1},
		{"minor", 1, 0, 2},
		{0, 0, 0, 0}
	};
	uint32_t values_read;
	uint32_t values_written;
	uint8_t fifo_empty;
	uint8_t fifo_full;
	uint8_t fifo_status;
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 FIFO PCI Request Example Program\n");
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
			   User entered '--minor'
			   ################################################################# */

		case 2:{
				char *invalid_char_p;

				/*
				 * Refuse to accept duplicate '--minor' options
				 */

				if (minor_option) {
					error(0, 0,
					      "ERROR: Duplicate option '--minor'");
					usage();
				}

				/*
				 * Convert option argument string to unsigned long integer
				 */

				errno = 0;

				minor_number =
				    strtoul(optarg, &invalid_char_p, 10);

				/*
				 * Catch unsigned long int overflow
				 */

				if ((minor_number == ULONG_MAX)
				    && (errno == ERANGE)) {
					error(0, 0,
					      "ERROR: Device minor number caused numeric overflow");
					usage();
				}

				/*
				 * Catch argument strings with valid decimal prefixes, for
				 * example "1q", and argument strings which cannot be converted,
				 * for example "abc1"
				 */

				if ((*invalid_char_p != '\0')
				    || (invalid_char_p == optarg)) {
					error(0, 0,
					      "ERROR: Non-decimal device minor number");
					usage();
				}

				/*
				 * '--minor' option has been seen
				 */

				minor_option = 0xFF;
				break;
			}

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

	/*
	 * '--minor' option must be given
	 */

	if (minor_option == 0x00) {
		error(0, 0, "ERROR: Option '--minor' is required");
		usage();
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Device initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Opening device with minor number %lu ...\n",
		minor_number);

	dm7820_status = DM7820_General_Open_Board(minor_number, &board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   FIFO generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable all FIFOs to put them into a known state
	 */

	fprintf(stdout, "Disabling FIFOs ...\n");

	fprintf(stdout, "    FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "    FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   FIFO 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing FIFO 0 ...\n");

	/*
	 * Set input clock to PCI write to FIFO 0 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting input clock ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Input_Clock(board, DM7820_FIFO_QUEUE_0,
					DM7820_FIFO_INPUT_CLOCK_PCI_WRITE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

	/*
	 * Set output clock to PCI read from FIFO 0 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting output clock ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Output_Clock(board, DM7820_FIFO_QUEUE_0,
					 DM7820_FIFO_OUTPUT_CLOCK_PCI_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*
	 * Set data input to PCI data
	 */

	fprintf(stdout, "    Setting data input ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Data_Input(board, DM7820_FIFO_QUEUE_0,
				       DM7820_FIFO_0_DATA_INPUT_PCI_DATA);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary FIFO 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*
	 * Clear FIFO status empty flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 0 status empty flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_EMPTY,
			&fifo_status);

	/*
	 * Clear FIFO status full flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 0 status full flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_FULL,
			&fifo_status);

	/*
	 * Clear FIFO status overflow flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 0 status overflow flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_OVERFLOW,
			&fifo_status);

	/*
	 * Clear FIFO status underflow flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 0 status underflow flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_0,
			DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Fill FIFO 0 by writing values to it via the PCI bus
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Filling FIFO 0 via PCI bus ...\n");

	/*#########################################################################
	   Write initialization
	   ######################################################################### */

	values_written = 0;

	/*#########################################################################
	   Determine initial FIFO status
	   ######################################################################### */

	/*
	 * FIFO should be empty
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_EMPTY,
			&fifo_status);

	if (!fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO is not empty");
	}

	/*
	 * FIFO should not have overflowed
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_OVERFLOW,
			&fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO has overflowed");
	}

	/*
	 * FIFO should not have underflowed
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0,
			DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO has underflowed");
	}

	/*
	 * Determine whether or not FIFO is full
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_FULL,
			&fifo_full);

	/*#########################################################################
	   Write values to FIFO
	   ######################################################################### */

	while (!fifo_full) {

		/*
		 * Write value to FIFO which can be easily recalculated on read so that
		 * values written can be verified
		 */

		dm7820_status =
		    DM7820_FIFO_Write(board, DM7820_FIFO_QUEUE_0,
				      (values_written % MODULUS_DIVISOR)
		    );
		DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Write()");

		/*
		 * Another value written successfully
		 */

		values_written++;

		/*
		 * Don't let program loop infinitely if something goes wrong
		 */

		if (values_written > EXPECTED_FIFO_ELEMENTS) {
			error(EXIT_FAILURE, 0,
			      "ERROR: Too many values written to FIFO");
		}

		/*
		 * Print progress information to inform user program is working
		 */

		fprintf(stdout, "    %u\r", values_written);

		/*
		 * FIFO should not be empty
		 */

		get_fifo_status(board, DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_EMPTY, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO is empty");
		}

		/*
		 * FIFO should not have overflowed
		 */

		get_fifo_status(board,
				DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_OVERFLOW, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO has overflowed");
		}

		/*
		 * FIFO should not have underflowed
		 */

		get_fifo_status(board,
				DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO has underflowed");
		}

		/*
		 * Determine whether or not FIFO is full
		 */

		get_fifo_status(board, DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_FULL, &fifo_full);
	}

	fprintf(stdout, "\n");

	/*
	 * Verify that expected number of values were written to FIFO
	 */

	fprintf(stdout, "%u values written to FIFO\n", values_written);

	if (values_written != EXPECTED_FIFO_ELEMENTS) {
		error(EXIT_FAILURE, 0,
		      "ERROR: Unexpected number of values written");
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Empty FIFO 0 by reading values from it via the PCI bus
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Emptying FIFO 0 via PCI bus ...\n");

	/*#########################################################################
	   Read initialization
	   ######################################################################### */

	values_read = 0;

	/*#########################################################################
	   Determine FIFO status
	   ######################################################################### */

	/*
	 * FIFO should be full
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_FULL,
			&fifo_status);

	if (!fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO is not full");
	}

	/*
	 * FIFO should not have overflowed
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_OVERFLOW,
			&fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO has overflowed");
	}

	/*
	 * FIFO should not have underflowed
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0,
			DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO has underflowed");
	}

	/*
	 * Determine whether or not FIFO is empty
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_EMPTY,
			&fifo_empty);

	/*#########################################################################
	   Read values from FIFO
	   ######################################################################### */

	while (!fifo_empty) {
		uint16_t fifo_value;

		/*
		 * Read value from FIFO and verify that it matches the value written
		 */

		dm7820_status =
		    DM7820_FIFO_Read(board, DM7820_FIFO_QUEUE_0, &fifo_value);
		DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Read()");

		if (fifo_value != (values_read % MODULUS_DIVISOR)) {
			error(EXIT_FAILURE, 0,
			      "ERROR: Incorrect value read from FIFO");
		}

		/*
		 * Another value read successfully
		 */

		values_read++;

		/*
		 * Don't let program loop infinitely if something goes wrong
		 */

		if (values_read > EXPECTED_FIFO_ELEMENTS) {
			error(EXIT_FAILURE, 0,
			      "ERROR: Too many values read from FIFO");
		}

		/*
		 * Print progress information to inform user program is working
		 */

		fprintf(stdout, "    %u\r", values_read);

		/*
		 * FIFO should not be full
		 */

		get_fifo_status(board, DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_FULL, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO is full");
		}

		/*
		 * FIFO should not have overflowed
		 */

		get_fifo_status(board,
				DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_OVERFLOW, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO has overflowed");
		}

		/*
		 * FIFO should not have underflowed
		 */

		get_fifo_status(board,
				DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO has underflowed");
		}

		/*
		 * Determine whether or not FIFO is empty
		 */

		get_fifo_status(board, DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_EMPTY, &fifo_empty);
	}

	fprintf(stdout, "\n");

	/*
	 * Verify that expected number of values were read from FIFO
	 */

	fprintf(stdout, "%u values read from FIFO\n", values_read);

	if (values_read != EXPECTED_FIFO_ELEMENTS) {
		error(EXIT_FAILURE, 0,
		      "ERROR: Unexpected number of values read");
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Disabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
