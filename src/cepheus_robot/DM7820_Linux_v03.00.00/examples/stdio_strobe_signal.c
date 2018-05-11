/**
    @file

    @brief
        Example program which demonstrates how to use the standard I/O block
        strobe signals.

    @note
        This program uses standard I/O block strobe signals 1 and 2.  Each
        signal is used to change the state of the other.  Therefore, CN11 pin 2
        should be connected to CN10 pin 2.

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

	$Id: stdio_strobe_signal.c 60252 2012-06-04 19:39:05Z rgroner $
*/

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include "dm7820_library.h"

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
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	uint8_t strobe_state;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout,
		"\tDM7820 Standard I/O Strobe Signal Example Program\n");
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
	   Set strobe signal 1 to input and strobe signal 2 to output.  Set signal 2
	   state and read signal 1 state.
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set strobe signal 1 to input
	 */

	fprintf(stdout, "Setting strobe signal 1 to input ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Mode(board, DM7820_STDIO_STROBE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");

	/*
	 * Set strobe signal 2 to output
	 */

	fprintf(stdout, "Setting strobe signal 2 to output ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Mode(board, DM7820_STDIO_STROBE_2, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");

	/*
	 * Set strobe signal 2 low
	 */

	fprintf(stdout, "Setting strobe signal 2 low ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Output(board, DM7820_STDIO_STROBE_2, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Output()");

	/*
	 * Make sure strobe signal 2 was set low
	 */

	fprintf(stdout, "Verifying strobe signal 2 state ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Input(board, DM7820_STDIO_STROBE_2,
				      &strobe_state);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Input()");

	if (strobe_state != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Strobe signal 2 not set low");
	}

	/*
	 * Get strobe signal 1 state
	 */

	fprintf(stdout, "Getting strobe signal 1 state ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Input(board, DM7820_STDIO_STROBE_1,
				      &strobe_state);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Input()");

	/*
	 * Verify that strobe signal 1 is also low
	 */

	if (strobe_state != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Strobe signal 1 is not low");
	}

	/*
	 * Set strobe signal 2 high
	 */

	fprintf(stdout, "Setting strobe signal 2 high ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Output(board, DM7820_STDIO_STROBE_2, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Output()");

	/*
	 * Make sure strobe signal 2 was set high
	 */

	fprintf(stdout, "Verifying strobe signal 2 state ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Input(board, DM7820_STDIO_STROBE_2,
				      &strobe_state);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Input()");

	if (strobe_state == 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Strobe signal 2 not set high");
	}

	/*
	 * Get strobe signal 1 state
	 */

	fprintf(stdout, "Getting strobe signal 1 state ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Input(board, DM7820_STDIO_STROBE_1,
				      &strobe_state);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Input()");

	/*
	 * Verify that strobe signal 1 is also high
	 */

	if (strobe_state == 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Strobe signal 1 is not high");
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set strobe signal 1 to output and strobe signal 2 to input.  Set signal 1
	   state and read signal 2 state.
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set strobe signal 1 to output
	 */

	fprintf(stdout, "Setting strobe signal 1 to output ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Mode(board, DM7820_STDIO_STROBE_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");

	/*
	 * Set strobe signal 2 to input
	 */

	fprintf(stdout, "Setting strobe signal 2 to input ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Mode(board, DM7820_STDIO_STROBE_2, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");

	/*
	 * Set strobe signal 1 low
	 */

	fprintf(stdout, "Setting strobe signal 1 low ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Output(board, DM7820_STDIO_STROBE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Output()");

	/*
	 * Make sure strobe signal 1 was set low
	 */

	fprintf(stdout, "Verifying strobe signal 2 state ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Input(board, DM7820_STDIO_STROBE_1,
				      &strobe_state);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Input()");

	if (strobe_state != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Strobe signal 1 not set low");
	}

	/*
	 * Get strobe signal 2 state
	 */

	fprintf(stdout, "Getting strobe signal 2 state ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Input(board, DM7820_STDIO_STROBE_2,
				      &strobe_state);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Input()");

	/*
	 * Verify that strobe signal 2 is also low
	 */

	if (strobe_state != 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Strobe signal 2 is not low");
	}

	/*
	 * Set strobe signal 1 high
	 */

	fprintf(stdout, "Setting strobe signal 1 high ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Output(board, DM7820_STDIO_STROBE_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Output()");

	/*
	 * Make sure strobe signal 1 was set high
	 */

	fprintf(stdout, "Verifying strobe signal 1 state ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Input(board, DM7820_STDIO_STROBE_1,
				      &strobe_state);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Input()");

	if (strobe_state == 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Strobe signal 1 not set high");
	}

	/*
	 * Get strobe signal 2 state
	 */

	fprintf(stdout, "Getting strobe signal 2 state ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Input(board, DM7820_STDIO_STROBE_2,
				      &strobe_state);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Input()");

	/*
	 * Verify that strobe signal 2 is also high
	 */

	if (strobe_state == 0x00) {
		error(EXIT_FAILURE, 0, "ERROR: Strobe signal 2 is not high");
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
