/**
    @file

    @brief
        Example program which demonstrates how to select peripheral outputs for
        standard I/O block digital I/O ports.

    @note
        This program does not set up the selected peripheral output.  It merely
        programs the digital I/O ports so that a peripheral can be output.

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

	$Id: stdio_peripheral_output.c 60252 2012-06-04 19:39:05Z rgroner $
*/

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
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
	fprintf(stderr,
		"Usage: %s [--help] --minor MINOR --port PORT --peripheral PERIPHERAL "
		"--bits BITS\n", program_name);
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --help:          Display usage information and exit.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    --minor:         Use specified DM7820 device.\n");
	fprintf(stderr,
		"        MINOR:             Device minor number (>= 0).\n");
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --port:          Use specified digital I/O input.\n");
	fprintf(stderr, "        PORT:              Port number (0, 1, 2).\n");
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --peripheral:    Set specified peripheral output.\n");
	fprintf(stderr,
		"        PERIPHERAL:        Peripheral output to set.\n");
	fprintf(stderr, "            pwm   = Pulse Width Modulator.\n");
	fprintf(stderr, "            clock = Clock/strobe.\n");
	fprintf(stderr, "            fifo0 = FIFO0,\n");
	fprintf(stderr, "            fifo1 = FIFO1.\n");
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --bits:          Port bits to set peripheral output for.\n");
	fprintf(stderr,
		"        BITS:              Port bit mask (0x0000 to 0xFFFF).\n");
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
	DM7820_StdIO_Periph_Mode peripheral_mode;
	int status;
	struct option options[] = {
		{"help", 0, 0, 1},
		{"minor", 1, 0, 2},
		{"port", 1, 0, 3},
		{"peripheral", 1, 0, 4},
		{"bits", 1, 0, 5},
		{0, 0, 0, 0}
	};
	uint8_t bits_option = 0x00;
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	uint8_t peripheral_option = 0x00;
	uint8_t port_option = 0x00;
	uint8_t want_clock = 0x00;
	uint8_t want_fifo0 = 0x00;
	uint8_t want_fifo1 = 0x00;
	uint8_t want_pwm = 0x00;
	unsigned long int bit_mask = 0x0000;
	unsigned long int minor_number = 0;
	unsigned long int port_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout,
		"\tDM7820 Standard I/O Peripheral Output Example Program\n");
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
			   User entered '--port'
			   ################################################################# */

		case 3:{
				char *invalid_char_p;

				/*
				 * Refuse to accept duplicate '--port' options
				 */

				if (port_option) {
					error(0, 0,
					      "ERROR: Duplicate option '--port'");
					usage();
				}

				/*
				 * Convert option argument string to unsigned long integer
				 */

				errno = 0;

				port_number =
				    strtoul(optarg, &invalid_char_p, 10);

				/*
				 * Catch unsigned long int overflow
				 */

				if ((port_number == ULONG_MAX)
				    && (errno == ERANGE)) {
					error(0, 0,
					      "ERROR: Port number caused numeric overflow");
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
					      "ERROR: Non-decimal port number");
					usage();
				}

				/*
				 * Valid port numbers are 0, 1, and 2
				 */

				switch (port_number) {
				case 0:
				case 1:
				case 2:
					break;

				default:
					error(0, 0,
					      "ERROR: Invalid port number");
					usage();
					break;
				}

				/*
				 * '--port' option has been seen
				 */

				port_option = 0xFF;
				break;
			}

			/*#################################################################
			   User entered '--peripheral'
			   ################################################################# */

		case 4:

			/*
			 * Refuse to accept duplicate '--peripheral' options
			 */

			if (peripheral_option) {
				error(0, 0,
				      "ERROR: Duplicate option '--peripheral'");
				usage();
			}

			want_clock = (strcasecmp(optarg, "clock") == 0);
			want_fifo0 = (strcasecmp(optarg, "fifo0") == 0);
			want_fifo1 = (strcasecmp(optarg, "fifo1") == 0);
			want_pwm = (strcasecmp(optarg, "pwm") == 0);

			if (!(want_clock | want_fifo0 | want_fifo1 | want_pwm)) {
				error(0, 0, "ERROR: Invalid peripheral mode");
				usage();
			}

			/*
			 * '--peripheral' option has been seen
			 */

			peripheral_option = 0xFF;
			break;

			/*#################################################################
			   User entered '--bits'
			   ################################################################# */

		case 5:{
				char *invalid_char_p;

				/*
				 * Refuse to accept duplicate '--bits' options
				 */

				if (bits_option) {
					error(0, 0,
					      "ERROR: Duplicate option '--bits'");
					usage();
				}

				/*
				 * Convert option argument string to unsigned long integer
				 */

				errno = 0;

				bit_mask = strtoul(optarg, &invalid_char_p, 16);

				/*
				 * Catch unsigned long int overflow
				 */

				if ((bit_mask == ULONG_MAX)
				    && (errno == ERANGE)) {
					error(0, 0,
					      "ERROR: Bit mask caused numeric overflow");
					usage();
				}

				/*
				 * Catch argument strings with valid hexadecimal prefixes, for
				 * example "0x1q", and argument strings which cannot be
				 * converted, for example "q0xFFFF"
				 */

				if ((*invalid_char_p != '\0')
				    || (invalid_char_p == optarg)) {
					error(0, 0,
					      "ERROR: Non-hexadecimal bit mask");
					usage();
				}

				if (bit_mask > 0xFFFF) {
					error(0, 0, "ERROR: Invalid bit mask");
					usage();
				}

				/*
				 * '--bits' option has been seen
				 */

				bits_option = 0xFF;
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

	/*
	 * '--port' option must be given
	 */

	if (port_option == 0x00) {
		error(0, 0, "ERROR: Option '--port' is required");
		usage();
	}

	/*
	 * '--peripheral' option must be given
	 */

	if (peripheral_option == 0x00) {
		error(0, 0, "ERROR: Option '--peripheral' is required");
		usage();
	}

	/*
	 * '--bits' option must be given
	 */

	if (bits_option == 0x00) {
		error(0, 0, "ERROR: Option '--bits' is required");
		usage();
	}

	/*
	 * Only port 2 supports clock/strobe and PWM peripheral outputs
	 */

	if (want_pwm | want_clock) {
		if (port_number != 2) {
			error(0, 0,
			      "ERROR: Only port 2 supports given peripheral mode");
			usage();
		}
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Device initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Opening device with minor number %lu ...\n",
		minor_number);

	dm7820_status = DM7820_General_Open_Board(minor_number, &board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Digital I/O port initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set given bits within specified port to peripheral output
	 */

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, port_number, bit_mask,
				     DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	/*
	 * Determine peripheral output mode based on user input
	 */

	if (want_pwm) {
		peripheral_mode = DM7820_STDIO_PERIPH_PWM;
	} else if (want_clock) {
		peripheral_mode = DM7820_STDIO_PERIPH_CLK_OTHER;
	} else if (want_fifo0) {
		peripheral_mode = DM7820_STDIO_PERIPH_FIFO_0;
	} else {
		peripheral_mode = DM7820_STDIO_PERIPH_FIFO_1;
	}

	/*
	 * Set given bits with specified port to desired peripheral output mode
	 */

	dm7820_status =
	    DM7820_StdIO_Set_Periph_Mode(board, port_number, bit_mask,
					 peripheral_mode);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
