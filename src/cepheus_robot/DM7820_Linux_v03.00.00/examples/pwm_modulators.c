/**
    @file

    @brief
        Example program which demonstrates how to use the pulse width modulator
        block modulators.

    @verbatim
        This program sets up the pulse width modulators as follows:

            PWM     Output    Duty Cyle
            ===========================
            0       A         20%
                    B         40%
                    C         60%
                    D         80%
            1       A         20%
                    B         40%
                    C         60%
                    D         80%

        PWMs 0 and 1 are set to use the 25 MHz clock as their period and width
        master clocks.  The period of PWM 0 is set to provide a frequency of 1
        MHz.  The period of PWM 1 is set to provide a frequency of 100 KHz.

        The standard I/O block port 2 pins are set up to output the pulse width
        modulators so that their frequencies and duty cycles may be measured.
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

	$Id: pwm_modulators.c 60252 2012-06-04 19:39:05Z rgroner $
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
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 PWM Modulator Example Program\n");
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
	   Pulse width modulator generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable all pulse width modulators to put them into a known state; any
	 * pulse width modulator should be disabled before programming it
	 */

	fprintf(stdout, "Disabling pulse width modulators ...\n");

	fprintf(stdout, "    PWM 0 ...\n");

	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

	fprintf(stdout, "    PWM 1 ...\n");

	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Standard I/O block digital I/O port 2 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing digital I/O port 2 ...\n");

	/*
	 * Set each port 2 bit to peripheral output
	 */

	fprintf(stdout, "    Setting bits to peripheral output ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF,
				     DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	/*
	 * Set each port 2 bit to output pulse width modulator peripheral
	 */

	fprintf(stdout, "    Setting bits to output PWM peripheral ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF,
					 DM7820_STDIO_PERIPH_PWM);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Pulse width modulator 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing pulse width modulator 0 ...\n");

	/*#########################################################################
	   Period initialization
	   ######################################################################### */

	/*
	 * Set period master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting period master clock ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_0,
					 DM7820_PWM_PERIOD_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

	/*
	 * Set pulse width modulator period to obtain 1 MHz frequency
	 */

	fprintf(stdout, "    Setting period ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, 25);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

	/*#########################################################################
	   Width initialization
	   ######################################################################### */

	/*
	 * Set width master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting width master clock ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_0,
					DM7820_PWM_WIDTH_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

	/*
	 * Set output A width to obtain 20% duty cycle
	 */

	fprintf(stdout, "    Setting output A width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0,
				 DM7820_PWM_OUTPUT_A, 5);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	/*
	 * Set output B width to obtain 40% duty cycle
	 */

	fprintf(stdout, "    Setting output B width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0,
				 DM7820_PWM_OUTPUT_B, 10);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	/*
	 * Set output C width to obtain 60% duty cycle
	 */

	fprintf(stdout, "    Setting output C width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0,
				 DM7820_PWM_OUTPUT_C, 15);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	/*
	 * Set output D width to obtain 80% duty cycle
	 */

	fprintf(stdout, "    Setting output D width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0,
				 DM7820_PWM_OUTPUT_D, 20);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	fprintf(stdout, "    Enabling PWM ...\n");

	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Pulse width modulator 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing pulse width modulator 1 ...\n");

	/*#########################################################################
	   Period initialization
	   ######################################################################### */

	/*
	 * Set period master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting period master clock ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_1,
					 DM7820_PWM_PERIOD_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

	/*
	 * Set pulse width modulator period to obtain 100 KHz frequency
	 */

	fprintf(stdout, "    Setting period ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_1, 250);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

	/*#########################################################################
	   Width initialization
	   ######################################################################### */

	/*
	 * Set width master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting width master clock ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_1,
					DM7820_PWM_WIDTH_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

	/*
	 * Set output A width to obtain 20% duty cycle
	 */

	fprintf(stdout, "    Setting output A width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1,
				 DM7820_PWM_OUTPUT_A, 50);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	/*
	 * Set output B width to obtain 40% duty cycle
	 */

	fprintf(stdout, "    Setting output B width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1,
				 DM7820_PWM_OUTPUT_B, 100);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	/*
	 * Set output C width to obtain 60% duty cycle
	 */

	fprintf(stdout, "    Setting output C width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1,
				 DM7820_PWM_OUTPUT_C, 150);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	/*
	 * Set output D width to obtain 80% duty cycle
	 */

	fprintf(stdout, "    Setting output D width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_1,
				 DM7820_PWM_OUTPUT_D, 200);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	fprintf(stdout, "    Enabling PWM ...\n");

	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
