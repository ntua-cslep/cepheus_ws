/**
    @file

    @brief
        Example program which demonstrates how to use the programmable clock
        block clocks.

    @verbatim
        This program sets up programmable clocks 0 and 1 so that they run at the
        same frequency but approximately 180 degrees out of phase.  The phase
        shift is only approximate because the delays inherent in starting clock
        2 via clock 0 and starting clock 1 via clock 2 must be considered when
        setting clock 2's period.  Clock 2 is used to delay clock 1 relative to
        clock 0.

        The programmable clocks are set up as follows:

                        Start       Stop
            Clock       Trigger     Trigger     Mode        Frequency
            =========================================================
            0           immediate   none        continuous  50 KHz
            1           clock 2     none        continuous  50 KHz
            2           clock 0     none        one-shot    100 KHz

        The above clocks are set to use the 25 MHz clock as their master clocks.

        The standard I/O block port 2 pins are set up to output the programmable
        clocks so that their frequencies may be measured.
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

	$Id: prgclk_clocks.c 60252 2012-06-04 19:39:05Z rgroner $
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
	fprintf(stdout, "\tDM7820 Programmable Clock Example Program\n");
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
	   Programmable clock generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable all clocks to put them into a known state; any clock should be
	 * disabled before programming it.
	 */

	fprintf(stdout, "Disabling programmable clocks ...\n");

	fprintf(stdout, "    Clock 0 ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
				   DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	fprintf(stdout, "    Clock 1 ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_1,
				   DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	fprintf(stdout, "    Clock 2 ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_2,
				   DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	fprintf(stdout, "    Clock 3 ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_3,
				   DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

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
	 * Set each port 2 bit to output clock/other peripheral
	 */

	fprintf(stdout,
		"    Setting bits to output clock/other peripheral ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF,
					 DM7820_STDIO_PERIPH_CLK_OTHER);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing programmable clock 0 ...\n");

	/*
	 * Set master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting master clock ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Master(board, DM7820_PRGCLK_CLOCK_0,
				     DM7820_PRGCLK_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");

	/*
	 * Set clock start trigger to start immediately
	 */

	fprintf(stdout, "    Setting start trigger ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Start_Trigger(board, DM7820_PRGCLK_CLOCK_0,
					    DM7820_PRGCLK_START_IMMEDIATE);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_PrgClk_Set_Start_Trigger()");

	/*
	 * Set clock stop trigger so that clock is never stopped
	 */

	fprintf(stdout, "    Setting stop trigger ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Stop_Trigger(board, DM7820_PRGCLK_CLOCK_0,
					   DM7820_PRGCLK_STOP_NONE);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");

	/*
	 * Set clock period to obtain 50 KHz frequency
	 */

	dm7820_status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 500);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

	/*
	 * Put clock into continuous mode and enable it
	 */

	fprintf(stdout, "    Setting clock mode and enabling ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
				   DM7820_PRGCLK_MODE_CONTINUOUS);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing programmable clock 1 ...\n");

	/*
	 * Set master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting master clock ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Master(board, DM7820_PRGCLK_CLOCK_1,
				     DM7820_PRGCLK_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");

	/*
	 * Set clock start trigger to programmable clock 2
	 */

	fprintf(stdout, "    Setting start trigger ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Start_Trigger(board, DM7820_PRGCLK_CLOCK_1,
					    DM7820_PRGCLK_START_PROG_CLOCK_2);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_PrgClk_Set_Start_Trigger()");

	/*
	 * Set clock stop trigger so that clock is never stopped
	 */

	fprintf(stdout, "    Setting stop trigger ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Stop_Trigger(board, DM7820_PRGCLK_CLOCK_1,
					   DM7820_PRGCLK_STOP_NONE);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");

	/*
	 * Set clock period to obtain 50 KHz frequency
	 */

	dm7820_status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_1, 500);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

	/*
	 * We cannot enable the clock yet.  Programmable clock 2 must be programmed
	 * before clock 1 can be enabled.
	 */

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock 2 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing programmable clock 2 ...\n");

	/*
	 * Set master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting master clock ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Master(board, DM7820_PRGCLK_CLOCK_2,
				     DM7820_PRGCLK_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");

	/*
	 * Set clock start trigger to programmable clock 0
	 */

	fprintf(stdout, "    Setting start trigger ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Start_Trigger(board, DM7820_PRGCLK_CLOCK_2,
					    DM7820_PRGCLK_START_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_PrgClk_Set_Start_Trigger()");

	/*
	 * Set clock stop trigger so that clock is never stopped
	 */

	fprintf(stdout, "    Setting stop trigger ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Stop_Trigger(board, DM7820_PRGCLK_CLOCK_2,
					   DM7820_PRGCLK_STOP_NONE);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");

	/*
	 * Set clock period to obtain 100 KHz frequency
	 */

	dm7820_status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_2, 250);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

	/*
	 * We cannot enable the clock yet.  Programmable clock 1 must be enabled
	 * before clock 2 can be enabled.
	 */

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary programmble clock 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Put programmble clock 1 into continuous mode and enable it; this readies
	 * it for triggering by clock 2
	 */

	fprintf(stdout,
		"Setting clock 1 to continuous mode and enabling ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_1,
				   DM7820_PRGCLK_MODE_CONTINUOUS);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary programmble clock 2 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Put programmble clock 2 into one-shot mode and enable it
	 */

	fprintf(stdout, "Setting clock 2 to one-shot mode and enabling ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_2,
				   DM7820_PRGCLK_MODE_ONE_SHOT);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

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
