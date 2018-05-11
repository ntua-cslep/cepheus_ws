/**
    @file

    @brief
        Example program which demonstrates how to get 8254 timer/counter block
        timer status when not using interrupts.

    @verbatim
        Timers A0, A1, and A2 are configured as follows:

                    Input   Count   Waveform
            Timer   Clock   Mode    Mode                Frequency
            =========================================================
            A0      5 MHz   binary  square wave         100 Hz
            A1      A0      binary  square wave         50 Hz
            A2      A1      binary  event counter

        Each timer has its gate set to logic 1 to enable counting.

        With the setup indicated above, timers A0, A1, and A2 are cascaded and
        timer A2 is decremented every tick of timer A1, i.e. every 20
        milliseconds.

        Timer A2 is loaded with a divisor value of 1000.  With this value, it
        will take 20 seconds for timer A2 to count down to zero.
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

    $Id: tmrctr_status.c 60252 2012-06-04 19:39:05Z rgroner $
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
Macros
 =============================================================================*/

/**
 * Timer/counter A0 divisor
 */

#define TIMER_A0_DIVISOR    50000

/**
 * Timer/counter A1 divisor
 */

#define TIMER_A1_DIVISOR    2

/**
 * Timer/counter A1 frequency in Hertz
 */

#define TIMER_A1_FREQUENCY  50

/**
 * Timer/counter A2 divisor
 */

#define TIMER_A2_DIVISOR    1000

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
    Disable all 8254 timer/counters by setting their gates to logic zero.

@param
    board

    Address of device's library board descriptor.
 *******************************************************************************
 */

static void disable_timers(DM7820_Board_Descriptor * board)
{
	DM7820_Error dm7820_status;

	fprintf(stdout, "Turning off timer/counters ...\n");

	fprintf(stdout, "    A0\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_0,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "    A1\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_1,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "    A2\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_2,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "    B0\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_0,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "    B1\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_1,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "    B2\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_2,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

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
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	uint8_t timer_status;
	uint8_t timer_zeroed = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 8254 Timer/Counter Status Example Program\n");
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
	   8254 timer/counter generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable all timer/counters to get them into a known state
	 */

	disable_timers(board);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   8254 timer/counter A0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing 8254 timer/counter A0 ...\n");

	/*
	 * Set timer clock source to 5 MHz clock
	 */

	fprintf(stdout, "    Selecting clock source ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Clock(board, DM7820_TMRCTR_TIMER_A_0,
				       DM7820_TMRCTR_CLOCK_5_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

	/*
	 * Set up the timer by 1) setting waveform mode to square wave generator,
	 * 2) setting count mode to binary, and 3) loading divisor value to obtain
	 * 100 Hz frequency
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_A_0,
					      DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      TIMER_A0_DIVISOR);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   8254 timer/counter A1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing 8254 timer/counter A1 ...\n");

	/*
	 * Set timer clock source to 8254 timer/counter A0
	 */

	fprintf(stdout, "    Selecting clock source ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Clock(board, DM7820_TMRCTR_TIMER_A_1,
				       DM7820_TMRCTR_CLOCK_8254_A_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

	/*
	 * Set up the timer by 1) setting waveform mode to square wave generator,
	 * 2) setting count mode to binary, and 3) loading divisor value to obtain
	 * 50 Hz frequency
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_A_1,
					      DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      TIMER_A1_DIVISOR);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   8254 timer/counter A2 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing 8254 timer/counter A2 ...\n");

	/*
	 * Set timer clock source to 8254 timer/counter A1
	 */

	fprintf(stdout, "    Selecting clock source ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Clock(board, DM7820_TMRCTR_TIMER_A_2,
				       DM7820_TMRCTR_CLOCK_8254_A_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

	/*
	 * Set up the timer by 1) setting waveform mode to event counter, 2) setting
	 * count mode to binary, and 3) loading divisor value of 1000
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_A_2,
					      DM7820_TMRCTR_WAVEFORM_EVENT_CTR,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      TIMER_A2_DIVISOR);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary 8254 timer/counter A2 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "Setting timer/counter A2 gate to logic high ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_2,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary 8254 timer/counter A1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "Setting timer/counter A1 gate to logic high ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_1,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary 8254 timer/counter A0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "Setting timer/counter A0 gate to logic high ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_0,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Clear timer/counter status flag without checking its state
	 */

	fprintf(stdout, "Clearing timer/counter A2 status flag ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Get_Status(board, DM7820_TMRCTR_TIMER_A_2,
				     &timer_status);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Get_Status()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Waiting for timer/counter A2 to count to zero.\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "Elapsed time (in seconds)\n");

	/*
	 * Loop reading timer A2 until timer counts down to zero
	 */

	while (!timer_zeroed) {
		uint16_t timer_value;

		dm7820_status =
		    DM7820_TmrCtr_Read(board, DM7820_TMRCTR_TIMER_A_2,
				       &timer_value);
		DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Read()");

		/*
		 * Elapsed time is number of timer A2 events divided by its input clock
		 * frequency
		 */

		fprintf(stdout,
			"%8.2f\r",
			(float)(TIMER_A2_DIVISOR -
				timer_value) / TIMER_A1_FREQUENCY);

		/*
		 * Get timer/counter A2 status and see if it reached zero
		 */

		dm7820_status =
		    DM7820_TmrCtr_Get_Status(board, DM7820_TMRCTR_TIMER_A_2,
					     &timer_status);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_TmrCtr_Get_Status()");

		if (timer_status) {

			/*
			 * Verify that timer status was cleared
			 */

			dm7820_status =
			    DM7820_TmrCtr_Get_Status(board,
						     DM7820_TMRCTR_TIMER_A_2,
						     &timer_status);
			DM7820_Return_Status(dm7820_status,
					     "DM7820_TmrCtr_Get_Status()");

			if (timer_status) {
				error(EXIT_FAILURE, 0,
				      "ERROR: Timer status not cleared");
			}

			fprintf(stdout, "\n");
			fprintf(stdout,
				"Timer/counter A2 counted down to zero.\n");

			timer_zeroed = 0xFF;
		}
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	disable_timers(board);

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
