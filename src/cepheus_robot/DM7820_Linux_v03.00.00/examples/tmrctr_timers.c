/**
    @file

    @brief
        Example program which demonstrates how to use the 8254 timer/counter
        block timers.

    @verbatim
        All timers are set to count in binary mode.  Timers A0, A1, A2, B0, and
        B1 are set to square wave generator waveform mode.  Timer B2 is set to
        event counter waveform mode.  Each timer has its gate set to logic 1 to
        enable counting.  The input clocks and frequencies are set as follows:

            Timer       Input Clock    Frequency
            ====================================
            A0          5 MHz          500 KHz
            A1          A0             50 KHz
            A2          A1             5 KHz
            B0          A2             500 Hz
            B1          B0             50 Hz
            B2          B1

        With the setup indicated above, all six timers are cascaded and timer B2
        is decremented every tick of timer B1, i.e. every 20 milliseconds.
        Thus, timer B2 indicates the length of time that timer B1 has been
        running.
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

    $Id: tmrctr_timers.c 79285 2014-05-21 20:37:42Z rgroner $
*/

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <signal.h>
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

/**
 * Flag used by SIGINT signal handler to tell main program to exit because the
 * user hit Control-C
 */

static volatile sig_atomic_t exit_program = 0;

/*=============================================================================
Program code
 =============================================================================*/

/**
*******************************************************************************
@brief
    Signal handler for SIGINT Control-C keyboard interrupt.

@param
    signal_number

    Signal number passed in from kernel.

@warning
    One must be extremely careful about what functions are called from a signal
    handler.  printf() and related functions are considered unsafe for use in
    signal handlers.  Therefore, this function uses write() instead.
 *******************************************************************************
 */

static void sigint_handler(int signal_number)
{
	char signal_error[] = "ERROR: SIGINT handler received wrong signal.\n";

	/*
	 * Make sure only SIGINT is received
	 */

	if (signal_number != SIGINT) {
		if (write(2, &(signal_error[0]), sizeof(signal_error))) {
			// This is just to prevent an un-used return
			// value warning
			;
		}
		_exit(EXIT_FAILURE);
	}

	exit_program = 1;
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
	struct sigaction signal_action;
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 8254 Timer/Counter Timer Example Program\n");
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
	   Generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set up a way for user to interrupt the program via Control-C
	 */

	fprintf(stdout, "Installing SIGINT signal handler ...\n");

	signal_action.sa_handler = sigint_handler;
	sigfillset(&(signal_action.sa_mask));
	signal_action.sa_flags = 0;

	if (sigaction(SIGINT, &signal_action, NULL) < 0) {
		error(EXIT_FAILURE, errno, "ERROR: sigaction() FAILED");
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Device initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Opening device with minor number %lu ...\n",
		minor_number);

	dm7820_status = DM7820_General_Open_Board(minor_number, &board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");

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
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "    Selecting gate ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_0,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*
	 * Set up the timer by 1) setting waveform mode to square wave generator,
	 * 2) setting count mode to binary, and 3) loading divisor value to cause
	 * timer to tick at 500 KHz
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_A_0,
					      DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      10);
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
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "    Selecting gate ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_1,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*
	 * Set up the timer by 1) setting waveform mode to square wave generator,
	 * 2) setting count mode to binary, and 3) loading divisor value to cause
	 * timer to tick at 50 KHz
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_A_1,
					      DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      10);
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
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "    Selecting gate ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_2,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*
	 * Set up the timer by 1) setting waveform mode to square wave generator,
	 * 2) setting count mode to binary, and 3) loading divisor value to cause
	 * timer to tick at 5 KHz
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_A_2,
					      DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      10);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   8254 timer/counter B0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing 8254 timer/counter B0 ...\n");

	/*
	 * Set timer clock source to 8254 timer/counter A2
	 */

	fprintf(stdout, "    Selecting clock source ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Clock(board, DM7820_TMRCTR_TIMER_B_0,
				       DM7820_TMRCTR_CLOCK_8254_A_2);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

	/*
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "    Selecting gate ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_0,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*
	 * Set up the timer by 1) setting waveform mode to square wave generator,
	 * 2) setting count mode to binary, and 3) loading divisor value to cause
	 * timer to tick at 500 Hz
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_B_0,
					      DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      10);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   8254 timer/counter B1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing 8254 timer/counter B1 ...\n");

	/*
	 * Set timer clock source to 8254 timer/counter B0
	 */

	fprintf(stdout, "    Selecting clock source ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Clock(board, DM7820_TMRCTR_TIMER_B_1,
				       DM7820_TMRCTR_CLOCK_8254_B_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

	/*
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "    Selecting gate ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_1,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*
	 * Set up the timer by 1) setting waveform mode to square wave generator,
	 * 2) setting count mode to binary, and 3) loading divisor value to cause
	 * timer to tick at 50 Hz
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_B_1,
					      DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      10);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   8254 timer/counter B2 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing 8254 timer/counter B2 ...\n");

	/*
	 * Set timer clock source to 8254 timer/counter B1
	 */

	fprintf(stdout, "    Selecting clock source ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Clock(board, DM7820_TMRCTR_TIMER_B_2,
				       DM7820_TMRCTR_CLOCK_8254_B_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

	/*
	 * Set timer gate to high to enable counting
	 */

	fprintf(stdout, "    Selecting gate ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_2,
				      DM7820_TMRCTR_GATE_LOGIC_1);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	/*
	 * Set up the timer by 1) setting waveform mode to event counter, 2) setting
	 * count mode to binary, and 3) loading divisor value to maximum value
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_B_2,
					      DM7820_TMRCTR_WAVEFORM_EVENT_CTR,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      0xFFFF);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "\n");
	fprintf(stdout, "Press Control-C to exit.\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "Elapsed time (in seconds)\n");

	/*
	 * Loop reading timer B2 until user hits Control-C
	 */

	while (!exit_program) {
		uint16_t timer_value;

		dm7820_status =
		    DM7820_TmrCtr_Read(board, DM7820_TMRCTR_TIMER_B_2,
				       &timer_value);
		DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Read()");

		/*
		 * Elapsed time is number of timer B2 events divided by its input clock
		 * frequency
		 */

		fprintf(stdout, "%8.2f\r",
			(float)(0xFFFF - timer_value) / 50.0);

	}

	fprintf(stdout, "\n");
	fprintf(stdout, "Interrupted by user ... exiting.\n");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Turn off all timer/counters by setting their gates to logic zero
	 */

	fprintf(stdout, "Turning off timer/counter A0 ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_0,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "Turning off timer/counter A1 ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_1,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "Turning off timer/counter A2 ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_2,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "Turning off timer/counter B0 ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_0,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "Turning off timer/counter B1 ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_1,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "Turning off timer/counter B2 ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_B_2,
				      DM7820_TMRCTR_GATE_LOGIC_0);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Gate()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
