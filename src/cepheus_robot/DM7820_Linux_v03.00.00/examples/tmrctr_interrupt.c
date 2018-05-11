/**
    @file

    @brief
        Example program which demonstrates how to use 8254 timer/counter block
        timer interrupts.

    @verbatim
        Timers A0 and A1 are configured as follows:

                    Input   Count   Waveform
            Timer   Clock   Mode    Mode                Frequency
            =========================================================
            A0      5 MHz   binary  square wave         100 Hz
            A1      A0      binary  square wave         .5 Hz

        Each timer has its gate set to logic 1 to enable counting.

        With the setup indicated above, timers A0 and A1 are cascaded.  Timer A1
        generates an interrupt once every two seconds.

    The program uses timer A1 interrupts and waits for the interrupts to
        occur.  Ten such interrupts are waited on and then the program exits.
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

    $Id: tmrctr_interrupt.c 79285 2014-05-21 20:37:42Z rgroner $
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

/**
 * Variable to count the number of DMA Done interrupts occurring
 */

static volatile int interrupts = 0;

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
			// This is just to prevent the un-used return
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
	fprintf(stderr,
		"Usage: %s [--help] --minor MINOR --wait WAIT\n", program_name);
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --help:     Display usage information and exit.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    --minor:    Use specified DM7820 device.\n");
	fprintf(stderr, "        MINOR:        Device minor number (>= 0).\n");
	fprintf(stderr, "\n");
	exit(EXIT_FAILURE);
}

/**
*******************************************************************************
@brief
    Userspace ISR

@param
    interrupt_info

    Information about the interrupt, returned by the kernel driver
*******************************************************************************
*/

void ISR(dm7820_interrupt_info interrupt_info)
{
	/*
	 * If this ISR is called that means an interrupt occurred, so
	 * check if it is timer/counter A1
	 */

	DM7820_Return_Status(interrupt_info.error, "ISR Failed\n");

	switch (interrupt_info.source) {
	case DM7820_INTERRUPT_TMRCTR_A_1:
		interrupts++;
		break;
	default:
		break;

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
	fprintf(stdout,
		"\tDM7820 8254 Timer/Counter Interrupt Example Program\n");
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
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

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

	/*
	 * Open the device file
	 */

	fprintf(stdout, "Opening device with minor number %lu ...\n",
		minor_number);

	dm7820_status = DM7820_General_Open_Board(minor_number, &board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Open_Board()");

	/*
	 * Reset device
	 */

	fprintf(stdout, "Resetting device ...\n");

	dm7820_status = DM7820_General_Reset(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Reset()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Interrupt generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Make sure timer/counter A1 interrupt is disabled to prevent stray
	 * interrupts
	 */

	fprintf(stdout, "Disabling timer/counter A1 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_TMRCTR_A_1,
					    0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*
	 * Install user-space ISR
	 */
	fprintf(stdout, "Installing user ISR ...\n");
	dm7820_status = DM7820_General_InstallISR(board, ISR);
	DM7820_Return_Status(dm7820_status, "DM7820_General_InstallISR()");

	fprintf(stdout, "Setting ISR priority ...\n");
	dm7820_status = DM7820_General_SetISRPriority(board, 99);
	DM7820_Return_Status(dm7820_status, "DM7820_General_SetISRPriority()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   8254 timer/counter generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable timer/counters A0 and A1 to get them into a known state
	 */

	fprintf(stdout, "Disabling timer/counters ...\n");

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
					      50000);
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
	 * .5 Hz frequency
	 */

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_A_1,
					      DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      200);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Program()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Timer/counter A1 interrupt initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling timer/counter A1 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_TMRCTR_A_1,
					    0xFF);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

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
	 * Loop until desired number of  have been seen
	 */
	int temp_interrupts = interrupts;
	while (temp_interrupts < 10 && !exit_program) {
		if (interrupts != temp_interrupts) {
			temp_interrupts = interrupts;
			fprintf(stdout, "    Interrupt number %u occurred.\n",
				interrupts);
			fflush(stdout);
		}
		usleep(0);
	}

	fprintf(stdout, "Interrupt count reached, exiting ...\n");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Disabling timer/counter A1 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_TMRCTR_A_1,
					    0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "Removing user ISR ...\n");

	dm7820_status = DM7820_General_RemoveISR(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_RemoveISR()");

	fprintf(stdout, "Disabling timer/counters ...\n");

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

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
