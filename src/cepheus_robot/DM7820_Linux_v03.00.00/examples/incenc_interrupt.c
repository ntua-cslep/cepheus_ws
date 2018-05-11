/**
    @file

    @brief
        Example program which demonstrates how to use the incremental encoder
        block interrupts.

    @verbatim
        Incremental encoder 0 is configured as follows: 1) master clock set to
        25 MHz clock, 2) value register hold disabled, 3) single-ended inputs,
        4) input filter disabled, 5) independent A/B channels, and 6) index
        input disabled.

        Incremental encoder 0 is further configured with an initial value of
        0x8000 and a phase filter to enable counter change on all transitions.

        This program assumes an incremental encoder is connected to incremental
        encoder 0 channel A.  The incremental encoder should be connected as
        follows:

            Encoder Pin     CN10 Pin
            ========================
            A               47
            B               43
            -               50
            +               49

        Channel B on incremental encoder 0 is not used.

        This program uses incremental encoder 0 channel A negative & positive
        rollover interrupts and waits for the interrupts to occur.  One such
        interrupt is waited on and then the program exits.
    @endverbatim

    @note
        Because all phase filter transitions are enabled in a particular
        counting direction (up or down), the counter value will update at four
        times the incremental encoder "cycles per revolution" rate.

    @note
        This program does not display the channel A value because sleepy waiting
        may be used.

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

    $Id: incenc_interrupt.c 60252 2012-06-04 19:39:05Z rgroner $
*/

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <strings.h>
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
		"Usage: %s [--help] --minor MINOR --wait WAIT\n", program_name);
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --help:     Display usage information and exit.\n");
	fprintf(stderr, "\n");
	fprintf(stderr, "    --minor:    Use specified DM7820 device.\n");
	fprintf(stderr, "        MINOR:        Device minor number (>= 0).\n");
	fprintf(stderr, "\n");
	fprintf(stderr,
		"    --wait:     Use specified interrupt wait method.\n");
	fprintf(stderr, "        WAIT:         Interrupt wait method.\n");
	fprintf(stderr, "            busy   = Busy wait.\n");
	fprintf(stderr, "            sleepy = Sleepy wait.\n");
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
	dm7820_incenc_phase_filter phase_filter;
	int status;
	struct option options[] = {
		{"help", 0, 0, 1},
		{"minor", 1, 0, 2},
		{"wait", 1, 0, 3},
		{0, 0, 0, 0}
	};
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	uint8_t wait_option = 0x00;
	uint8_t want_busy_wait = 0x00;
	uint8_t want_sleepy_wait = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout,
		"\tDM7820 Incremental Encoder Interrupt Example Program\n");
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
			   User entered '--wait'
			   ################################################################# */

		case 3:

			/*
			 * Refuse to accept duplicate '--wait' options
			 */

			if (wait_option) {
				error(0, 0, "ERROR: Duplicate option '--wait'");
				usage();
			}

			/*
			 * Validate '--wait' argument
			 */

			want_busy_wait = (strcasecmp(optarg, "busy") == 0);
			want_sleepy_wait = (strcasecmp(optarg, "sleepy") == 0);

			if (!(want_busy_wait | want_sleepy_wait)) {
				error(0, 0, "ERROR: Invalid wait mode");
				usage();
			}

			/*
			 * '--wait' option has been seen
			 */

			wait_option = 0xFF;
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

	/*
	 * '--minor' option must be given
	 */

	if (minor_option == 0x00) {
		error(0, 0, "ERROR: Option '--minor' is required");
		usage();
	}

	/*
	 * '--wait' option must be given
	 */

	if (wait_option == 0x00) {
		error(0, 0, "ERROR: Option '--wait' is required");
		usage();
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Device initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

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
	 * Make sure incremental encoder 0 channel A negative and positive rollover
	 * interrupts are disabled to prevent stray interrupts
	 */

	fprintf(stdout,
		"Disabling incremental encoder channel A interrupts ...\n");

	fprintf(stdout, "    Negative rollover\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER,
							0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "    Positive rollover\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER,
							0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Incremental encoder generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable incremental encoder 0 to put it into a known state; any
	 * incremental encoder should be disabled before programming it
	 */

	fprintf(stdout, "Disabling incremental encoder 0 ...\n");

	dm7820_status =
	    DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Standard I/O block digital I/O port 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing digital I/O port 0 ...\n");

	/*
	 * Set each port 0 bit to input which enables incremental encoder inputs
	 */

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF,
				     DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Incremental encoder 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing incremental encoder 0 ...\n");

	/*
	 * Set master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting master clock ...\n");

	dm7820_status =
	    DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_0,
				     DM7820_INCENC_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");

	/*
	 * Disable value register hold
	 */

	fprintf(stdout, "    Disabling value register hold ...\n");

	dm7820_status =
	    DM7820_IncEnc_Enable_Hold(board, DM7820_INCENC_ENCODER_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

	/*
	 * Configure the incremental encoder as follows: 1) enable counter update on
	 * all transitions 2) set single-ended input mode, 3) disable the input
	 * filter, 4) set channels A and B to operate independently of each other,
	 * and 5) disable index input
	 */

	fprintf(stdout, "    Configuring encoder ...\n");

	DM7820_INCENC_RESET_PHASE_FILTER(phase_filter);

	dm7820_status = DM7820_IncEnc_Configure(board,
						DM7820_INCENC_ENCODER_0,
						phase_filter,
						DM7820_INCENC_INPUT_SINGLE_ENDED,
						0x00,
						DM7820_INCENC_CHANNEL_INDEPENDENT,
						0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

	/*
	 * Set initial value for channel A counter
	 */

	fprintf(stdout, "    Setting initial value for channel A ...\n");

	dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
							    DM7820_INCENC_ENCODER_0,
							    DM7820_INCENC_CHANNEL_A,
							    0x8000);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_IncEnc_Set_Independent_Value()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Incremental encoder 0 channel A interrupt secondary initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Enable incremental encoder 0 channel A negative and positive rollover
	 * interrupts
	 */

	fprintf(stdout,
		"Enabling incremental encoder channel A interrupts ...\n");

	fprintf(stdout, "    Negative rollover\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER,
							0xFF);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "    Positive rollover\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER,
							0xFF);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Incremental encoder 0 secondary initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Enable encoder 0
	 */

	fprintf(stdout, "Enabling incremental encoder 0 ...\n");

	dm7820_status =
	    DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Waiting for interrupts using ");
	if (want_busy_wait) {
		fprintf(stdout, "busy wait ...\n");
	} else {
		fprintf(stdout, "sleepy wait ...\n");
	}

	/*
	 * Loop until an interrupt occurs on incremental encoder 0 channel A
	 */

	for (;;) {
		dm7820_interrupt_info interrupt_info;
		uint8_t negative_rollover;
		uint8_t positive_rollover;

		/*#####################################################################
		   Wait for interrupt to occur
		   ##################################################################### */

		if (want_busy_wait) {

			/*
			 * Busy wait for an interrupt
			 */

			dm7820_status =
			    DM7820_General_Get_Interrupt_Status(board,
								&interrupt_info,
								0x00);
			DM7820_Return_Status(dm7820_status,
					     "DM7820_General_Get_Interrupt_Status()");

			/*
			 * If neither rollover interrupt is pending, check status again
			 */

			negative_rollover =
			    (interrupt_info.source ==
			     DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER);

			positive_rollover =
			    (interrupt_info.source ==
			     DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER);

			if ((!negative_rollover) && (!positive_rollover)) {
				continue;
			}
		} else {

			/*
			 * Put the process to sleep waiting for an interrupt
			 */

			dm7820_status =
			    DM7820_General_Get_Interrupt_Status(board,
								&interrupt_info,
								0xFF);
			if (dm7820_status != 0) {

				/*
				 * If a signal interrupted the wait, wait again because an
				 * interrupt did not occur
				 */

				if (errno == EINTR) {
					continue;
				}

				/*
				 * Unrecoverable error occurred
				 */

				error(EXIT_FAILURE,
				      errno,
				      "ERROR: DM7820_General_Get_Interrupt_Status() FAILED");
			}

			/*
			 * Rollover interrupt should have occurred
			 */

			/*
			 * Verify interrupt status
			 */

			negative_rollover =
			    (interrupt_info.source ==
			     DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER);

			positive_rollover =
			    (interrupt_info.source ==
			     DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER);

			if ((!negative_rollover) && (!positive_rollover)) {
				error(EXIT_FAILURE, 0,
				      "ERROR: No encoder 0 interrupt occurred");
			}
		}

		/*#####################################################################
		   Rollover interrupt has occurred
		   ##################################################################### */

		/*
		 * Print the type of rollover interrupt
		 */

		if (negative_rollover) {
			fprintf(stdout, "    Negative ");
		} else {
			fprintf(stdout, "    Positive ");
		}

		fprintf(stdout, "rollover interrupt occurred.\n");

		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout,
		"Disabling incremental encoder channel A interrupts ...\n");

	fprintf(stdout, "    Negative rollover\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER,
							0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "    Positive rollover\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER,
							0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "Disabling incremental encoder 0 ...\n");

	dm7820_status =
	    DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
