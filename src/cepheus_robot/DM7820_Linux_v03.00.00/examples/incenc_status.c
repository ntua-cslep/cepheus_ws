/**
    @file

    @brief
        Example program which demonstrates how to get incremental encoder status
        when not using interrupts.

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

        Encoder 0 channel A values are read and printed until either negative
        rollover or positive rollover occurs.

        Channel B on incremental encoder 0 is not used.
    @endverbatim

    @note
        Because all phase filter transitions are enabled in a particular
        counting direction (up or down), the counter value will update at four
        times the incremental encoder "cycles per revolution" rate.

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

    $Id: incenc_status.c 60252 2012-06-04 19:39:05Z rgroner $
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
	dm7820_incenc_phase_filter phase_filter;
	int status;
	struct option options[] = {
		{"help", 0, 0, 1},
		{"minor", 1, 0, 2},
		{0, 0, 0, 0}
	};
	uint8_t encoder_status;
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	uint8_t rollover_occurred = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout,
		"\tDM7820 Incremental Encoder Status Example Program\n");
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
	   Secondary incremental encoder 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Enable encoder 0
	 */

	fprintf(stdout, "Enabling incremental encoder 0 ...\n");

	dm7820_status =
	    DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	/*
	 * Clear channel A negative rollover status flag without checking its state
	 */

	dm7820_status = DM7820_IncEnc_Get_Status(board,
						 DM7820_INCENC_ENCODER_0,
						 DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER,
						 &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	/*
	 * Clear channel A positive rollover status flag without checking its state
	 */

	dm7820_status = DM7820_IncEnc_Get_Status(board,
						 DM7820_INCENC_ENCODER_0,
						 DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER,
						 &encoder_status);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Get_Status()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "\n");
	fprintf(stdout, "Waiting for channel A rollover.\n");
	fprintf(stdout, "\n");
	fprintf(stdout, "Incremental encoder 0 channel A value\n");

	/*
	 * Loop reading encoder 0 channel A value until negative or positive
	 * rollover occurs
	 */

	while (!rollover_occurred) {
		uint16_t encoder_value;

		/*
		 * Read encoder 0 channel A value
		 */

		dm7820_status = DM7820_IncEnc_Get_Independent_Value(board,
								    DM7820_INCENC_ENCODER_0,
								    DM7820_INCENC_CHANNEL_A,
								    &encoder_value);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_IncEnc_Get_Independent_Value()");

		/*
		 * Print channel A value
		 */

		fprintf(stdout, "    0x%04x\r", encoder_value);

		/*
		 * Get encoder 0 channel A negative rollover status and see if rollover
		 * occurred
		 */

		dm7820_status = DM7820_IncEnc_Get_Status(board,
							 DM7820_INCENC_ENCODER_0,
							 DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER,
							 &encoder_status);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_IncEnc_Get_Status()");

		if (encoder_status) {

			/*
			 * Verify that negative rollover status was cleared
			 */

			dm7820_status = DM7820_IncEnc_Get_Status(board,
								 DM7820_INCENC_ENCODER_0,
								 DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER,
								 &encoder_status);
			DM7820_Return_Status(dm7820_status,
					     "DM7820_IncEnc_Get_Status()");

			if (encoder_status) {
				error(EXIT_FAILURE,
				      0,
				      "ERROR: Channel A negative rollover status not cleared");
			}

			fprintf(stdout, "\n");
			fprintf(stdout, "Channel A negative rollover.\n");

			rollover_occurred = 0xFF;
		}

		/*
		 * Get encoder 0 channel A positive rollover status and see if rollover
		 * occurred
		 */

		dm7820_status = DM7820_IncEnc_Get_Status(board,
							 DM7820_INCENC_ENCODER_0,
							 DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER,
							 &encoder_status);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_IncEnc_Get_Status()");

		if (encoder_status) {

			/*
			 * Verify that positive rollover status was cleared
			 */

			dm7820_status = DM7820_IncEnc_Get_Status(board,
								 DM7820_INCENC_ENCODER_0,
								 DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER,
								 &encoder_status);
			DM7820_Return_Status(dm7820_status,
					     "DM7820_IncEnc_Get_Status()");

			if (encoder_status) {
				error(EXIT_FAILURE,
				      0,
				      "ERROR: Channel A positive rollover status not cleared");
			}

			fprintf(stdout, "\n");
			fprintf(stdout, "Channel A positive rollover.\n");

			rollover_occurred = 0xFF;
		}
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

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
