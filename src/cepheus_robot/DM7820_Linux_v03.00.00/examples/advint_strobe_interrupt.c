/**
    @file

    @brief
        Example program which demonstrates how to use advanced interrupt block
        strobe mode interrupts.

    @verbatim
        Standard I/O block ports are configured so that port 0 is set to output
        and port 1 is set to input.  Port 2 is not used.

        This program initializes advanced interrupt 0 by selecting strobe signal
        1 as the strobe source and setting interrupt mode to strobe.

        Standard I/O block port 0 feeds values to port 1.  Therefore, each port
        0 bit should be connected to the corresponding port 1 bit.

        This program initializes the strobe signals as follows: 1) strobe signal
        1 set to input, 2) strobe signal 2 set to output, and 3) strobe signal 2
        set low.

        Strobe signal 2 provides the input for strobe signal 1.  Therefore, CN11
        pin 2 should be connected to CN10 pin 2.

        This program uses advanced interrupt block 0 interrupts and waits for
        the interrupts to occur.  Only one such interrupt should occur.
    @endverbatim

    @note
        This program does not use interrupt sleepy-wait.  For this example, it
        is difficult to use sleep-waiting without making the program too complex
        and hard to understand.

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

    $Id: advint_strobe_interrupt.c 60252 2012-06-04 19:39:05Z rgroner $
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
	uint32_t interrupt_count = 0;
	uint32_t output_value;
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout,
		"\tDM7820 Advanced Interrupt Strobe Mode Interrupt Example Program\n");
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

	/*
	 * Open device file
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
	 * Make sure advance interrupt block 0 interrupt is disabled to prevent
	 * stray interrupts
	 */

	fprintf(stdout, "Disabling advanced interrupt block 0 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_ADVINT_0,
					    0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Advanced interrupt generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable all advanced interrupt blocks to prevent unexpected interrupts
	 */

	fprintf(stdout, "Disabling all advanced interrupt blocks ...\n");

	fprintf(stdout, "    Block 0 ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mode(board, DM7820_ADVINT_INTERRUPT_0,
				   DM7820_ADVINT_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mode()");

	fprintf(stdout, "    Block 1 ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mode(board, DM7820_ADVINT_INTERRUPT_1,
				   DM7820_ADVINT_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Strobe signal 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set signal to input
	 */

	fprintf(stdout, "Setting strobe signal 1 to input ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Mode(board, DM7820_STDIO_STROBE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Strobe signal 2 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing strobe signal 2 ...\n");

	/*
	 * Set signal to output
	 */

	fprintf(stdout, "    Setting signal to output ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Mode(board, DM7820_STDIO_STROBE_2, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Mode()");

	/*
	 * Pull signal low to get strobe signal 1 into a known state
	 */

	fprintf(stdout, "    Setting signal low ...\n");

	dm7820_status =
	    DM7820_StdIO_Strobe_Output(board, DM7820_STDIO_STROBE_2, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Strobe_Output()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Standard I/O block digital I/O port 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing digital I/O port 0 ...\n");

	/*
	 * Set each port bit to output
	 */

	fprintf(stdout, "    Setting bits to output ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF,
				     DM7820_STDIO_MODE_OUTPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Standard I/O block digital I/O port 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing digital I/O port 1 ...\n");

	/*
	 * Set each port bit to input
	 */

	fprintf(stdout, "    Setting bits to input ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_1, 0xFFFF,
				     DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Advanced interrupt block 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing advanced interrupt block 0 ...\n");

	/*
	 * Set strobe source to strobe signal 1
	 */

	fprintf(stdout, "    Setting strobe source ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Master(board, DM7820_ADVINT_INTERRUPT_0,
				     DM7820_ADVINT_MASTER_STROBE_1);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Master()");

	/*
	 * Set advanced interrupt mode to strobe
	 */

	fprintf(stdout, "    Setting advanced interrupt mode ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mode(board, DM7820_ADVINT_INTERRUPT_0,
				   DM7820_ADVINT_MODE_STROBE);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Advanced interrupt block 0 interrupt initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling advanced interrupt block 0 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_ADVINT_0,
					    0xFF);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Loop writing all possible 16-bit values to port 0
	 */

	fprintf(stdout, "Writing values to port 0 ...\n");

	for (output_value = 0x0000; output_value <= 0xFFFF; output_value++) {
		dm7820_interrupt_info interrupt_info;
		uint16_t capture_value;
		uint8_t advint_0_pending;

		/*#####################################################################
		   Display progress information
		   ##################################################################### */

		fprintf(stdout, "    0x%04x\r", output_value);

		/*#####################################################################
		   Write current digital value to port 0
		   ##################################################################### */

		dm7820_status =
		    DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_0,
					    output_value);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_StdIO_Set_Output()");

		/*#####################################################################
		   See if a strobe should be generated
		   ##################################################################### */

		/*
		 * Generate strobe after value 0x1234 is written
		 */

		if (output_value == 0x1234) {

			/*
			 * Set strobe signal 2 high which also pulls strobe signal 1 high
			 * and causes a strobe interrupt
			 */

			dm7820_status =
			    DM7820_StdIO_Strobe_Output(board,
						       DM7820_STDIO_STROBE_2,
						       0xFF);

			if (dm7820_status != 0) {
				error(EXIT_FAILURE,
				      errno,
				      "ERROR: DM7820_StdIO_Strobe_Output() FAILED");
			}

			/*
			 * Give the signal 1 microsecond to go high.
			 */

			usleep(1);

			/*
			 * Set strobe signal 2 low
			 */

			dm7820_status =
			    DM7820_StdIO_Strobe_Output(board,
						       DM7820_STDIO_STROBE_2,
						       0x00);

			if (dm7820_status != 0) {
				error(EXIT_FAILURE,
				      errno,
				      "ERROR: DM7820_StdIO_Strobe_Output() FAILED");
			}
		}

		/*#####################################################################
		   Get and examine interrupt status
		   ##################################################################### */

		/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		   Retrieve interrupt status
		   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

		/*
		 * Do not sleep waiting for an interrupt
		 */

		dm7820_status =
		    DM7820_General_Get_Interrupt_Status(board,
							&interrupt_info, 0x00);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_General_Get_Interrupt_Status()");

		/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		   Examine interrupt status
		   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

		advint_0_pending =
		    interrupt_info.source == DM7820_INTERRUPT_ADVINT_0;

		/*
		 * Compare output value to interrupt status
		 */

		if (output_value != 0x1234) {

			/*
			 * Strobe should not have been generated, so make sure no interrupt
			 * occurred
			 */

			if (advint_0_pending) {
				error(EXIT_FAILURE, 0,
				      "ERROR: Unexpected strobe occurred");
			}
		} else {

			/*
			 * Strobe should have been generated, so make sure an interrupt
			 * occurred
			 */

			if (!advint_0_pending) {
				error(EXIT_FAILURE, 0,
				      "ERROR: No strobe occurred");
			}
		}

		/*
		 * If an interrupt did not occur, write next output value
		 */

		if (!advint_0_pending) {
			continue;
		}

		/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		   Advanced interrupt block 0 strobe interrupt occurred
		   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

		fprintf(stdout, "\n");
		fprintf(stdout, "        Strobe interrupt occurred\n");

		/*
		 * Read port 1 capture register and verify its value equals the value
		 * which should have caused a strobe
		 */

		dm7820_status = DM7820_AdvInt_Read_Capture(board,
							   DM7820_ADVINT_INTERRUPT_0,
							   DM7820_STDIO_PORT_1,
							   &capture_value);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_AdvInt_Read_Capture()");

		fprintf(stdout, "            Capture register: 0x%04x\n",
			capture_value);

		if (capture_value != output_value) {
			error(EXIT_FAILURE,
			      0,
			      "ERROR: Incorrect value read from capture register");
		}

		/*
		 * Count the interrupt
		 */

		interrupt_count++;
	}

	fprintf(stdout, "\n");

	/*
	 * Make sure only a single interrupt occurred
	 */

	if (interrupt_count != 1) {
		error(EXIT_FAILURE,
		      0,
		      "ERROR: Expected 1 interrupt, received %u\n",
		      interrupt_count);
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Disabling advanced interrupt block 0 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_ADVINT_0,
					    0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "Disabling advanced interrupt block 0 ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mode(board, DM7820_ADVINT_INTERRUPT_0,
				   DM7820_ADVINT_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mode()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
