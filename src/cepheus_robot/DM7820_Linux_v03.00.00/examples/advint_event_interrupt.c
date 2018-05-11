/**
    @file

    @brief
        Example program which demonstrates how to use advanced interrupt block
        event mode interrupts.

    @verbatim
        Standard I/O block ports are configured as follows: 1) port 0 set to
        output, 2) port 1 set to input, and 3) port 2 set to output.

        Advanced interrupt 1 is configured as follows: 1) interrupt mode set to
        event, 2) sampling clock set to 25 MHz clock, 3) port 0 mask register
        set to ignore all bits, 4) port 1 mask register set to enable only bit 1
        to generate an interrupt, and 5) port 2 mask register set to ignore all
        bits.

        Standard I/O block port 0 feeds values to port 1.  Therefore, each port
        0 bit should be connected to the corresponding port 1 bit.  An initial
        value of 0x0000 is written to port 0.

        With the setup indicated above, an event interrupt should occur on port
        1 whenever there is a value evenly divisible by 2 present except for
        0x0000.

        This program uses advanced interrupt block 1 interrupts and waits for
        the interrupts to occur.

        32767 event interrupts should occur.  65536 values are output but only
        half of those (32768) should cause an interrupt.  The value 0x0000 does
        not cause an interrupt since that value is already present on the input
        port when it is written.  Thus, 32678 - 1 (32767) interrupts occur.
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

    $Id: advint_event_interrupt.c 60252 2012-06-04 19:39:05Z rgroner $
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

/**
 * Counter for advint interrupts received
 */
volatile unsigned int advint1_interrupts;

/*=============================================================================
Program code
 =============================================================================*/

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
	 * Receive the interrupt and examine it
	 */

	DM7820_Return_Status(interrupt_info.error, "ISR Failed\n");

	switch (interrupt_info.source) {
	case DM7820_INTERRUPT_ADVINT_1:
		advint1_interrupts++;
		break;

	default:

		break;
	}

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
	uint32_t interrupt_count = 0;
	uint32_t output_value;
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout,
		"\tDM7820 Advanced Interrupt Event Mode Interrupt Example Program\n");
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
	 * Make sure advance interrupt block 1 interrupt is disabled to prevent
	 * stray interrupts
	 */

	fprintf(stdout, "Disabling advanced interrupt block 1 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_ADVINT_1,
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

	/*
	 * Set all port bits low to feed a known value to port 1
	 */

	dm7820_status =
	    DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_0, 0x0000);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Output()");

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
	   Standard I/O block digital I/O port 2 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing digital I/O port 2 ...\n");

	/*
	 * Set each port bit to output
	 */

	fprintf(stdout, "    Setting bits to output ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0xFFFF,
				     DM7820_STDIO_MODE_OUTPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Advanced interrupt block 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing advanced interrupt block 1 ...\n");

	/*
	 * Set master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting master clock ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Master(board, DM7820_ADVINT_INTERRUPT_1,
				     DM7820_ADVINT_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Master()");

	/*
	 * Load port 0 mask register so that all bits are ignored; any bits not used
	 * to generate an interrupt MUST be ignored for interrupt to work properly
	 */

	fprintf(stdout, "    Loading port 0 mask register ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mask(board, DM7820_ADVINT_INTERRUPT_1,
				   DM7820_STDIO_PORT_0, 0xFFFF);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mask()");

	/*
	 * Load port 1 mask register so that only bit 1 can cause an interrupt
	 */

	fprintf(stdout, "    Loading port 1 mask register ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mask(board, DM7820_ADVINT_INTERRUPT_1,
				   DM7820_STDIO_PORT_1, 0xFFFD);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mask()");

	/*
	 * Load port 2 mask register so that all bits are ignored; any bits not used
	 * to generate an interrupt MUST be ignored for interrupt to work properly
	 */

	fprintf(stdout, "    Loading port 2 mask register ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mask(board, DM7820_ADVINT_INTERRUPT_1,
				   DM7820_STDIO_PORT_2, 0xFFFF);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mask()");

	/*
	 * Set advanced interrupt mode to event
	 */

	fprintf(stdout, "    Setting advanced interrupt mode ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mode(board, DM7820_ADVINT_INTERRUPT_1,
				   DM7820_ADVINT_MODE_EVENT);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Advanced interrupt block 1 interrupt initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling advanced interrupt block 1 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_ADVINT_1,
					    0xFF);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Install the user ISR routine to capture the advint interrupt
	 */
	fprintf(stdout, "Installing user ISR ...\n");
	dm7820_status = DM7820_General_InstallISR(board, ISR);
	DM7820_Return_Status(dm7820_status, "DM7820_General_InstallISR()");

	fprintf(stdout, "Setting ISR priority ...\n");
	dm7820_status = DM7820_General_SetISRPriority(board, 99);
	DM7820_Return_Status(dm7820_status, "DM7820_General_SetISRPriority()");

	/*
	 * Loop writing all possible 16-bit values to port 0
	 */

	fprintf(stdout, "Writing values to port 0 ...\n");

	for (output_value = 0x0000; output_value <= 0xFFFF; output_value++) {
		uint16_t capture_value;
		unsigned int old_advint1_interrupt_val;

		/* Take the old value to compare to the new value, so we know when
		 * an interrupt has happened.
		 */
		old_advint1_interrupt_val = advint1_interrupts;

		/*#####################################################################
		   Display progress information
		   ##################################################################### */

		fprintf(stdout, "    0x%04x\n", output_value);

		/*#####################################################################
		   Write current digital value to port 0
		   ##################################################################### */

		dm7820_status =
		    DM7820_StdIO_Set_Output(board, DM7820_STDIO_PORT_0,
					    output_value);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_StdIO_Set_Output()");

		/*#####################################################################
		   Get and examine interrupt status
		   ##################################################################### */

		/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		   Retrieve interrupt status
		   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

		while (advint1_interrupts == old_advint1_interrupt_val) {

			if (output_value % 2 != 0 || output_value == 0x0000) {
				break;
			}
		}

		/*
		 * Compare output value to interrupt status
		 */

		if ((output_value % 2) == 0) {

			/*
			 * Output value is evenly divisible by 2
			 */

			/*
			 * Behavior depends on whether or not the value 0x0000 was output
			 */

			if (output_value != 0x0000) {

				/*
				 * Make sure output value, which should have caused an
				 * interrupt, did cause an interrupt
				 */

				if (advint1_interrupts ==
				    old_advint1_interrupt_val) {
					error(EXIT_FAILURE, 0,
					      "ERROR: No event occurred");
				}
			} else {

				/*
				 * Make sure output value, which shouldn't have caused an
				 * interrupt, did not cause an interrupt
				 */

				if (advint1_interrupts !=
				    old_advint1_interrupt_val) {
					error(EXIT_FAILURE, 0,
					      "ERROR: Unexpected event occurred");
				}
			}
		} else {

			/*
			 * Output value is not evenly divisible by 2
			 */

			/*
			 * Make sure output value, which shouldn't have caused an interrupt,
			 * did not cause an interrupt
			 */

			if (advint1_interrupts != old_advint1_interrupt_val) {
				error(EXIT_FAILURE, 0,
				      "ERROR: Unexpected event occurred");
			}
		}

		/*
		 * If an interrupt did not occur, write next output value
		 */

		if (advint1_interrupts == old_advint1_interrupt_val) {
			continue;
		}

		/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		   Advanced interrupt block 1 event interrupt occurred
		   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

		fprintf(stdout, "        Event interrupt occurred\n");

		/*
		 * Read port 1 capture register and verify its value equals the value
		 * which should have caused an event
		 */

		dm7820_status = DM7820_AdvInt_Read_Capture(board,
							   DM7820_ADVINT_INTERRUPT_1,
							   DM7820_STDIO_PORT_1,
							   &capture_value);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_AdvInt_Read_Capture()");

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

	/*
	 * Make sure expected number of interrupts occurred
	 */

	if (interrupt_count != 32767) {
		error(EXIT_FAILURE,
		      0,
		      "ERROR: Expected 32767 interrupts, received %u\n",
		      interrupt_count);
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Disabling advanced interrupt block 1 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_ADVINT_1,
					    0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "Disabling advanced interrupt block 1 ...\n");

	dm7820_status =
	    DM7820_AdvInt_Set_Mode(board, DM7820_ADVINT_INTERRUPT_1,
				   DM7820_ADVINT_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_AdvInt_Set_Mode()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
