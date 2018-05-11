/**
    @file

    @brief
        Example program which demonstrates a loopback accross the FIFO's using
        DMA.

    @verbatim

        This program uses FIFO 0, which is configured as follows: 1) input clock
        set to PCI write request, 2) output clock set to programmable clock 1,
        and 3) data input set to PCI data.

        The programmable clocks are set up as follows:

                    Master      Start       Stop
            Clock   Clock       Trigger     Trigger     Mode        Frequency
            ==================================================================
            0       25 MHz      immediate   none        continuous  500 Hz
            1       Clock 0     immediate   none        continuous  1 Hz

        With the setup indicated above, programmable clock 1 will cause a value
        to be read from FIFO 0 once every second.

        Any value clocked out of FIFO 0 is discarded.

        The program loops performing the following sequence of actions:
        1) writing 5 values to FIFO 0, 2) enabling FIFO 0 empty interrupt,
        3) starting programmable clocks 0 and 1, 4) waiting for a FIFO 0 empty
        interrupt, and 5) disabling programmable clocks 0 and 1.

        The above sequence of actions is performed 4 times and then the program
        exits.

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

    $Id: fifo_interrupt.c 60252 2012-06-04 19:39:05Z rgroner $
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
	dm7820_interrupt_info interrupt_info;
	int status;
	struct option options[] = {
		{"help", 0, 0, 1},
		{"minor", 1, 0, 2},
		{"wait", 1, 0, 3},
		{0, 0, 0, 0}
	};
	uint8_t help_option = 0x00;
	uint8_t interrupt_count = 0;
	uint8_t minor_option = 0x00;
	uint8_t wait_option = 0x00;
	uint8_t want_busy_wait = 0x00;
	uint8_t want_sleepy_wait = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 FIFO Interrupt Example Program\n");
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
	 * Make sure FIFO 0 empty interrupt is disabled to prevent stray interrupts
	 */

	fprintf(stdout, "Disabling FIFO 0 empty interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board,
					    DM7820_INTERRUPT_FIFO_0_EMPTY,
					    0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   FIFO generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable FIFO 0 to put it into a known state
	 */

	fprintf(stdout, "Disabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable clocks 0 and 1 to put them into a known state; any clock should
	 * be disabled before programming it.
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

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   FIFO 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing FIFO 0 ...\n");

	/*
	 * Set input clock to PCI write to FIFO 0 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting input clock ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Input_Clock(board, DM7820_FIFO_QUEUE_0,
					DM7820_FIFO_INPUT_CLOCK_PCI_WRITE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

	/*
	 * Set output clock to programmable clock 1
	 */

	fprintf(stdout, "    Setting output clock ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Output_Clock(board, DM7820_FIFO_QUEUE_0,
					 DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_1);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*
	 * Set data input to PCI data
	 */

	fprintf(stdout, "    Setting data input ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Data_Input(board, DM7820_FIFO_QUEUE_0,
				       DM7820_FIFO_0_DATA_INPUT_PCI_DATA);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

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
	 * Set clock start trigger so that clock starts immediately
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
	 * Set clock period to obtain 500 Hz frequency
	 */

	dm7820_status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 50000);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing programmable clock 1 ...\n");

	/*
	 * Set master clock to programmable clock 0
	 */

	fprintf(stdout, "    Setting master clock ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Master(board, DM7820_PRGCLK_CLOCK_1,
				     DM7820_PRGCLK_MASTER_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");

	/*
	 * Set clock start trigger so that clock starts immediately
	 */

	fprintf(stdout, "    Setting start trigger ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Start_Trigger(board, DM7820_PRGCLK_CLOCK_1,
					    DM7820_PRGCLK_START_IMMEDIATE);
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
	 * Set clock period to obtain 1 Hz frequency
	 */

	dm7820_status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_1, 500);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary FIFO 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Load/empty FIFO 0 until desired number of interrupts have been seen
	 */

	fprintf(stdout, "Looping to process FIFO 0 ...\n");

	while (interrupt_count < 4) {
		uint8_t fifo_value;

		/*#####################################################################
		   Load some values into FIFO 0
		   ##################################################################### */

		fprintf(stdout, "    Writing values to FIFO ...\n");

		/*
		 * Write 5 values into FIFO 0.  With programmable clock 1 running at 1
		 * Hz, it will take 5 seconds to empty the FIFO.
		 */

		for (fifo_value = 0; fifo_value < 5; fifo_value++) {
			dm7820_status =
			    DM7820_FIFO_Write(board, DM7820_FIFO_QUEUE_0,
					      fifo_value);
			DM7820_Return_Status(dm7820_status,
					     "DM7820_FIFO_Write()");

		}

		/*#####################################################################
		   FIFO 0 empty interrupt initialization
		   ##################################################################### */

		/*
		 * Enable FIFO 0 empty interrupt after values are written to FIFO.  If
		 * the interrupt was enabled before the values were written, an unwanted
		 * interrupt would occur because the FIFO is empty at that point.
		 */

		fprintf(stdout, "    Enabling FIFO 0 empty interrupt ...\n");

		dm7820_status =
		    DM7820_General_Enable_Interrupt(board,
						    DM7820_INTERRUPT_FIFO_0_EMPTY,
						    0xFF);
		DM7820_Return_Status(dm7820_status,
				     "DM7820_General_Enable_Interrupt()");

		/*#####################################################################
		   Get programmable clocks 0 and 1 going
		   ##################################################################### */

		/*
		 * Put clock 1 into continuous mode and enable it
		 */

		fprintf(stdout,
			"    Setting programmable clock 1 mode and enabling ...\n");

		dm7820_status =
		    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_1,
					   DM7820_PRGCLK_MODE_CONTINUOUS);
		DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

		/*
		 * Put clock 0 into continuous mode and enable it
		 */

		fprintf(stdout,
			"    Setting programmable clock 0 mode and enabling ...\n");

		dm7820_status =
		    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
					   DM7820_PRGCLK_MODE_CONTINUOUS);
		DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

		/*#####################################################################
		   Wait for FIFO 0 empty interrupt
		   ##################################################################### */

		fprintf(stdout, "    Waiting for FIFO empty interrupt using ");
		if (want_busy_wait) {
			fprintf(stdout, "busy wait ...\n");
		} else {
			fprintf(stdout, "sleepy wait ...\n");
		}

		if (want_busy_wait) {

			/*
			 * Busy wait for an interrupt
			 */

			for (;;) {
				dm7820_status =
				    DM7820_General_Get_Interrupt_Status(board,
									&interrupt_info,
									0x00);
				DM7820_Return_Status(dm7820_status,
						     "DM7820_General_Get_Interrupt_Status()");

				/*
				 * If FIFO 0 empty interrupt is pending, we're done waiting
				 */

				if (interrupt_info.source ==
				    DM7820_INTERRUPT_FIFO_0_EMPTY) {
					break;
				}
			}
		} else {

			/*
			 * Put the process to sleep waiting for an interrupt
			 */

			for (;;) {
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
				 * Verify interrupt status
				 */

				if (interrupt_info.source ==
				    DM7820_INTERRUPT_FIFO_0_EMPTY) {
					break;
				}

				error(EXIT_FAILURE,
				      0,
				      "ERROR: No FIFO 0 empty interrupt occurred");
			}
		}

		/*#####################################################################
		   FIFO 0 empty interrupt occurred.  Since the driver's interrupt handler
		   disables any FIFO interrupt, it must be reenabled before it can be used
		   again.  This will occur the next time the enclosing loop is executed.
		   ##################################################################### */

		/*
		 * Count this interrupt
		 */

		interrupt_count++;

		fprintf(stdout, "        Interrupt number %u occurred.\n",
			interrupt_count);

		/*#####################################################################
		   Disable programmable clocks 0 and 1.  This will lessen the chances of
		   (but will not prevent) a garbage value being read from the FIFO before
		   more values can be written to the FIFO.
		   ##################################################################### */

		fprintf(stdout,
			"    Disabling programmable clocks 0 and 1 ...\n");

		dm7820_status =
		    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_1,
					   DM7820_PRGCLK_MODE_DISABLED);
		DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

		dm7820_status =
		    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
					   DM7820_PRGCLK_MODE_DISABLED);
		DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	}

	fprintf(stdout, "Interrupt count reached, exiting ...\n");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Normally, the FIFO 0 empty interrupt would be disabled here.  However,
	 * there's no need to do that because each FIFO interrupt is disabled by the
	 * driver's interrupt handler.
	 */

	fprintf(stdout, "Disabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
