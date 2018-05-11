/**
    @file

    @brief
        Example program which demonstrates how to use the pulse width modulator
        block interrupts.

    @verbatim
        Programmable clock 0 is set up as follows:

            Master      Start       Stop
            Clock       Trigger     Trigger     Mode        Frequency
            =========================================================
            25 MHz      immediate   none        continuous  500 Hz

        This program configures pulse width modulator 0 as follows: 1) master
        clock set to programmable clock 0, 2) period set to obtain frequency of
        1 Hz, 3) width master clock set to programmable clock 0, and 4) output A
        width initially set to obtain 20% positive duty cycle.

        The standard I/O block port 2 pins are set up to output the pulse width
        modulators so that PWM 0 frequency and duty cycle may be measured.

        With the setup indicated about, PWM 0 generates an interrupt once a
        second.

        This program uses pulse width modulator 0 interrupts and waits for the
        interrupts to occur.  20 such interrupts are waited on and then the
        program exits.  After 10 interrupts occur, the PWM 0 output A width is
        changed to obtain 80% positive duty cycle.
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

    $Id: pwm_interrupt.c 60252 2012-06-04 19:39:05Z rgroner $
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
	fprintf(stdout, "\tDM7820 PWM Interrupt Example Program\n");
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
	 * Make sure pulse width modulator 0 interrupt is disabled to prevent stray
	 * interrupts
	 */

	fprintf(stdout, "Disabling pulse width modulator 0 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_PWM_0,
					    0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable programmabel clock 0 to put it into a known state; any clock
	 * should be disabled before programming it.
	 */

	fprintf(stdout, "Disabling programmable clock 0 ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
				   DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Pulse width modulator generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable pulse width modulator 0 to put it into a known state; any pulse
	 * width modulator should be disabled before programming it
	 */

	fprintf(stdout, "Disabling pulse width modulator 0 ...\n");

	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0x00);
	if (dm7820_status != 0) {
		error(EXIT_FAILURE, errno, "ERROR: DM7820_PWM_Enable() FAILED");
	}

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
	 * Set clock period to obtain 500 Hz frequency
	 */

	dm7820_status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 50000);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

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
	 * Set period master clock to programmable clock 0
	 */

	fprintf(stdout, "    Setting period master clock ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_0,
					 DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

	/*
	 * Set pulse width modulator period to obtain 1 Hz frequency
	 */

	fprintf(stdout, "    Setting period ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, 500);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

	/*#########################################################################
	   Width initialization
	   ######################################################################### */

	/*
	 * Set width master clock to programmable clock 0
	 */

	fprintf(stdout, "    Setting width master clock ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_0,
					DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

	/*
	 * Set output A width to obtain 20% duty cycle
	 */

	fprintf(stdout, "    Setting output A width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0,
				 DM7820_PWM_OUTPUT_A, 100);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	/*
	 * Note that outputs B,C, and D are not initialized
	 */

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Pulse width modulator 0 interrupt initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling pulse width modulator 0 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_PWM_0,
					    0xFF);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Pulse width modulator 0 secondary initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling pulse width modulator 0 ...\n");

	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock 0 secondary initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Put clock into continuous mode and enable it
	 */

	fprintf(stdout, "Setting programmable clock 0 mode and enabling ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
				   DM7820_PRGCLK_MODE_CONTINUOUS);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

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
	 * Loop until desired number of interrupts have been seen
	 */

	while (interrupt_count < 20) {
		dm7820_interrupt_info interrupt_info;

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
			 * If PWM 0 interrupt is not pending, check status again
			 */

			if (interrupt_info.source != DM7820_INTERRUPT_PWM_0) {
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
			 * Verify interrupt status
			 */

			if (interrupt_info.source != DM7820_INTERRUPT_PWM_0) {
				error(EXIT_FAILURE, 0,
				      "ERROR: No PWM 0 interrupt occurred");
			}
		}

		/*#####################################################################
		   Interrupt has occurred
		   ##################################################################### */

		/*
		 * Count this interrupt
		 */

		interrupt_count++;

		fprintf(stdout, "    Interrupt number %u occurred.\n",
			interrupt_count);

		/*
		 * If 10 interrupts have occurred, set PWM 0 output A width to obtain
		 * 80% duty cycle
		 */

		if (interrupt_count == 10) {
			fprintf(stdout,
				"        Changing PWM 0 output A width ...\n");

			dm7820_status =
			    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0,
						 DM7820_PWM_OUTPUT_A, 400);
			DM7820_Return_Status(dm7820_status,
					     "DM7820_PWM_Set_Width()");

		}
	}

	fprintf(stdout, "Interrupt count reached, exiting ...\n");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Disabling pulse width modulator 0 interrupt ...\n");

	dm7820_status =
	    DM7820_General_Enable_Interrupt(board, DM7820_INTERRUPT_PWM_0,
					    0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "Disabling pulse width modulator 0 ...\n");

	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");

	fprintf(stdout, "Disabling programmable clock 0 ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
				   DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
