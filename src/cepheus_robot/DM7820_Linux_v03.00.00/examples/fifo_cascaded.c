/**
    @file

    @brief
        Example program which demonstrates how to use the FIFO block FIFOs such
        that the output of one FIFO serves as the input of another FIFO.

    @verbatim
        This program transfers FIFO 0 data out standard I/O port 0, in standard
        I/O port 1, and into FIFO 1.

        FIFO 0 is configured as follows: 1) input clock set to PCI write
        request, 2) output clock set to programmable clock 1, and 3) data input
        set to PCI data.

        FIFO 1 is configured as follows: 1) input clock set to programmable
        clock 1, 2) output clock set to PCI read request, and 3) data input set
        to standard I/O port 1.

        The programmable clocks are set up as follows:

                    Master      Start       Stop
            Clock   Clock       Trigger     Trigger     Mode        Frequency
            ==================================================================
            0       25 MHz      immediate   none        continuous  100 KHz
            1       Clock 0     immediate   none        continuous  2 Hz

        Standard I/O ports 0 and 1 should be connected as follows:

            CN10 Pin    CN11 Pin
            ====================
            17          17
            19          19
            21          21
            23          23
            25          25
            27          27
            29          29
            31          31
            33          33
            35          35
            37          37
            39          39
            41          41
            43          43
            45          45
            47          47

        A character string is written to FIFO 0.  Once the programmable clocks
        are started, the string is transferred to FIFO 1 at the rate of two
        characters per second.  While the characters are being transferred, the
        program reads the characters from FIFO 1 and reassembles the string.
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

    $Id: fifo_cascaded.c 60252 2012-06-04 19:39:05Z rgroner $
*/

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
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
    Determine whether or not the specified status condition has occurred for the
    given FIFO.

@param
    board

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to determine status of.

@param
    condition

    The status condition to check for.

@param
    status

    Address where occurrence flag should be stored.
 *******************************************************************************
 */

static void
get_fifo_status(DM7820_Board_Descriptor * board,
		dm7820_fifo_queue fifo,
		dm7820_fifo_status_condition condition, uint8_t * status)
{
	if (DM7820_FIFO_Get_Status(board, fifo, condition, status) == -1) {
		error(EXIT_FAILURE, errno,
		      "ERROR: DM7820_FIFO_Get_Status() FAILED");
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
	char source_message[] =
	    "This message will be clocked out of FIFO "
	    "0 via standard I/O port 0 into FIFO 1 via " "standard I/O port 1.";
	char target_message[sizeof(source_message)];
	int status;
	struct option options[] = {
		{"help", 0, 0, 1},
		{"minor", 1, 0, 2},
		{0, 0, 0, 0}
	};
	uint16_t fifo_value;
	uint8_t fifo_status;
	uint8_t help_option = 0x00;
	uint8_t message_index;
	uint8_t minor_option = 0x00;
	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 FIFO Cascade Example Program\n");
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
	   FIFO generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable all FIFOs to put them into a known state
	 */

	fprintf(stdout, "Disabling FIFOs ...\n");

	fprintf(stdout, "    FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "    FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

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
	   Standard I/O block digital I/O port 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing digital I/O port 0 ...\n");

	/*
	 * Set each port bit to peripheral output
	 */

	fprintf(stdout, "    Setting bits to peripheral output ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF,
				     DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	/*
	 * Set each port bit to output FIFO 0
	 */

	fprintf(stdout, "    Selecting peripheral output ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_0, 0xFFFF,
					 DM7820_STDIO_PERIPH_FIFO_0);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Standard I/O block digital I/O port 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing digital I/O port 1 ...\n");

	/*
	 * Set each port bit to peripheral input
	 */

	fprintf(stdout, "    Setting bits to input ...\n");

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_1, 0xFFFF,
				     DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

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
	 * Set output clock to PCI read from FIFO 0 Read/Write Port Register; this
	 * will be changed to programmable clock 1 after FIFO 0 has been loaded and
	 * first value has been clocked out of it
	 */

	fprintf(stdout, "    Setting output clock ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Output_Clock(board, DM7820_FIFO_QUEUE_0,
					 DM7820_FIFO_OUTPUT_CLOCK_PCI_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

	/*
	 * Set data input to PCI data
	 */

	fprintf(stdout, "    Setting data input ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Data_Input(board, DM7820_FIFO_QUEUE_0,
				       DM7820_FIFO_0_DATA_INPUT_PCI_DATA);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   FIFO 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing FIFO 1 ...\n");

	/*
	 * Set input clock to programmable clock 1
	 */

	fprintf(stdout, "    Setting input clock ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Input_Clock(board, DM7820_FIFO_QUEUE_1,
					DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_1);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

	/*
	 * Set output clock to PCI read from FIFO 1 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting output clock ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Output_Clock(board, DM7820_FIFO_QUEUE_1,
					 DM7820_FIFO_OUTPUT_CLOCK_PCI_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*
	 * Set data input to standard I/O block port 1
	 */

	fprintf(stdout, "    Setting data input ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Data_Input(board, DM7820_FIFO_QUEUE_1,
				       DM7820_FIFO_1_DATA_INPUT_PORT_1);
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
	 * Set clock period to obtain 100 KHz frequency
	 */

	dm7820_status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_0, 250);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

	/*
	 * The clock cannot be enabled yet.  We need to wait until FIFO 0 has been
	 * loaded and FIFO 1 has been enabled.
	 */

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
	 * Set clock start trigger to start immediately
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
	 * Set clock period to obtain 2 Hz frequency
	 */

	dm7820_status =
	    DM7820_PrgClk_Set_Period(board, DM7820_PRGCLK_CLOCK_1, 50000);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

	/*
	 * The clock cannot be enabled yet.  We need to wait until FIFO 0 has been
	 * loaded and FIFO 1 has been enabled.
	 */

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary FIFO 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*
	 * Clear FIFO status empty flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 0 status empty flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_EMPTY,
			&fifo_status);

	/*
	 * Clear FIFO status full flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 0 status full flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_FULL,
			&fifo_status);

	/*
	 * Clear FIFO status overflow flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 0 status overflow flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_OVERFLOW,
			&fifo_status);

	/*
	 * Clear FIFO status underflow flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 0 status underflow flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_0,
			DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine initial FIFO 0 status
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Checking initial FIFO 0 status ...\n");

	/*
	 * FIFO should be empty
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_EMPTY,
			&fifo_status);

	if (!fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO is not empty");
	}

	/*
	 * FIFO should not be full
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_FULL,
			&fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO is full");
	}

	/*
	 * FIFO should not have overflowed
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0, DM7820_FIFO_STATUS_OVERFLOW,
			&fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO has overflowed");
	}

	/*
	 * FIFO should not have underflowed
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_0,
			DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO has underflowed");
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write source message to FIFO 0 via the PCI bus
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Writing source message to FIFO 0 via PCI bus ...\n");

	/*#########################################################################
	   Write each character of source message to FIFO 0
	   ######################################################################### */

	for (message_index = 0;
	     message_index < sizeof(source_message); message_index++) {

		/*
		 * Write current character of source message to FIFO
		 */

		dm7820_status =
		    DM7820_FIFO_Write(board, DM7820_FIFO_QUEUE_0,
				      source_message[message_index]
		    );
		if (dm7820_status != 0) {
			error(EXIT_FAILURE, errno,
			      "ERROR: DM7820_FIFO_Write() FAILED");
		}

		fprintf(stdout, "    0x%02x\r", source_message[message_index]);

		/*
		 * FIFO should not be empty
		 */

		get_fifo_status(board, DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_EMPTY, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO is empty");
		}

		/*
		 * FIFO should not be full
		 */

		get_fifo_status(board, DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_FULL, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO is full");
		}

		/*
		 * FIFO should not have overflowed
		 */

		get_fifo_status(board,
				DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_OVERFLOW, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO has overflowed");
		}

		/*
		 * FIFO should not have underflowed
		 */

		get_fifo_status(board,
				DM7820_FIFO_QUEUE_0,
				DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

		if (fifo_status) {
			error(EXIT_FAILURE, 0, "ERROR: FIFO has underflowed");
		}
	}

	fprintf(stdout, "\n");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Account for data clocking differences between FIFO 0 and 1 to prevent
	   transfer of invalid values
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Via PCI access, clock first value out of FIFO 0 to make it available to
	 * FIFO 1 on first input clock pulse
	 */

	fprintf(stdout, "Clocking first value out of FIFO 0 ...\n");

	dm7820_status =
	    DM7820_FIFO_Read(board, DM7820_FIFO_QUEUE_0, &fifo_value);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Read()");

	/*
	 * Set FIFO 0 output clock to programmable clock 1
	 */

	fprintf(stdout,
		"Setting FIFO 0 output clock to programmable clock 1 ...\n");

	dm7820_status =
	    DM7820_FIFO_Set_Output_Clock(board, DM7820_FIFO_QUEUE_0,
					 DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_1);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary FIFO 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Enabling FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*
	 * Clear FIFO status empty flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 1 status empty flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_1, DM7820_FIFO_STATUS_EMPTY,
			&fifo_status);

	/*
	 * Clear FIFO status full flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 1 status full flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_1, DM7820_FIFO_STATUS_FULL,
			&fifo_status);

	/*
	 * Clear FIFO status overflow flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 1 status overflow flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_1, DM7820_FIFO_STATUS_OVERFLOW,
			&fifo_status);

	/*
	 * Clear FIFO status underflow flag without checking its state
	 */

	fprintf(stdout, "Clearing FIFO 1 status underflow flag ...\n");

	get_fifo_status(board, DM7820_FIFO_QUEUE_1,
			DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine initial FIFO 1 status
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Checking initial FIFO 1 status ...\n");

	/*
	 * FIFO should be empty because input clock has not been started yet
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_1, DM7820_FIFO_STATUS_EMPTY,
			&fifo_status);

	if (!fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO is not empty");
	}

	/*
	 * FIFO should not be full
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_1, DM7820_FIFO_STATUS_FULL,
			&fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO is full");
	}

	/*
	 * FIFO should not have overflowed
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_1, DM7820_FIFO_STATUS_OVERFLOW,
			&fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO has overflowed");
	}

	/*
	 * FIFO should not have underflowed
	 */

	get_fifo_status(board, DM7820_FIFO_QUEUE_1,
			DM7820_FIFO_STATUS_UNDERFLOW, &fifo_status);

	if (fifo_status) {
		error(EXIT_FAILURE, 0, "ERROR: FIFO has underflowed");
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary programmable clock 0 initialization
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
	   Secondary programmable clock 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Put clock into continuous mode and enable it
	 */

	fprintf(stdout, "Setting programmable clock 1 mode and enabling ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_1,
				   DM7820_PRGCLK_MODE_CONTINUOUS);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read target message from FIFO 1 via the PCI bus
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Reading target message from FIFO 1 via PCI bus ...\n");

	/*#########################################################################
	   Read expected number of target message characters from FIFO 1
	   ######################################################################### */

	for (message_index = 0;
	     message_index < sizeof(target_message); message_index++) {

		/*
		 * Wait until FIFO 1 is not empty
		 */

		for (;;) {

			/*
			 * FIFO should not be full
			 */

			get_fifo_status(board,
					DM7820_FIFO_QUEUE_1,
					DM7820_FIFO_STATUS_FULL, &fifo_status);

			if (fifo_status) {
				error(EXIT_FAILURE, errno,
				      "ERROR: FIFO is full");
			}

			/*
			 * FIFO should not have overflowed
			 */

			get_fifo_status(board,
					DM7820_FIFO_QUEUE_1,
					DM7820_FIFO_STATUS_OVERFLOW,
					&fifo_status);

			if (fifo_status) {
				error(EXIT_FAILURE, errno,
				      "ERROR: FIFO overflowed");
			}

			/*
			 * FIFO should not have underflowed
			 */

			get_fifo_status(board,
					DM7820_FIFO_QUEUE_1,
					DM7820_FIFO_STATUS_UNDERFLOW,
					&fifo_status);

			if (fifo_status) {
				error(EXIT_FAILURE, errno,
				      "ERROR: FIFO underflowed");
			}

			/*
			 * If FIFO is not empty, a character can be read from it
			 */

			get_fifo_status(board,
					DM7820_FIFO_QUEUE_1,
					DM7820_FIFO_STATUS_EMPTY, &fifo_status);

			if (!fifo_status) {
				break;
			}
		}

		/*
		 * Read value from FIFO
		 */

		dm7820_status =
		    DM7820_FIFO_Read(board, DM7820_FIFO_QUEUE_1, &fifo_value);
		DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Read()");

		/*
		 * Print progress information to inform user program is working
		 */

		fprintf(stdout, "    0x%02x\r", (char)fifo_value);

		/*
		 * Save FIFO value in target message
		 */

		target_message[message_index] = (char)fifo_value;
	}

	fprintf(stdout, "\n");

	/*#########################################################################
	   Validate target message
	   ######################################################################### */

	/*
	 * Make sure last character of target message is NUL character so that
	 * string functions don't cause segmentation faults
	 */

	if (target_message[sizeof(target_message) - 1] != '\0') {
		error(EXIT_FAILURE, 0,
		      "ERROR: Target message is not NUL terminated");
	}

	/*
	 * Validate target message length
	 */

	if (strlen(&(target_message[0])) != (sizeof(target_message) - 1)) {
		error(EXIT_FAILURE, 0, "Invalid target message length");
	}

	/*
	 * Make sure target message matches source message
	 */

	if (strcmp(&(source_message[0]), &(target_message[0])) != 0) {
		error(EXIT_FAILURE, errno,
		      "Source and target messages don't match");
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Final processing before exit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Disabling programmable clock 0 ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_0,
				   DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	fprintf(stdout, "Disabling programmable clock 1 ...\n");

	dm7820_status =
	    DM7820_PrgClk_Set_Mode(board, DM7820_PRGCLK_CLOCK_1,
				   DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	fprintf(stdout, "Disabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Disabling FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
