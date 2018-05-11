/**
    @file

    @brief
        Example program which demonstrates how to use the FIFO block interrupts.
        Note: This example requires a loopback cable!

    @verbatim

        The Program does the following
        1) Allocate a 4MB Block of output data.
        2) Allocate a 4MB input buffer.
        3) FIFO 0:
            a. Input Clock = PCI Write
            b. Output Clock = Prog Clock 0
            c. In Data = PCI Data
            d. DREQ = Write Request
        4) FIFO 1:
            a. Input Clock = Prog Clock 0
            b. Out Clock = PCI Read
            c. In Data = Port 1
            d. DREQ = Read Request
        5) Port 0:
            a. All Outputs
            b. Select FIFO 0 as peripheral
        6) Prog Clock 0:
            a. Continuous
            b. Clock Source = 25MHz
            c. Period = 3 (results in 6.25 MHz Clock)
        7) Setup and starts both DMA's.
        8) Wait until FIFO 0 is not empty.
        9) Clock first data value to loopback
            a. Change FIFO 0 output clock to PCI Read
            b. Read from FIFO 0 data register
            c. Change FIFO 0 output clock to Prog Clock 0
        10) Start Prog Clock 0
        11) Wait until DMA1 is complete
        13) Test the Data
        12) Turn off Prog Clock 0 and the FIFO's

    @endverbatim

    @verbatim

        Warning:  This example program is only intended to run on a system with
        enough RAM to support the large buffer's which will be allocated.
        Preferably 128MB minimum.

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

    $Id: loopback.c 60252 2012-06-04 19:39:05Z rgroner $
*/

#include <errno.h>
#include <error.h>
#include <getopt.h>
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <strings.h>
#include <string.h>
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

volatile int dma_done_interrupts = 0;

/**
 * Variable to count the number of FIFO0 empty interrupts occurring
 */

volatile int fifo0_empty_interrupts = 0;

/**
 * Device descriptor
 */

DM7820_Board_Descriptor *board;

/**
 * The size of the individual buffers we are asking to driver to create for
 * each DMA channel.
 */

#define BUF_SIZE 0x40000

/**
 * The number of buffers we want for each DMA channel.
 */

#define BUF_NUM 16

/**
 * The number of samples that can be held by the list of buffers we created.
 */

#define SAMPLES ( ( BUF_SIZE * BUF_NUM ) / 2 )

/**
 * The size of the buffer required to handle SAMPLES number of samples.
 */

#define DMA_BUF_SIZE ( BUF_SIZE * BUF_NUM )

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
	 * If this ISR is called that means a DMA transfer has completed.
	 */

	DM7820_Return_Status(interrupt_info.error, "ISR Failed\n");

	switch (interrupt_info.source) {
	case DM7820_INTERRUPT_FIFO_1_DMA_DONE:
		dma_done_interrupts++;
		break;

	case DM7820_INTERRUPT_FIFO_0_EMPTY:
		fifo0_empty_interrupts++;
		break;

	default:

		break;
	}

}

/**
*******************************************************************************
@brief
    Disable and clean up

*******************************************************************************
*/

void clean_up()
{
	DM7820_Error dm7820_status;
	/*
	 * Normally, the FIFO 0 empty interrupt would be disabled here.  However,
	 * there's no need to do that because each FIFO interrupt is disabled by the
	 * driver's interrupt handler.
	 */

	fprintf(stdout, "\nDisabling programmable clock 0 ...\n");

	dm7820_status = DM7820_PrgClk_Set_Mode(board,
					       DM7820_PRGCLK_CLOCK_0,
					       DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	fprintf(stdout, "\nDisabling DMA 0 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Enable(board,
					       DM7820_FIFO_QUEUE_0, 0x00, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

	fprintf(stdout, "\nDisabling DMA 1 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Enable(board,
					       DM7820_FIFO_QUEUE_1, 0x00, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

	fprintf(stdout, "\nDisabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "\nDisabling FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");
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
	DM7820_Error dm7820_status;
	int status;
	uint32_t i;
	uint16_t data;
	uint16_t *dma_out_buf;
	uint16_t *dma_in_buf;

	struct option options[] = {
		{"help", 0, 0, 1},
		{"minor", 1, 0, 2},
		{0, 0, 0, 0}
	};
	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;

	unsigned long int minor_number = 0;

	program_name = arguments[0];

	dma_out_buf = NULL;
	dma_in_buf = NULL;

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 FIFO/DMA Loopback Example Program\n");
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

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_FIFO_0_EMPTY,
							0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	fprintf(stdout, "Disabling FIFO 1 empty interrupt ...\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_FIFO_1_EMPTY,
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

	/*
	 * Disable FIFO 1 to put it into a known state
	 */

	fprintf(stdout, "Disabling FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable clock 0 to put it into a known state; any clock should
	 * be disabled before programming it.
	 */

	fprintf(stdout, "Disabling programmable clocks ...\n");

	fprintf(stdout, "    Clock 0 ...\n");

	dm7820_status = DM7820_PrgClk_Set_Mode(board,
					       DM7820_PRGCLK_CLOCK_0,
					       DM7820_PRGCLK_MODE_DISABLED);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   FIFO 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set input clock to PCI write for FIFO 0 Read/Write Port Register
	 */

	fprintf(stdout, "Initializing FIFO 0 ...\n");

	fprintf(stdout, "    Setting FIFO 0 input clock ...\n");

	dm7820_status = DM7820_FIFO_Set_Input_Clock(board,
						    DM7820_FIFO_QUEUE_0,
						    DM7820_FIFO_INPUT_CLOCK_PCI_WRITE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

	/*
	 * Set output clock to Prog Clock 0 for FIFO 0 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting FIFO 0 output clock ...\n");

	dm7820_status = DM7820_FIFO_Set_Output_Clock(board,
						     DM7820_FIFO_QUEUE_0,
						     DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*
	 * Set data input to PCI data for FIFO 0
	 */

	fprintf(stdout, "    Setting FIFO 0 data input ...\n");

	dm7820_status = DM7820_FIFO_Set_Data_Input(board,
						   DM7820_FIFO_QUEUE_0,
						   DM7820_FIFO_0_DATA_INPUT_PCI_DATA);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

	/*
	 * Set FIFO 0 DREQ to Read Request
	 */

	fprintf(stdout, "    Setting FIFO 0 DREQ source ...\n");

	dm7820_status = DM7820_FIFO_Set_DMA_Request(board,
						    DM7820_FIFO_QUEUE_0,
						    DM7820_FIFO_DMA_REQUEST_WRITE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_DMA_Request()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   FIFO 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Set input clock to Prog Clock 0 FIFO 1 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting FIFO 1 input clock ...\n");

	dm7820_status = DM7820_FIFO_Set_Input_Clock(board,
						    DM7820_FIFO_QUEUE_1,
						    DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

	/*
	 * Set output clock to PCI Read for FIFO 1 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting FIFO 1 output clock ...\n");

	dm7820_status = DM7820_FIFO_Set_Output_Clock(board,
						     DM7820_FIFO_QUEUE_1,
						     DM7820_FIFO_INPUT_CLOCK_PCI_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*
	 * Set data input to Port 1 for FIFO 1
	 */

	fprintf(stdout, "    Setting FIFO 1 data input ...\n");

	dm7820_status = DM7820_FIFO_Set_Data_Input(board,
						   DM7820_FIFO_QUEUE_1,
						   DM7820_FIFO_1_DATA_INPUT_PORT_1);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

	/*
	 *Set FIFO 1 DREQ to Read Request
	 */

	fprintf(stdout, "    Setting FIFO 1 DREQ source ...\n");

	dm7820_status = DM7820_FIFO_Set_DMA_Request(board,
						    DM7820_FIFO_QUEUE_1,
						    DM7820_FIFO_DMA_REQUEST_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_DMA_Request()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Port 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Configuring Port 0 ...\n");

	dm7820_status = DM7820_StdIO_Set_IO_Mode(board,
						 DM7820_STDIO_PORT_0,
						 0xFFFF,
						 DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	dm7820_status = DM7820_StdIO_Set_Periph_Mode(board,
						     DM7820_STDIO_PORT_0,
						     0xFFFF,
						     DM7820_STDIO_PERIPH_FIFO_0);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Port 1 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Configuring Port 1 ...\n");

	dm7820_status = DM7820_StdIO_Set_IO_Mode(board,
						 DM7820_STDIO_PORT_1,
						 0x0000,
						 DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	dm7820_status = DM7820_StdIO_Set_Periph_Mode(board,
						     DM7820_STDIO_PORT_1,
						     0xFFFF,
						     DM7820_STDIO_PERIPH_FIFO_1);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Programmable clock 0 initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fprintf(stdout, "Initializing programmable clock 0 ...\n");

	/*
	 * Set master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting master clock ...\n");

	dm7820_status = DM7820_PrgClk_Set_Master(board,
						 DM7820_PRGCLK_CLOCK_0,
						 DM7820_PRGCLK_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Master()");

	/*
	 * Set clock start trigger so that clock starts immediately
	 */

	fprintf(stdout, "    Setting start trigger ...\n");

	dm7820_status = DM7820_PrgClk_Set_Start_Trigger(board,
							DM7820_PRGCLK_CLOCK_0,
							DM7820_PRGCLK_START_IMMEDIATE);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_PrgClk_Set_Start_Trigger()");

	/*
	 * Set clock stop trigger so that clock is never stopped
	 */

	fprintf(stdout, "    Setting stop trigger ...\n");

	dm7820_status = DM7820_PrgClk_Set_Stop_Trigger(board,
						       DM7820_PRGCLK_CLOCK_0,
						       DM7820_PRGCLK_STOP_NONE);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Stop_Trigger()");

	/*
	 * Set clock period to obtain 3.125 MHz frequency
	 */

	dm7820_status = DM7820_PrgClk_Set_Period(board,
						 DM7820_PRGCLK_CLOCK_0, 21);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary FIFO initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Enable FIFO's
	 */

	fprintf(stdout, "Enabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Enabling FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   DMA 0 Configuration
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 *  Initializing DMA 0
	 */

	fprintf(stdout, "Initializing DMA 0 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Initialize(board,
						   DM7820_FIFO_QUEUE_0,
						   BUF_NUM, BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Initialize()");

	/*
	 *  Configuring DMA 0
	 */

	fprintf(stdout, "    Configuring DMA 0 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Configure(board,
						  DM7820_FIFO_QUEUE_0,
						  DM7820_DMA_DEMAND_ON_PCI_TO_DM7820,
						  BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   DMA 1 Configuration
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 *  Initializing DMA 1
	 */

	fprintf(stdout, "Initializing DMA 1 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Initialize(board,
						   DM7820_FIFO_QUEUE_1,
						   BUF_NUM, BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Initialize()");

	/*
	 *  Configuring DMA 1
	 */

	fprintf(stdout, "    Configuring DMA 1 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Configure(board,
						  DM7820_FIFO_QUEUE_1,
						  DM7820_DMA_DEMAND_ON_DM7820_TO_PCI,
						  BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Create two buffers of size 4 megs, one for each side
	 * of the loopback.
	 */

	dm7820_status =
	    DM7820_FIFO_DMA_Create_Buffer(&dma_out_buf, DMA_BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()")

	    dm7820_status =
	    DM7820_FIFO_DMA_Create_Buffer(&dma_in_buf, DMA_BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");

	fprintf(stdout, "Installing user ISR ...\n");
	dm7820_status = DM7820_General_InstallISR(board, ISR);
	DM7820_Return_Status(dm7820_status, "DM7820_General_InstallISR()");

	fprintf(stdout, "Setting ISR priority ...\n");
	dm7820_status = DM7820_General_SetISRPriority(board, 99);
	DM7820_Return_Status(dm7820_status, "DM7820_General_SetISRPriority()");

	fprintf(stdout, "Filling Buffer\n");

	for (i = 0; i < SAMPLES; i++) {
		dma_out_buf[i] = i % 65535;
	}

	/*
	 * Writing user buffer to kernel DMA buffers.
	 */

	fprintf(stdout, "Copying user buffer to DMA buffers\n");

	dm7820_status = DM7820_FIFO_DMA_Write(board,
					      DM7820_FIFO_QUEUE_0,
					      dma_out_buf, BUF_NUM);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Write");

	/*
	 *  Enabling DMA channels 0 and 1
	 */

	fprintf(stdout, "Enabling and Starting DMA 0 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Enable(board,
					       DM7820_FIFO_QUEUE_0, 0xFF, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

	/*
	 * Allow for some time between starting second DMA channel
	 */

	sleep(1);

	dm7820_status = DM7820_FIFO_DMA_Enable(board,
					       DM7820_FIFO_QUEUE_1, 0xFF, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

	uint8_t fifo_status = 0;

	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Transfer from DMA 0 to FIFO 0 should now have started.
	 * Let's test to make sure the FIFO 0 is filling with data.
	 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

	fprintf(stdout, "Waiting for FIFO 0 to receive data ... \n");

	do {

		dm7820_status = DM7820_FIFO_Get_Status(board,
						       DM7820_FIFO_QUEUE_0,
						       DM7820_FIFO_STATUS_EMPTY,
						       &fifo_status);
		DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Get_Status()");

	} while (fifo_status);

	fprintf(stdout, "FIFO 0 is no longer empty\n");

	/*
	 * Enable FIFO 0 Empty Interrupt again.
	 * This is so we can watch FIFO 0 empty its data into FIFO 1
	 */

	fprintf(stdout, "Enabling FIFO 0 empty interrupt ...\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_FIFO_0_EMPTY,
							0xFF);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * The first ouput of FIFO 0 must be primed.  To do this you must
	 * temporarily set the output lock to PCI Read, then actually Read from
	 * the FIFO.  This will put the first value on the output port ready to
	 * be read by FIFO 1.  Only then can you set the output clock back to
	 * Prog Clock 0 and begin FIFO to FIFO transfer.
	 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

	fprintf(stdout, "    Loading first FIFO 0 Value ...\n");

	dm7820_status = DM7820_FIFO_Set_Output_Clock(board,
						     DM7820_FIFO_QUEUE_0,
						     DM7820_FIFO_INPUT_CLOCK_PCI_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	dm7820_status = DM7820_FIFO_Read(board, DM7820_FIFO_QUEUE_0, &data);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Read()");

	dm7820_status = DM7820_FIFO_Set_Output_Clock(board,
						     DM7820_FIFO_QUEUE_0,
						     DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_0);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * FIFO 0 primed and ready -- Now you can start the clock to begin output
	 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

	fprintf(stdout, "    Starting clock ...\n");

	dm7820_status = DM7820_PrgClk_Set_Mode(board,
					       DM7820_PRGCLK_CLOCK_0,
					       DM7820_PRGCLK_MODE_CONTINUOUS);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	fprintf(stdout, "Waiting for FIFO 1 to receive data ... \n");

	/*
	 * Wait for FIFO1 not empty.
	 */

	do {

		dm7820_status = DM7820_FIFO_Get_Status(board,
						       DM7820_FIFO_QUEUE_1,
						       DM7820_FIFO_STATUS_EMPTY,
						       &fifo_status);
		DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Get_Status()");

	} while (fifo_status);

	fprintf(stdout, "FIFO 1 is no longer empty\n");

	/*
	 * Sleep while waiting for FIFO 0 Empty.
	 */

	fprintf(stdout, "Waiting for FIFO 0 empty interrupt ...\n");

	do {

	} while (fifo0_empty_interrupts == 0);

	fprintf(stdout, "FIFO 0 empty interrupt received ...\n");

	/*
	 * Loop while waiting for interrupts from both DMA channels
	 * before attempting to read back that data.
	 */

	fprintf(stdout,
		"Waiting to receive the correct number of DMA 1 interrupts ...\n");
	do {

		fprintf(stdout, "\r dma_done_interrupts: %d",
			dma_done_interrupts);
		fflush(stdout);

	} while (dma_done_interrupts < BUF_NUM);

	/*
	 * Uninstall the user ISR
	 */

	fprintf(stdout, "\r dma_done_interrupts: %d", dma_done_interrupts);
	fprintf(stdout, "\nInterrupts received removing user ISR ...\n");

	dm7820_status = DM7820_General_RemoveISR(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_RemoveISR()");

	/*
	 * Reading DMA 1
	 */

	fprintf(stdout, "Reading DMA 1 ... \n");
	dm7820_status =
	    DM7820_FIFO_DMA_Read(board, DM7820_FIFO_QUEUE_1, dma_in_buf,
				 BUF_NUM);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Read()");

	/*
	 * Verify the data was successfully sent
	 */

	printf("Read Complete ... Testing Data ...\n");
	for (i = 0; i < SAMPLES; i++) {

		if (dma_in_buf[i] != dma_out_buf[i]) {

			int j;
			int start_index;
			int end_index;

			start_index = i - 10;
			end_index = i + 10;
			if (start_index < 0) {
				start_index = 0;
				end_index = 20;
			}

			fprintf(stdout, "\t     INDEX\tVALUE\tBUFFER\n");
			for (j = start_index; j < end_index; j++) {

				if (j == i) {
					fprintf(stdout, "-> ");
				}

				fprintf(stdout, "\tDMA: %d\t%d\t%d\n", j,
					j % 65535, dma_in_buf[j]);
			}

			error(EXIT_FAILURE, errno,
			      "ERROR: DMA Readback FAILED");
		}
		printf("\r");
	}

	printf("%d samples received.\n", SAMPLES);

	/*
	 * Clean up and exit
	 */

	dm7820_status = DM7820_FIFO_DMA_Free_Buffer(&dma_out_buf, DMA_BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Free_Buffer()");

	dm7820_status = DM7820_FIFO_DMA_Free_Buffer(&dma_in_buf, DMA_BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Free_Buffer()");

	clean_up();

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
