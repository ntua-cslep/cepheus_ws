/**
    @file

    @brief
        Example program which demonstrates how to use DMA to continuously
        acquire samples.

    @verbatim

        This example program demonstrates the DM7820's ability to continuously
        sample data from FIFO 0 by using DMA.  This example will sample 8Mb of
        data (4 Million Samples) at a rate of 1 Mhz.  We read DMA samples from
        the device then dump them to a file on disk.  We use FILE IO here as
        attempting to create buffers large enough to hold all the data we expect
        to gather can cause failures.  Because we are using FILE I/O the speed
        at which we can continuously sample is restricted by how fast the I/O
        operations can finish.

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

    $Id: dma_capture_file.c 79285 2014-05-21 20:37:42Z rgroner $
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
#include <fcntl.h>

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
 * Device descriptor
 */

DM7820_Board_Descriptor *board;

/**
 * File handle for the data file.
 */

int test_data;

/**
 * Constant pathname and filename for the data file.
 */

#define DAT_FILE "./test.dat"

/**
 * The size of the individual buffers we are asking to driver to create for
 * each DMA channel.  If you receive a memory error when trying to run
 * this example, decrease this size.
 */

#define BUF_SIZE 0x40000

/**
 * The number of buffers we want for each DMA channel.
 */

#define BUF_NUM 8

/**
 * The number of samples that can be held by the list of buffers we created.
 */

#define SAMPLES ((BUF_SIZE * BUF_NUM)/2)

/**
 * The total space available in the device for DMA at any one time (4MB).
 */

#define DMA_BUF_SIZE (BUF_SIZE * BUF_NUM)

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
	case DM7820_INTERRUPT_FIFO_0_DMA_DONE:
		dma_done_interrupts++;
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

	fprintf(stdout, "\nDisabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Closing device ...\n");

	dm7820_status = DM7820_General_Close_Board(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_Close_Board()");

	fprintf(stdout, "Removing temporary data file ...\n");

	remove(DAT_FILE);

	sleep(2);
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
	uint16_t *temp_buf = NULL;
	uint16_t read_data;
	int status;
	uint32_t i;

	struct option options[] = {
		{"help", 0, 0, 1},
		{"minor", 1, 0, 2},
		{0, 0, 0, 0}
	};

	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;

	unsigned long int minor_number = 0;

	program_name = arguments[0];

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 DMA Capture Example Program\n");
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

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   FIFO generic initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable FIFO 0 to put it into a known state
	 */

	fprintf(stdout, "Disabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Initializing FIFO 0 ...\n");

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

	fprintf(stdout, "    Setting FIFO 0 input clock ...\n");

	dm7820_status = DM7820_FIFO_Set_Input_Clock(board,
						    DM7820_FIFO_QUEUE_0,
						    DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_0);

	/*
	 * Set output clock to Prog Clock 0 for FIFO 0 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting FIFO 0 output clock ...\n");

	dm7820_status = DM7820_FIFO_Set_Output_Clock(board,
						     DM7820_FIFO_QUEUE_0,
						     DM7820_FIFO_OUTPUT_CLOCK_PCI_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*
	 * Set data input to Port 0 for FIFO 0
	 */

	fprintf(stdout, "    Setting FIFO 0 data input ...\n");

	dm7820_status = DM7820_FIFO_Set_Data_Input(board,
						   DM7820_FIFO_QUEUE_0,
						   DM7820_FIFO_0_DATA_INPUT_PORT_0);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

	/*
	 * Set FIFO 0 DREQ to Read Request
	 */

	fprintf(stdout, "    Setting FIFO 0 DREQ source ...\n");

	dm7820_status = DM7820_FIFO_Set_DMA_Request(board,
						    DM7820_FIFO_QUEUE_0,
						    DM7820_FIFO_DMA_REQUEST_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_DMA_Request()");

	fprintf(stdout, "Configuring Port 0 ...\n");

	dm7820_status = DM7820_StdIO_Set_IO_Mode(board,
						 DM7820_STDIO_PORT_0,
						 0x0000,
						 DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

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
	 * Set clock period to obtain 1 MHz frequency
	 */

	dm7820_status = DM7820_PrgClk_Set_Period(board,
						 DM7820_PRGCLK_CLOCK_0, 24);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Period()");

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
						  DM7820_DMA_DEMAND_ON_DM7820_TO_PCI,
						  BUF_SIZE);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Configure()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	dm7820_status =
	    DM7820_FIFO_DMA_Create_Buffer(&temp_buf, DMA_BUF_SIZE / 2);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");

	fprintf(stdout, "Installing user ISR ...\n");
	dm7820_status = DM7820_General_InstallISR(board, ISR);
	DM7820_Return_Status(dm7820_status, "DM7820_General_InstallISR()");

	fprintf(stdout, "Setting ISR priority ...\n");
	dm7820_status = DM7820_General_SetISRPriority(board, 99);
	DM7820_Return_Status(dm7820_status, "DM7820_General_SetISRPriority()");

	/*
	 * Starting Clock
	 */

	fprintf(stdout, "    Starting clock ...\n");

	dm7820_status = DM7820_PrgClk_Set_Mode(board,
					       DM7820_PRGCLK_CLOCK_0,
					       DM7820_PRGCLK_MODE_CONTINUOUS);
	DM7820_Return_Status(dm7820_status, "DM7820_PrgClk_Set_Mode()");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Secondary FIFO initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Enable FIFO's
	 */

	fprintf(stdout, "Enabling FIFO 0 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*
	 *  Enabling DMA channel 0
	 */

	fprintf(stdout, "Enabling and Starting DMA 0 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Enable(board,
					       DM7820_FIFO_QUEUE_0, 0xFF, 0xFF);
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
	 * Gather the data from the device and dump it to file
	 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

	int temp_interrupts = dma_done_interrupts;

	/*
	 * Create/Open a file to dump the DMA data to.
	 */

	test_data = open(DAT_FILE, O_WRONLY | O_CREAT, 0666);

	if (test_data < 0)
		error(EXIT_FAILURE, errno, "open() FAILED\n");

	/*
	 * Loop here waiting for 4 of the 8 buffer we allocated to fill with
	 * data from the FIFO/DMA.  This will be equivalent to a FIFO half-full
	 * DMA transfer.  The ISR in the driver will keep the DMA/FIFO going while
	 * we take some time to read the data from the DMA buffer in the device and
	 * dump this data to a space on disk.
	 */

	fprintf(stdout, "Waiting to receive all data from DMA ...\n\n");

	do {
		/*
		 * Check to see if we received an interrupt this time
		 */

		if (dma_done_interrupts != temp_interrupts) {
			temp_interrupts = dma_done_interrupts;
			//fprintf(stdout, "Interrupt: %d\n", temp_interrupts);

			/*
			 * Check to see if 4 DMA transfers have successfully completed.
			 * This also denotes our FIFO was half full and has been transferred
			 * to DMA in 4 small transfers (based on our DMA configuration).
			 */

			if ((temp_interrupts % 4 == 0) && (temp_interrupts > 0)) {
				/*
				 * Read the 4 buffers that have completed transfer.
				 */

				dm7820_status =
				    DM7820_FIFO_DMA_Read(board,
							 DM7820_FIFO_QUEUE_0,
							 temp_buf, BUF_NUM / 2);
				DM7820_Return_Status(dm7820_status,
						     "DM7820_DMA_Read()");

				/*
				 * Dump the buffers we read from the device to a file on disk.
				 */

				dm7820_status =
				    write(test_data, temp_buf,
					  DMA_BUF_SIZE / 2);

				if (dm7820_status != DMA_BUF_SIZE / 2) {

					dm7820_status = close(test_data);

					clean_up();

					error(EXIT_FAILURE, errno,
					      "ERROR: write() FAILED %d",
					      dm7820_status);
				}

				fprintf(stdout, "Data written to file ... \n");
			}
		}

		/*
		 * Loop until we receive 8 megs or 4M samples.
		 */

	} while (temp_interrupts < 32);

	dm7820_status = close(test_data);

	if (dm7820_status != 0) {
		error(EXIT_FAILURE, errno, "ERROR: close() FAILED\n");
	}

	/*
	 * Uninstall the user ISR
	 */

	fprintf(stdout, "\nInterrupts received removing user ISR ...\n");

	dm7820_status = DM7820_General_RemoveISR(board);
	DM7820_Return_Status(dm7820_status, "DM7820_General_RemoveISR()");

	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Verify data has been successfully sent
	 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

	fprintf(stdout, "Read Complete ... Testing Data ...\n");
	fprintf(stdout, "\n\t     INDEX\tVALUE\n");

	/*
	 * Loop Buffer Size/2 times to print out all the samples.
	 */

	test_data = open(DAT_FILE, O_RDONLY);

	if (test_data < 0) {
		error(EXIT_FAILURE, errno, "ERROR: open() FAILED\n");
	}

	for (i = 0; i < DMA_BUF_SIZE * 2; i++) {

		/*
		 * Read each sample from the file and print it.
		 */
		dm7820_status = read(test_data, &read_data, sizeof(uint16_t));

		if (dm7820_status != sizeof(uint16_t)) {

			/*
			 * An error occurred, try to fail gracefully by cleaing up first.
			 */

			fprintf(stdout, "\n\t     %d\t%d", i, read_data);

			close(test_data);

			clean_up();

			error(EXIT_FAILURE,
			      errno,
			      "ERROR: read() FAILED %d\n", dm7820_status);
		}

		fprintf(stdout, "\t     %d\t%d\r", i, read_data);
	}

	/*
	 * We have finished reading, close the file.
	 */

	close(test_data);

	fprintf(stdout, "\n%d samples received.\n", i);

	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Clean up and exit.
	 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

	dm7820_status =
	    DM7820_FIFO_DMA_Free_Buffer(&temp_buf, DMA_BUF_SIZE / 2);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Free_Buffer()");

	clean_up();

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
