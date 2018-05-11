/**
    @file

    @brief
        This program demonstrates how an incoming pulse width can be measured
        using the DM7820.

    @verbatim

    The example program implements the block diagram drawn below

                           5MHz
                          __|_
                         |    | out
                  |-gate-|8254|------------o PIN -----o
                  |      |____|   ______              | CONNECT
                  |              |     A|--o PIN -----o
        PWM_IN ---|      |-count-|IncEnc|
       (strobe1)  |      |       |_____B|---o PIN -- connect to ground
                  |      |        ______
                  |      |-input-|      |
                  |--------clock-| FIFO |
                                 |______|

    The pulse width coming from the position sensor should be connected to
    strobe1 (Pin 2 on CN11).  This example uses PWM0 Output A (Pin 15 CN10) to
    emulate the position sensor input on strobe1.

    The strobe1 input is then internally used as the gate intput for
    Timer/Counter A0.  Timer/Counter A0 out is then externally used as the input
    for Incremental Encoder 1 Channel A (connect Pin 11 CN10 to Pin 47 CN11).
    The incremental encoder's count value is then inserted into the FIFO.

    Encoder count values are then read from the FIFO.  Taking a difference
    from those values will give the number of 8254 User Timer/Counter periods
    that have occured.  If that period is known value, the length of the Duty
    Cycle can then be calculated.

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

    $Id: dma_capture_buffers.c 33597 2008-11-26 16:46:30Z wtate $
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
 * Flag indicating user intent to exit the program
 */

uint8_t exit_program = 0;

/**
 * The size of the individual buffers we are asking to driver to create for
 * each DMA channel.
 */

#define BUF_SIZE 0x4000

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
/**
 * User Timer/Counter divisor, the smaller this number is the better your
 * resolution for measuring PW is.
 */
#define UTC_RATE 2

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
	 * Process the interrupt received
	 */

	DM7820_Return_Status(interrupt_info.error, "ISR Failed\n");

	switch (interrupt_info.source) {
	case DM7820_INTERRUPT_FIFO_1_DMA_DONE:
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

	fprintf(stdout, "\nDisabling DMA 1 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Enable(board,
					       DM7820_FIFO_QUEUE_1, 0x00, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

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
    Signal handler for SIGINT Control-C keyboard interrupt.

@param
    signal_number

    Signal number passed in from the kernel.

@warning
    One must be extremely careful about what functions are called from a signal
    handler.
 *******************************************************************************
*/

static void sigint_handler(int signal_number)
{
	exit_program = 0xff;
}

/**
*******************************************************************************
 * @brief
 *      Perform all digital I/O port initialization
 ******************************************************************************/
void do_digital_io()
{
	DM7820_Error dm7820_status;

	fprintf(stdout, "Setting Digital I/O Port 2 to Output ...\n");
	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_2, 0x0005,
				     DM7820_STDIO_MODE_PER_OUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");

	fprintf(stdout,
		"Setting Digital I/O Peripheral to User Timer/Counter A0 ...\n");
	dm7820_status =
	    DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0x0004,
					 DM7820_STDIO_PERIPH_CLK_OTHER);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	fprintf(stdout, "Setting Digital I/O Peripheral to PWM Out ...\n");
	dm7820_status =
	    DM7820_StdIO_Set_Periph_Mode(board, DM7820_STDIO_PORT_2, 0x0001,
					 DM7820_STDIO_PERIPH_PWM);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_Periph_Mode()");

	fprintf(stdout, "Initializing Digital I/O Port 1 as Input ...\n");

	/*
	 * Set each port 1 bit to input which enables incremental encoder inputs
	 */

	dm7820_status =
	    DM7820_StdIO_Set_IO_Mode(board, DM7820_STDIO_PORT_1, 0xFFFF,
				     DM7820_STDIO_MODE_INPUT);
	DM7820_Return_Status(dm7820_status, "DM7820_StdIO_Set_IO_Mode()");
}

/**
*******************************************************************************
 * @brief
 *      Perform all incremental encoder initialization
 ******************************************************************************/
void do_incenc()
{
	DM7820_Error dm7820_status;
	fprintf(stdout, "Disabling incremental encoder 1 ...\n");

	dm7820_status =
	    DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");

	fprintf(stdout, "Initializing incremental encoder 1 ...\n");

	/*
	 * Set master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting master clock ... \n");

	dm7820_status = DM7820_IncEnc_Set_Master(board, DM7820_INCENC_ENCODER_1,
						 DM7820_INCENC_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Set_Master()");

	fprintf(stdout, "    Disabling value register hold ...\n");

	dm7820_status =
	    DM7820_IncEnc_Enable_Hold(board, DM7820_INCENC_ENCODER_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable_Hold()");

	/*
	 * Enable only Inputs B/A transition from 0/0 to 0/1 when counting up
	 * Set single-ended
	 * Filter disabled
	 */

	fprintf(stdout, "    Configuring encoder ... \n");

	dm7820_status = DM7820_IncEnc_Configure(board,
						DM7820_INCENC_ENCODER_1,
						0xFE,
						DM7820_INCENC_INPUT_SINGLE_ENDED,
						0x00,
						DM7820_INCENC_CHANNEL_INDEPENDENT,
						0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Configure()");

	fprintf(stdout, "    Setting initial value for channel A ...\n");

	dm7820_status = DM7820_IncEnc_Set_Independent_Value(board,
							    DM7820_INCENC_ENCODER_1,
							    DM7820_INCENC_CHANNEL_A,
							    0x8000);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_IncEnc_Set_Independent_Value()");

	fprintf(stdout, "Enabling incremental encoder 1 ...\n");

	dm7820_status =
	    DM7820_IncEnc_Enable(board, DM7820_INCENC_ENCODER_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_IncEnc_Enable()");
}

/**
*******************************************************************************
 * @brief
 *      Perform all 8254 Timer/Counter initialization
 ******************************************************************************/
void do_8254()
{
	DM7820_Error dm7820_status;

	fprintf(stdout, "Initializing 8254 timer/counter A0 ...\n");

	/*
	 * Set timer clock source to 5 MHz clock
	 */

	fprintf(stdout, "    Selecting clock source ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Clock(board, DM7820_TMRCTR_TIMER_A_0,
				       DM7820_TMRCTR_CLOCK_5_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_TmrCtr_Select_Clock()");

	fprintf(stdout, "    Selecting clock gate ...\n");

	dm7820_status =
	    DM7820_TmrCtr_Select_Gate(board, DM7820_TMRCTR_TIMER_A_0,
				      DM7820_TMRCTR_GATE_STROBE_1);

	fprintf(stdout, "    Programming timer ...\n");

	dm7820_status = DM7820_TmrCtr_Program(board,
					      DM7820_TMRCTR_TIMER_A_0,
					      DM7820_TMRCTR_WAVEFORM_RATE_GENERATOR,
					      DM7820_TMRCTR_COUNT_MODE_BINARY,
					      UTC_RATE);
}

/**
*******************************************************************************
 * @brief
 *      Perform all PWM initialization
 ******************************************************************************/
void do_pwm()
{
	DM7820_Error dm7820_status;

	fprintf(stdout, "Initializing pulse width modulator 0 ...\n");

	/*
	 * Set period master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting period master clock ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Period_Master(board, DM7820_PWM_MODULATOR_0,
					 DM7820_PWM_PERIOD_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period_Master()");

	/*
	 * Set pulse width modulator period to obtain 10 kHz frequency.  The
	 * period is set as the number of master clock cycles, so 25 Mhz / 2500
	 * equals a period of 10 kHz
	 */

	fprintf(stdout, "    Setting period ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Period(board, DM7820_PWM_MODULATOR_0, 2500);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Period()");

	/*
	 * Set width master clock to 25 MHz clock
	 */

	fprintf(stdout, "    Setting width master clock ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width_Master(board, DM7820_PWM_MODULATOR_0,
					DM7820_PWM_WIDTH_MASTER_25_MHZ);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width_Master()");

	/*
	 * Set output A width to obtain 60% duty cycle.  The width is set as
	 * the number of master clock cycles, same as the period.  So a width
	 * of 1500 output / 2500 period = 60% duty cycle.  The period time is
	 * equal to the master clock period * the output width, so 40 ns * 1500
	 * equals 60000 nsec.
	 */

	fprintf(stdout, "    Setting output A width ...\n");

	dm7820_status =
	    DM7820_PWM_Set_Width(board, DM7820_PWM_MODULATOR_0,
				 DM7820_PWM_OUTPUT_A, 1500);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Set_Width()");

	fprintf(stdout, "    Enabling PWM ...\n");

	dm7820_status = DM7820_PWM_Enable(board, DM7820_PWM_MODULATOR_0, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_PWM_Enable()");
}

/**
*******************************************************************************
 * @brief
 *      Perform all FIFO initialization
 ******************************************************************************/
void do_fifo()
{
	DM7820_Error dm7820_status;

	/*
	 * Disable FIFO 0 to put it into a known state
	 */

	fprintf(stdout, "Disabling FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0x00);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	fprintf(stdout, "Initializing FIFO 1 ...\n");

	/*
	 * Set input clock to strobe 1 for FIFO 1 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting FIFO 1 input clock ...\n");

	dm7820_status = DM7820_FIFO_Set_Input_Clock(board,
						    DM7820_FIFO_QUEUE_1,
						    DM7820_FIFO_INPUT_CLOCK_STROBE_1);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Input_Clock()");

	/*
	 * Set output clock to PCI Read for FIFO 1 Read/Write Port Register
	 */

	fprintf(stdout, "    Setting FIFO 1 output clock ...\n");

	dm7820_status = DM7820_FIFO_Set_Output_Clock(board,
						     DM7820_FIFO_QUEUE_1,
						     DM7820_FIFO_OUTPUT_CLOCK_PCI_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Output_Clock()");

	/*
	 * Set data input to Incremental Encoder 1 A for FIFO 1
	 */

	fprintf(stdout, "    Setting FIFO 1 data input ...\n");

	dm7820_status = DM7820_FIFO_Set_Data_Input(board,
						   DM7820_FIFO_QUEUE_1,
						   DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_A);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_Data_Input()");

	/*
	 * Set FIFO 1 DREQ to Read Request
	 */

	fprintf(stdout, "    Setting FIFO 1 DREQ source ...\n");

	dm7820_status = DM7820_FIFO_Set_DMA_Request(board,
						    DM7820_FIFO_QUEUE_1,
						    DM7820_FIFO_DMA_REQUEST_READ);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Set_DMA_Request()");
}

/**
*******************************************************************************
 * @brief
 *      Perform all DMA initialization
 ******************************************************************************/
void do_dma()
{
	DM7820_Error dm7820_status;

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

	struct sigaction signal_action;

	uint8_t help_option = 0x00;
	uint8_t minor_option = 0x00;

	unsigned long int minor_number = 0;

	program_name = arguments[0];

	signal_action.sa_handler = sigint_handler;
	sigfillset(&(signal_action.sa_mask));
	signal_action.sa_flags = 0;

	if (sigaction(SIGINT, &signal_action, NULL) < 0) {
		error(EXIT_FAILURE, errno, "ERROR: sigaction() FAILED");
	}

	fprintf(stdout, "\n");
	fprintf(stdout, "\tDM7820 DMA Capture Example Program\n");
	fprintf(stdout, "\n");

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Process command line options
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

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
			   ############################################################## */

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
			   ############################################################## */

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
			   ############################################################## */

		case '?':
			usage();
			break;

			/*#################################################################
			   getopt_long() returned unexpected value
			   ############################################################## */

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
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

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

	/*
	 * Make sure FIFO 1 empty interrupt is disabled to prevent stray interrupts
	 */

	fprintf(stdout, "Disabling FIFO 1 empty interrupt ...\n");

	dm7820_status = DM7820_General_Enable_Interrupt(board,
							DM7820_INTERRUPT_FIFO_1_EMPTY,
							0x00);
	DM7820_Return_Status(dm7820_status,
			     "DM7820_General_Enable_Interrupt()");

	do_digital_io();
	do_incenc();
	do_8254();
	do_pwm();
	do_fifo();
	do_dma();

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Main processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Create a large buffer to place data from DMA.  This should be done AFTER
	 * the driver has created its buffers as the kernel space DMA buffers must
	 * be contiguous and ours do not have to be.
	 */

	dm7820_status =
	    DM7820_FIFO_DMA_Create_Buffer(&temp_buf, DMA_BUF_SIZE * 8);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Create_Buffer()");

	fprintf(stdout, "Installing user ISR ...\n");
	dm7820_status = DM7820_General_InstallISR(board, ISR);
	DM7820_Return_Status(dm7820_status, "DM7820_General_InstallISR()");

	fprintf(stdout, "Setting ISR priority ...\n");
	dm7820_status = DM7820_General_SetISRPriority(board, 99);
	DM7820_Return_Status(dm7820_status, "DM7820_General_SetISRPriority()");

	/*
	 * Enable FIFO
	 */

	fprintf(stdout, "Enabling FIFO 1 ...\n");

	dm7820_status = DM7820_FIFO_Enable(board, DM7820_FIFO_QUEUE_1, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_Enable()");

	/*
	 *  Enabling DMA channel 0
	 */

	fprintf(stdout, "Enabling and Starting DMA 1 ...\n");

	dm7820_status = DM7820_FIFO_DMA_Enable(board,
					       DM7820_FIFO_QUEUE_1, 0xFF, 0xFF);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Enable()");

	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Gather the data from the device and place it in our user buffer
	 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

	int temp_interrupts = dma_done_interrupts;

	/*
	 * Loop here waiting for 4 of the 8 buffers we allocated to fill with
	 * data from the FIFO/DMA.  This will be equivalent to a FIFO half-full
	 * DMA transfer.  The ISR in the driver will keep the DMA/FIFO going while
	 * we take some time to read the data from the DMA buffer in the device and
	 * place this data in our buffer.
	 */

	fprintf(stdout, "Waiting to receive all data from DMA ...\n\n");

	uint32_t offset = 0;

	do {

		/*
		 * Check to see if we received an interrupt this time
		 */

		if (dma_done_interrupts != temp_interrupts) {
			temp_interrupts = dma_done_interrupts;

			/*
			 * Check to see if 4 DMA transfers have successfully completed.
			 * This also denotes our FIFO was half full and has been transferred
			 * to DMA in 4 small transfers (based on our DMA configuration).
			 */

			if ((temp_interrupts % 4 == 0) && (temp_interrupts > 0)) {
				/*
				 * Read the 4 buffers that have completed transfer.
				 * Also the offset must be adjusted here so the DMA data is read
				 * into the next block of our buffer.
				 */

				dm7820_status =
				    DM7820_FIFO_DMA_Read(board,
							 DM7820_FIFO_QUEUE_1,
							 (temp_buf +
							  (BUF_SIZE * offset)),
							 4);
				DM7820_Return_Status(dm7820_status,
						     "DM7820_FIFO_DMA_Read()");

				fprintf(stdout, "Retrieved DMA data ... \n");

				/*
				 * Increase the offset of the user-space buffer to which our DMA
				 * data will be copied.
				 */

				offset += 2;
			}

		}

		/*
		 * Loop until we receive 8 megs or 4M samples.
		 */

	} while (temp_interrupts < 32 && !exit_program);

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
	 * Loop Buffer Size*2 times to print out all the samples.
	 */

	for (i = 1; i < DMA_BUF_SIZE * 2; i++) {

		/*
		 * Read each sample from the buffer and print it.
		 */

		read_data = temp_buf[i] - temp_buf[i - 1];

		fprintf(stdout, "\t     %d\t%6.2f ns\n", i,
			(float)(read_data) * 1000. * ((float)UTC_RATE / 5));
	}

	fprintf(stdout, "\n%d samples received.\n", i);

	/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	 * Clean up and exit.
	 @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@*/

	/*
	 * Free our large buffer
	 */

	dm7820_status =
	    DM7820_FIFO_DMA_Free_Buffer(&temp_buf, DMA_BUF_SIZE * 8);
	DM7820_Return_Status(dm7820_status, "DM7820_FIFO_DMA_Free_Buffer()");

	clean_up();

	fprintf(stdout, "\n");
	fprintf(stdout, "Successful end of example program.\n");

	exit(EXIT_SUCCESS);
}
