/**
    @file

    @brief
        DM7820 user library source code

    @verbatim
//----------------------------------------------------------------------------
//  COPYRIGHT (C) RTD EMBEDDED TECHNOLOGIES, INC.  ALL RIGHTS RESERVED.
//
//  This software package is dual-licensed.  Source code that is compiled for
//  kernel mode execution is licensed under the GNU General Public License
//  version 2.  For a copy of this license, refer to the file
//  LICENSE_GPLv2.TXT (which should be included with this software) or contact
//  the Free Software Foundation.  Source code that is compiled for user mode
//  execution is licensed under the RTD End-User Software License Agreement.
//  For a copy of this license, refer to LICENSE.TXT or contact RTD Embedded
//  Technologies, Inc.  Using this software indicates agreement with the
//  license terms listed above.
//----------------------------------------------------------------------------
    @endverbatim

    $Id: librtd-dm7820.c 89872 2015-07-08 16:05:17Z rgroner $
*/

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include "dm7820_globals.h"
#include "dm7820_ioctl.h"
#include "dm7820_library.h"
#include "dm7820_registers.h"

/**
 * @defgroup DM7820_Library_Source DM7820 user library source code
 * @{
 */

/*=============================================================================
Private functions
 =============================================================================*/

/**
 * @defgroup DM7820_Library_Private_Functions DM7820 user library source code private functions
 * @{
 */

 /**
  * @internal
  */

/**
*******************************************************************************
@brief
    Validate an advanced interrupt passed into the library.

@param
    interrupt

    The advanced interrupt to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      interrupt is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_AdvInt_Interrupt(dm7820_advint_interrupt interrupt)
{
	switch (interrupt) {
	case DM7820_ADVINT_INTERRUPT_0:
	case DM7820_ADVINT_INTERRUPT_1:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an advanced interrupt master clock passed into the library.

@param
    master

    The advanced interrupt master clock to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_ADVINT_MASTER_RESERVED.
 *******************************************************************************
 */

static int DM7820_Validate_AdvInt_Master(dm7820_advint_master_clock master)
{
	switch (master) {
	case DM7820_ADVINT_MASTER_25_MHZ:
	case DM7820_ADVINT_MASTER_8254_A_0:
	case DM7820_ADVINT_MASTER_8254_A_1:
	case DM7820_ADVINT_MASTER_8254_A_2:
	case DM7820_ADVINT_MASTER_8254_B_0:
	case DM7820_ADVINT_MASTER_8254_B_1:
	case DM7820_ADVINT_MASTER_8254_B_2:
	case DM7820_ADVINT_MASTER_PROG_CLOCK_0:
	case DM7820_ADVINT_MASTER_PROG_CLOCK_1:
	case DM7820_ADVINT_MASTER_PROG_CLOCK_2:
	case DM7820_ADVINT_MASTER_PROG_CLOCK_3:
	case DM7820_ADVINT_MASTER_STROBE_1:
	case DM7820_ADVINT_MASTER_STROBE_2:
	case DM7820_ADVINT_MASTER_INV_STROBE_1:
	case DM7820_ADVINT_MASTER_INV_STROBE_2:
		break;

	case DM7820_ADVINT_MASTER_RESERVED:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an advanced interrupt mode passed into the library.

@param
    mode

    The advanced interrupt mode to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      mode is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_AdvInt_Mode(dm7820_advint_mode mode)
{
	switch (mode) {
	case DM7820_ADVINT_MODE_DISABLED:
	case DM7820_ADVINT_MODE_STROBE:
	case DM7820_ADVINT_MODE_MATCH:
	case DM7820_ADVINT_MODE_EVENT:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a FIFO data input passed into the library.

@param
    fifo

    FIFO to validate data input for.

@param
    input

    The FIFO data input to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      input is not valid.

        @arg \c
            EOPNOTSUPP  input is not supported by fifo.
 *******************************************************************************
 */

static int
DM7820_Validate_FIFO_Data_Input(dm7820_fifo_queue fifo,
				dm7820_fifo_data_input input)
{
	switch (input) {
	case DM7820_FIFO_0_DATA_INPUT_PCI_DATA:
	case DM7820_FIFO_0_DATA_INPUT_PORT_0:
	case DM7820_FIFO_0_DATA_INPUT_PORT_2:
	case DM7820_FIFO_0_DATA_INPUT_FIFO_0_OUTPUT:
	case DM7820_FIFO_1_DATA_INPUT_PCI_DATA:
	case DM7820_FIFO_1_DATA_INPUT_PORT_1:
	case DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_A:
	case DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_B:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		switch (input) {
		case DM7820_FIFO_0_DATA_INPUT_PCI_DATA:
		case DM7820_FIFO_0_DATA_INPUT_PORT_0:
		case DM7820_FIFO_0_DATA_INPUT_PORT_2:
		case DM7820_FIFO_0_DATA_INPUT_FIFO_0_OUTPUT:
			break;

		default:
			errno = EOPNOTSUPP;
			return -1;
			break;
		}

		break;

	case DM7820_FIFO_QUEUE_1:
		switch (input) {
		case DM7820_FIFO_1_DATA_INPUT_PCI_DATA:
		case DM7820_FIFO_1_DATA_INPUT_PORT_1:
		case DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_A:
		case DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_B:
			break;

		default:
			errno = EOPNOTSUPP;
			return -1;
			break;
		}

		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a FIFO DMA request source passed into the library.

@param
    source

    The FIFO DMA request source to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      source is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_FIFO_DMA_Request(dm7820_fifo_dma_request source)
{
	switch (source) {
	case DM7820_FIFO_DMA_REQUEST_READ:
	case DM7820_FIFO_DMA_REQUEST_NOT_EMPTY:
	case DM7820_FIFO_DMA_REQUEST_WRITE:
	case DM7820_FIFO_DMA_REQUEST_NOT_FULL:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a FIFO input clock passed into the library.

@param
    clock

    The FIFO input clock to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      clock is not valid.

        @arg \c
            EOPNOTSUPP  clock is equal to DM7820_FIFO_INPUT_CLOCK_RESERVED_1,
                        DM7820_FIFO_INPUT_CLOCK_RESERVED_2,
                        DM7820_FIFO_INPUT_CLOCK_RESERVED_3, or
                        DM7820_FIFO_INPUT_CLOCK_RESERVED_4.
 *******************************************************************************
 */

static int DM7820_Validate_FIFO_Input_Clock(dm7820_fifo_input_clock clock)
{
	switch (clock) {
	case DM7820_FIFO_INPUT_CLOCK_25_MHZ:
	case DM7820_FIFO_INPUT_CLOCK_8254_A_0:
	case DM7820_FIFO_INPUT_CLOCK_8254_A_1:
	case DM7820_FIFO_INPUT_CLOCK_8254_A_2:
	case DM7820_FIFO_INPUT_CLOCK_8254_B_0:
	case DM7820_FIFO_INPUT_CLOCK_8254_B_1:
	case DM7820_FIFO_INPUT_CLOCK_8254_B_2:
	case DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_0:
	case DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_1:
	case DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_2:
	case DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_3:
	case DM7820_FIFO_INPUT_CLOCK_STROBE_1:
	case DM7820_FIFO_INPUT_CLOCK_STROBE_2:
	case DM7820_FIFO_INPUT_CLOCK_INV_STROBE_1:
	case DM7820_FIFO_INPUT_CLOCK_INV_STROBE_2:
	case DM7820_FIFO_INPUT_CLOCK_ADVANCED_INT_0:
	case DM7820_FIFO_INPUT_CLOCK_ADVANCED_INT_1:
	case DM7820_FIFO_INPUT_CLOCK_8254_INT:
	case DM7280_FIFO_INPUT_CLOCK_INC_ENCODER_0_INT:
	case DM7280_FIFO_INPUT_CLOCK_INC_ENCODER_1_INT:
	case DM7820_FIFO_INPUT_CLOCK_PWM_0_INT:
	case DM7820_FIFO_INPUT_CLOCK_PWM_1_INT:
	case DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_0_INT:
	case DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_1_INT:
	case DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_2_INT:
	case DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_3_INT:
	case DM7820_FIFO_INPUT_CLOCK_PCI_READ:
	case DM7820_FIFO_INPUT_CLOCK_PCI_WRITE:
		break;

	case DM7820_FIFO_INPUT_CLOCK_RESERVED_1:
	case DM7820_FIFO_INPUT_CLOCK_RESERVED_2:
	case DM7820_FIFO_INPUT_CLOCK_RESERVED_3:
	case DM7820_FIFO_INPUT_CLOCK_RESERVED_4:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a FIFO output clock passed into the library.

@param
    clock

    The FIFO output clock to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      clock is not valid.

        @arg \c
            EOPNOTSUPP  clock is equal to DM7820_FIFO_OUTPUT_CLOCK_RESERVED_1,
                        DM7820_FIFO_OUTPUT_CLOCK_RESERVED_2,
                        DM7820_FIFO_OUTPUT_CLOCK_RESERVED_3, or
                        DM7820_FIFO_OUTPUT_CLOCK_RESERVED_4.
 *******************************************************************************
 */

static int DM7820_Validate_FIFO_Output_Clock(dm7820_fifo_output_clock clock)
{
	switch (clock) {
	case DM7820_FIFO_OUTPUT_CLOCK_25_MHZ:
	case DM7820_FIFO_OUTPUT_CLOCK_8254_A_0:
	case DM7820_FIFO_OUTPUT_CLOCK_8254_A_1:
	case DM7820_FIFO_OUTPUT_CLOCK_8254_A_2:
	case DM7820_FIFO_OUTPUT_CLOCK_8254_B_0:
	case DM7820_FIFO_OUTPUT_CLOCK_8254_B_1:
	case DM7820_FIFO_OUTPUT_CLOCK_8254_B_2:
	case DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_0:
	case DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_1:
	case DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_2:
	case DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_3:
	case DM7820_FIFO_OUTPUT_CLOCK_STROBE_1:
	case DM7820_FIFO_OUTPUT_CLOCK_STROBE_2:
	case DM7820_FIFO_OUTPUT_CLOCK_INV_STROBE_1:
	case DM7820_FIFO_OUTPUT_CLOCK_INV_STROBE_2:
	case DM7820_FIFO_OUTPUT_CLOCK_ADVANCED_INT_0:
	case DM7820_FIFO_OUTPUT_CLOCK_ADVANCED_INT_1:
	case DM7820_FIFO_OUTPUT_CLOCK_8254_INT:
	case DM7280_FIFO_OUTPUT_CLOCK_INC_ENCODER_0_INT:
	case DM7280_FIFO_OUTPUT_CLOCK_INC_ENCODER_1_INT:
	case DM7820_FIFO_OUTPUT_CLOCK_PWM_0_INT:
	case DM7820_FIFO_OUTPUT_CLOCK_PWM_1_INT:
	case DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_0_INT:
	case DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_1_INT:
	case DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_2_INT:
	case DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_3_INT:
	case DM7820_FIFO_OUTPUT_CLOCK_PCI_READ:
	case DM7820_FIFO_OUTPUT_CLOCK_PCI_WRITE:
		break;

	case DM7820_FIFO_OUTPUT_CLOCK_RESERVED_1:
	case DM7820_FIFO_OUTPUT_CLOCK_RESERVED_2:
	case DM7820_FIFO_OUTPUT_CLOCK_RESERVED_3:
	case DM7820_FIFO_OUTPUT_CLOCK_RESERVED_4:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a FIFO passed into the library.

@param
    queue

    The FIFO to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      queue is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_FIFO_Queue(dm7820_fifo_queue queue)
{
	switch (queue) {
	case DM7820_FIFO_QUEUE_0:
	case DM7820_FIFO_QUEUE_1:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a FIFO status condition passed into the library

@param
    condition

    The FIFO status condition to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      condition is not valid.
 *******************************************************************************
 */

static int
DM7820_Validate_FIFO_Status_Condition(dm7820_fifo_status_condition condition)
{
	switch (condition) {
	case DM7820_FIFO_STATUS_READ_REQUEST:
	case DM7820_FIFO_STATUS_WRITE_REQUEST:
	case DM7820_FIFO_STATUS_FULL:
	case DM7820_FIFO_STATUS_EMPTY:
	case DM7820_FIFO_STATUS_OVERFLOW:
	case DM7820_FIFO_STATUS_UNDERFLOW:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
};

/**
*******************************************************************************
@brief
    Validate an interrupt source passed into the library

@param
    source

    The interrupt source to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      source is not valid.

        @arg \c
            EOPNOTSUPP  source is valid but cannot be modified from user space.
 *******************************************************************************
 */

static int
DM7820_Validate_General_Interrupt_Source(dm7820_interrupt_source source)
{
	switch (source) {
	case DM7820_INTERRUPT_ADVINT_0:
	case DM7820_INTERRUPT_ADVINT_1:
	case DM7820_INTERRUPT_FIFO_0_EMPTY:
	case DM7820_INTERRUPT_FIFO_0_FULL:
	case DM7820_INTERRUPT_FIFO_0_OVERFLOW:
	case DM7820_INTERRUPT_FIFO_0_READ_REQUEST:
	case DM7820_INTERRUPT_FIFO_0_UNDERFLOW:
	case DM7820_INTERRUPT_FIFO_0_WRITE_REQUEST:
	case DM7820_INTERRUPT_FIFO_1_EMPTY:
	case DM7820_INTERRUPT_FIFO_1_FULL:
	case DM7820_INTERRUPT_FIFO_1_OVERFLOW:
	case DM7820_INTERRUPT_FIFO_1_READ_REQUEST:
	case DM7820_INTERRUPT_FIFO_1_UNDERFLOW:
	case DM7820_INTERRUPT_FIFO_1_WRITE_REQUEST:
	case DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER:
	case DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER:
	case DM7820_INTERRUPT_INCENC_0_CHANNEL_B_NEGATIVE_ROLLOVER:
	case DM7820_INTERRUPT_INCENC_0_CHANNEL_B_POSITIVE_ROLLOVER:
	case DM7820_INTERRUPT_INCENC_1_CHANNEL_A_NEGATIVE_ROLLOVER:
	case DM7820_INTERRUPT_INCENC_1_CHANNEL_A_POSITIVE_ROLLOVER:
	case DM7820_INTERRUPT_INCENC_1_CHANNEL_B_NEGATIVE_ROLLOVER:
	case DM7820_INTERRUPT_INCENC_1_CHANNEL_B_POSITIVE_ROLLOVER:
	case DM7820_INTERRUPT_PRGCLK_0:
	case DM7820_INTERRUPT_PRGCLK_1:
	case DM7820_INTERRUPT_PRGCLK_2:
	case DM7820_INTERRUPT_PRGCLK_3:
	case DM7820_INTERRUPT_PWM_0:
	case DM7820_INTERRUPT_PWM_1:
	case DM7820_INTERRUPT_TMRCTR_A_0:
	case DM7820_INTERRUPT_TMRCTR_A_1:
	case DM7820_INTERRUPT_TMRCTR_A_2:
	case DM7820_INTERRUPT_TMRCTR_B_0:
	case DM7820_INTERRUPT_TMRCTR_B_1:
	case DM7820_INTERRUPT_TMRCTR_B_2:
		break;

	case DM7820_INTERRUPT_FIFO_0_DMA_DONE:
	case DM7820_INTERRUPT_FIFO_1_DMA_DONE:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an incremental encoder channel passed into the library

@param
    channel

    The incremental encoder channel to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      channel is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_IncEnc_Channel(dm7820_incenc_channel channel)
{
	switch (channel) {
	case DM7820_INCENC_CHANNEL_A:
	case DM7820_INCENC_CHANNEL_B:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an incremental encoder channel mode passed into the library

@param
    mode

    The incremental encoder channel mode to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      mode is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_IncEnc_Channel_Mode(dm7820_incenc_channel_mode mode)
{
	switch (mode) {
	case DM7820_INCENC_CHANNEL_INDEPENDENT:
	case DM7820_INCENC_CHANNEL_JOINED:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an incremental encoder passed into the library

@param
    encoder

    The incremental encoder to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_IncEnc_Encoder(dm7820_incenc_encoder encoder)
{
	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
	case DM7820_INCENC_ENCODER_1:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an incremental encoder input mode passed into the library

@param
    mode

    The incremental encoder input mode to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      mode is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_IncEnc_Input_Mode(dm7820_incenc_input_mode mode)
{
	switch (mode) {
	case DM7820_INCENC_INPUT_SINGLE_ENDED:
	case DM7820_INCENC_INPUT_DIFFERENTIAL:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an incremental encoder master clock passed into the library.

@param
    master

    The incremental encoder master clock to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_INCENC_MASTER_RESERVED.
 *******************************************************************************
 */

static int DM7820_Validate_IncEnc_Master(dm7820_incenc_master_clock master)
{
	switch (master) {
	case DM7820_INCENC_MASTER_25_MHZ:
	case DM7820_INCENC_MASTER_8254_A_0:
	case DM7820_INCENC_MASTER_8254_A_1:
	case DM7820_INCENC_MASTER_8254_A_2:
	case DM7820_INCENC_MASTER_8254_B_0:
	case DM7820_INCENC_MASTER_8254_B_1:
	case DM7820_INCENC_MASTER_8254_B_2:
	case DM7820_INCENC_MASTER_PROG_CLOCK_0:
	case DM7820_INCENC_MASTER_PROG_CLOCK_1:
	case DM7820_INCENC_MASTER_PROG_CLOCK_2:
	case DM7820_INCENC_MASTER_PROG_CLOCK_3:
	case DM7820_INCENC_MASTER_STROBE_1:
	case DM7820_INCENC_MASTER_STROBE_2:
	case DM7820_INCENC_MASTER_INV_STROBE_1:
	case DM7820_INCENC_MASTER_INV_STROBE_2:
		break;

	case DM7820_INCENC_MASTER_RESERVED:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an incremental encoder status condition passed into the library

@param
    condition

    The encoder status condition to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      condition is not valid.
 *******************************************************************************
 */

static int
DM7820_Validate_IncEnc_Status_Condition(dm7820_incenc_status_condition
					condition)
{
	switch (condition) {
	case DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER:
	case DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER:
	case DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER:
	case DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a pulse width modulator passed into the library

@param
    pwm

    The pulse width modulator to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      pwm is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_PWM_Modulator(dm7820_pwm_modulator pwm)
{
	switch (pwm) {
	case DM7820_PWM_MODULATOR_0:
	case DM7820_PWM_MODULATOR_1:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a pulse width modulator output passed into the library

@param
    output

    The pulse width modulator output to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      output is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_PWM_Output(dm7820_pwm_output output)
{
	switch (output) {
	case DM7820_PWM_OUTPUT_A:
	case DM7820_PWM_OUTPUT_B:
	case DM7820_PWM_OUTPUT_C:
	case DM7820_PWM_OUTPUT_D:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a pulse width modulator period master clock passed into the
    library.

@param
    master

    The pulse width modulator period master clock to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_PWM_PERIOD_MASTER_RESERVED.
 *******************************************************************************
 */

static int
DM7820_Validate_PWM_Period_Master(dm7820_pwm_period_master_clock master)
{
	switch (master) {
	case DM7820_PWM_PERIOD_MASTER_25_MHZ:
	case DM7820_PWM_PERIOD_MASTER_8254_A_0:
	case DM7820_PWM_PERIOD_MASTER_8254_A_1:
	case DM7820_PWM_PERIOD_MASTER_8254_A_2:
	case DM7820_PWM_PERIOD_MASTER_8254_B_0:
	case DM7820_PWM_PERIOD_MASTER_8254_B_1:
	case DM7820_PWM_PERIOD_MASTER_8254_B_2:
	case DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_0:
	case DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_1:
	case DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_2:
	case DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_3:
	case DM7820_PWM_PERIOD_MASTER_STROBE_1:
	case DM7820_PWM_PERIOD_MASTER_STROBE_2:
	case DM7820_PWM_PERIOD_MASTER_INV_STROBE_1:
	case DM7820_PWM_PERIOD_MASTER_INV_STROBE_2:
		break;

	case DM7820_PWM_PERIOD_MASTER_RESERVED:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a pulse width modulator width master clock passed into the library.

@param
    master

    The pulse width modulator width master clock to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_PWM_WIDTH_MASTER_RESERVED.
 *******************************************************************************
 */

static int
DM7820_Validate_PWM_Width_Master(dm7820_pwm_width_master_clock master)
{
	switch (master) {
	case DM7820_PWM_WIDTH_MASTER_25_MHZ:
	case DM7820_PWM_WIDTH_MASTER_8254_A_0:
	case DM7820_PWM_WIDTH_MASTER_8254_A_1:
	case DM7820_PWM_WIDTH_MASTER_8254_A_2:
	case DM7820_PWM_WIDTH_MASTER_8254_B_0:
	case DM7820_PWM_WIDTH_MASTER_8254_B_1:
	case DM7820_PWM_WIDTH_MASTER_8254_B_2:
	case DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_0:
	case DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_1:
	case DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_2:
	case DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_3:
	case DM7820_PWM_WIDTH_MASTER_STROBE_1:
	case DM7820_PWM_WIDTH_MASTER_STROBE_2:
	case DM7820_PWM_WIDTH_MASTER_INV_STROBE_1:
	case DM7820_PWM_WIDTH_MASTER_INV_STROBE_2:
		break;

	case DM7820_PWM_WIDTH_MASTER_RESERVED:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a programmable clock passed into the library

@param
    clock

    The programmable clock to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      clock is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_PrgClk_Clock(dm7820_prgclk_clock clock)
{
	switch (clock) {
	case DM7820_PRGCLK_CLOCK_0:
	case DM7820_PRGCLK_CLOCK_1:
	case DM7820_PRGCLK_CLOCK_2:
	case DM7820_PRGCLK_CLOCK_3:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a programmable clock master clock passed into the library.

@param
    master

    The programmable clock master clock to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      master is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_PrgClk_Master(dm7820_prgclk_master_clock master)
{
	switch (master) {
	case DM7820_PRGCLK_MASTER_25_MHZ:
	case DM7820_PRGCLK_MASTER_SAMPLE_CLOCK:
	case DM7820_PRGCLK_MASTER_8254_A_0:
	case DM7820_PRGCLK_MASTER_8254_A_1:
	case DM7820_PRGCLK_MASTER_8254_A_2:
	case DM7820_PRGCLK_MASTER_8254_B_0:
	case DM7820_PRGCLK_MASTER_8254_B_1:
	case DM7820_PRGCLK_MASTER_8254_B_2:
	case DM7820_PRGCLK_MASTER_PROG_CLOCK_0:
	case DM7820_PRGCLK_MASTER_PROG_CLOCK_1:
	case DM7820_PRGCLK_MASTER_PROG_CLOCK_2:
	case DM7820_PRGCLK_MASTER_PROG_CLOCK_3:
	case DM7820_PRGCLK_MASTER_STROBE_1:
	case DM7820_PRGCLK_MASTER_STROBE_2:
	case DM7820_PRGCLK_MASTER_INV_STROBE_1:
	case DM7820_PRGCLK_MASTER_INV_STROBE_2:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a programmable clock mode passed into the library.

@param
    mode

    The programmable clock mode to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      mode is not valid.

        @arg \c
            EOPNOTSUPP  mode is equal to DM7820_PRGCLK_MODE_RESERVED.
 *******************************************************************************
 */

static int DM7820_Validate_PrgClk_Mode(dm7820_prgclk_mode mode)
{
	switch (mode) {
	case DM7820_PRGCLK_MODE_DISABLED:
	case DM7820_PRGCLK_MODE_CONTINUOUS:
	case DM7820_PRGCLK_MODE_ONE_SHOT:
		break;

	case DM7820_PRGCLK_MODE_RESERVED:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a programmable clock start trigger passed into the library.

@param
    start

    The programmable clock start trigger to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      start is not valid.

        @arg \c
            EOPNOTSUPP  start is equal to DM7820_PRGCLK_START_RESERVED_1,
                        DM7820_PRGCLK_START_RESERVED_2,
                        DM7820_PRGCLK_START_RESERVED_3, or
                        DM7820_PRGCLK_START_RESERVED_4.
 *******************************************************************************
 */

static int
DM7820_Validate_PrgClk_Start_Trigger(dm7820_prgclk_start_trigger start)
{
	switch (start) {
	case DM7820_PRGCLK_START_IMMEDIATE:
	case DM7820_PRGCLK_START_8254_A_0:
	case DM7820_PRGCLK_START_8254_A_1:
	case DM7820_PRGCLK_START_8254_A_2:
	case DM7820_PRGCLK_START_8254_B_0:
	case DM7820_PRGCLK_START_8254_B_1:
	case DM7820_PRGCLK_START_8254_B_2:
	case DM7820_PRGCLK_START_PROG_CLOCK_0:
	case DM7820_PRGCLK_START_PROG_CLOCK_1:
	case DM7820_PRGCLK_START_PROG_CLOCK_2:
	case DM7820_PRGCLK_START_PROG_CLOCK_3:
	case DM7820_PRGCLK_START_STROBE_1:
	case DM7820_PRGCLK_START_STROBE_2:
	case DM7820_PRGCLK_START_INV_STROBE_1:
	case DM7820_PRGCLK_START_INV_STROBE_2:
	case DM7820_PRGCLK_START_ADVANCED_INT_0:
	case DM7820_PRGCLK_START_ADVANCED_INT_1:
	case DM7820_PRGCLK_START_8254_INT:
	case DM7820_PRGCLK_START_INC_ENCODER_0_INT:
	case DM7820_PRGCLK_START_INC_ENCODER_1_INT:
	case DM7820_PRGCLK_START_PWM_0_INT:
	case DM7820_PRGCLK_START_PWM_1_INT:
	case DM7820_PRGCLK_START_PROG_CLOCK_0_INT:
	case DM7820_PRGCLK_START_PROG_CLOCK_1_INT:
	case DM7820_PRGCLK_START_PROG_CLOCK_2_INT:
	case DM7820_PRGCLK_START_PROG_CLOCK_3_INT:
	case DM7820_PRGCLK_START_FIFO_0_INT:
	case DM7820_PRGCLK_START_FIFO_1_INT:
		break;

	case DM7820_PRGCLK_START_RESERVED_1:
	case DM7820_PRGCLK_START_RESERVED_2:
	case DM7820_PRGCLK_START_RESERVED_3:
	case DM7820_PRGCLK_START_RESERVED_4:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a programmable clock stop trigger passed into the library.

@param
    stop

    The programmable clock stop trigger to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      stop is not valid.

        @arg \c
            EOPNOTSUPP  stop is equal to DM7820_PRGCLK_STOP_RESERVED_1,
                        DM7820_PRGCLK_STOP_RESERVED_2,
                        DM7820_PRGCLK_STOP_RESERVED_3, or
                        DM7820_PRGCLK_STOP_RESERVED_4.
 *******************************************************************************
 */

static int DM7820_Validate_PrgClk_Stop_Trigger(dm7820_prgclk_stop_trigger stop)
{
	switch (stop) {
	case DM7820_PRGCLK_STOP_NONE:
	case DM7820_PRGCLK_STOP_8254_A_0:
	case DM7820_PRGCLK_STOP_8254_A_1:
	case DM7820_PRGCLK_STOP_8254_A_2:
	case DM7820_PRGCLK_STOP_8254_B_0:
	case DM7820_PRGCLK_STOP_8254_B_1:
	case DM7820_PRGCLK_STOP_8254_B_2:
	case DM7820_PRGCLK_STOP_PROG_CLOCK_0:
	case DM7820_PRGCLK_STOP_PROG_CLOCK_1:
	case DM7820_PRGCLK_STOP_PROG_CLOCK_2:
	case DM7820_PRGCLK_STOP_PROG_CLOCK_3:
	case DM7820_PRGCLK_STOP_STROBE_1:
	case DM7820_PRGCLK_STOP_STROBE_2:
	case DM7820_PRGCLK_STOP_INV_STROBE_1:
	case DM7820_PRGCLK_STOP_INV_STROBE_2:
	case DM7820_PRGCLK_STOP_ADVANCED_INT_0:
	case DM7820_PRGCLK_STOP_ADVANCED_INT_1:
	case DM7820_PRGCLK_STOP_8254_INT:
	case DM7820_PRGCLK_STOP_INC_ENCODER_0_INT:
	case DM7820_PRGCLK_STOP_INC_ENCODER_1_INT:
	case DM7820_PRGCLK_STOP_PWM_0_INT:
	case DM7820_PRGCLK_STOP_PWM_1_INT:
	case DM7820_PRGCLK_STOP_PROG_CLOCK_0_INT:
	case DM7820_PRGCLK_STOP_PROG_CLOCK_1_INT:
	case DM7820_PRGCLK_STOP_PROG_CLOCK_2_INT:
	case DM7820_PRGCLK_STOP_PROG_CLOCK_3_INT:
	case DM7820_PRGCLK_STOP_FIFO_0_INT:
	case DM7820_PRGCLK_STOP_FIFO_1_INT:
		break;

	case DM7820_PRGCLK_STOP_RESERVED_1:
	case DM7820_PRGCLK_STOP_RESERVED_2:
	case DM7820_PRGCLK_STOP_RESERVED_3:
	case DM7820_PRGCLK_STOP_RESERVED_4:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a standard I/O port mode passed into the library.

@param
    mode

    The standard I/O port mode to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      mode is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_StdIO_IO_Mode(DM7820_StdIO_IO_Mode mode)
{
	switch (mode) {
	case DM7820_STDIO_MODE_INPUT:
	case DM7820_STDIO_MODE_OUTPUT:
	case DM7820_STDIO_MODE_PER_OUT:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a standard I/O port peripheral output mode passed into the library.

@param
    mode

    The standard I/O port peripheral output mode to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      mode is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_StdIO_Peripheral_Mode(DM7820_StdIO_Periph_Mode mode)
{
	switch (mode) {
	case DM7820_STDIO_PERIPH_PWM:
	case DM7820_STDIO_PERIPH_CLK_OTHER:
	case DM7820_STDIO_PERIPH_FIFO_0:
	case DM7820_STDIO_PERIPH_FIFO_1:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a standard I/O port passed into the library.

@param
    port

    The standard I/O port to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      port is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_StdIO_Port(DM7820_StdIO_Port port)
{
	switch (port) {
	case DM7820_STDIO_PORT_0:
	case DM7820_STDIO_PORT_1:
	case DM7820_STDIO_PORT_2:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate a standard I/O strobe signal passed into the library.

@param
    strobe

    The standard I/O strobe signal to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      strobe is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_StdIO_Strobe(DM7820_StdIO_Strobe strobe)
{
	switch (strobe) {
	case DM7820_STDIO_STROBE_1:
	case DM7820_STDIO_STROBE_2:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an 8254 timer/counter clock input passed into the library.

@param
    clock

    The 8254 timer/counter clock input to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      clock is not valid.

        @arg \c
            EOPNOTSUPP  clock is equal to DM7820_TMRCTR_CLOCK_RESERVED.
 *******************************************************************************
 */

static int DM7820_Validate_TmrCtr_Clock(dm7820_tmrctr_clock clock)
{
	switch (clock) {
	case DM7820_TMRCTR_CLOCK_5_MHZ:
	case DM7820_TMRCTR_CLOCK_8254_A_0:
	case DM7820_TMRCTR_CLOCK_8254_A_1:
	case DM7820_TMRCTR_CLOCK_8254_A_2:
	case DM7820_TMRCTR_CLOCK_8254_B_0:
	case DM7820_TMRCTR_CLOCK_8254_B_1:
	case DM7820_TMRCTR_CLOCK_8254_B_2:
	case DM7820_TMRCTR_CLOCK_PROG_CLOCK_0:
	case DM7820_TMRCTR_CLOCK_PROG_CLOCK_1:
	case DM7820_TMRCTR_CLOCK_PROG_CLOCK_2:
	case DM7820_TMRCTR_CLOCK_PROG_CLOCK_3:
	case DM7820_TMRCTR_CLOCK_STROBE_1:
	case DM7820_TMRCTR_CLOCK_STROBE_2:
	case DM7820_TMRCTR_CLOCK_INV_STROBE_1:
	case DM7820_TMRCTR_CLOCK_INV_STROBE_2:
		break;

	case DM7820_TMRCTR_CLOCK_RESERVED:
		errno = EOPNOTSUPP;
		return -1;
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an 8254 timer/counter count mode passed into the library.

@param
    count_mode

    The 8254 timer/counter count mode to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      count_mode is not valid.
 *******************************************************************************
 */

static int
DM7820_Validate_TmrCtr_Count_Mode(dm7820_tmrctr_count_mode count_mode)
{
	switch (count_mode) {
	case DM7820_TMRCTR_COUNT_MODE_BINARY:
	case DM7820_TMRCTR_COUNT_MODE_BCD:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an 8254 timer/counter gate input passed into the library.

@param
    gate

    The 8254 timer/counter gate input to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      gate is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_TmrCtr_Gate(dm7820_tmrctr_gate gate)
{
	switch (gate) {
	case DM7820_TMRCTR_GATE_LOGIC_0:
	case DM7820_TMRCTR_GATE_LOGIC_1:
	case DM7820_TMRCTR_GATE_8254_A_0:
	case DM7820_TMRCTR_GATE_8254_A_1:
	case DM7820_TMRCTR_GATE_8254_A_2:
	case DM7820_TMRCTR_GATE_8254_B_0:
	case DM7820_TMRCTR_GATE_8254_B_1:
	case DM7820_TMRCTR_GATE_8254_B_2:
	case DM7820_TMRCTR_GATE_PROG_CLOCK_0:
	case DM7820_TMRCTR_GATE_PROG_CLOCK_1:
	case DM7820_TMRCTR_GATE_PROG_CLOCK_2:
	case DM7820_TMRCTR_GATE_PROG_CLOCK_3:
	case DM7820_TMRCTR_GATE_STROBE_1:
	case DM7820_TMRCTR_GATE_STROBE_2:
	case DM7820_TMRCTR_GATE_INV_STROBE_1:
	case DM7820_TMRCTR_GATE_INV_STROBE_2:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_0:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_1:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_2:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_3:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_4:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_5:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_6:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_7:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_8:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_9:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_10:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_11:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_12:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_13:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_14:
	case DM7820_TMRCTR_GATE_PORT_2_BIT_15:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an 8254 timer/counter passed into the library.

@param
    timer

    The 8254 timer/counter to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      timer is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_TmrCtr_Timer(dm7820_tmrctr_timer timer)
{
	switch (timer) {
	case DM7820_TMRCTR_TIMER_A_0:
	case DM7820_TMRCTR_TIMER_A_1:
	case DM7820_TMRCTR_TIMER_A_2:
	case DM7820_TMRCTR_TIMER_B_0:
	case DM7820_TMRCTR_TIMER_B_1:
	case DM7820_TMRCTR_TIMER_B_2:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
*******************************************************************************
@brief
    Validate an 8254 timer/counter waveform mode passed into the library.

@param
    waveform

    The 8254 timer/counter waveform mode to validate.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      waveform is not valid.
 *******************************************************************************
 */

static int DM7820_Validate_TmrCtr_Waveform(dm7820_tmrctr_waveform waveform)
{
	switch (waveform) {
	case DM7820_TMRCTR_WAVEFORM_EVENT_CTR:
	case DM7820_TMRCTR_WAVEFORM_PROG_ONE_SHOT:
	case DM7820_TMRCTR_WAVEFORM_RATE_GENERATOR:
	case DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE:
	case DM7820_TMRCTR_WAVEFORM_SOFTWARE_STROBE:
	case DM7820_TMRCTR_WAVEFORM_HARDWARE_STROBE:
		break;

	default:
		errno = EINVAL;
		return -1;
		break;
	}

	return 0;
}

/**
 * @} DM7820_Library_Private_Functions
 */

/**
 * @} DM7820_Library_Source
 */

/*=============================================================================
Public functions
 =============================================================================*/

DM7820_Error
DM7820_AdvInt_Get_Status(DM7820_Board_Descriptor * handle,
			 dm7820_advint_interrupt interrupt, uint8_t * occurred)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mask = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_AdvInt_Interrupt(interrupt) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine mask for advanced interrupt status bit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (interrupt) {
	case DM7820_ADVINT_INTERRUPT_0:
		mask = 0x0001;
		break;

	case DM7820_ADVINT_INTERRUPT_1:
		mask = 0x0002;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read Interrupt Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_INTERRUPT_STATUS;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	*occurred = (ioctl_request.readwrite.access.data.data16 & mask);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Clear Interrupt Register advanced interrupt status bit if set
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (*occurred) {

		/*
		 * Preserve reserved bits (bits 3, 6, and 7)
		 */

		ioctl_request.readwrite.access.data.data16 &= 0x00C8;

		ioctl_request.readwrite.access.data.data16 |= mask;
		ioctl_request.readwrite.access.offset =
		    DM7820_BAR2_INTERRUPT_STATUS;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			     &ioctl_request);
	}

	return 0;
}

DM7820_Error
DM7820_AdvInt_Read_Capture(DM7820_Board_Descriptor * handle,
			   dm7820_advint_interrupt interrupt,
			   DM7820_StdIO_Port port, uint16_t * value)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t capture_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_AdvInt_Interrupt(interrupt) == -1) {
		return -1;
	}

	if (DM7820_Validate_StdIO_Port(port) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Advanced Interrupt Port Capture Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (interrupt) {
	case DM7820_ADVINT_INTERRUPT_0:
		switch (port) {
		case DM7820_STDIO_PORT_0:
			capture_offset = DM7820_BAR2_ADVINT0_PORT0_CAPT;
			break;

		case DM7820_STDIO_PORT_1:
			capture_offset = DM7820_BAR2_ADVINT0_PORT1_CAPT;
			break;

		case DM7820_STDIO_PORT_2:
			capture_offset = DM7820_BAR2_ADVINT0_PORT2_CAPT;
			break;
		}

		break;

	case DM7820_ADVINT_INTERRUPT_1:
		switch (port) {
		case DM7820_STDIO_PORT_0:
			capture_offset = DM7820_BAR2_ADVINT1_PORT0_CAPT;
			break;

		case DM7820_STDIO_PORT_1:
			capture_offset = DM7820_BAR2_ADVINT1_PORT1_CAPT;
			break;

		case DM7820_STDIO_PORT_2:
			capture_offset = DM7820_BAR2_ADVINT1_PORT2_CAPT;
			break;
		}

		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read Advanced Interrupt Port Capture Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = capture_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == 0) {
		*value = ioctl_request.readwrite.access.data.data16;
	}

	return status;
}

DM7820_Error
DM7820_AdvInt_Set_Compare(DM7820_Board_Descriptor * handle,
			  dm7820_advint_interrupt interrupt,
			  DM7820_StdIO_Port port, uint16_t value)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t compare_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_AdvInt_Interrupt(interrupt) == -1) {
		return -1;
	}

	if (DM7820_Validate_StdIO_Port(port) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Advanced Interrupt Port Compare Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (interrupt) {
	case DM7820_ADVINT_INTERRUPT_0:
		switch (port) {
		case DM7820_STDIO_PORT_0:
			compare_offset = DM7820_BAR2_ADVINT0_PORT0_CMP;
			break;

		case DM7820_STDIO_PORT_1:
			compare_offset = DM7820_BAR2_ADVINT0_PORT1_CMP;
			break;

		case DM7820_STDIO_PORT_2:
			compare_offset = DM7820_BAR2_ADVINT0_PORT2_CMP;
			break;
		}

		break;

	case DM7820_ADVINT_INTERRUPT_1:
		switch (port) {
		case DM7820_STDIO_PORT_0:
			compare_offset = DM7820_BAR2_ADVINT1_PORT0_CMP;
			break;

		case DM7820_STDIO_PORT_1:
			compare_offset = DM7820_BAR2_ADVINT1_PORT1_CMP;
			break;

		case DM7820_STDIO_PORT_2:
			compare_offset = DM7820_BAR2_ADVINT1_PORT2_CMP;
			break;
		}

		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write to Advanced Interrupt Port Compare Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = value;
	ioctl_request.readwrite.access.offset = compare_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_AdvInt_Set_Mask(DM7820_Board_Descriptor * handle,
		       dm7820_advint_interrupt interrupt,
		       DM7820_StdIO_Port port, uint16_t value)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mask_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_AdvInt_Interrupt(interrupt) == -1) {
		return -1;
	}

	if (DM7820_Validate_StdIO_Port(port) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Advanced Interrupt Port Mask Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (interrupt) {
	case DM7820_ADVINT_INTERRUPT_0:
		switch (port) {
		case DM7820_STDIO_PORT_0:
			mask_offset = DM7820_BAR2_ADVINT0_PORT0_MASK;
			break;

		case DM7820_STDIO_PORT_1:
			mask_offset = DM7820_BAR2_ADVINT0_PORT1_MASK;
			break;

		case DM7820_STDIO_PORT_2:
			mask_offset = DM7820_BAR2_ADVINT0_PORT2_MASK;
			break;
		}

		break;

	case DM7820_ADVINT_INTERRUPT_1:
		switch (port) {
		case DM7820_STDIO_PORT_0:
			mask_offset = DM7820_BAR2_ADVINT1_PORT0_MASK;
			break;

		case DM7820_STDIO_PORT_1:
			mask_offset = DM7820_BAR2_ADVINT1_PORT1_MASK;
			break;

		case DM7820_STDIO_PORT_2:
			mask_offset = DM7820_BAR2_ADVINT1_PORT2_MASK;
			break;
		}

		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write to Advanced Interrupt Port Mask Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = value;
	ioctl_request.readwrite.access.offset = mask_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_AdvInt_Set_Master(DM7820_Board_Descriptor * handle,
			 dm7820_advint_interrupt interrupt,
			 dm7820_advint_master_clock master)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t clock_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_AdvInt_Interrupt(interrupt) == -1) {
		return -1;
	}

	if (DM7820_Validate_AdvInt_Master(master) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Advanced Interrupt Clock Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (interrupt) {
	case DM7820_ADVINT_INTERRUPT_0:
		clock_offset = DM7820_BAR2_ADVINT0_CLK;
		break;

	case DM7820_ADVINT_INTERRUPT_1:
		clock_offset = DM7820_BAR2_ADVINT1_CLK;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify Advanced Interrupt Clock Register to select desired mode
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = master;
	ioctl_request.modify.access.offset = clock_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x000F;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_AdvInt_Set_Mode(DM7820_Board_Descriptor * handle,
		       dm7820_advint_interrupt interrupt,
		       dm7820_advint_mode mode)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_AdvInt_Interrupt(interrupt) == -1) {
		return -1;
	}

	if (DM7820_Validate_AdvInt_Mode(mode) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Advanced Interrupt Mode Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (interrupt) {
	case DM7820_ADVINT_INTERRUPT_0:
		mode_offset = DM7820_BAR2_ADVINT0_INT_MODE;
		break;

	case DM7820_ADVINT_INTERRUPT_1:
		mode_offset = DM7820_BAR2_ADVINT1_INT_MODE;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify Advanced Interrupt Mode Register to select desired mode
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = mode;
	ioctl_request.modify.access.offset = mode_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x0003;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_General_Get_Version_Info(DM7820_Board_Descriptor * handle,
				uint8_t * fpga_type_id,
				uint8_t * fpga_version, uint16_t * svn_version)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t saved_fpga_version;

	/*
	 * Read FPGA Version Register and save value
	 */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_FPGA_VERSION;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	if (ioctl
	    (handle->file_descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request)
	    == -1) {
		return -1;
	}

	saved_fpga_version = ioctl_request.readwrite.access.data.data16;

	/*
	 * Read SVN Version Register
	 */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_SVN_VERSION;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == 0) {
		*svn_version = ioctl_request.readwrite.access.data.data16;
		*fpga_type_id = ((saved_fpga_version & 0xFF00) >> 8);
		*fpga_version = (saved_fpga_version & 0x00FF);
	}

	return status;
}

DM7820_Error
DM7820_General_Is_PCI_Master(DM7820_Board_Descriptor * handle,
			     uint8_t * pci_master)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_BRD_STAT;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == 0) {
		*pci_master =
		    !(ioctl_request.readwrite.access.data.data16 & 0x0001);
	}

	return status;
}

DM7820_Error DM7820_General_Reset(DM7820_Board_Descriptor * handle)
{
	dm7820_ioctl_argument_t ioctl_request;

	ioctl_request.readwrite.access.data.data16 =
	    DM7820_BOARD_RESET_DO_RESET;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_BOARD_RESET;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_FIFO_Enable(DM7820_Board_Descriptor * handle,
		   dm7820_fifo_queue fifo, uint8_t enable)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t control_offset = 0x0000;
	uint16_t control_value;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Control/Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		control_offset = DM7820_BAR2_FIFO0_CON_STAT;
		break;

	case DM7820_FIFO_QUEUE_1:
		control_offset = DM7820_BAR2_FIFO1_CON_STAT;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up FIFO Control/Status Register value and modify the register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	control_value = 0x0000;

	if (enable) {
		control_value = 0x0001;
	}

	ioctl_request.modify.access.data.data16 = control_value;
	ioctl_request.modify.access.offset = control_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x0001;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_FIFO_Get_Status(DM7820_Board_Descriptor * handle,
		       dm7820_fifo_queue fifo,
		       dm7820_fifo_status_condition condition,
		       uint8_t * occurred)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t int_offset = 0x0000;
	uint16_t mask = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	if (DM7820_Validate_FIFO_Status_Condition(condition) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Interrupt Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		int_offset = DM7820_BAR2_FIFO0_INT;
		break;

	case DM7820_FIFO_QUEUE_1:
		int_offset = DM7820_BAR2_FIFO1_INT;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine mask for given FIFO status bit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (condition) {
	case DM7820_FIFO_STATUS_READ_REQUEST:
		mask = 0x0100;
		break;

	case DM7820_FIFO_STATUS_WRITE_REQUEST:
		mask = 0x0200;
		break;

	case DM7820_FIFO_STATUS_FULL:
		mask = 0x0400;
		break;

	case DM7820_FIFO_STATUS_EMPTY:
		mask = 0x0800;
		break;

	case DM7820_FIFO_STATUS_OVERFLOW:
		mask = 0x1000;
		break;

	case DM7820_FIFO_STATUS_UNDERFLOW:
		mask = 0x2000;
		break;

	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read FIFO Interrupt Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = int_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Process FIFO empty and full status conditions
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if ((condition == DM7820_FIFO_STATUS_EMPTY)
	    || (condition == DM7820_FIFO_STATUS_FULL)
	    ) {

		/*#####################################################################
		   Clear FIFO Interrupt Register status empty or full bit
		   ##################################################################### */

		/*
		 * Preserve interrupt enable bits and mask off all interrupt status bits
		 */

		ioctl_request.readwrite.access.data.data16 &= 0x00FF;

		ioctl_request.readwrite.access.data.data16 |= mask;
		ioctl_request.readwrite.access.offset = int_offset;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		status =
		    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		if (status == -1) {
			return -1;
		}

		/*#####################################################################
		   Read FIFO Interrupt Register again to get updated status
		   ##################################################################### */

		ioctl_request.readwrite.access.data.data16 = 0x0000;
		ioctl_request.readwrite.access.offset = int_offset;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		status =
		    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
			  &ioctl_request);
		if (status == 0) {
			*occurred =
			    ((ioctl_request.readwrite.access.data.
			      data16 & mask) >> 8);
		}

		return status;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Process overflow, read request, underflow, and write request status
	   conditions
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Determine if status condition occurred
	 */

	*occurred = ((ioctl_request.readwrite.access.data.data16 & mask) >> 8);

	/*#########################################################################
	   Clear condition status flag if it is set
	   ######################################################################### */

	if (*occurred) {

		/*
		 * Preserve interrupt enable bits and mask off all interrupt status bits
		 */

		ioctl_request.readwrite.access.data.data16 &= 0x00FF;

		ioctl_request.readwrite.access.data.data16 |= mask;
		ioctl_request.readwrite.access.offset = int_offset;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			     &ioctl_request);
	}

	return 0;
}

DM7820_Error
DM7820_FIFO_DMA_Configure(DM7820_Board_Descriptor * handle,
			  dm7820_fifo_queue fifo,
			  uint8_t direction, uint32_t transfer_size)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t control_offset = 0;
	uint32_t control_value = 0;
	uint8_t ioctl_status = 0;
	uint32_t buffer_phys_addr = 0;
	uint32_t local_address = 0;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	if (direction > DM7820_DMA_DEMAND_ON_DM7820_TO_PCI) {
		return -1;
	}

	if (transfer_size % 2 != 0) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 * Configure the Mode
	 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		control_offset = DM7820_BAR0_DMAMODE0;
		local_address = 0xCC;
		break;
	case DM7820_FIFO_QUEUE_1:
		control_offset = DM7820_BAR0_DMAMODE1;
		local_address = 0xDC;
		break;
	}

	control_value = 0x00000000;
	control_value |= PLX9056_DMA_WIDTH_16 | PLX9056_DMA_READY |
	    PLX9056_DMA_BURST | PLX9056_DMA_LOCAL_BURST |
	    PLX9056_DMA_LOCAL_ADDRESSING_MODE;

	if ((direction & DM7820_DMA_DEMAND_ON_PCI_TO_DM7820) ==
	    DM7820_DMA_DEMAND_ON_PCI_TO_DM7820) {
		control_value |= PLX9056_DMA_DEMAND_MODE;
	}

	ioctl_request.modify.mask.mask32 =
	    control_value | PLX9056_DMA_WIDTH_MASK |
	    PLX9056_DMA_WAITSTATES_MASK;

	ioctl_request.modify.access.data.data32 = control_value;
	ioctl_request.modify.access.offset = control_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_32;

	ioctl_status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		  &ioctl_request);

	if (ioctl_status != 0) {
		return ioctl_status;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 * Configure the PCI address
	 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		control_offset = DM7820_BAR0_DMAPADR0;
		break;
	case DM7820_FIFO_QUEUE_1:
		control_offset = DM7820_BAR0_DMAPADR1;
		break;
	}

	/*
	 * Get a usable buffers physical address from the driver
	 */

	ioctl_request.dma_function.function = DM7820_DMA_GET_BUFFER_ADDR;
	ioctl_request.dma_function.fifo = fifo;

	buffer_phys_addr =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_DMA_FUNCTION,
		  &ioctl_request);

	ioctl_request.readwrite.access.data.data32 = buffer_phys_addr;
	ioctl_request.readwrite.access.offset = control_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_32;

	ioctl_status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		  &ioctl_request);

	if (ioctl_status != 0) {
		return ioctl_status;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 * Configure the Local address
	 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		control_offset = DM7820_BAR0_DMALADR0;
		break;
	case DM7820_FIFO_QUEUE_1:
		control_offset = DM7820_BAR0_DMALADR1;
		break;
	}

	ioctl_request.readwrite.access.data.data32 = local_address;
	ioctl_request.readwrite.access.offset = control_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_32;

	ioctl_status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		  &ioctl_request);

	if (ioctl_status != 0) {
		return ioctl_status;
	}
	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 * Configure the transfer size.
	 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		control_offset = DM7820_BAR0_DMASIZ0;
		break;
	case DM7820_FIFO_QUEUE_1:
		control_offset = DM7820_BAR0_DMASIZ1;
		break;
	}

	ioctl_request.modify.access.data.data32 = transfer_size;
	ioctl_request.modify.access.offset = control_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_32;
	ioctl_request.modify.mask.mask32 = 0x007fffff;

	ioctl_status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		  &ioctl_request);

	if (ioctl_status != 0) {
		return ioctl_status;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	 * Configure the transfer direction.
	 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		control_offset = DM7820_BAR0_DMADPR0;
		break;
	case DM7820_FIFO_QUEUE_1:
		control_offset = DM7820_BAR0_DMADPR1;
		break;
	}

	if ((direction & DM7820_DMA_DEMAND_OFF_DM7820_TO_PCI) ==
	    DM7820_DMA_DEMAND_OFF_DM7820_TO_PCI) {
		control_value = 0x00000008;
	} else {
		control_value = 0x00000000;
	}

	ioctl_request.readwrite.access.data.data32 = control_value;
	ioctl_request.readwrite.access.offset = control_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_32;

	ioctl_status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		  &ioctl_request);

	return ioctl_status;
}

DM7820_Error
DM7820_FIFO_DMA_Enable(DM7820_Board_Descriptor * handle,
		       dm7820_fifo_queue fifo, uint8_t enable, uint8_t start)
{

	dm7820_ioctl_argument_t ioctl_request;
	uint16_t control_offset = 0x0000;
	uint8_t control_value;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Control/Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		control_offset = DM7820_BAR0_DMACSR0;
		break;

	case DM7820_FIFO_QUEUE_1:
		control_offset = DM7820_BAR0_DMACSR1;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up FIFO Control/Status Register value and modify the register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	control_value = 0x00;

	if (enable) {
		control_value |= 0x01;
	}
	if (start) {
		control_value |= 0x02;
	}

	ioctl_request.modify.access.data.data8 = control_value;
	ioctl_request.modify.access.offset = control_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_PLX_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_8;
	ioctl_request.modify.mask.mask8 = 0x03;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);

}

DM7820_Error
DM7820_Stop_DMA(DM7820_Board_Descriptor * handle, dm7820_fifo_queue fifo)
{
	dm7820_ioctl_argument_t ioctl_request;

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	ioctl_request.dma_function.fifo = fifo;
	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_STOP;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_DMA_FUNCTION,
		     &ioctl_request);
}

DM7820_Error
DM7820_FIFO_DMA_Read(DM7820_Board_Descriptor * handle,
		     dm7820_fifo_queue fifo, void *user_buffer,
		     uint32_t num_bufs)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	if (user_buffer == NULL) {
		return -1;
	}

	if (num_bufs < 0) {
		return -1;
	}

	ioctl_request.dma_function.fifo = fifo;
	ioctl_request.dma_function.transfer_size = num_bufs;
	ioctl_request.dma_function.user_buffer = user_buffer;
	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_READ;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_DMA_FUNCTION,
		  &ioctl_request);

	return status;
}

DM7820_Error
DM7820_FIFO_DMA_Write(DM7820_Board_Descriptor * handle,
		      dm7820_fifo_queue fifo, void *user_buffer,
		      uint32_t num_bufs)
{
	dm7820_ioctl_argument_t ioctl_request;

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	if (user_buffer == NULL) {
		return -1;
	}

	if (num_bufs < 0) {
		return -1;
	}

	ioctl_request.dma_function.fifo = fifo;
	ioctl_request.dma_function.transfer_size = num_bufs;
	ioctl_request.dma_function.user_buffer = user_buffer;
	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_WRITE;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_DMA_FUNCTION,
		     &ioctl_request);
}

DM7820_Error DM7820_FIFO_DMA_Create_Buffer(uint16_t ** buf, uint32_t size)
{

	*buf = malloc(size);

	if (*buf == NULL) {
		return -1;
	}
	if (mlock(*buf, size) != 0) {
		return -1;
	}
	return 0;
}

DM7820_Error DM7820_FIFO_DMA_Free_Buffer(uint16_t ** buf, uint32_t size)
{

	if (*buf != NULL) {
		if (munlock(*buf, size) != 0) {
			return -1;
		}
		free(*buf);
	} else {
		return -1;
	}

	return 0;
}

DM7820_Error
DM7820_FIFO_DMA_Initialize(DM7820_Board_Descriptor * handle,
			   dm7820_fifo_queue fifo,
			   uint32_t buffer_count, uint32_t buffer_size)
{
	dm7820_ioctl_argument_t ioctl_request;

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	ioctl_request.dma_function.fifo = fifo;
	ioctl_request.dma_function.function = DM7820_DMA_FUNCTION_INITIALIZE;
	ioctl_request.dma_function.arguments.dma_init.buffer_count =
	    buffer_count;
	ioctl_request.dma_function.arguments.dma_init.buffer_size = buffer_size;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_DMA_FUNCTION,
		     &ioctl_request);
}

DM7820_Error
DM7820_FIFO_Read(DM7820_Board_Descriptor * handle,
		 dm7820_fifo_queue fifo, uint16_t * data)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t read_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Read/Write Port Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		read_offset = DM7820_BAR2_FIFO0_RW_PORT;
		break;

	case DM7820_FIFO_QUEUE_1:
		read_offset = DM7820_BAR2_FIFO1_RW_PORT;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read FIFO Read/Write Port Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = read_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == 0) {
		*data = ioctl_request.readwrite.access.data.data16;
	}

	return status;
}

DM7820_Error
DM7820_FIFO_Set_DMA_Request(DM7820_Board_Descriptor * handle,
			    dm7820_fifo_queue fifo,
			    dm7820_fifo_dma_request source)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t source_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	if (DM7820_Validate_FIFO_DMA_Request(source) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Input Data/DMA Request Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		source_offset = DM7820_BAR2_FIFO0_IN_DATA_DREQ;
		break;

	case DM7820_FIFO_QUEUE_1:
		source_offset = DM7820_BAR2_FIFO1_IN_DATA_DREQ;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify FIFO Input Data/DMA Request Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = (source << 8);
	ioctl_request.modify.access.offset = source_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x0300;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_FIFO_Set_Data_Input(DM7820_Board_Descriptor * handle,
			   dm7820_fifo_queue fifo, dm7820_fifo_data_input input)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t data_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	if (DM7820_Validate_FIFO_Data_Input(fifo, input) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Input Data/DMA Request Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		data_offset = DM7820_BAR2_FIFO0_IN_DATA_DREQ;
		break;

	case DM7820_FIFO_QUEUE_1:
		data_offset = DM7820_BAR2_FIFO1_IN_DATA_DREQ;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify FIFO Input Data/DMA Request Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = input;
	ioctl_request.modify.access.offset = data_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x0003;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_FIFO_Set_Input_Clock(DM7820_Board_Descriptor * handle,
			    dm7820_fifo_queue fifo,
			    dm7820_fifo_input_clock clock)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t clock_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	if (DM7820_Validate_FIFO_Input_Clock(clock) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Input Clock Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		clock_offset = DM7820_BAR2_FIFO0_IN_CLK;
		break;

	case DM7820_FIFO_QUEUE_1:
		clock_offset = DM7820_BAR2_FIFO1_IN_CLK;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify FIFO Input Clock Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = clock;
	ioctl_request.modify.access.offset = clock_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x001F;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_FIFO_Set_Output_Clock(DM7820_Board_Descriptor * handle,
			     dm7820_fifo_queue fifo,
			     dm7820_fifo_output_clock clock)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t clock_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	if (DM7820_Validate_FIFO_Output_Clock(clock) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Output Clock Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		clock_offset = DM7820_BAR2_FIFO0_OUT_CLK;
		break;

	case DM7820_FIFO_QUEUE_1:
		clock_offset = DM7820_BAR2_FIFO1_OUT_CLK;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify FIFO Output Clock Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = clock;
	ioctl_request.modify.access.offset = clock_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x001F;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_FIFO_Write(DM7820_Board_Descriptor * handle,
		  dm7820_fifo_queue fifo, uint16_t data)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t write_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_FIFO_Queue(fifo) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of FIFO Read/Write Port Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (fifo) {
	case DM7820_FIFO_QUEUE_0:
		write_offset = DM7820_BAR2_FIFO0_RW_PORT;
		break;

	case DM7820_FIFO_QUEUE_1:
		write_offset = DM7820_BAR2_FIFO1_RW_PORT;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write FIFO Read/Write Port Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = data;
	ioctl_request.readwrite.access.offset = write_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error DM7820_General_Close_Board(DM7820_Board_Descriptor * handle)
{
	if (handle == NULL) {
		errno = ENODATA;
		return -1;
	}

	if (close(handle->file_descriptor) == -1) {
		free(handle);
		return -1;
	}

	free(handle);
	return 0;
}

DM7820_Error
DM7820_General_Enable_Interrupt(DM7820_Board_Descriptor * handle,
				dm7820_interrupt_source source, uint8_t enable)
{
	DM7820_Error status;
	dm7820_interrupt_control_t *int_control;
	dm7820_ioctl_argument_t ioctl_request;
	dm7820_minor_int_reg_layout_t *minor_reg;
	uint16_t int_enable_value;
	uint16_t minor_value = 0x0000;
	uint8_t disable_local;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_General_Interrupt_Source(source) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Get shortcuts to interrupt control and minor interrupt register layout
	   information
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	int_control = &(dm7820_interrupt_control[source]);

	minor_reg = NULL;
	if (int_control->minor_reg != DM7820_MINOR_INT_REG_NONE) {
		minor_reg =
		    &(dm7820_minor_int_reg_layout[int_control->minor_reg]);
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read Interrupt Enable Register and possibly minor interrupt enable register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*#########################################################################
	   Read Interrupt Enable Register to determine current state
	   ######################################################################### */

	/*
	 * Read Interrupt Enable Register
	 */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_INTERRUPT_ENABLE;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	/*
	 * Save current value of Interrupt Enable Register
	 */

	int_enable_value = ioctl_request.readwrite.access.data.data16;

	/*#########################################################################
	   If needed, read minor interrupt enable register to determine current state
	   ######################################################################### */

	if (minor_reg != NULL) {

		/*
		 * Read minor interrupt enable register
		 */

		ioctl_request.readwrite.access.data.data16 = 0x0000;
		ioctl_request.readwrite.access.offset = minor_reg->offset;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		status =
		    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
			  &ioctl_request);
		if (status == -1) {
			return -1;
		}

		/*
		 * Save current value of minor interrupt enable register
		 */

		minor_value = ioctl_request.readwrite.access.data.data16;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Process interrupt enable
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (enable) {

		/*#####################################################################
		   If needed, enable interrupt in minor interrupt enable register
		   ##################################################################### */

		if (minor_reg != NULL) {

			/*
			 * Preserve interrupt enable & reserved bits and mask off all
			 * interrupt status bits
			 */

			minor_value &=
			    (minor_reg->enable_mask | minor_reg->reserved_mask);

			/*
			 * Enable interrupt
			 */

			minor_value |= int_control->minor_enable;

			/*
			 * Clear interrupt status flag
			 */

			minor_value |= int_control->minor_status;

			/*
			 * Write to minor interrupt enable register
			 */

			ioctl_request.readwrite.access.data.data16 =
			    minor_value;
			ioctl_request.readwrite.access.offset =
			    minor_reg->offset;
			ioctl_request.readwrite.access.region =
			    DM7820_PCI_REGION_FPGA_MEM;
			ioctl_request.readwrite.access.size =
			    DM7820_PCI_REGION_ACCESS_16;

			status = ioctl(handle->file_descriptor,
				       DM7820_IOCTL_REGION_WRITE,
				       &ioctl_request);
			if (status == -1) {
				return -1;
			}
		}

		/*#####################################################################
		   Enable interrupt in Interrupt Enable Register
		   ##################################################################### */

		/*
		 * Enable interrupt
		 */

		int_enable_value |= int_control->int_enable_bit;

		/*
		 * Write to Interrupt Enable Register
		 */

		ioctl_request.readwrite.access.data.data16 = int_enable_value;
		ioctl_request.readwrite.access.offset =
		    DM7820_BAR2_INTERRUPT_ENABLE;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		status =
		    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		if (status == -1) {
			return -1;
		}

		/*#####################################################################
		   Clear interrupt status flag in Interrupt Status Register
		   ##################################################################### */

		/*
		 * Write to Interrupt Status Register
		 */

		ioctl_request.readwrite.access.data.data16 =
		    int_control->int_status_bit;
		ioctl_request.readwrite.access.offset =
		    DM7820_BAR2_INTERRUPT_STATUS;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		status =
		    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		if (status == -1) {
			return -1;
		}

		return 0;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Process interrupt disable
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Initially assume that interrupt should be disabled in Interrupt Enable
	 * Register
	 */

	disable_local = 0xFF;

	/*#########################################################################
	   If needed, disable interrupt in minor interrupt enable register
	   ######################################################################### */

	if (minor_reg != NULL) {

		/*
		 * Preserve interrupt enable & reserved bits and mask off all interrupt
		 * status bits
		 */

		minor_value &=
		    (minor_reg->enable_mask | minor_reg->reserved_mask);

		/*
		 * Disable interrupt
		 */

		minor_value &= ~int_control->minor_enable;

		/*
		 * If there are interrupts still enabled, then do not disable the
		 * interrupt in the Interrupt Enable Register
		 */

		if (minor_value & minor_reg->enable_mask) {
			disable_local = 0x00;
		}

		/*
		 * Write to minor interrupt enable register
		 */

		ioctl_request.readwrite.access.data.data16 = minor_value;
		ioctl_request.readwrite.access.offset = minor_reg->offset;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		status =
		    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		if (status == -1) {
			return -1;
		}
	}

	/*#########################################################################
	   If needed, disable interrupt in Interrupt Enable Register
	   ######################################################################### */

	if (disable_local) {

		/*
		 * Disable interrupt
		 */

		int_enable_value &= ~int_control->int_enable_bit;

		/*
		 * Write to Interrupt Enable Register
		 */

		ioctl_request.readwrite.access.data.data16 = int_enable_value;
		ioctl_request.readwrite.access.offset =
		    DM7820_BAR2_INTERRUPT_ENABLE;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		status =
		    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			  &ioctl_request);
		if (status == -1) {
			return -1;
		}
	}

	return 0;
}

DM7820_Error
DM7820_General_Get_Interrupt_Status(DM7820_Board_Descriptor * handle,
				    dm7820_interrupt_info * interrupt_info,
				    uint8_t wait_for_interrupt)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;

	ioctl_request.int_status.wait_for_interrupt = wait_for_interrupt;

	status = ioctl(handle->file_descriptor,
		       DM7820_IOCTL_GET_INTERRUPT_STATUS, &ioctl_request);
	if (status == 0) {
		*interrupt_info = ioctl_request.int_status.int_source_info;
	}

	return status;
}

DM7820_Error
DM7820_General_Open_Board(uint8_t dev_num, DM7820_Board_Descriptor ** handle)
{
	char device_name[25];
	int descriptor;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Form the device file name and attempt to open the file
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	(void)snprintf(&(device_name[0]), sizeof(device_name),
		       "/dev/rtd-dm7820-%u", dev_num);

	descriptor = open(&(device_name[0]), O_RDWR);
	if (descriptor == -1) {
		if (errno != EBUSY) {
			*handle = NULL;
		}
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Allocate and initialize memory for the library device descriptor
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	*handle =
	    (DM7820_Board_Descriptor *) malloc(sizeof(DM7820_Board_Descriptor)
	    );
	if (*handle == NULL) {
		return -1;
	}

	(void)memset(*handle, 0x00, sizeof(DM7820_Board_Descriptor));

	(*handle)->file_descriptor = descriptor;
	(*handle)->isr = NULL;
	return 0;
}

DM7820_Error
DM7820_IncEnc_Configure(DM7820_Board_Descriptor * handle,
			dm7820_incenc_encoder encoder,
			dm7820_incenc_phase_filter phase_filter,
			dm7820_incenc_input_mode input_mode,
			uint8_t enable_input_filter,
			dm7820_incenc_channel_mode channel_mode,
			uint8_t enable_index)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;
	uint16_t mode_value;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	if (DM7820_Validate_IncEnc_Input_Mode(input_mode) == -1) {
		return -1;
	}

	if (DM7820_Validate_IncEnc_Channel_Mode(channel_mode) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Incremental Encoder Mode Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		mode_offset = DM7820_BAR2_INCENC0_MODE;
		break;

	case DM7820_INCENC_ENCODER_1:
		mode_offset = DM7820_BAR2_INCENC1_MODE;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up Incremental Encoder Mode Register value and modify the register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Initial register value; by default, both the input filter and the index
	 * input are disabled
	 */

	mode_value = 0x0000;

	/*
	 * Fold phase filter into bits 8 through 15
	 */

	mode_value |= (phase_filter << 8);

	/*
	 * Fold input mode into bit 5
	 */

	mode_value |= (input_mode << 5);

	/*
	 * Set bit 4 if input filter is to be enabled
	 */

	if (enable_input_filter) {
		mode_value |= (1 << 4);
	}

	/*
	 * Fold channel mode into bit 3
	 */

	mode_value |= (channel_mode << 3);

	/*
	 * Set bit 2 if index input is to be enabled
	 */

	if (enable_index) {
		mode_value |= (1 << 2);
	}

	/*
	 * Modify the Incremental Encoder Mode Register value
	 */

	ioctl_request.modify.access.data.data16 = mode_value;
	ioctl_request.modify.access.offset = mode_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0xFF3C;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_IncEnc_Enable(DM7820_Board_Descriptor * handle,
		     dm7820_incenc_encoder encoder, uint8_t enable)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;
	uint16_t mode_value;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Incremental Encoder Mode Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		mode_offset = DM7820_BAR2_INCENC0_MODE;
		break;

	case DM7820_INCENC_ENCODER_1:
		mode_offset = DM7820_BAR2_INCENC1_MODE;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up Incremental Encoder Mode Register value and modify the register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	mode_value = 0x0000;

	if (enable) {
		mode_value = 0x0001;
	}

	ioctl_request.modify.access.data.data16 = mode_value;
	ioctl_request.modify.access.offset = mode_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x0001;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_IncEnc_Enable_Hold(DM7820_Board_Descriptor * handle,
			  dm7820_incenc_encoder encoder, uint8_t enable)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;
	uint16_t mode_value;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Incremental Encoder Mode Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		mode_offset = DM7820_BAR2_INCENC0_MODE;
		break;

	case DM7820_INCENC_ENCODER_1:
		mode_offset = DM7820_BAR2_INCENC1_MODE;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up Incremental Encoder Mode Register value and modify the register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	mode_value = 0x0000;

	if (enable) {
		mode_value = 0x0002;
	}

	ioctl_request.modify.access.data.data16 = (mode_value << 1);
	ioctl_request.modify.access.offset = mode_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x0002;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_IncEnc_Get_Independent_Value(DM7820_Board_Descriptor * handle,
				    dm7820_incenc_encoder encoder,
				    dm7820_incenc_channel channel,
				    uint16_t * value)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;
	uint16_t value_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	if (DM7820_Validate_IncEnc_Channel(channel) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offsets of Incremental Encoder Value and Mode Registers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		mode_offset = DM7820_BAR2_INCENC0_MODE;

		switch (channel) {
		case DM7820_INCENC_CHANNEL_A:
			value_offset = DM7820_BAR2_INCENC0_VALUEA;
			break;

		case DM7820_INCENC_CHANNEL_B:
			value_offset = DM7820_BAR2_INCENC0_VALUEB;
			break;
		}

		break;

	case DM7820_INCENC_ENCODER_1:
		mode_offset = DM7820_BAR2_INCENC1_MODE;

		switch (channel) {
		case DM7820_INCENC_CHANNEL_A:
			value_offset = DM7820_BAR2_INCENC1_VALUEA;
			break;

		case DM7820_INCENC_CHANNEL_B:
			value_offset = DM7820_BAR2_INCENC1_VALUEB;
			break;
		}

		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Verify that encoder channels are not joined
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read Incremental Encoder Mode Register
	 */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = mode_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	/*
	 * Make sure encoder channels are not joined
	 */

	if (ioctl_request.readwrite.access.data.data16 & 0x0008) {
		errno = EOPNOTSUPP;
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read Incremental Encoder Value Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = value_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);

	if (status == 0) {
		*value = ioctl_request.readwrite.access.data.data16;
	}

	return status;
}

DM7820_Error
DM7820_IncEnc_Get_Joined_Value(DM7820_Board_Descriptor * handle,
			       dm7820_incenc_encoder encoder, uint32_t * value)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;
	uint16_t value_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offsets of Incremental Encoder Value and Mode Registers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		mode_offset = DM7820_BAR2_INCENC0_MODE;
		value_offset = DM7820_BAR2_INCENC0_VALUEA;
		break;

	case DM7820_INCENC_ENCODER_1:
		mode_offset = DM7820_BAR2_INCENC1_MODE;
		value_offset = DM7820_BAR2_INCENC1_VALUEA;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Verify that encoder channels are joined
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read Incremental Encoder Mode Register
	 */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = mode_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	/*
	 * Make sure encoder channels are joined
	 */

	if (!(ioctl_request.readwrite.access.data.data16 & 0x0008)) {
		errno = EOPNOTSUPP;
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read Incremental Encoder Value Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data32 = 0x00000000;
	ioctl_request.readwrite.access.offset = value_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_32;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);

	if (status == 0) {
		*value = ioctl_request.readwrite.access.data.data32;
	}

	return status;
}

DM7820_Error
DM7820_IncEnc_Get_Status(DM7820_Board_Descriptor * handle,
			 dm7820_incenc_encoder encoder,
			 dm7820_incenc_status_condition condition,
			 uint8_t * occurred)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t int_offset = 0x0000;
	uint16_t mask = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	if (DM7820_Validate_IncEnc_Status_Condition(condition) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Incremental Encoder Interrupt Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		int_offset = DM7820_BAR2_INCENC0_INT;
		break;

	case DM7820_INCENC_ENCODER_1:
		int_offset = DM7820_BAR2_INCENC1_INT;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine mask for given incremental encoder status bit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (condition) {
	case DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER:
		mask = 0x0100;
		break;

	case DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER:
		mask = 0x0200;
		break;

	case DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER:
		mask = 0x0400;
		break;

	case DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER:
		mask = 0x0800;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read Incremental Encoder Interrupt Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = int_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	*occurred = ((ioctl_request.readwrite.access.data.data16 & mask) >> 8);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Clear Incremental Encoder Interrupt register condition status bit if set
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (*occurred) {

		/*
		 * Preserve interrupt enable bits (bits 0 through 3) and reserved bits
		 * (bits 4 through 7 and 12 through 15)
		 */

		ioctl_request.readwrite.access.data.data16 &= 0xF0FF;

		ioctl_request.readwrite.access.data.data16 |= mask;
		ioctl_request.readwrite.access.offset = int_offset;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			     &ioctl_request);
	}

	return 0;
}

DM7820_Error
DM7820_IncEnc_Set_Independent_Value(DM7820_Board_Descriptor * handle,
				    dm7820_incenc_encoder encoder,
				    dm7820_incenc_channel channel,
				    uint16_t value)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;
	uint16_t value_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	if (DM7820_Validate_IncEnc_Channel(channel) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offsets of Incremental Encoder Value and Mode Registers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		mode_offset = DM7820_BAR2_INCENC0_MODE;

		switch (channel) {
		case DM7820_INCENC_CHANNEL_A:
			value_offset = DM7820_BAR2_INCENC0_VALUEA;
			break;

		case DM7820_INCENC_CHANNEL_B:
			value_offset = DM7820_BAR2_INCENC0_VALUEB;
			break;
		}

		break;

	case DM7820_INCENC_ENCODER_1:
		mode_offset = DM7820_BAR2_INCENC1_MODE;

		switch (channel) {
		case DM7820_INCENC_CHANNEL_A:
			value_offset = DM7820_BAR2_INCENC1_VALUEA;
			break;

		case DM7820_INCENC_CHANNEL_B:
			value_offset = DM7820_BAR2_INCENC1_VALUEB;
			break;
		}

		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Verify that encoder channels are not joined
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read Incremental Encoder Mode Register
	 */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = mode_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	/*
	 * Make sure encoder channels are not joined
	 */

	if (ioctl_request.readwrite.access.data.data16 & 0x0008) {
		errno = EOPNOTSUPP;
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write Incremental Encoder Value Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = value;
	ioctl_request.readwrite.access.offset = value_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_IncEnc_Set_Joined_Value(DM7820_Board_Descriptor * handle,
			       dm7820_incenc_encoder encoder, uint32_t value)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;
	uint16_t value_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offsets of Incremental Encoder Value and Mode Registers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		mode_offset = DM7820_BAR2_INCENC0_MODE;
		value_offset = DM7820_BAR2_INCENC0_VALUEA;
		break;

	case DM7820_INCENC_ENCODER_1:
		mode_offset = DM7820_BAR2_INCENC1_MODE;
		value_offset = DM7820_BAR2_INCENC1_VALUEA;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Verify that encoder channels are joined
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read Incremental Encoder Mode Register
	 */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = mode_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	/*
	 * Make sure encoder channels are joined
	 */

	if (!(ioctl_request.readwrite.access.data.data16 & 0x0008)) {
		errno = EOPNOTSUPP;
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write Incremental Encoder Value Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data32 = value;
	ioctl_request.readwrite.access.offset = value_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_32;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_IncEnc_Set_Master(DM7820_Board_Descriptor * handle,
			 dm7820_incenc_encoder encoder,
			 dm7820_incenc_master_clock master)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t clock_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_IncEnc_Encoder(encoder) == -1) {
		return -1;
	}

	if (DM7820_Validate_IncEnc_Master(master) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Incremental Encoder Clock Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (encoder) {
	case DM7820_INCENC_ENCODER_0:
		clock_offset = DM7820_BAR2_INCENC0_CLOCK;
		break;

	case DM7820_INCENC_ENCODER_1:
		clock_offset = DM7820_BAR2_INCENC1_CLOCK;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify Incremental Encoder Clock Register to select master clock
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = master;
	ioctl_request.modify.access.offset = clock_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x000F;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_PWM_Enable(DM7820_Board_Descriptor * handle,
		  dm7820_pwm_modulator pwm, uint8_t enable)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;
	uint16_t mode_value;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PWM_Modulator(pwm) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of PWM Mode Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (pwm) {
	case DM7820_PWM_MODULATOR_0:
		mode_offset = DM7820_BAR2_PWM0_MODE;
		break;

	case DM7820_PWM_MODULATOR_1:
		mode_offset = DM7820_BAR2_PWM1_MODE;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up PWM Mode Register value and modify the register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	mode_value = 0x0000;

	if (enable) {
		mode_value = 0x0001;
	}

	ioctl_request.modify.access.data.data16 = mode_value;
	ioctl_request.modify.access.offset = mode_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x0001;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_PWM_Set_Period(DM7820_Board_Descriptor * handle,
		      dm7820_pwm_modulator pwm, uint32_t period)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t period_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PWM_Modulator(pwm) == -1) {
		return -1;
	}

	if (period == 0x00000000) {
		errno = ERANGE;
		return -1;
	}

	if (period > 0x00010000) {
		errno = ERANGE;
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of PWM Period Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (pwm) {
	case DM7820_PWM_MODULATOR_0:
		period_offset = DM7820_BAR2_PWM0_PERIOD;
		break;

	case DM7820_PWM_MODULATOR_1:
		period_offset = DM7820_BAR2_PWM1_PERIOD;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write period to PWM Period Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * The hardware thinks the period is one less than the real value, so make
	 * this adjustment
	 */

	ioctl_request.readwrite.access.data.data16 = (period - 1);
	ioctl_request.readwrite.access.offset = period_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_PWM_Set_Period_Master(DM7820_Board_Descriptor * handle,
			     dm7820_pwm_modulator pwm,
			     dm7820_pwm_period_master_clock master)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t clock_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PWM_Modulator(pwm) == -1) {
		return -1;
	}

	if (DM7820_Validate_PWM_Period_Master(master) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of PWM Clock Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (pwm) {
	case DM7820_PWM_MODULATOR_0:
		clock_offset = DM7820_BAR2_PWM0_CLK;
		break;

	case DM7820_PWM_MODULATOR_1:
		clock_offset = DM7820_BAR2_PWM1_CLK;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify PWM Clock Register value to select period master clock
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = (master << 4);
	ioctl_request.modify.access.offset = clock_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x00F0;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_PWM_Set_Width(DM7820_Board_Descriptor * handle,
		     dm7820_pwm_modulator pwm,
		     dm7820_pwm_output output, uint16_t width)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t width_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PWM_Modulator(pwm) == -1) {
		return -1;
	}

	if (DM7820_Validate_PWM_Output(output) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of PWM Width Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (pwm) {
	case DM7820_PWM_MODULATOR_0:
		switch (output) {
		case DM7820_PWM_OUTPUT_A:
			width_offset = DM7820_BAR2_PWM0_WIDTHA;
			break;

		case DM7820_PWM_OUTPUT_B:
			width_offset = DM7820_BAR2_PWM0_WIDTHB;
			break;

		case DM7820_PWM_OUTPUT_C:
			width_offset = DM7820_BAR2_PWM0_WIDTHC;
			break;

		case DM7820_PWM_OUTPUT_D:
			width_offset = DM7820_BAR2_PWM0_WIDTHD;
			break;
		}

		break;

	case DM7820_PWM_MODULATOR_1:
		switch (output) {
		case DM7820_PWM_OUTPUT_A:
			width_offset = DM7820_BAR2_PWM1_WIDTHA;
			break;

		case DM7820_PWM_OUTPUT_B:
			width_offset = DM7820_BAR2_PWM1_WIDTHB;
			break;

		case DM7820_PWM_OUTPUT_C:
			width_offset = DM7820_BAR2_PWM1_WIDTHC;
			break;

		case DM7820_PWM_OUTPUT_D:
			width_offset = DM7820_BAR2_PWM1_WIDTHD;
			break;
		}

		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write width to PWM Width Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = width;
	ioctl_request.readwrite.access.offset = width_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_PWM_Set_Width_Master(DM7820_Board_Descriptor * handle,
			    dm7820_pwm_modulator pwm,
			    dm7820_pwm_width_master_clock master)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t clock_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PWM_Modulator(pwm) == -1) {
		return -1;
	}

	if (DM7820_Validate_PWM_Width_Master(master) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of PWM Clock Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (pwm) {
	case DM7820_PWM_MODULATOR_0:
		clock_offset = DM7820_BAR2_PWM0_CLK;
		break;

	case DM7820_PWM_MODULATOR_1:
		clock_offset = DM7820_BAR2_PWM1_CLK;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify PWM Clock Register value to select width master clock
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = master;
	ioctl_request.modify.access.offset = clock_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x000F;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_PrgClk_Set_Master(DM7820_Board_Descriptor * handle,
			 dm7820_prgclk_clock clock,
			 dm7820_prgclk_master_clock master)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t master_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PrgClk_Clock(clock) == -1) {
		return -1;
	}

	if (DM7820_Validate_PrgClk_Master(master) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Programmable Clock Master Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (clock) {
	case DM7820_PRGCLK_CLOCK_0:
		master_offset = DM7820_BAR2_PRGCLK0_CLK;
		break;

	case DM7820_PRGCLK_CLOCK_1:
		master_offset = DM7820_BAR2_PRGCLK1_CLK;
		break;

	case DM7820_PRGCLK_CLOCK_2:
		master_offset = DM7820_BAR2_PRGCLK2_CLK;
		break;

	case DM7820_PRGCLK_CLOCK_3:
		master_offset = DM7820_BAR2_PRGCLK3_CLK;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Modify Programmable Clock Master Register to select master clock
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = master;
	ioctl_request.modify.access.offset = master_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x000F;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_PrgClk_Set_Mode(DM7820_Board_Descriptor * handle,
		       dm7820_prgclk_clock clock, dm7820_prgclk_mode mode)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PrgClk_Clock(clock) == -1) {
		return -1;
	}

	if (DM7820_Validate_PrgClk_Mode(mode) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Programmable Clock Mode Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (clock) {
	case DM7820_PRGCLK_CLOCK_0:
		mode_offset = DM7820_BAR2_PRGCLK0_MODE;
		break;

	case DM7820_PRGCLK_CLOCK_1:
		mode_offset = DM7820_BAR2_PRGCLK1_MODE;
		break;

	case DM7820_PRGCLK_CLOCK_2:
		mode_offset = DM7820_BAR2_PRGCLK2_MODE;
		break;

	case DM7820_PRGCLK_CLOCK_3:
		mode_offset = DM7820_BAR2_PRGCLK3_MODE;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up Programmable Clock Mode Register value and modify it
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = mode;
	ioctl_request.modify.access.offset = mode_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x0003;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_PrgClk_Set_Period(DM7820_Board_Descriptor * handle,
			 dm7820_prgclk_clock clock, uint32_t period)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t period_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PrgClk_Clock(clock) == -1) {
		return -1;
	}

	if (period == 0x00000000) {
		errno = ERANGE;
		return -1;
	}

	if (period > 0x00010000) {
		errno = ERANGE;
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Programmable Clock Period Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (clock) {
	case DM7820_PRGCLK_CLOCK_0:
		period_offset = DM7820_BAR2_PRGCLK0_PERIOD;
		break;

	case DM7820_PRGCLK_CLOCK_1:
		period_offset = DM7820_BAR2_PRGCLK1_PERIOD;
		break;

	case DM7820_PRGCLK_CLOCK_2:
		period_offset = DM7820_BAR2_PRGCLK2_PERIOD;
		break;

	case DM7820_PRGCLK_CLOCK_3:
		period_offset = DM7820_BAR2_PRGCLK3_PERIOD;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write period to Programmable Clock Period Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * The hardware thinks the period is one less than the real value, so make
	 * this adjustment
	 */

	ioctl_request.readwrite.access.data.data16 = (period - 1);
	ioctl_request.readwrite.access.offset = period_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_PrgClk_Set_Start_Trigger(DM7820_Board_Descriptor * handle,
				dm7820_prgclk_clock clock,
				dm7820_prgclk_start_trigger start)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t start_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PrgClk_Clock(clock) == -1) {
		return -1;
	}

	if (DM7820_Validate_PrgClk_Start_Trigger(start) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Programmable Clock Start/Stop Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (clock) {
	case DM7820_PRGCLK_CLOCK_0:
		start_offset = DM7820_BAR2_PRGCLK0_START_STOP;
		break;

	case DM7820_PRGCLK_CLOCK_1:
		start_offset = DM7820_BAR2_PRGCLK1_START_STOP;
		break;

	case DM7820_PRGCLK_CLOCK_2:
		start_offset = DM7820_BAR2_PRGCLK2_START_STOP;
		break;

	case DM7820_PRGCLK_CLOCK_3:
		start_offset = DM7820_BAR2_PRGCLK3_START_STOP;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up Programmable Clock Start/Stop Register value and modify it
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = start;
	ioctl_request.modify.access.offset = start_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x001F;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_PrgClk_Set_Stop_Trigger(DM7820_Board_Descriptor * handle,
			       dm7820_prgclk_clock clock,
			       dm7820_prgclk_stop_trigger stop)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t stop_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_PrgClk_Clock(clock) == -1) {
		return -1;
	}

	if (DM7820_Validate_PrgClk_Stop_Trigger(stop) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Programmable Clock Start/Stop Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (clock) {
	case DM7820_PRGCLK_CLOCK_0:
		stop_offset = DM7820_BAR2_PRGCLK0_START_STOP;
		break;

	case DM7820_PRGCLK_CLOCK_1:
		stop_offset = DM7820_BAR2_PRGCLK1_START_STOP;
		break;

	case DM7820_PRGCLK_CLOCK_2:
		stop_offset = DM7820_BAR2_PRGCLK2_START_STOP;
		break;

	case DM7820_PRGCLK_CLOCK_3:
		stop_offset = DM7820_BAR2_PRGCLK3_START_STOP;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up Programmable Clock Start/Stop Register value and modify it
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = (stop << 8);
	ioctl_request.modify.access.offset = stop_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x1F00;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_StdIO_Get_Input(DM7820_Board_Descriptor * handle,
		       DM7820_StdIO_Port port, uint16_t * value)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t input_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_StdIO_Port(port) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset for port Input Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (port) {
	case DM7820_STDIO_PORT_0:
		input_offset = DM7820_BAR2_PORT0_INPUT;
		break;

	case DM7820_STDIO_PORT_1:
		input_offset = DM7820_BAR2_PORT1_INPUT;
		break;

	case DM7820_STDIO_PORT_2:
		input_offset = DM7820_BAR2_PORT2_INPUT;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read from port Input Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = input_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == 0) {
		*value = ioctl_request.readwrite.access.data.data16;
	}

	return status;
}

DM7820_Error
DM7820_StdIO_Set_IO_Mode(DM7820_Board_Descriptor * handle,
			 DM7820_StdIO_Port port,
			 uint16_t bits, DM7820_StdIO_IO_Mode mode)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_mask = 0x0000;
	uint16_t mode_offset = 0x0000;
	uint16_t mode_value = 0x0000;
	uint16_t tristate_mask = 0x0000;
	uint16_t tristate_offset = 0x0000;
	uint16_t tristate_value = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_StdIO_Port(port) == -1) {
		return -1;
	}

	if (DM7820_Validate_StdIO_IO_Mode(mode) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offsets for Mode and Tristate Registers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (port) {
	case DM7820_STDIO_PORT_0:
		mode_offset = DM7820_BAR2_PORT0_MODE;
		tristate_offset = DM7820_BAR2_PORT0_TRISTATE;
		break;

	case DM7820_STDIO_PORT_1:
		mode_offset = DM7820_BAR2_PORT1_MODE;
		tristate_offset = DM7820_BAR2_PORT1_TRISTATE;
		break;

	case DM7820_STDIO_PORT_2:
		mode_offset = DM7820_BAR2_PORT2_MODE;
		tristate_offset = DM7820_BAR2_PORT2_TRISTATE;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine masks & values for Mode and Tristate Registers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	mode_mask = bits;
	tristate_mask = bits;

	switch (mode) {
	case DM7820_STDIO_MODE_INPUT:
		mode_value = 0x0000;
		tristate_value = 0x0000;
		break;

	case DM7820_STDIO_MODE_OUTPUT:
		mode_value = 0x0000;
		tristate_value = bits;
		break;

	case DM7820_STDIO_MODE_PER_OUT:
		mode_value = bits;
		tristate_value = bits;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up port's Tristate Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = tristate_value;
	ioctl_request.modify.access.offset = tristate_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = tristate_mask;

	if (ioctl
	    (handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
	     &ioctl_request)
	    == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up port's Mode Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = mode_value;
	ioctl_request.modify.access.offset = mode_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = mode_mask;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_StdIO_Set_Output(DM7820_Board_Descriptor * handle,
			DM7820_StdIO_Port port, uint16_t value)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t output_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_StdIO_Port(port) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset for port Output Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (port) {
	case DM7820_STDIO_PORT_0:
		output_offset = DM7820_BAR2_PORT0_OUTPUT;
		break;

	case DM7820_STDIO_PORT_1:
		output_offset = DM7820_BAR2_PORT1_OUTPUT;
		break;

	case DM7820_STDIO_PORT_2:
		output_offset = DM7820_BAR2_PORT2_OUTPUT;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Write to port Output Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = value;
	ioctl_request.readwrite.access.offset = output_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_StdIO_Set_Periph_Mode(DM7820_Board_Descriptor * handle,
			     DM7820_StdIO_Port port,
			     uint16_t bits, DM7820_StdIO_Periph_Mode mode)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t high_mask = 0x0000;
	uint16_t high_offset = 0x0000;
	uint16_t high_value = 0x0000;
	uint16_t low_mask = 0x0000;
	uint16_t low_offset = 0x0000;
	uint16_t low_value = 0x0000;
	uint8_t port_bit;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_StdIO_Port(port) == -1) {
		return -1;
	}

	if (DM7820_Validate_StdIO_Peripheral_Mode(mode) == -1) {
		return -1;
	}

	/*
	 * Verify that port supports the desired mode
	 */

	switch (mode) {
	case DM7820_STDIO_PERIPH_PWM:
	case DM7820_STDIO_PERIPH_CLK_OTHER:
		if (port != DM7820_STDIO_PORT_2) {
			errno = EOPNOTSUPP;
			return -1;
		}

		break;

	default:
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of port's Low and High Peripheral Select Registers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (port) {
	case DM7820_STDIO_PORT_0:
		low_offset = DM7820_BAR2_PORT0_PERIPH_SEL_L;
		high_offset = DM7820_BAR2_PORT0_PERIPH_SEL_H;
		break;

	case DM7820_STDIO_PORT_1:
		low_offset = DM7820_BAR2_PORT1_PERIPH_SEL_L;
		high_offset = DM7820_BAR2_PORT1_PERIPH_SEL_H;
		break;

	case DM7820_STDIO_PORT_2:
		low_offset = DM7820_BAR2_PORT2_PERIPH_SEL_L;
		high_offset = DM7820_BAR2_PORT2_PERIPH_SEL_H;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine masks & values of port's Low and High Peripheral Select Registers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Process each of the 16 bits in the port bit mask
	 */

	for (port_bit = 0; port_bit < 16; port_bit++) {

		/*
		 * Is the current port bit supposed to be modified?
		 */

		if (bits & (1 << port_bit)) {

			/*
			 * Caller wants to change current port bit
			 */

			/*
			 * Which bit is the current port bit?
			 */

			if (port_bit < 8) {

				/*
				 * Current port bit is between 0 and 7
				 */

				/*
				 * Modify Low Peripheral Select Register mask and value to
				 * change behavior of current port bit
				 */

				low_mask |= (0x0003 << (port_bit * 2));
				low_value |= (mode << (port_bit * 2));
			} else {

				/*
				 * Current port bit is between 8 and 15
				 */

				/*
				 * Modify High Peripheral Select Register mask and value to
				 * change behavior of current port bit
				 */

				high_mask |= (0x0003 << ((port_bit - 8) * 2));
				high_value |= (mode << ((port_bit - 8) * 2));
			}
		}
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up port's Low Peripheral Select Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = low_value;
	ioctl_request.modify.access.offset = low_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = low_mask;

	if (ioctl
	    (handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
	     &ioctl_request)
	    == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up port's High Peripheral Select Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = high_value;
	ioctl_request.modify.access.offset = high_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = high_mask;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_StdIO_Strobe_Input(DM7820_Board_Descriptor * handle,
			  DM7820_StdIO_Strobe strobe, uint8_t * state)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_StdIO_Strobe(strobe) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read Strobe Status Register and determine state of desired signal
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read Strobe Status Register
	 */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_STROBE_STATUS;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == 0) {
		uint16_t state_mask = 0x0000;

		/*
		 * Determine mask for accessing desired signal state
		 */

		switch (strobe) {
		case DM7820_STDIO_STROBE_1:
			state_mask = 0x0001;
			break;

		case DM7820_STDIO_STROBE_2:
			state_mask = 0x0002;
			break;
		}

		/*
		 * Return signal state to caller
		 */

		*state =
		    (ioctl_request.readwrite.access.data.data16 & state_mask);
	}

	return status;
}

DM7820_Error
DM7820_StdIO_Strobe_Mode(DM7820_Board_Descriptor * handle,
			 DM7820_StdIO_Strobe strobe, uint8_t output)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mode_mask = 0x0000;
	uint16_t mode_value;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_StdIO_Strobe(strobe) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine mask and value for Strobe Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Figure out which bit controls the strobe signal mode
	 */

	switch (strobe) {
	case DM7820_STDIO_STROBE_1:
		mode_mask = 0x0100;
		break;

	case DM7820_STDIO_STROBE_2:
		mode_mask = 0x0200;
		break;
	}

	/*
	 * Set register value to assume signal is an input
	 */

	mode_value = 0x0000;

	/*
	 * If signal is to be an output, use mode mask to set it to an output
	 */

	if (output) {
		mode_value = mode_mask;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set Strobe Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = mode_value;
	ioctl_request.modify.access.offset = DM7820_BAR2_STROBE_STATUS;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = mode_mask;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_StdIO_Strobe_Output(DM7820_Board_Descriptor * handle,
			   DM7820_StdIO_Strobe strobe, uint8_t state)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t state_mask = 0x0000;
	uint16_t state_value;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_StdIO_Strobe(strobe) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine mask and value for Strobe Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Figure out which bit controls the strobe signal state
	 */

	switch (strobe) {
	case DM7820_STDIO_STROBE_1:
		state_mask = 0x0010;
		break;

	case DM7820_STDIO_STROBE_2:
		state_mask = 0x0020;
		break;
	}

	/*
	 * Set register value to assume signal is low
	 */

	state_value = 0x0000;

	/*
	 * If signal is to be high, use state mask to set it high
	 */

	if (state) {
		state_value = state_mask;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set Strobe Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = state_value;
	ioctl_request.modify.access.offset = DM7820_BAR2_STROBE_STATUS;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = state_mask;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_TmrCtr_Get_Status(DM7820_Board_Descriptor * handle,
			 dm7820_tmrctr_timer timer, uint8_t * occurred)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t mask = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_TmrCtr_Timer(timer) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine mask for given timer/counter status bit
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (timer) {
	case DM7820_TMRCTR_TIMER_A_0:
		mask = 0x0100;
		break;

	case DM7820_TMRCTR_TIMER_A_1:
		mask = 0x0200;
		break;

	case DM7820_TMRCTR_TIMER_A_2:
		mask = 0x0400;
		break;

	case DM7820_TMRCTR_TIMER_B_0:
		mask = 0x0800;
		break;

	case DM7820_TMRCTR_TIMER_B_1:
		mask = 0x1000;
		break;

	case DM7820_TMRCTR_TIMER_B_2:
		mask = 0x2000;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read Timer/Counter Interrupt Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.readwrite.access.data.data16 = 0x0000;
	ioctl_request.readwrite.access.offset = DM7820_BAR2_TC_INT;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_16;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == -1) {
		return -1;
	}

	*occurred = ((ioctl_request.readwrite.access.data.data16 & mask) >> 8);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Clear Timer/Counter Interrupt register timer status bit if set
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (*occurred) {

		/*
		 * Preserve interrupt enable bits (bits 0 through 5) and reserved bits
		 * (bits 6, 7, 14, and 15)
		 */

		ioctl_request.readwrite.access.data.data16 &= 0xC0FF;

		ioctl_request.readwrite.access.data.data16 |= mask;
		ioctl_request.readwrite.access.offset = DM7820_BAR2_TC_INT;
		ioctl_request.readwrite.access.region =
		    DM7820_PCI_REGION_FPGA_MEM;
		ioctl_request.readwrite.access.size =
		    DM7820_PCI_REGION_ACCESS_16;

		return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
			     &ioctl_request);
	}

	return 0;
}

DM7820_Error
DM7820_TmrCtr_Program(DM7820_Board_Descriptor * handle,
		      dm7820_tmrctr_timer timer,
		      dm7820_tmrctr_waveform waveform,
		      dm7820_tmrctr_count_mode count_mode, uint16_t divisor)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t control_offset = 0x0000;
	uint16_t timer_offset = 0x0000;
	uint8_t control_word;
	uint8_t timer_select = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_TmrCtr_Timer(timer) == -1) {
		return -1;
	}

	if (DM7820_Validate_TmrCtr_Waveform(waveform) == -1) {
		return -1;
	}

	if (DM7820_Validate_TmrCtr_Count_Mode(count_mode) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Timer/Counter Control Word Register, offset of
	   Timer/Counter Register, and timer selector in control word
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (timer) {
	case DM7820_TMRCTR_TIMER_A_0:
		control_offset = DM7820_BAR2_TCA_CON_WORD;
		timer_offset = DM7820_BAR2_TCA_COUNTER_0;
		timer_select = 0x00;
		break;

	case DM7820_TMRCTR_TIMER_A_1:
		control_offset = DM7820_BAR2_TCA_CON_WORD;
		timer_offset = DM7820_BAR2_TCA_COUNTER_1;
		timer_select = 0x01;
		break;

	case DM7820_TMRCTR_TIMER_A_2:
		control_offset = DM7820_BAR2_TCA_CON_WORD;
		timer_offset = DM7820_BAR2_TCA_COUNTER_2;
		timer_select = 0x02;
		break;

	case DM7820_TMRCTR_TIMER_B_0:
		control_offset = DM7820_BAR2_TCB_CON_WORD;
		timer_offset = DM7820_BAR2_TCB_COUNTER_0;
		timer_select = 0x00;
		break;

	case DM7820_TMRCTR_TIMER_B_1:
		control_offset = DM7820_BAR2_TCB_CON_WORD;
		timer_offset = DM7820_BAR2_TCB_COUNTER_1;
		timer_select = 0x01;
		break;

	case DM7820_TMRCTR_TIMER_B_2:
		control_offset = DM7820_BAR2_TCB_CON_WORD;
		timer_offset = DM7820_BAR2_TCB_COUNTER_2;
		timer_select = 0x02;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up Timer/Counter Control Word Register value and program it
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	control_word = 0x00;

	/*
	 * Place count mode into bit 0
	 */

	control_word |= count_mode;

	/*
	 * Place waveform mode into bits 3 through 1
	 */

	control_word |= (waveform << 1);

	/*
	 * Set bits 5 and 4 to ones which sets the timer to be read/loaded least
	 * significant byte first then most significant byte
	 */

	control_word |= (0x3 << 4);

	/*
	 * Place timer into bits 7 and 6
	 */

	control_word |= (timer_select << 6);

	ioctl_request.readwrite.access.data.data8 = control_word;
	ioctl_request.readwrite.access.offset = control_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;

	if (ioctl
	    (handle->file_descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request)
	    == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Load timer divisor
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Write least significant byte of divisor into Timer/Counter Register
	 */

	ioctl_request.readwrite.access.data.data8 = (divisor & 0x00FF);
	ioctl_request.readwrite.access.offset = timer_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;

	if (ioctl
	    (handle->file_descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request)
	    == -1) {
		return -1;
	}

	/*
	 * Write most significant byte of divisor into Timer/Counter Register
	 */

	ioctl_request.readwrite.access.data.data8 = ((divisor & 0xFF00) >> 8);
	ioctl_request.readwrite.access.offset = timer_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_WRITE,
		     &ioctl_request);
}

DM7820_Error
DM7820_TmrCtr_Read(DM7820_Board_Descriptor * handle,
		   dm7820_tmrctr_timer timer, uint16_t * value)
{
	DM7820_Error status;
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t control_offset = 0x0000;
	uint16_t timer_offset = 0x0000;
	uint16_t timer_value;
	uint8_t control_word;
	uint8_t timer_select = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_TmrCtr_Timer(timer) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Timer/Counter Control Word Register, offset of
	   Timer/Counter Register, and timer selector in control word
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (timer) {
	case DM7820_TMRCTR_TIMER_A_0:
		control_offset = DM7820_BAR2_TCA_CON_WORD;
		timer_offset = DM7820_BAR2_TCA_COUNTER_0;
		timer_select = 0x00;
		break;

	case DM7820_TMRCTR_TIMER_A_1:
		control_offset = DM7820_BAR2_TCA_CON_WORD;
		timer_offset = DM7820_BAR2_TCA_COUNTER_1;
		timer_select = 0x01;
		break;

	case DM7820_TMRCTR_TIMER_A_2:
		control_offset = DM7820_BAR2_TCA_CON_WORD;
		timer_offset = DM7820_BAR2_TCA_COUNTER_2;
		timer_select = 0x02;
		break;

	case DM7820_TMRCTR_TIMER_B_0:
		control_offset = DM7820_BAR2_TCB_CON_WORD;
		timer_offset = DM7820_BAR2_TCB_COUNTER_0;
		timer_select = 0x00;
		break;

	case DM7820_TMRCTR_TIMER_B_1:
		control_offset = DM7820_BAR2_TCB_CON_WORD;
		timer_offset = DM7820_BAR2_TCB_COUNTER_1;
		timer_select = 0x01;
		break;

	case DM7820_TMRCTR_TIMER_B_2:
		control_offset = DM7820_BAR2_TCB_CON_WORD;
		timer_offset = DM7820_BAR2_TCB_COUNTER_2;
		timer_select = 0x02;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Program Timer/Counter Control Word Register to latch counter value
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Timer/Counter Control Word Register bits 3 through zero are ignored when
	 * latching the counter value.  Set register bits 5 and 4 to zero which
	 * enables counter latch.
	 */

	control_word = 0x00;

	/*
	 * Place timer into bits 7 and 6
	 */

	control_word |= (timer_select << 6);

	ioctl_request.readwrite.access.data.data8 = control_word;
	ioctl_request.readwrite.access.offset = control_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;

	if (ioctl
	    (handle->file_descriptor, DM7820_IOCTL_REGION_WRITE, &ioctl_request)
	    == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Read timer value
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read least significant byte of value from Timer/Counter Register
	 */

	ioctl_request.readwrite.access.data.data8 = 0x0000;
	ioctl_request.readwrite.access.offset = timer_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;

	if (ioctl
	    (handle->file_descriptor, DM7820_IOCTL_REGION_READ, &ioctl_request)
	    == -1) {
		return -1;
	}

	timer_value = ioctl_request.readwrite.access.data.data8;

	/*
	 * Read most significant byte of value from Timer/Counter Register
	 */

	ioctl_request.readwrite.access.data.data8 = 0x0000;
	ioctl_request.readwrite.access.offset = timer_offset;
	ioctl_request.readwrite.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.readwrite.access.size = DM7820_PCI_REGION_ACCESS_8;

	status =
	    ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_READ,
		  &ioctl_request);
	if (status == 0) {
		timer_value |= (ioctl_request.readwrite.access.data.data8 << 8);
		*value = timer_value;
	}

	return status;
}

DM7820_Error
DM7820_TmrCtr_Select_Clock(DM7820_Board_Descriptor * handle,
			   dm7820_tmrctr_timer timer, dm7820_tmrctr_clock clock)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t control_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_TmrCtr_Timer(timer) == -1) {
		return -1;
	}

	if (DM7820_Validate_TmrCtr_Clock(clock) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Timer/Counter Control Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (timer) {
	case DM7820_TMRCTR_TIMER_A_0:
		control_offset = DM7820_BAR2_TC_A0_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_A_1:
		control_offset = DM7820_BAR2_TC_A1_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_A_2:
		control_offset = DM7820_BAR2_TC_A2_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_B_0:
		control_offset = DM7820_BAR2_TC_B0_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_B_1:
		control_offset = DM7820_BAR2_TC_B1_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_B_2:
		control_offset = DM7820_BAR2_TC_B2_CONTROL;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set clock input in Timer/Counter Control Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = clock;
	ioctl_request.modify.access.offset = control_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x000F;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_TmrCtr_Select_Gate(DM7820_Board_Descriptor * handle,
			  dm7820_tmrctr_timer timer, dm7820_tmrctr_gate gate)
{
	dm7820_ioctl_argument_t ioctl_request;
	uint16_t control_offset = 0x0000;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Argument validation
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (DM7820_Validate_TmrCtr_Timer(timer) == -1) {
		return -1;
	}

	if (DM7820_Validate_TmrCtr_Gate(gate) == -1) {
		return -1;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine offset of Timer/Counter Control Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (timer) {
	case DM7820_TMRCTR_TIMER_A_0:
		control_offset = DM7820_BAR2_TC_A0_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_A_1:
		control_offset = DM7820_BAR2_TC_A1_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_A_2:
		control_offset = DM7820_BAR2_TC_A2_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_B_0:
		control_offset = DM7820_BAR2_TC_B0_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_B_1:
		control_offset = DM7820_BAR2_TC_B1_CONTROL;
		break;

	case DM7820_TMRCTR_TIMER_B_2:
		control_offset = DM7820_BAR2_TC_B2_CONTROL;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set gate input in Timer/Counter Control Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	ioctl_request.modify.access.data.data16 = (gate << 8);
	ioctl_request.modify.access.offset = control_offset;
	ioctl_request.modify.access.region = DM7820_PCI_REGION_FPGA_MEM;
	ioctl_request.modify.access.size = DM7820_PCI_REGION_ACCESS_16;
	ioctl_request.modify.mask.mask16 = 0x1F00;

	return ioctl(handle->file_descriptor, DM7820_IOCTL_REGION_MODIFY,
		     &ioctl_request);
}

DM7820_Error
DM7820_General_InstallISR(DM7820_Board_Descriptor * handle, void (*isr_fnct))
{
	/*
	 * Check for ISR already installed
	 */

	if (handle->isr != NULL) {
		return -EBUSY;
	}

	/*
	 * Set devices isr to the passed userspace isr
	 */

	handle->isr = isr_fnct;

	/*
	 * Start the thread to wait for the interrupt
	 */

	if (pthread_create
	    (&(handle->pid), NULL, DM7820_General_WaitForInterrupt,
	     handle) != 0) {
		errno = EFAULT;
		return -1;
	}

	return 0;
}

DM7820_Error DM7820_General_RemoveISR(DM7820_Board_Descriptor * handle)
{

	/*
	 * Check to make sure there exists an ISR to remove
	 */

	if (!handle->isr) {
		return -EFAULT;
	}

	/*
	 * Make ISR pointer NULL this will be seen by the thread and is a signal
	 * for it to quit.
	 */

	handle->isr = NULL;
	/*
	 * Join back up with ISR thread
	 */

	ioctl(handle->file_descriptor, DM7820_IOCTL_WAKEUP);

	return pthread_join(handle->pid, NULL);
}

void *DM7820_General_WaitForInterrupt(void *ptr)
{
	dm7820_interrupt_info interrupt_status;
	fd_set exception_fds;
	fd_set read_fds;
	int status;
	DM7820_Board_Descriptor *handle;
	handle = (DM7820_Board_Descriptor *) ptr;

	interrupt_status.error = 0;

	while (1) {

		/*
		 * Set up the set of file descriptors that will be watched for input
		 * activity.  Only the DM7820 device file descriptor is of interest.
		 */

		FD_ZERO(&read_fds);
		FD_SET(handle->file_descriptor, &read_fds);

		/*
		 * Set up the set of file descriptors that will be watched for exception
		 * activity.  Only the DM7820 file descriptor is of interest.
		 */

		FD_ZERO(&exception_fds);
		FD_SET(handle->file_descriptor, &exception_fds);

		/*
		 * Wait for the interrupt to happen.  No timeout is given, which means
		 * the process will not be woken up until either an interrupt occurs or
		 * a signal is delivered
		 */

		status = select((handle->file_descriptor) + 1,
				&read_fds, NULL, &exception_fds, NULL);

		/*
		 * The isr should be a null pointer if RemoveISR has been called
		 * This checks if the user has asked the thread to quit.
		 */

		if (handle->isr == NULL) {
			break;
		}

		/*
		 * Check select() error status
		 */

		if (status == -1) {

			/*
			 * Some error occurred.
			 */

			interrupt_status.error = -1;
			(*(handle->isr)) (interrupt_status);
			break;
		}

		if (status == 0) {

			/*
			 * No file descriptors have data available.  Something is broken in the
			 * driver.
			 */

			interrupt_status.error = -1;
			errno = -ENODATA;
			(*(handle->isr)) (interrupt_status);
			break;
		}
		/*
		 * An exception occured, this means that no IRQ line was allocated to
		 * the device when the driver was loaded.
		 */

		if (FD_ISSET(handle->file_descriptor, &exception_fds)) {
			interrupt_status.error = -1;
			errno = -EIO;
			(*(handle->isr)) (interrupt_status);
			break;
		}

		/*
		 * At least one file descriptor has data available and no exception
		 * occured.  Check the device file descriptor to see if it is readable.
		 */

		if (!FD_ISSET(handle->file_descriptor, &read_fds)) {

			/*
			 * The device file is not readable.  This means something is broken.
			 */

			interrupt_status.error = -1;
			errno = -ENODATA;
			(*(handle->isr)) (interrupt_status);
			break;
		}

		status =
		    ioctl(handle->file_descriptor, DM7820_IOCTL_INTERRUPT_INFO,
			  &interrupt_status);

		if (status != 0) {
			interrupt_status.error = -1;
			(*(handle->isr)) (interrupt_status);
			return handle;
		}

		while (interrupt_status.int_remaining != -1) {

			/*
			 * Call the Interrupt Service Routine and pass through the status
			 */

			(*(handle->isr)) (interrupt_status);

			/*
			 * Get Next Status
			 */

			status =
			    ioctl(handle->file_descriptor,
				  DM7820_IOCTL_INTERRUPT_INFO,
				  &interrupt_status);

			if (status != 0) {
				interrupt_status.error = -1;
				(*(handle->isr)) (interrupt_status);
			}
		}
	}

	/*
	 * Terminate waiting thread
	 */

	return 0;
}

DM7820_Error
DM7820_General_SetISRPriority(DM7820_Board_Descriptor * handle, int priority)
{
	struct sched_param param;

	param.sched_priority = priority;
	if (handle->isr == NULL) {
		errno = -EFAULT;
		return -1;
	}

	if (getuid() != 0) {
		return 0;
	}

	return pthread_setschedparam(handle->pid, SCHED_FIFO, &param);
}
