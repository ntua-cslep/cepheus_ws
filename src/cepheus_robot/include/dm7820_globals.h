/**
    @file

    @brief
        Global variables used both in the kernel and in the library

    @warning
        Only the driver and library source code should include this file.

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

    $Id: dm7820_globals.h 89872 2015-07-08 16:05:17Z rgroner $
*/

#ifndef __dm7820_globals_h__
#define __dm7820_globals_h__

#include "dm7820_registers.h"
#include "dm7820_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup DM7820_Globals_Header DM7820 global variable header file
 * @{
 */

/**
 * @brief
 *      Table of information needed to acknowledge, disable, and enable all
 *      interrupt sources
 */

	static dm7820_interrupt_control_t dm7820_interrupt_control[] = {

		/*
		 * Advanced interrupt 0
		 */

		[DM7820_INTERRUPT_ADVINT_0] = {
					       .int_enable_bit = 0x0001,
					       .int_status_bit = 0x0001,
					       .minor_reg =
					       DM7820_MINOR_INT_REG_NONE,
					       .minor_enable = 0x0000,
					       .minor_status = 0x0000,
					       },

		/*
		 * Advanced interrupt 1
		 */

		[DM7820_INTERRUPT_ADVINT_1] = {
					       .int_enable_bit = 0x0002,
					       .int_status_bit = 0x0002,
					       .minor_reg =
					       DM7820_MINOR_INT_REG_NONE,
					       .minor_enable = 0x0000,
					       .minor_status = 0x0000,
					       },

		/*
		 * FIFO 0 empty interrupt
		 */

		[DM7820_INTERRUPT_FIFO_0_EMPTY] = {
						   .int_enable_bit = 0x4000,
						   .int_status_bit = 0x4000,
						   .minor_reg =
						   DM7820_MINOR_INT_REG_FIFO_0_INT,
						   .minor_enable = 0x0008,
						   .minor_status = 0x0800,
						   },

		/*
		 * FIFO 0 full interrupt
		 */

		[DM7820_INTERRUPT_FIFO_0_FULL] = {
						  .int_enable_bit = 0x4000,
						  .int_status_bit = 0x4000,
						  .minor_reg =
						  DM7820_MINOR_INT_REG_FIFO_0_INT,
						  .minor_enable = 0x0004,
						  .minor_status = 0x0400,
						  },

		/*
		 * FIFO 0 overflow interrupt
		 */

		[DM7820_INTERRUPT_FIFO_0_OVERFLOW] = {
						      .int_enable_bit = 0x4000,
						      .int_status_bit = 0x4000,
						      .minor_reg =
						      DM7820_MINOR_INT_REG_FIFO_0_INT,
						      .minor_enable = 0x0010,
						      .minor_status = 0x1000,
						      },

		/*
		 * FIFO 0 read request interrupt
		 */

		[DM7820_INTERRUPT_FIFO_0_READ_REQUEST] = {
							  .int_enable_bit =
							  0x4000,
							  .int_status_bit =
							  0x4000,
							  .minor_reg =
							  DM7820_MINOR_INT_REG_FIFO_0_INT,
							  .minor_enable =
							  0x0001,
							  .minor_status =
							  0x0100,
							  },

		/*
		 * FIFO 0 underflow interrupt
		 */

		[DM7820_INTERRUPT_FIFO_0_UNDERFLOW] = {
						       .int_enable_bit = 0x4000,
						       .int_status_bit = 0x4000,
						       .minor_reg =
						       DM7820_MINOR_INT_REG_FIFO_0_INT,
						       .minor_enable = 0x0020,
						       .minor_status = 0x2000,
						       },

		/*
		 * FIFO 0 write request interrupt
		 */

		[DM7820_INTERRUPT_FIFO_0_WRITE_REQUEST] = {
							   .int_enable_bit =
							   0x4000,
							   .int_status_bit =
							   0x4000,
							   .minor_reg =
							   DM7820_MINOR_INT_REG_FIFO_0_INT,
							   .minor_enable =
							   0x0002,
							   .minor_status =
							   0x0200,
							   },

		/*
		 * FIFO 1 empty interrupt
		 */

		[DM7820_INTERRUPT_FIFO_1_EMPTY] = {
						   .int_enable_bit = 0x8000,
						   .int_status_bit = 0x8000,
						   .minor_reg =
						   DM7820_MINOR_INT_REG_FIFO_1_INT,
						   .minor_enable = 0x0008,
						   .minor_status = 0x0800,
						   },

		/*
		 * FIFO 1 full interrupt
		 */

		[DM7820_INTERRUPT_FIFO_1_FULL] = {
						  .int_enable_bit = 0x8000,
						  .int_status_bit = 0x8000,
						  .minor_reg =
						  DM7820_MINOR_INT_REG_FIFO_1_INT,
						  .minor_enable = 0x0004,
						  .minor_status = 0x0400,
						  },

		/*
		 * FIFO 1 overflow interrupt
		 */

		[DM7820_INTERRUPT_FIFO_1_OVERFLOW] = {
						      .int_enable_bit = 0x8000,
						      .int_status_bit = 0x8000,
						      .minor_reg =
						      DM7820_MINOR_INT_REG_FIFO_1_INT,
						      .minor_enable = 0x0010,
						      .minor_status = 0x1000,
						      },

		/*
		 * FIFO 1 read request interrupt
		 */

		[DM7820_INTERRUPT_FIFO_1_READ_REQUEST] = {
							  .int_enable_bit =
							  0x8000,
							  .int_status_bit =
							  0x8000,
							  .minor_reg =
							  DM7820_MINOR_INT_REG_FIFO_1_INT,
							  .minor_enable =
							  0x0001,
							  .minor_status =
							  0x0100,
							  },

		/*
		 * FIFO 1 underflow interrupt
		 */

		[DM7820_INTERRUPT_FIFO_1_UNDERFLOW] = {
						       .int_enable_bit = 0x8000,
						       .int_status_bit = 0x8000,
						       .minor_reg =
						       DM7820_MINOR_INT_REG_FIFO_1_INT,
						       .minor_enable = 0x0020,
						       .minor_status = 0x2000,
						       },

		/*
		 * FIFO 1 write request interrupt
		 */

		[DM7820_INTERRUPT_FIFO_1_WRITE_REQUEST] = {
							   .int_enable_bit =
							   0x8000,
							   .int_status_bit =
							   0x8000,
							   .minor_reg =
							   DM7820_MINOR_INT_REG_FIFO_1_INT,
							   .minor_enable =
							   0x0002,
							   .minor_status =
							   0x0200,
							   },

		/*
		 * Incremental encoder 0 channel A negative rollover interrupt
		 */

		[DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER] = {
									   .int_enable_bit
									   =
									   0x0010,
									   .int_status_bit
									   =
									   0x0010,
									   .
									   minor_reg
									   =
									   DM7820_MINOR_INT_REG_INCENC_0_INT,
									   .minor_enable
									   =
									   0x0002,
									   .minor_status
									   =
									   0x0200,
									   },

		/*
		 * Incremental encoder 0 channel A positive rollover interrupt
		 */

		[DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER] = {
									   .int_enable_bit
									   =
									   0x0010,
									   .int_status_bit
									   =
									   0x0010,
									   .
									   minor_reg
									   =
									   DM7820_MINOR_INT_REG_INCENC_0_INT,
									   .minor_enable
									   =
									   0x0001,
									   .minor_status
									   =
									   0x0100,
									   },

		/*
		 * Incremental encoder 0 channel B negative rollover interrupt
		 */

		[DM7820_INTERRUPT_INCENC_0_CHANNEL_B_NEGATIVE_ROLLOVER] = {
									   .int_enable_bit
									   =
									   0x0010,
									   .int_status_bit
									   =
									   0x0010,
									   .
									   minor_reg
									   =
									   DM7820_MINOR_INT_REG_INCENC_0_INT,
									   .minor_enable
									   =
									   0x0008,
									   .minor_status
									   =
									   0x0800,
									   },

		/*
		 * Incremental encoder 0 channel B positive rollover interrupt
		 */

		[DM7820_INTERRUPT_INCENC_0_CHANNEL_B_POSITIVE_ROLLOVER] = {
									   .int_enable_bit
									   =
									   0x0010,
									   .int_status_bit
									   =
									   0x0010,
									   .
									   minor_reg
									   =
									   DM7820_MINOR_INT_REG_INCENC_0_INT,
									   .minor_enable
									   =
									   0x0004,
									   .minor_status
									   =
									   0x0400,
									   },

		/*
		 * Incremental encoder 1 channel A negative rollover interrupt
		 */

		[DM7820_INTERRUPT_INCENC_1_CHANNEL_A_NEGATIVE_ROLLOVER] = {
									   .int_enable_bit
									   =
									   0x0020,
									   .int_status_bit
									   =
									   0x0020,
									   .
									   minor_reg
									   =
									   DM7820_MINOR_INT_REG_INCENC_1_INT,
									   .minor_enable
									   =
									   0x0002,
									   .minor_status
									   =
									   0x0200,
									   },

		/*
		 * Incremental encoder 1 channel A positive rollover interrupt
		 */

		[DM7820_INTERRUPT_INCENC_1_CHANNEL_A_POSITIVE_ROLLOVER] = {
									   .int_enable_bit
									   =
									   0x0020,
									   .int_status_bit
									   =
									   0x0020,
									   .
									   minor_reg
									   =
									   DM7820_MINOR_INT_REG_INCENC_1_INT,
									   .minor_enable
									   =
									   0x0001,
									   .minor_status
									   =
									   0x0100,
									   },

		/*
		 * Incremental encoder 1 channel B negative rollover interrupt
		 */

		[DM7820_INTERRUPT_INCENC_1_CHANNEL_B_NEGATIVE_ROLLOVER] = {
									   .int_enable_bit
									   =
									   0x0020,
									   .int_status_bit
									   =
									   0x0020,
									   .
									   minor_reg
									   =
									   DM7820_MINOR_INT_REG_INCENC_1_INT,
									   .minor_enable
									   =
									   0x0008,
									   .minor_status
									   =
									   0x0800,
									   },

		/*
		 * Incremental encoder 1 channel B positive rollover interrupt
		 */

		[DM7820_INTERRUPT_INCENC_1_CHANNEL_B_POSITIVE_ROLLOVER] = {
									   .int_enable_bit
									   =
									   0x0020,
									   .int_status_bit
									   =
									   0x0020,
									   .
									   minor_reg
									   =
									   DM7820_MINOR_INT_REG_INCENC_1_INT,
									   .minor_enable
									   =
									   0x0004,
									   .minor_status
									   =
									   0x0400,
									   },

		/*
		 * Programmable clock 0 interrupt
		 */

		[DM7820_INTERRUPT_PRGCLK_0] = {
					       .int_enable_bit = 0x0400,
					       .int_status_bit = 0x0400,
					       .minor_reg =
					       DM7820_MINOR_INT_REG_NONE,
					       .minor_enable = 0x0000,
					       .minor_status = 0x0000,
					       },

		/*
		 * Programmable clock 1 interrupt
		 */

		[DM7820_INTERRUPT_PRGCLK_1] = {
					       .int_enable_bit = 0x0800,
					       .int_status_bit = 0x0800,
					       .minor_reg =
					       DM7820_MINOR_INT_REG_NONE,
					       .minor_enable = 0x0000,
					       .minor_status = 0x0000,
					       },

		/*
		 * Programmable clock 2 interrupt
		 */

		[DM7820_INTERRUPT_PRGCLK_2] = {
					       .int_enable_bit = 0x1000,
					       .int_status_bit = 0x1000,
					       .minor_reg =
					       DM7820_MINOR_INT_REG_NONE,
					       .minor_enable = 0x0000,
					       .minor_status = 0x0000,
					       },

		/*
		 * Programmable clock 3 interrupt
		 */

		[DM7820_INTERRUPT_PRGCLK_3] = {
					       .int_enable_bit = 0x2000,
					       .int_status_bit = 0x2000,
					       .minor_reg =
					       DM7820_MINOR_INT_REG_NONE,
					       .minor_enable = 0x0000,
					       .minor_status = 0x0000,
					       },

		/*
		 * Pulse width modulator 0 interrupt
		 */

		[DM7820_INTERRUPT_PWM_0] = {
					    .int_enable_bit = 0x0100,
					    .int_status_bit = 0x0100,
					    .minor_reg =
					    DM7820_MINOR_INT_REG_NONE,
					    .minor_enable = 0x0000,
					    .minor_status = 0x0000,
					    },

		/*
		 * Pulse width modulator 1 interrupt
		 */

		[DM7820_INTERRUPT_PWM_1] = {
					    .int_enable_bit = 0x0200,
					    .int_status_bit = 0x0200,
					    .minor_reg =
					    DM7820_MINOR_INT_REG_NONE,
					    .minor_enable = 0x0000,
					    .minor_status = 0x0000,
					    },

		/*
		 * 8254 timer/counter A0 interrupt
		 */

		[DM7820_INTERRUPT_TMRCTR_A_0] = {
						 .int_enable_bit = 0x0004,
						 .int_status_bit = 0x0004,
						 .minor_reg =
						 DM7820_MINOR_INT_REG_TMRCTR_INT,
						 .minor_enable = 0x0001,
						 .minor_status = 0x0100,
						 },

		/*
		 * 8254 timer/counter A1 interrupt
		 */

		[DM7820_INTERRUPT_TMRCTR_A_1] = {
						 .int_enable_bit = 0x0004,
						 .int_status_bit = 0x0004,
						 .minor_reg =
						 DM7820_MINOR_INT_REG_TMRCTR_INT,
						 .minor_enable = 0x0002,
						 .minor_status = 0x0200,
						 },

		/*
		 * 8254 timer/counter A2 interrupt
		 */

		[DM7820_INTERRUPT_TMRCTR_A_2] = {
						 .int_enable_bit = 0x0004,
						 .int_status_bit = 0x0004,
						 .minor_reg =
						 DM7820_MINOR_INT_REG_TMRCTR_INT,
						 .minor_enable = 0x0004,
						 .minor_status = 0x0400,
						 },

		/*
		 * 8254 timer/counter B0 interrupt
		 */

		[DM7820_INTERRUPT_TMRCTR_B_0] = {
						 .int_enable_bit = 0x0004,
						 .int_status_bit = 0x0004,
						 .minor_reg =
						 DM7820_MINOR_INT_REG_TMRCTR_INT,
						 .minor_enable = 0x0008,
						 .minor_status = 0x0800,
						 },

		/*
		 * 8254 timer/counter B1 interrupt
		 */

		[DM7820_INTERRUPT_TMRCTR_B_1] = {
						 .int_enable_bit = 0x0004,
						 .int_status_bit = 0x0004,
						 .minor_reg =
						 DM7820_MINOR_INT_REG_TMRCTR_INT,
						 .minor_enable = 0x0010,
						 .minor_status = 0x1000,
						 },

		/*
		 * 8254 timer/counter B2 interrupt
		 */

		[DM7820_INTERRUPT_TMRCTR_B_2] = {
						 .int_enable_bit = 0x0004,
						 .int_status_bit = 0x0004,
						 .minor_reg =
						 DM7820_MINOR_INT_REG_TMRCTR_INT,
						 .minor_enable = 0x0020,
						 .minor_status = 0x2000,
						 }
	};

/**
 * @brief
 *      Table of information providing layout of all minor interrupt registers
 */

	dm7820_minor_int_reg_layout_t dm7820_minor_int_reg_layout[] = {

    /**
     * FIFO 0 Interrupt Register
     */

		[DM7820_MINOR_INT_REG_FIFO_0_INT] = {
						     .offset =
						     DM7820_BAR2_FIFO0_INT,
						     .enable_mask = 0x00FF,
						     .reserved_mask = 0x0000,
						     .status_mask = 0xFF00},

    /**
     * FIFO 1 Interrupt Register
     */

		[DM7820_MINOR_INT_REG_FIFO_1_INT] = {
						     .offset =
						     DM7820_BAR2_FIFO1_INT,
						     .enable_mask = 0x00FF,
						     .reserved_mask = 0x0000,
						     .status_mask = 0xFF00},

    /**
     * Incremental Encoder 0 Interrupt Register
     */

		[DM7820_MINOR_INT_REG_INCENC_0_INT] = {
						       .offset =
						       DM7820_BAR2_INCENC0_INT,
						       .enable_mask = 0x000F,
						       .reserved_mask = 0xF0F0,
						       .status_mask = 0x0F00},

    /**
     * Incremental Encoder 1 Interrupt Register
     */

		[DM7820_MINOR_INT_REG_INCENC_1_INT] = {
						       .offset =
						       DM7820_BAR2_INCENC1_INT,
						       .enable_mask = 0x000F,
						       .reserved_mask = 0xF0F0,
						       .status_mask = 0x0F00},

    /**
     * 8254 Timer/Counter Interrupt Register
     */

		[DM7820_MINOR_INT_REG_TMRCTR_INT] = {
						     .offset =
						     DM7820_BAR2_TC_INT,
						     .enable_mask = 0x003F,
						     .reserved_mask = 0xC0C0,
						     .status_mask = 0x3F00}
	};

/**
 * @} DM7820_Globals_Header
 */

#ifdef __cplusplus
}
#endif

#endif /* __dm7820_globals_h__ */
