/**
    @file

    @brief
        Type definitions used both in kernel and user space

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

    $Id: dm7820_types.h 86294 2015-03-04 21:36:57Z rgroner $
*/

#ifndef __dm7820_types_h__
#define __dm7820_types_h__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup DM7820_Types_Header DM7820 type definition header file
 * @{
 */

/*=============================================================================
Enumerations
 =============================================================================*/

/**
 * @defgroup DM7820_Types_Enumerations DM7820 type enumerations
 * @{
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PCI region access enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_PCI_Enumerations DM7820 type PCI enumerations
 * @{
 */

/**
 * @brief
 *      Standard PCI region number
 */

	enum dm7820_pci_region_num {

    /**
     * Memory-mapped PLX registers (BAR0)
     */

		DM7820_PCI_REGION_PLX_MEM = 0,

    /**
     * I/O-mapped PLX registers (BAR1)
     */

		DM7820_PCI_REGION_PLX_IO,

    /**
     * Memory-mapped FPGA registers (BAR2)
     */

		DM7820_PCI_REGION_FPGA_MEM
	};

/**
 * @brief
 *      Standard PCI region number type
 */

	typedef enum dm7820_pci_region_num dm7820_pci_region_num_t;

/**
 * @brief
 *      Desired size in bits of access to standard PCI region
 */

	enum dm7820_pci_region_access_size {

    /**
     * 8-bit access
     */

		DM7820_PCI_REGION_ACCESS_8 = 0,

    /**
     * 16-bit access
     */

		DM7820_PCI_REGION_ACCESS_16,

    /**
     * 32-bit access
     */

		DM7820_PCI_REGION_ACCESS_32
	};

/**
 * @brief
 *      Standard PCI region access size type
 */

	typedef enum dm7820_pci_region_access_size
	 dm7820_pci_region_access_size_t;

/**
 * @} DM7820_Types_PCI_Enumerations
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Standard I/O block enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_StdIO_Enumerations DM7820 type standard I/O enumerations
 * @{
 */

/**
 * @brief
 *      Standard I/O ports
 */

	enum _DM7820_StdIO_Port {

    /**
     * Port 0
     */

		DM7820_STDIO_PORT_0 = 0,

    /**
     * Port 1
     */

		DM7820_STDIO_PORT_1,

    /**
     * Port 2
     */

		DM7820_STDIO_PORT_2
	};

/**
 * @brief
 *      Standard I/O port type
 */

	typedef enum _DM7820_StdIO_Port DM7820_StdIO_Port;

/**
 * @brief
 *      Standard I/O port modes
 */

	enum _DM7820_StdIO_IO_Mode {

    /**
     * Input
     */

		DM7820_STDIO_MODE_INPUT = 0,

    /**
     * Output
     */

		DM7820_STDIO_MODE_OUTPUT,

    /**
     * Peripheral output
     */

		DM7820_STDIO_MODE_PER_OUT
	};

/**
 * @brief
 *      Standard I/O port mode type
 */

	typedef enum _DM7820_StdIO_IO_Mode DM7820_StdIO_IO_Mode;

/**
 * @brief
 *      Standard I/O port peripheral output modes
 */

	enum _DM7820_StdIO_Periph_Mode {

    /**
     * Pulse Width Modulator (PWM) mode; valid for port 2 only
     */

		DM7820_STDIO_PERIPH_PWM = 0x0,

    /**
     * Clock/other mode; valid for port 2 only
     */

		DM7820_STDIO_PERIPH_CLK_OTHER,

    /**
     * FIFO 0 mode; valid for all ports
     */

		DM7820_STDIO_PERIPH_FIFO_0,

    /**
     * FIFO 1 mode; valid for all ports
     */

		DM7820_STDIO_PERIPH_FIFO_1
	};

/**
 * @brief
 *      Standard I/O port peripheral output mode type
 */

	typedef enum _DM7820_StdIO_Periph_Mode DM7820_StdIO_Periph_Mode;

/**
 * @brief
 *      Strobe signals
 */

	enum _DM7820_StdIO_Strobe {

    /**
     * Strobe signal 1
     */

		DM7820_STDIO_STROBE_1 = 0,

    /**
     * Strobe signal 2
     */

		DM7820_STDIO_STROBE_2
	};

/**
 * @brief
 *      Standard I/O port strobe signal type
 */

	typedef enum _DM7820_StdIO_Strobe DM7820_StdIO_Strobe;

/**
 * @} DM7820_Types_StdIO_Enumerations
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
8254 timer/counter enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_TmrCtr_Enumerations DM7820 type timer/counter enumerations
 * @{
 */

/**
 * @brief
 *      8254 timers/counters
 */

	enum _dm7820_tmrctr_timer {

    /**
     * Timer 0 on first 8254 chip
     */

		DM7820_TMRCTR_TIMER_A_0 = 0,

    /**
     * Timer 1 on first 8254 chip
     */

		DM7820_TMRCTR_TIMER_A_1,

    /**
     * Timer 2 on first 8254 chip
     */

		DM7820_TMRCTR_TIMER_A_2,

    /**
     * Timer 0 on second 8254 chip
     */

		DM7820_TMRCTR_TIMER_B_0,

    /**
     * Timer 1 on second 8254 chip
     */

		DM7820_TMRCTR_TIMER_B_1,

    /**
     * Timer 2 on second 8254 chip
     */

		DM7820_TMRCTR_TIMER_B_2
	};

/**
 * @brief
 *      8254 timer/counter type
 */

	typedef enum _dm7820_tmrctr_timer dm7820_tmrctr_timer;

/**
 * @brief
 *      8254 timer/counter clock selectors
 */

	enum _dm7820_tmrctr_clock {

    /**
     * 5 MHz clock
     */

		DM7820_TMRCTR_CLOCK_5_MHZ = 0,

    /**
     * Reserved; do not use
     */

		DM7820_TMRCTR_CLOCK_RESERVED,

    /**
     * 8254 timer/counter A0
     */

		DM7820_TMRCTR_CLOCK_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_TMRCTR_CLOCK_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_TMRCTR_CLOCK_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_TMRCTR_CLOCK_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_TMRCTR_CLOCK_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_TMRCTR_CLOCK_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_TMRCTR_CLOCK_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_TMRCTR_CLOCK_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_TMRCTR_CLOCK_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_TMRCTR_CLOCK_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_TMRCTR_CLOCK_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_TMRCTR_CLOCK_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_TMRCTR_CLOCK_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_TMRCTR_CLOCK_INV_STROBE_2
	};

/**
 * @brief
 *      8254 timer/counter clock selector type
 */

	typedef enum _dm7820_tmrctr_clock dm7820_tmrctr_clock;

/**
 * @brief
 *      8254 timer/counter gate selectors
 */

	enum _dm7820_tmrctr_gate {

    /**
     * Logic 0
     */

		DM7820_TMRCTR_GATE_LOGIC_0 = 0,

    /**
     * Logic 1
     */

		DM7820_TMRCTR_GATE_LOGIC_1,

    /**
     * 8254 timer/counter A0
     */

		DM7820_TMRCTR_GATE_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_TMRCTR_GATE_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_TMRCTR_GATE_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_TMRCTR_GATE_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_TMRCTR_GATE_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_TMRCTR_GATE_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_TMRCTR_GATE_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_TMRCTR_GATE_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_TMRCTR_GATE_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_TMRCTR_GATE_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_TMRCTR_GATE_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_TMRCTR_GATE_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_TMRCTR_GATE_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_TMRCTR_GATE_INV_STROBE_2,

    /**
     * Digital I/O port 2 bit 0
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_0,

    /**
     * Digital I/O port 2 bit 1
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_1,

    /**
     * Digital I/O port 2 bit 2
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_2,

    /**
     * Digital I/O port 2 bit 3
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_3,

    /**
     * Digital I/O port 2 bit 4
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_4,

    /**
     * Digital I/O port 2 bit 5
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_5,

    /**
     * Digital I/O port 2 bit 6
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_6,

    /**
     * Digital I/O port 2 bit 7
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_7,

    /**
     * Digital I/O port 2 bit 8
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_8,

    /**
     * Digital I/O port 2 bit 9
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_9,

    /**
     * Digital I/O port 2 bit 10
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_10,

    /**
     * Digital I/O port 2 bit 11
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_11,

    /**
     * Digital I/O port 2 bit 12
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_12,

    /**
     * Digital I/O port 2 bit 13
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_13,

    /**
     * Digital I/O port 2 bit 14
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_14,

    /**
     * Digital I/O port 2 bit 15
     */

		DM7820_TMRCTR_GATE_PORT_2_BIT_15
	};

/**
 * @brief
 *      8254 timer/counter gate selector type
 */

	typedef enum _dm7820_tmrctr_gate dm7820_tmrctr_gate;

/**
 * @brief
 *      8254 timer/counter waveform mode selectors
 */

	enum _dm7820_tmrctr_waveform {

    /**
     * Event counter
     */

		DM7820_TMRCTR_WAVEFORM_EVENT_CTR = 0,

    /**
     * Programmable one shot
     */

		DM7820_TMRCTR_WAVEFORM_PROG_ONE_SHOT,

    /**
     * Rate generator
     */

		DM7820_TMRCTR_WAVEFORM_RATE_GENERATOR,

    /**
     * Square wave generator
     */

		DM7820_TMRCTR_WAVEFORM_SQUARE_WAVE,

    /**
     * Software triggered strobe
     */

		DM7820_TMRCTR_WAVEFORM_SOFTWARE_STROBE,

    /**
     * Hardware triggered strobe
     */

		DM7820_TMRCTR_WAVEFORM_HARDWARE_STROBE
	};

/**
 * @brief
 *      8254 timer/counter waverform mode selector type
 */

	typedef enum _dm7820_tmrctr_waveform dm7820_tmrctr_waveform;

/**
 * @brief
 *      8254 timer/counter count mode selectors
 */

	enum _dm7820_tmrctr_count_mode {

    /**
     * 16-bit binary mode
     */

		DM7820_TMRCTR_COUNT_MODE_BINARY = 0,

    /**
     * Binary Coded Decimal (BCD) mode
     */

		DM7820_TMRCTR_COUNT_MODE_BCD
	};

/**
 * @brief
 *      8254 timer/counter count mode selector type
 */

	typedef enum _dm7820_tmrctr_count_mode dm7820_tmrctr_count_mode;

/**
 * @} DM7820_Types_TmrCtr_Enumerations
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Programmable clock enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_PrgClk_Enumerations DM7820 type programmable clock enumerations
 * @{
 */

/**
 * @brief
 *      Programmable clocks
 */

	enum _dm7820_prgclk_clock {

    /**
     * Programmable clock 0
     */

		DM7820_PRGCLK_CLOCK_0 = 0,

    /**
     * Programmable clock 1
     */

		DM7820_PRGCLK_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_PRGCLK_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_PRGCLK_CLOCK_3
	};

/**
 * @brief
 *      Programmable clock type
 */

	typedef enum _dm7820_prgclk_clock dm7820_prgclk_clock;

/**
 * @brief
 *      Programmable clock modes
 */

	enum _dm7820_prgclk_mode {

    /**
     * Disabled
     */

		DM7820_PRGCLK_MODE_DISABLED = 0,

    /**
     * Continuous mode
     */

		DM7820_PRGCLK_MODE_CONTINUOUS,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_MODE_RESERVED,

    /**
     * One shot mode
     */

		DM7820_PRGCLK_MODE_ONE_SHOT
	};

/**
 * @brief
 *      Programmable clock mode type
 */

	typedef enum _dm7820_prgclk_mode dm7820_prgclk_mode;

/**
 * @brief
 *      Programmable clock master clocks
 */

	enum _dm7820_prgclk_master_clock {

    /**
     * 25 MHz clock
     */

		DM7820_PRGCLK_MASTER_25_MHZ = 0,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_MASTER_SAMPLE_CLOCK,

    /**
     * 8254 timer/counter A0
     */

		DM7820_PRGCLK_MASTER_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_PRGCLK_MASTER_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_PRGCLK_MASTER_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_PRGCLK_MASTER_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_PRGCLK_MASTER_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_PRGCLK_MASTER_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_PRGCLK_MASTER_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_PRGCLK_MASTER_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_PRGCLK_MASTER_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_PRGCLK_MASTER_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_PRGCLK_MASTER_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_PRGCLK_MASTER_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_PRGCLK_MASTER_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_PRGCLK_MASTER_INV_STROBE_2
	};

/**
 * @brief
 *      Programmable clock master clock type
 */

	typedef enum _dm7820_prgclk_master_clock dm7820_prgclk_master_clock;

/**
 * @brief
 *      Programmable clock start triggers
 */

	enum _dm7820_prgclk_start_trigger {

    /**
     * Start the clock immediately
     */

		DM7820_PRGCLK_START_IMMEDIATE = 0,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_START_RESERVED_1,

    /**
     * 8254 timer/counter A0
     */

		DM7820_PRGCLK_START_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_PRGCLK_START_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_PRGCLK_START_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_PRGCLK_START_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_PRGCLK_START_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_PRGCLK_START_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_PRGCLK_START_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_PRGCLK_START_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_PRGCLK_START_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_PRGCLK_START_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_PRGCLK_START_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_PRGCLK_START_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_PRGCLK_START_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_PRGCLK_START_INV_STROBE_2,

    /**
     * Advanced interrupt 0
     */

		DM7820_PRGCLK_START_ADVANCED_INT_0,

    /**
     * Advanced interrupt 1
     */

		DM7820_PRGCLK_START_ADVANCED_INT_1,

    /**
     * 8254 timer/counter interrupt
     */

		DM7820_PRGCLK_START_8254_INT,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_START_RESERVED_2,

    /**
     * Incremental Encoder 0 interrupt
     */

		DM7820_PRGCLK_START_INC_ENCODER_0_INT,

    /**
     * Incremental Encoder 1 interrupt
     */

		DM7820_PRGCLK_START_INC_ENCODER_1_INT,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_START_RESERVED_3,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_START_RESERVED_4,

    /**
     * Pulse Width Modulator 0 interrupt
     */

		DM7820_PRGCLK_START_PWM_0_INT,

    /**
     * Pulse Width Modulator 1 interrupt
     */

		DM7820_PRGCLK_START_PWM_1_INT,

    /**
     * Programmable clock 0 interrupt
     */

		DM7820_PRGCLK_START_PROG_CLOCK_0_INT,

    /**
     * Programmable clock 1 interrupt
     */

		DM7820_PRGCLK_START_PROG_CLOCK_1_INT,

    /**
     * Programmable clock 2 interrupt
     */

		DM7820_PRGCLK_START_PROG_CLOCK_2_INT,

    /**
     * Programmable clock 3 interrupt
     */

		DM7820_PRGCLK_START_PROG_CLOCK_3_INT,

    /**
     * FIFO 0 interrupt
     */

		DM7820_PRGCLK_START_FIFO_0_INT,

    /**
     * FIFO 1 interrupt
     */

		DM7820_PRGCLK_START_FIFO_1_INT
	};

/**
 * @brief
 *      Programmable clock start trigger type
 */

	typedef enum _dm7820_prgclk_start_trigger dm7820_prgclk_start_trigger;

/**
 * @brief
 *      Programmable clock stop triggers
 */

	enum _dm7820_prgclk_stop_trigger {

    /**
     * Do not stop the clock
     */

		DM7820_PRGCLK_STOP_NONE = 0,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_STOP_RESERVED_1,

    /**
     * 8254 timer/counter A0
     */

		DM7820_PRGCLK_STOP_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_PRGCLK_STOP_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_PRGCLK_STOP_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_PRGCLK_STOP_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_PRGCLK_STOP_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_PRGCLK_STOP_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_PRGCLK_STOP_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_PRGCLK_STOP_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_PRGCLK_STOP_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_PRGCLK_STOP_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_PRGCLK_STOP_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_PRGCLK_STOP_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_PRGCLK_STOP_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_PRGCLK_STOP_INV_STROBE_2,

    /**
     * Advanced interrupt 0
     */

		DM7820_PRGCLK_STOP_ADVANCED_INT_0,

    /**
     * Advanced interrupt 1
     */

		DM7820_PRGCLK_STOP_ADVANCED_INT_1,

    /**
     * 8254 timer/counter interrupt
     */

		DM7820_PRGCLK_STOP_8254_INT,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_STOP_RESERVED_2,

    /**
     * Incremental Encoder 0 interrupt
     */

		DM7820_PRGCLK_STOP_INC_ENCODER_0_INT,

    /**
     * Incremental Encoder 1 interrupt
     */

		DM7820_PRGCLK_STOP_INC_ENCODER_1_INT,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_STOP_RESERVED_3,

    /**
     * Reserved; do not use
     */

		DM7820_PRGCLK_STOP_RESERVED_4,

    /**
     * Pulse Width Modulator 0 interrupt
     */

		DM7820_PRGCLK_STOP_PWM_0_INT,

    /**
     * Pulse Width Modulator 1 interrupt
     */

		DM7820_PRGCLK_STOP_PWM_1_INT,

    /**
     * Programmable clock 0 interrupt
     */

		DM7820_PRGCLK_STOP_PROG_CLOCK_0_INT,

    /**
     * Programmable clock 1 interrupt
     */

		DM7820_PRGCLK_STOP_PROG_CLOCK_1_INT,

    /**
     * Programmable clock 2 interrupt
     */

		DM7820_PRGCLK_STOP_PROG_CLOCK_2_INT,

    /**
     * Programmable clock 3 interrupt
     */

		DM7820_PRGCLK_STOP_PROG_CLOCK_3_INT,

    /**
     * FIFO 0 interrupt
     */

		DM7820_PRGCLK_STOP_FIFO_0_INT,

    /**
     * FIFO 1 interrupt
     */

		DM7820_PRGCLK_STOP_FIFO_1_INT
	};

/**
 * @brief
 *      Programmable clock stop trigger type
 */

	typedef enum _dm7820_prgclk_stop_trigger dm7820_prgclk_stop_trigger;

/**
 * @} DM7820_Types_PrgClk_Enumerations
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Pulse width modulator enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_PWM_Enumerations DM7820 type pulse width modulator enumerations
 * @{
 */

/**
 * @brief
 *      Pulse width modulators
 */

	enum _dm7820_pwm_modulator {

		/*
		 * Pulse width modulator 0
		 */

		DM7820_PWM_MODULATOR_0 = 0,

		/*
		 * Pulse width modulator 1
		 */

		DM7820_PWM_MODULATOR_1
	};

/**
 * @brief
 *      Pulse width modulator type
 */

	typedef enum _dm7820_pwm_modulator dm7820_pwm_modulator;

/**
 * @brief
 *      Pulse width modulator period master clocks
 */

	enum _dm7820_pwm_period_master_clock {

    /**
     * 25 MHz clock
     */

		DM7820_PWM_PERIOD_MASTER_25_MHZ = 0,

    /**
     * Reserved; do not use
     */

		DM7820_PWM_PERIOD_MASTER_RESERVED,

    /**
     * 8254 timer/counter A0
     */

		DM7820_PWM_PERIOD_MASTER_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_PWM_PERIOD_MASTER_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_PWM_PERIOD_MASTER_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_PWM_PERIOD_MASTER_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_PWM_PERIOD_MASTER_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_PWM_PERIOD_MASTER_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_PWM_PERIOD_MASTER_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_PWM_PERIOD_MASTER_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_PWM_PERIOD_MASTER_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_PWM_PERIOD_MASTER_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_PWM_PERIOD_MASTER_INV_STROBE_2
	};

/**
 * @brief
 *      Pulse width modulator period master clock type
 */

	typedef enum _dm7820_pwm_period_master_clock
	 dm7820_pwm_period_master_clock;

/**
 * @brief
 *      Pulse width modulator outputs
 */

	enum _dm7820_pwm_output {

    /**
     * PWM output A
     */

		DM7820_PWM_OUTPUT_A = 0,

    /**
     * PWM output B
     */

		DM7820_PWM_OUTPUT_B,

    /**
     * PWM output C
     */

		DM7820_PWM_OUTPUT_C,

    /**
     * PWM output D
     */

		DM7820_PWM_OUTPUT_D
	};

/**
 * @brief
 *      Pulse width modulator output type
 */

	typedef enum _dm7820_pwm_output dm7820_pwm_output;

/**
 * @brief
 *      Pulse width modulator width master clocks
 */

	enum _dm7820_pwm_width_master_clock {

    /**
     * 25 MHz clock
     */

		DM7820_PWM_WIDTH_MASTER_25_MHZ = 0,

    /**
     * Reserved; do not use
     */

		DM7820_PWM_WIDTH_MASTER_RESERVED,

    /**
     * 8254 timer/counter A0
     */

		DM7820_PWM_WIDTH_MASTER_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_PWM_WIDTH_MASTER_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_PWM_WIDTH_MASTER_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_PWM_WIDTH_MASTER_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_PWM_WIDTH_MASTER_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_PWM_WIDTH_MASTER_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_PWM_WIDTH_MASTER_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_PWM_WIDTH_MASTER_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_PWM_WIDTH_MASTER_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_PWM_WIDTH_MASTER_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_PWM_WIDTH_MASTER_INV_STROBE_2
	};

/**
 * @brief
 *      Pulse width modulator width master clock type
 */

	typedef enum _dm7820_pwm_width_master_clock
	 dm7820_pwm_width_master_clock;

/**
 * @} DM7820_Types_PWM_Enumerations
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Incremental encoder enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_IncEnc_Enumerations DM7820 type incremental encoder enumerations
 * @{
 */

/**
 * @brief
 *      Incremental encoders
 */

	enum _dm7820_incenc_encoder {

    /**
     * Incremental encoder 0
     */

		DM7820_INCENC_ENCODER_0 = 0,

    /**
     * Incremental encoder 1
     */

		DM7820_INCENC_ENCODER_1
	};

/**
 * @brief
 *      Incremental encoder type
 */

	typedef enum _dm7820_incenc_encoder dm7820_incenc_encoder;

/**
 * @brief
 *      Incremental encoder master clocks
 */

	enum _dm7820_incenc_master_clock {

    /**
     * 25 MHz clock
     */

		DM7820_INCENC_MASTER_25_MHZ = 0,

    /**
     * Reserved; do not use
     */

		DM7820_INCENC_MASTER_RESERVED,

    /**
     * 8254 timer/counter A0
     */

		DM7820_INCENC_MASTER_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_INCENC_MASTER_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_INCENC_MASTER_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_INCENC_MASTER_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_INCENC_MASTER_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_INCENC_MASTER_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_INCENC_MASTER_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_INCENC_MASTER_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_INCENC_MASTER_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_INCENC_MASTER_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_INCENC_MASTER_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_INCENC_MASTER_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_INCENC_MASTER_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_INCENC_MASTER_INV_STROBE_2
	};

/**
 * @brief
 *      Incremental encoder master clock type
 */

	typedef enum _dm7820_incenc_master_clock dm7820_incenc_master_clock;

/**
 * @brief
 *      Incremental encoder input modes
 */

	enum _dm7820_incenc_input_mode {

    /**
     * Single ended
     */

		DM7820_INCENC_INPUT_SINGLE_ENDED = 0,

    /**
     * Pseudo differential
     */

		DM7820_INCENC_INPUT_DIFFERENTIAL
	};

/**
 * @brief
 *      Incremental encoder input mode type
 */

	typedef enum _dm7820_incenc_input_mode dm7820_incenc_input_mode;

/**
 * @brief
 *      Incremental encoder channel modes
 */

	enum _dm7820_incenc_channel_mode {

    /**
     * Independent 16-bit channels
     */

		DM7820_INCENC_CHANNEL_INDEPENDENT = 0,

    /**
     * Channels joined into single 32-bit channel
     */

		DM7820_INCENC_CHANNEL_JOINED
	};

/**
 * @brief
 *      Incremental encoder channel mode type
 */

	typedef enum _dm7820_incenc_channel_mode dm7820_incenc_channel_mode;

/**
 * @brief
 *      Incremental encoder phase filter transitions
 */

	enum _dm7820_incenc_phase_transition {

    /**
     * Inputs B/A transition from 0/0 to 0/1 when counting up
     */

		DM7820_INCENC_PHASE_BA_00_TO_01_UP = 0,

    /**
     * Inputs B/A transition from 0/1 to 1/1 when counting up
     */

		DM7820_INCENC_PHASE_BA_01_TO_11_UP,

    /**
     * Inputs B/A transition from 1/1 to 1/0 when counting up
     */

		DM7820_INCENC_PHASE_BA_11_TO_10_UP,

    /**
     * Inputs B/A transition from 1/0 to 0/0 when counting up
     */

		DM7820_INCENC_PHASE_BA_10_TO_00_UP,

    /**
     * Inputs B/A transition from 0/1 to 0/0 when counting down
     */

		DM7820_INCENC_PHASE_BA_01_TO_00_DOWN,

    /**
     * Inputs B/A transition from 1/1 to 0/1 when counting down
     */

		DM7820_INCENC_PHASE_BA_11_TO_01_DOWN,

    /**
     * Inputs B/A transition from 1/0 to 1/1 when counting down
     */

		DM7820_INCENC_PHASE_BA_10_TO_11_DOWN,

    /**
     * Inputs B/A transition from 0/0 to 1/0 when counting down
     */

		DM7820_INCENC_PHASE_BA_00_TO_10_DOWN
	};

/**
 * @brief
 *      Incremental encoder phase filter transition type
 */

	typedef enum _dm7820_incenc_phase_transition
	 dm7820_incenc_phase_transition;

/**
 * @brief
 *      Incremental encoder channels
 */

	enum _dm7820_incenc_channel {

    /**
     * Channel A
     */

		DM7820_INCENC_CHANNEL_A = 0,

    /**
     * Channel B
     */

		DM7820_INCENC_CHANNEL_B
	};

/**
 * @brief
 *      Incremental encoder channel type
 */

	typedef enum _dm7820_incenc_channel dm7820_incenc_channel;

/**
 * @brief
 *      Incremental encoder status conditions
 */

	enum _dm7820_incenc_status_condition {

    /**
     * Channel A positive rollover
     */

		DM7820_INCENC_STATUS_CHANNEL_A_POSITIVE_ROLLOVER = 0,

    /**
     * Channel A negative rollover
     */

		DM7820_INCENC_STATUS_CHANNEL_A_NEGATIVE_ROLLOVER,

    /**
     * Channel B positive rollover
     */

		DM7820_INCENC_STATUS_CHANNEL_B_POSITIVE_ROLLOVER,

    /**
     * Channel B negative rollover
     */

		DM7820_INCENC_STATUS_CHANNEL_B_NEGATIVE_ROLLOVER
	};

/**
 * @brief
 *      Incremental encoder status condition type
 */

	typedef enum _dm7820_incenc_status_condition
	 dm7820_incenc_status_condition;

/**
 * @} DM7820_Types_IncEnc_Enumerations
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FIFO enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_FIFO_Enumerations DM7820 type FIFO enumerations
 * @{
 */

/**
 * @brief
 *      FIFOs
 */

	enum _dm7820_fifo_queue {

    /**
     * FIFO 0
     */

		DM7820_FIFO_QUEUE_0 = 0,

    /**
     * FIFO 1
     */

		DM7820_FIFO_QUEUE_1
	};

/**
 * @brief
 *      FIFO type
 */

	typedef enum _dm7820_fifo_queue dm7820_fifo_queue;

/**
 * @brief
 *      FIFO input clocks
 */

	enum _dm7820_fifo_input_clock {

    /**
     * 25 MHz clock
     */

		DM7820_FIFO_INPUT_CLOCK_25_MHZ = 0,

    /**
     * Reserved; do not use
     */

		DM7820_FIFO_INPUT_CLOCK_RESERVED_1,

    /**
     * 8254 timer/counter A0
     */

		DM7820_FIFO_INPUT_CLOCK_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_FIFO_INPUT_CLOCK_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_FIFO_INPUT_CLOCK_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_FIFO_INPUT_CLOCK_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_FIFO_INPUT_CLOCK_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_FIFO_INPUT_CLOCK_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_FIFO_INPUT_CLOCK_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_FIFO_INPUT_CLOCK_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_FIFO_INPUT_CLOCK_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_FIFO_INPUT_CLOCK_INV_STROBE_2,

    /**
     * Advanced interrupt 0
     */

		DM7820_FIFO_INPUT_CLOCK_ADVANCED_INT_0,

    /**
     * Advanced interrupt 1
     */

		DM7820_FIFO_INPUT_CLOCK_ADVANCED_INT_1,

    /**
     * 8254 timer/counter interrupt
     */

		DM7820_FIFO_INPUT_CLOCK_8254_INT,

    /**
     * Reserved; do not use
     */

		DM7820_FIFO_INPUT_CLOCK_RESERVED_2,

    /**
     * Incremental encoder 0 interrupt
     */

		DM7280_FIFO_INPUT_CLOCK_INC_ENCODER_0_INT,

    /**
     * Incremental encoder 1 interrupt
     */

		DM7280_FIFO_INPUT_CLOCK_INC_ENCODER_1_INT,

    /**
     * Reserved; do not use
     */

		DM7820_FIFO_INPUT_CLOCK_RESERVED_3,

    /**
     * Reserved; do not use
     */

		DM7820_FIFO_INPUT_CLOCK_RESERVED_4,

    /**
     * PWM 0 interrupt
     */

		DM7820_FIFO_INPUT_CLOCK_PWM_0_INT,

    /**
     * PWM 1 interrupt
     */

		DM7820_FIFO_INPUT_CLOCK_PWM_1_INT,

    /**
     * Programmable clock 0 interrupt
     */

		DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_0_INT,

    /**
     * Programmable clock 1 interrupt
     */

		DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_1_INT,

    /**
     * Programmable clock 2 interrupt
     */

		DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_2_INT,

    /**
     * Programmable clock 3 interrupt
     */

		DM7820_FIFO_INPUT_CLOCK_PROG_CLOCK_3_INT,

    /**
     * PCI read from FIFO
     */

		DM7820_FIFO_INPUT_CLOCK_PCI_READ,

    /**
     * PCI write to FIFO
     */

		DM7820_FIFO_INPUT_CLOCK_PCI_WRITE
	};

/**
 * @brief
 *      FIFO input clock type
 */

	typedef enum _dm7820_fifo_input_clock dm7820_fifo_input_clock;

/**
 * @brief
 *      FIFO output clocks
 */

	enum _dm7820_fifo_output_clock {

    /**
     * 25 MHz clock
     */

		DM7820_FIFO_OUTPUT_CLOCK_25_MHZ = 0,

    /**
     * Reserved; do not use
     */

		DM7820_FIFO_OUTPUT_CLOCK_RESERVED_1,

    /**
     * 8254 timer/counter A0
     */

		DM7820_FIFO_OUTPUT_CLOCK_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_FIFO_OUTPUT_CLOCK_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_FIFO_OUTPUT_CLOCK_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_FIFO_OUTPUT_CLOCK_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_FIFO_OUTPUT_CLOCK_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_FIFO_OUTPUT_CLOCK_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_FIFO_OUTPUT_CLOCK_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_FIFO_OUTPUT_CLOCK_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_FIFO_OUTPUT_CLOCK_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_FIFO_OUTPUT_CLOCK_INV_STROBE_2,

    /**
     * Advanced interrupt 0
     */

		DM7820_FIFO_OUTPUT_CLOCK_ADVANCED_INT_0,

    /**
     * Advanced interrupt 1
     */

		DM7820_FIFO_OUTPUT_CLOCK_ADVANCED_INT_1,

    /**
     * 8254 timer/counter interrupt
     */

		DM7820_FIFO_OUTPUT_CLOCK_8254_INT,

    /**
     * Reserved; do not use
     */

		DM7820_FIFO_OUTPUT_CLOCK_RESERVED_2,

    /**
     * Incremental encoder 0 interrupt
     */

		DM7280_FIFO_OUTPUT_CLOCK_INC_ENCODER_0_INT,

    /**
     * Incremental encoder 1 interrupt
     */

		DM7280_FIFO_OUTPUT_CLOCK_INC_ENCODER_1_INT,

    /**
     * Reserved; do not use
     */

		DM7820_FIFO_OUTPUT_CLOCK_RESERVED_3,

    /**
     * Reserved; do not use
     */

		DM7820_FIFO_OUTPUT_CLOCK_RESERVED_4,

    /**
     * PWM 0 interrupt
     */

		DM7820_FIFO_OUTPUT_CLOCK_PWM_0_INT,

    /**
     * PWM 1 interrupt
     */

		DM7820_FIFO_OUTPUT_CLOCK_PWM_1_INT,

    /**
     * Programmable clock 0 interrupt
     */

		DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_0_INT,

    /**
     * Programmable clock 1 interrupt
     */

		DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_1_INT,

    /**
     * Programmable clock 2 interrupt
     */

		DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_2_INT,

    /**
     * Programmable clock 3 interrupt
     */

		DM7820_FIFO_OUTPUT_CLOCK_PROG_CLOCK_3_INT,

    /**
     * PCI read from FIFO
     */

		DM7820_FIFO_OUTPUT_CLOCK_PCI_READ,

    /**
     * PCI write to FIFO
     */

		DM7820_FIFO_OUTPUT_CLOCK_PCI_WRITE
	};

/**
 * @brief
 *      FIFO output clock type
 */

	typedef enum _dm7820_fifo_output_clock dm7820_fifo_output_clock;

/**
 * @brief
 *      FIFO DMA request sources
 */

	enum _dm7820_fifo_dma_request {

    /**
     * Read request
     */

		DM7820_FIFO_DMA_REQUEST_READ = 0,

    /**
     * Not empty
     */

		DM7820_FIFO_DMA_REQUEST_NOT_EMPTY,

    /**
     * Write request
     */

		DM7820_FIFO_DMA_REQUEST_WRITE,

    /**
     * Not full
     */

		DM7820_FIFO_DMA_REQUEST_NOT_FULL
	};

/**
 * @brief
 *      FIFO DMA request source type
 */

	typedef enum _dm7820_fifo_dma_request dm7820_fifo_dma_request;

/**
 * @brief
 *      FIFO data inputs
 */

	enum _dm7820_fifo_data_input {

    /**
     * FIFO 0 data input is PCI data
     */

		DM7820_FIFO_0_DATA_INPUT_PCI_DATA = 0,

    /**
     * FIFO 0 data input is digital I/O port 0
     */

		DM7820_FIFO_0_DATA_INPUT_PORT_0,

    /**
     * FIFO 0 data input is digital I/O port 2
     */

		DM7820_FIFO_0_DATA_INPUT_PORT_2,

    /**
     * FIFO 0 data input is FIFO 0 output
     */

		DM7820_FIFO_0_DATA_INPUT_FIFO_0_OUTPUT,

    /**
     * FIFO 1 data input is PCI data
     */

		DM7820_FIFO_1_DATA_INPUT_PCI_DATA,

    /**
     * FIFO 1 data input is digital I/O port 1
     */

		DM7820_FIFO_1_DATA_INPUT_PORT_1,

    /**
     * FIFO 1 data input is incremental encoder 1 channel A counter value
     */

		DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_A,

    /**
     * FIFO 1 data input is incremental encoder 1 channel B counter value
     */

		DM7820_FIFO_1_DATA_INPUT_INC_ENCODER_1_B
	};

/**
 * @brief
 *      FIFO data input type
 */

	typedef enum _dm7820_fifo_data_input dm7820_fifo_data_input;

/**
 * @brief
 *      FIFO status conditions
 */

	enum _dm7820_fifo_status_condition {

    /**
     * FIFO read request
     */

		DM7820_FIFO_STATUS_READ_REQUEST = 0,

    /**
     * FIFO read request
     */

		DM7820_FIFO_STATUS_WRITE_REQUEST,

    /**
     * FIFO full
     */

		DM7820_FIFO_STATUS_FULL,

    /**
     * FIFO empty
     */

		DM7820_FIFO_STATUS_EMPTY,

    /**
     * FIFO overflow
     */

		DM7820_FIFO_STATUS_OVERFLOW,

    /**
     * FIFO underflow
     */

		DM7820_FIFO_STATUS_UNDERFLOW
	};

/**
 * @brief
 *      FIFO status condition type
 */

	typedef enum _dm7820_fifo_status_condition dm7820_fifo_status_condition;

/**
 * @} DM7820_Types_FIFO_Enumerations
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Advanced interrupt enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_AdvInt_Enumerations DM7820 type advanced interrupt enumerations
 * @{
 */

/**
 * @brief
 *      Advanced interrupts
 */

	enum _dm7820_advint_interrupt {

    /**
     * Advanced interrupt 0
     */

		DM7820_ADVINT_INTERRUPT_0 = 0,

    /**
     * Advanced interrupt 1
     */

		DM7820_ADVINT_INTERRUPT_1
	};

/**
 * @brief
 *      Advanced interrupt type
 */

	typedef enum _dm7820_advint_interrupt dm7820_advint_interrupt;

/**
 * @brief
 *      Advanced interrupt modes
 */

	enum _dm7820_advint_mode {

    /**
     * Disabled
     */

		DM7820_ADVINT_MODE_DISABLED = 0,

    /**
     * Strobe mode
     */

		DM7820_ADVINT_MODE_STROBE,

    /**
     * Match mode
     */

		DM7820_ADVINT_MODE_MATCH,

    /**
     * Event mode
     */

		DM7820_ADVINT_MODE_EVENT
	};

/**
 * @brief
 *      Advanced interrupt mode type
 */

	typedef enum _dm7820_advint_mode dm7820_advint_mode;

/**
 * @brief
 *      Advanced interrupt master clocks
 */

	enum _dm7820_advint_master_clock {

    /**
     * 25 MHz clock
     */

		DM7820_ADVINT_MASTER_25_MHZ = 0,

    /**
     * Reserved; do not use
     */

		DM7820_ADVINT_MASTER_RESERVED,

    /**
     * 8254 timer/counter A0
     */

		DM7820_ADVINT_MASTER_8254_A_0,

    /**
     * 8254 timer/counter A1
     */

		DM7820_ADVINT_MASTER_8254_A_1,

    /**
     * 8254 timer/counter A2
     */

		DM7820_ADVINT_MASTER_8254_A_2,

    /**
     * 8254 timer/counter B0
     */

		DM7820_ADVINT_MASTER_8254_B_0,

    /**
     * 8254 timer/counter B1
     */

		DM7820_ADVINT_MASTER_8254_B_1,

    /**
     * 8254 timer/counter B2
     */

		DM7820_ADVINT_MASTER_8254_B_2,

    /**
     * Programmable clock 0
     */

		DM7820_ADVINT_MASTER_PROG_CLOCK_0,

    /**
     * Programmable clock 1
     */

		DM7820_ADVINT_MASTER_PROG_CLOCK_1,

    /**
     * Programmable clock 2
     */

		DM7820_ADVINT_MASTER_PROG_CLOCK_2,

    /**
     * Programmable clock 3
     */

		DM7820_ADVINT_MASTER_PROG_CLOCK_3,

    /**
     * Strobe signal 1
     */

		DM7820_ADVINT_MASTER_STROBE_1,

    /**
     * Strobe signal 2
     */

		DM7820_ADVINT_MASTER_STROBE_2,

    /**
     * Inverted strobe signal 1
     */

		DM7820_ADVINT_MASTER_INV_STROBE_1,

    /**
     * Inverted strobe signal 2
     */

		DM7820_ADVINT_MASTER_INV_STROBE_2
	};

/**
 * @brief
 *      Advanced interrupt master clock type
 */

	typedef enum _dm7820_advint_master_clock dm7820_advint_master_clock;

/**
 * @} DM7820_Types_AdvInt_Enumerations
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Interrupt enumerations
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Types_Interrupt_Enumerations DM7820 type interrupt enumerations
 * @{
 */

/**
 * @brief
 *      Interrupt sources
 */

	enum _dm7820_interrupt_source {

    /**
     * Advanced interrupt block 0 interrupt
     */

		DM7820_INTERRUPT_ADVINT_0 = 0,

    /**
     * Advanced interrupt block 1 interrupt (1)
     */

		DM7820_INTERRUPT_ADVINT_1,

    /**
     * FIFO block FIFO 0 empty interrupt (2)
     */

		DM7820_INTERRUPT_FIFO_0_EMPTY,

    /**
     * FIFO block FIFO 0 full interrupt (3)
     */

		DM7820_INTERRUPT_FIFO_0_FULL,

    /**
     * FIFO block FIFO 0 overflow interrupt (4)
     */

		DM7820_INTERRUPT_FIFO_0_OVERFLOW,

    /**
     * FIFO block FIFO 0 read request interrupt (5)
     */

		DM7820_INTERRUPT_FIFO_0_READ_REQUEST,

    /**
     * FIFO block FIFO 0 underflow interrupt (6)
     */

		DM7820_INTERRUPT_FIFO_0_UNDERFLOW,

    /**
     * FIFO block FIFO 0 write request interrupt (7)
     */

		DM7820_INTERRUPT_FIFO_0_WRITE_REQUEST,

    /**
     * FIFO block FIFO 1 empty interrupt (8)
     */

		DM7820_INTERRUPT_FIFO_1_EMPTY,

    /**
     * FIFO block FIFO 1 full interrupt (9)
     */

		DM7820_INTERRUPT_FIFO_1_FULL,

    /**
     * FIFO block FIFO 1 overflow interrupt (10)
     */

		DM7820_INTERRUPT_FIFO_1_OVERFLOW,

    /**
     * FIFO block FIFO 1 read request interrupt (11)
     */

		DM7820_INTERRUPT_FIFO_1_READ_REQUEST,

    /**
     * FIFO block FIFO 1 underflow interrupt (12)
     */

		DM7820_INTERRUPT_FIFO_1_UNDERFLOW,

    /**
     * FIFO block FIFO 1 write request interrupt (13)
     */

		DM7820_INTERRUPT_FIFO_1_WRITE_REQUEST,

    /**
     * Incremental encoder block 0 channel A negative rollover interrupt (14)
     */

		DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER,

    /**
     * Incremental encoder block 0 channel A positive rollover interrupt (15)
     */

		DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER,

    /**
     * Incremental encoder block 0 channel B negative rollover interrupt (16)
     */

		DM7820_INTERRUPT_INCENC_0_CHANNEL_B_NEGATIVE_ROLLOVER,

    /**
     * Incremental encoder block 0 channel B positive rollover interrupt (17)
     */

		DM7820_INTERRUPT_INCENC_0_CHANNEL_B_POSITIVE_ROLLOVER,

    /**
     * Incremental encoder block 1 channel A negative rollover interrupt (18)
     */

		DM7820_INTERRUPT_INCENC_1_CHANNEL_A_NEGATIVE_ROLLOVER,

    /**
     * Incremental encoder block 1 channel A positive rollover interrupt (19)
     */

		DM7820_INTERRUPT_INCENC_1_CHANNEL_A_POSITIVE_ROLLOVER,

    /**
     * Incremental encoder block 1 channel B negative rollover interrupt (20)
     */

		DM7820_INTERRUPT_INCENC_1_CHANNEL_B_NEGATIVE_ROLLOVER,

    /**
     * Incremental encoder block 1 channel B positive rollover interrupt (21)
     */

		DM7820_INTERRUPT_INCENC_1_CHANNEL_B_POSITIVE_ROLLOVER,

    /**
     *  Programmable clock block 0 interrupt (22)
     */

		DM7820_INTERRUPT_PRGCLK_0,

    /**
     *  Programmable clock block 1 interrupt (23)
     */

		DM7820_INTERRUPT_PRGCLK_1,

    /**
     *  Programmable clock block 2 interrupt (24)
     */

		DM7820_INTERRUPT_PRGCLK_2,

    /**
     *  Programmable clock block 3 interrupt (25)
     */

		DM7820_INTERRUPT_PRGCLK_3,

    /**
     * Pulse width modulator block 0 interrupt (26)
     */

		DM7820_INTERRUPT_PWM_0,

    /**
     * Pulse width modulator block 1 interrupt (27)
     */

		DM7820_INTERRUPT_PWM_1,

    /**
     * 8254 timer/counter A0 interrupt (28)
     */

		DM7820_INTERRUPT_TMRCTR_A_0,

    /**
     * 8254 timer/counter A1 interrupt (29)
     */

		DM7820_INTERRUPT_TMRCTR_A_1,

    /**
     * 8254 timer/counter A2 interrupt (30)
     */

		DM7820_INTERRUPT_TMRCTR_A_2,

    /**
     * 8254 timer/counter B0 interrupt (31)
     */

		DM7820_INTERRUPT_TMRCTR_B_0,

    /**
     * 8254 timer/counter B1 interrupt (32)
     */

		DM7820_INTERRUPT_TMRCTR_B_1,

    /**
     * 8254 timer/counter B2 interrupt (33)
     */

		DM7820_INTERRUPT_TMRCTR_B_2,

    /**
     * FIFO block FIFO 0 DMA done interrupt.  Applications cannot control this
     * interrupt but they can get its status. (34)
     */

		DM7820_INTERRUPT_FIFO_0_DMA_DONE,

    /**
     * FIFO block FIFO 1 DMA done interrupt.  Applications cannot control this
     * interrupt but they can get its status. (35)
     */

		DM7820_INTERRUPT_FIFO_1_DMA_DONE,

    /**
     * Value which indicates no interrupt source.  User level ignores this.  The
     * kernel uses this in the interrupt handler.  This must be the last entry. (36)
     */

		DM7820_INTERRUPT_NONE
	};

/**
 * @brief
 *      Interrupt source type
 */

	typedef enum _dm7820_interrupt_source dm7820_interrupt_source;

/**
 * @brief
 *      Minor interrupt control/status registers
 */

	enum _dm7820_minor_interrupt_register {

    /**
     * FIFO 0 Interrupt Register
     */

		DM7820_MINOR_INT_REG_FIFO_0_INT = 0,

    /**
     * FIFO 1 Interrupt Register
     */

		DM7820_MINOR_INT_REG_FIFO_1_INT,

    /**
     * Incremental Encoder 0 Interrupt Register
     */

		DM7820_MINOR_INT_REG_INCENC_0_INT,

    /**
     * Incremental Encoder 1 Interrupt Register
     */

		DM7820_MINOR_INT_REG_INCENC_1_INT,

    /**
     * 8254 Timer/Counter Interrupt Register
     */

		DM7820_MINOR_INT_REG_TMRCTR_INT,

    /**
     * Value which indicates no minor interrupt register.  This must be the last
     * entry.
     */

		DM7820_MINOR_INT_REG_NONE
	};

/**
 * @brief
 *      Minor interrupt control/status register type
 */

	typedef enum _dm7820_minor_interrupt_register
	 dm7820_minor_interrupt_register;

/**
 * @brief
 *      Interrupt source information
 */

	struct _dm7820_interrupt_info {
    /**
     * The interrupt sources for the last acknowledged interrupt
     */
		dm7820_interrupt_source source;
    /**
     * The number of logged interrupt in the driver that still need attention
     */
		int int_remaining;
    /**
     * The number of interrupt missed due to a full log in the driver.
     */
		int int_missed;
    /**
     * Error Code: 0 = success, -1 = failure
     */
		int error;
	};

/**
 * @brief
 *      Interrupt source information type
 */

	typedef struct _dm7820_interrupt_info dm7820_interrupt_info;

/**
 * @} DM7820_Types_Interrupt_Enumerations
 */

/**
 * @} DM7820_Types_Enumerations
 */

/*=============================================================================
Structures
 =============================================================================*/

/**
 * @defgroup DM7820_Types_Structures DM7820 type definition structures
 * @{
 */

/**
 * @brief
 *      PCI region access request descriptor.  This structure holds information
 *      about a request to read data from or write data to one of a device's
 *      PCI regions.
 */

	struct dm7820_pci_access_request {

    /**
     * Size of access in bits
     */

		dm7820_pci_region_access_size_t size;

    /**
     * The PCI region to access
     */

		dm7820_pci_region_num_t region;

    /**
     * Offset within region to access
     */

		uint16_t offset;

    /**
     * Data to write or the data read
     */

		union {

    /**
     * 8-bit value
     */

			uint8_t data8;

    /**
     * 16-bit value
     */

			uint16_t data16;

    /**
     * 32-bit value
     */

			uint32_t data32;
		} data;
	};

/**
 * PCI region access request descriptor type
 */

	typedef struct dm7820_pci_access_request dm7820_pci_access_request_t;

/**
 * @brief
 *      Structure containing information needed to acknowledge, disable, and
 *      enable a particular interrupt source
 */

	struct dm7820_interrupt_control {

    /**
     * Interrupt Enable Register mask that indicates which single bit controls
     * the interrupt
     */

		uint16_t int_enable_bit;

    /**
     * Interrupt Status Register mask that indicates which single bit gives the
     * status of the interrupt
     */

		uint16_t int_status_bit;

    /**
     * Minor interrupt register
     */

		dm7820_minor_interrupt_register minor_reg;

    /**
     * Minor interrupt enable register mask that indicates which single bit in
     * the register controls the interrupt
     */

		uint16_t minor_enable;

    /**
     * Minor interrupt enable register mask that indicates which single bit in
     * the register gives the status of the interrupt
     */

		uint16_t minor_status;
	};

/**
 * @brief
 *      Interrupt control information type
 */

	typedef struct dm7820_interrupt_control dm7820_interrupt_control_t;

/**
 * @brief
 *      Minor interrupt register bit layout
 */

	struct dm7820_minor_int_reg_layout {

    /**
     * BAR2 register offset
     */

		uint16_t offset;

    /**
     * Bit mask that indicates which register bits are interrupt enable bits.  A
     * zero in a bit position means the corresponding register bit does not
     * control an interrupt.  A one in a bit position means the corresponding
     * register bit controls an interrupt.
     */

		uint16_t enable_mask;

    /**
     * Bit mask that indicates which register bits are reserved.  A zero in a
     * bit position means the corresponding register bit is not reserved.  A one
     * in a bit position means the corresponding register bit is reserved.
     */

		uint16_t reserved_mask;

    /**
     * Bit mask that indicates which register bits are interrupt status bits.  A
     * zero in a bit position means the corresponding register bit does not
     * indicate interrupt status.  A one in a bit position means the
     * corresponding register bit indicates interrupt status.
     */

		uint16_t status_mask;
	};

/**
 * @brief
 *      Minor interrupt register bit layout type
 */

	typedef struct dm7820_minor_int_reg_layout
	 dm7820_minor_int_reg_layout_t;

/**
 * @} DM7820_Types_Structures
 */

/*=============================================================================
Type definitions
 =============================================================================*/

/**
 * @defgroup DM7820_Types_Typedefs DM7820 type definition typedefs
 * @{
 */

/**
 * @brief
 *      Interrupt source status type
 */

	typedef uint64_t dm7820_int_source_status_t;

/**
 * @} DM7820_Types_Typedefs
 */

/**
 * @} DM7820_Types_Header
 */

#ifdef __cplusplus
}
#endif
#endif				/* __dm7820_types_h__ */
