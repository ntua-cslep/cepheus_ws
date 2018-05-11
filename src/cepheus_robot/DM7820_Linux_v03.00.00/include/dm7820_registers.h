/**
    @file

    @brief
        Register definitions for DM7820 devices

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

    $Id: dm7820_registers.h 86275 2015-03-04 15:53:23Z rgroner $
*/

#ifndef __dm7820_registers_h__
#define __dm7820_registers_h__

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup DM7820_Register_Header DM7820 register header file
 * @{
 */

/*=============================================================================
BAR0 register offsets (memory-mapped PLX registers).  Each register is defined
as an offset from the first address in BAR0.
 =============================================================================*/

/**
 * @defgroup DM7820_Register_BAR0_Offsets DM7820 register BAR0 (memory-mapped PLX registers) offsets
 * @{
 */

/**
 * @brief
 *      PLX interrupt control/status
 */

#define DM7820_BAR0_INTCSR          0x68

/**
 * @brief
 *      PLX DMA channel 0 mode
 */

#define DM7820_BAR0_DMAMODE0            0x80

/**
 * @brief
 *      PLX DMA channel 0 PCI address
 */

#define DM7820_BAR0_DMAPADR0            0x84

/**
 * @brief
 *      PLX DMA channel 0 local address
 */

#define DM7820_BAR0_DMALADR0            0x88

/**
 * @brief
 *      PLX DMA channel 0 transfer size
 */

#define DM7820_BAR0_DMASIZ0         0x8C

/**
 * @brief
 *      PLX DMA channel 0 descriptor pointer
 */

#define DM7820_BAR0_DMADPR0         0x90

/**
 * @brief
 *      PLX DMA channel 1 mode
 */

#define DM7820_BAR0_DMAMODE1            0x94

/**
 * @brief
 *      PLX DMA channel 1 PCI address
 */

#define DM7820_BAR0_DMAPADR1            0x98

/**
 * @brief
 *      PLX DMA channel 1 local address
 */

#define DM7820_BAR0_DMALADR1            0x9C

/**
 * @brief
 *      PLX DMA channel 1 transfer size
 */

#define DM7820_BAR0_DMASIZ1         0xA0

/**
 * @brief
 *      PLX DMA channel 1 descriptor pointer
 */

#define DM7820_BAR0_DMADPR1         0xA4

/**
 * @brief
 *      PLX DMA channel 0 command/status
 */

#define DM7820_BAR0_DMACSR0         0xA8

/**
 * @brief
 *      PLX DMA channel 1 command/status
 */

#define DM7820_BAR0_DMACSR1         0xA9

/**
 * @brief
 *      PLX DMA arbitration
 */

#define DM7820_BAR0_DMAARB          0xAC

/**
 * @brief
 *      PLX DMA threshold
 */

#define DM7820_BAR0_DMATHR          0xB0

/**
 * @brief
 *      PLX DMA channel 0 PCI dual address cycles upper address
 */

#define DM7820_BAR0_DMADA0          0xB4

/**
 * @brief
 *      PLX DMA channel 1 PCI dual address cycles upper address
 */

#define DM7820_BAR0_DMADA1          0xB8

/**
 * @} DM7820_Register_BAR0_Offsets
 */

/*=============================================================================
BAR1 register offsets (I/O-mapped PLX registers).  Each register is defined as
an offset from the first port in BAR1.
 =============================================================================*/

/**
 * @defgroup DM7820_Register_BAR1_Offsets DM7820 register BAR1 (I/O-mapped PLX registers) offsets
 * @{
 */

/**
 * @brief
 *      PLX interrupt control/status
 */

#define DM7820_BAR1_INTCSR          0x68

/**
 * @brief
 *      PLX DMA channel 0 mode
 */

#define DM7820_BAR1_DMAMODE0            0x80

/**
 * @brief
 *      PLX DMA channel 0 PCI address
 */

#define DM7820_BAR1_DMAPADR0            0x84

/**
 * @brief
 *      PLX DMA channel 0 local address
 */

#define DM7820_BAR1_DMALADR0            0x88

/**
 * @brief
 *      PLX DMA channel 0 transfer size
 */

#define DM7820_BAR1_DMASIZ0         0x8C

/**
 * @brief
 *      PLX DMA channel 0 descriptor pointer
 */

#define DM7820_BAR1_DMADPR0         0x90

/**
 * @brief
 *      PLX DMA channel 1 mode
 */

#define DM7820_BAR1_DMAMODE1            0x94

/**
 * @brief
 *      PLX DMA channel 1 PCI address
 */

#define DM7820_BAR1_DMAPADR1            0x98

/**
 * @brief
 *      PLX DMA channel 1 local address
 */

#define DM7820_BAR1_DMALADR1            0x9C

/**
 * @brief
 *      PLX DMA channel 1 transfer size
 */

#define DM7820_BAR1_DMASIZ1         0xA0

/**
 * @brief
 *      PLX DMA channel 1 descriptor pointer
 */

#define DM7820_BAR1_DMADPR1         0xA4

/**
 * @brief
 *      PLX DMA channel 0 command/status
 */

#define DM7820_BAR1_DMACSR0         0xA8

/**
 * @brief
 *      PLX DMA channel 1 command/status
 */

#define DM7820_BAR1_DMACSR1         0xA9

/**
 * @brief
 *      PLX DMA arbitration
 */

#define DM7820_BAR1_DMAARB          0xAC

/**
 * @brief
 *      PLX DMA threshold
 */

#define DM7820_BAR1_DMATHR          0xB0

/**
 * @brief
 *      PLX DMA channel 0 PCI dual address cycles upper address
 */

#define DM7820_BAR1_DMADA0          0xB4

/**
 * @brief
 *      PLX DMA channel 1 PCI dual address cycles upper address
 */

#define DM7820_BAR1_DMADA1          0xB8

/**
 * @} DM7820_Register_BAR1_Offsets
 */

/*=============================================================================
BAR2 register offsets (memory-mapped FPGA registers).  Each register is defined
as an offset from the first address in BAR2.
 =============================================================================*/

/**
 * @defgroup DM7820_Register_BAR2_Offsets DM7820 register BAR2 (memory-mapped FPGA registers) offsets
 * @{
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Board control register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_General DM7820 register board control offsets
 * @{
 */

/**
 * @brief
 *      FPGA version and type identifiers
 */

#define DM7820_BAR2_FPGA_VERSION    0x0000

/**
 * @brief
 *      FPGA source code revision control version identifier
 */

#define DM7820_BAR2_SVN_VERSION     0x0002

/**
 * @brief
 *      Board reset
 */

#define DM7820_BAR2_BOARD_RESET     0x0004

/**
 * @brief
 *      Board status
 */

#define DM7820_BAR2_BRD_STAT        0x0008

/**
 * @brief
 *      Local interrupt enable
 */

#define DM7820_BAR2_INTERRUPT_ENABLE    0x0010

/**
 * @brief
 *      Local interrupt status
 */

#define DM7820_BAR2_INTERRUPT_STATUS    0x0012

/**
 * @} DM7820_Register_General
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Standard I/O register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_StdIO DM7820 register standard I/O offsets
 * @{
 */

/**
 * @brief
 *      Port 0 output value
 */

#define DM7820_BAR2_PORT0_OUTPUT    0x0040

/**
 * @brief
 *      Port 0 input value
 */

#define DM7820_BAR2_PORT0_INPUT     0x0042

/**
 * @brief
 *      Port 0 direction
 */

#define DM7820_BAR2_PORT0_TRISTATE  0x0044

/**
 * @brief
 *      Port 0 operating mode
 */

#define DM7820_BAR2_PORT0_MODE      0x0046

/**
 * @brief
 *      Port 1 output value
 */

#define DM7820_BAR2_PORT1_OUTPUT    0x0048

/**
 * @brief
 *      Port 1 input value
 */

#define DM7820_BAR2_PORT1_INPUT     0x004A

/**
 * @brief
 *      Port 1 direction
 */

#define DM7820_BAR2_PORT1_TRISTATE  0x004C

/**
 * @brief
 *      Port 1 operating mode
 */

#define DM7820_BAR2_PORT1_MODE      0x004E

/**
 * @brief
 *      Port 2 output value
 */

#define DM7820_BAR2_PORT2_OUTPUT    0x0050

/**
 * @brief
 *      Port 2 input value
 */

#define DM7820_BAR2_PORT2_INPUT     0x0052

/**
 * @brief
 *      Port 2 direction
 */

#define DM7820_BAR2_PORT2_TRISTATE  0x0054

/**
 * @brief
 *      Port 2 operating mode
 */

#define DM7820_BAR2_PORT2_MODE      0x0056

/**
 * @brief
 *      Port 2 strobe signal status
 */

#define DM7820_BAR2_STROBE_STATUS   0x0058

/**
 * @brief
 *      Port 0 bits 0-7 peripheral select
 */

#define DM7820_BAR2_PORT0_PERIPH_SEL_L  0x0060

/**
 * @brief
 *      Port 0 bits 8-15 peripheral select
 */

#define DM7820_BAR2_PORT0_PERIPH_SEL_H  0x0062

/**
 * @brief
 *      Port 1 bits 0-7 peripheral select
 */

#define DM7820_BAR2_PORT1_PERIPH_SEL_L  0x0064

/**
 * @brief
 *      Port 1 bits 8-15 peripheral select
 */

#define DM7820_BAR2_PORT1_PERIPH_SEL_H  0x0066

/**
 * @brief
 *      Port 2 bits 0-7 peripheral select
 */

#define DM7820_BAR2_PORT2_PERIPH_SEL_L  0x0068

/**
 * @brief
 *      Port 2 bits 8-15 peripheral select
 */

#define DM7820_BAR2_PORT2_PERIPH_SEL_H  0x006A

/**
 * @} DM7820_Register_StdIO
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
PLX Registers
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

    /**
     * Local Bus Data Width Mask
     */

#define PLX9056_DMA_WIDTH_MASK   0x00000003

     /**
     * Local Bus Data Width 8 Bits.
     */

#define PLX9056_DMA_WIDTH_8  0x00000000

     /**
     * Local Bus Data Width 16 Bits.
     */

#define PLX9056_DMA_WIDTH_16 0x00000001

     /**
     * Local Bus Data Width 32 Bits.
     */

#define PLX9056_DMA_WIDTH_32 0x00000002

     /**
     * Internal wait state counter mask.  This is a 4 bit
     * value that is 0 by default.
     */

#define PLX9056_DMA_WAITSTATES_MASK 0x0000003c

     /**
     * TA#/READY# Input enable (use as mask and control)
     */

#define PLX9056_DMA_READY 0x00000040

     /**
     * Continuous burst enable (use as mask and control)
     */

#define PLX9056_DMA_BURST 0x00000080

    /**
     * Lecal burst enable (use as mask and control)
     */

#define PLX9056_DMA_LOCAL_BURST 0x00000100

     /**
     * Scatter/Gather mode enabled (use as mask and control)
     * When this bit is enabled() the PLX will get address
     * and length of data from a table in the PCI memory.
     */

#define PLX9056_DMA_SCATTERGATHER 0x00000200

     /**
     * DMA done interrupt enable (use as mask and control)
     * At the time the DMA cycle is completed an interrupt will
     * occur on the PCI bus if the master interrupt is enabled
     * IF PLX9056_DMA_CLEAR_COUNT is enabled then the interrupt
     * will not occur until the byte count is cleared in the PCI
     * memory table (scatter/gather mode is enabled in this case).
     */

#define PLX9056_DMA_DONE_INTERRUPT 0x00000400

     /**
     * DMA local addressing mode (use as mask and control)
     * Enabling this bit holds the local address bus pointer
     * constant, disabling causes it to autoincrement.
     */

#define PLX9056_DMA_LOCAL_ADDRESSING_MODE 0x00000800

     /**
     * DMA Demand mode (use as mask and control)
     * Enabling this mode causes the PLX to transfer data when
     * the DREQ# input is asserted.
     */

#define PLX9056_DMA_DEMAND_MODE 0x00001000

     /**
     * Memory write and invalidate mode.
     */

#define PLX9056_DMA_MEMWRITE_INV 0x00002000

    /**
     * EOT# Enable
     */

#define PLX9056_DMA_EOT_ENABLE 0x00004000

     /**
     * Fast/Slow Terminate mode selected
     */

#define PLX9056_DMA_FAST_SLOW_TERM 0x00008000

     /**
     * Clear count mode (use as mask and control)
     * When enabled this mdoe will clear our the length
     * field of the scatter/gather descriptor table entry
     * when the DMA is completed.  Interrupts are delayed
     * until this clearing operation is comlpeted.
     */

#define PLX9056_DMA_CLEAR_COUNT 0x00010000

     /**
     * Interrupt Select
     * when set, the DMA channel's interrupt is routed
     * to the PCI buss
     */

#define PLX9056_DMA_INTERRUPT_SEL 0x00020000

     /**
     * DAC Chain load (not supported most chipsets)
     */

#define PLX9056_DMA_DAC_CHAIN 0x00040000

     /**
     * EOT# End Link
     */

#define PLX9056_DMA_EOT_END 0x00080000

     /**
     * Ring Management Valid Mode Enabled
     * When set the Ring Management Valid bit in DMASIZx[31] register
     * bit controls the processing of DMA descriptors.
     * If the valid bit is set, the transfer count is 0 and the
     * descriptor is not the last descriptor in the chain then the DMA
     * controller will move to the next descriptor in the chain
     */

#define PLX9056_DMA_RING_MODE 0x00100000

     /**
     * Ring Management Void Stop Control
     * Valid when PLX9056_DMA_RING_MODE is set
     * When this bit is 0 the Scatter/Gather controller
     * will load the descriptor and execute the DMA.
     * If this bit is 1 then the Scatter/Gather controller will stop
     * polling when the valid bit goes 0.  The host will need
     * to restart the DMA controller by setting the DMACSRx[1] to 1
     */

#define PLX9056_DMA_RING_CONTROL 0x00200000

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
8254 timer/counter register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_TmrCtr DM7820 register 8254 timer/counter offsets
 * @{
 */

/**
 * @brief
 *      8254 timer/counter block identifier
 */

#define DM7820_BAR2_TC_ID       0x0080

/**
 * @brief
 *      8254 timer/counter interrupt control/status
 */

#define DM7820_BAR2_TC_INT      0x0082

/**
 * @brief
 *      8254 timer/counter A0 control
 */

#define DM7820_BAR2_TC_A0_CONTROL   0x0084

/**
 * @brief
 *      8254 timer/counter A1 control
 */

#define DM7820_BAR2_TC_A1_CONTROL   0x0086

/**
 * @brief
 *      8254 timer/counter A2 control
 */

#define DM7820_BAR2_TC_A2_CONTROL   0x0088

/**
 * @brief
 *      8254 timer/counter B0 control
 */

#define DM7820_BAR2_TC_B0_CONTROL   0x008A

/**
 * @brief
 *      8254 timer/counter B1 control
 */

#define DM7820_BAR2_TC_B1_CONTROL   0x008C

/**
 * @brief
 *      8254 timer/counter B2 control
 */

#define DM7820_BAR2_TC_B2_CONTROL   0x008E

/**
 * @} DM7820_Register_TmrCtr
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FIFO channel 0 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_FIFO_0 DM7820 register FIFO 0 offsets
 * @{
 */

/**
 * @brief
 *      FIFO 0 block identifier
 */

#define DM7820_BAR2_FIFO0_ID        0x00C0

/**
 * @brief
 *      FIFO 0 interrupt control/status
 */

#define DM7820_BAR2_FIFO0_INT       0x00C2

/**
 * @brief
 *      FIFO 0 input clock
 */

#define DM7820_BAR2_FIFO0_IN_CLK    0x00C4

/**
 * @brief
 *      FIFO 0 output clock
 */

#define DM7820_BAR2_FIFO0_OUT_CLK   0x00C6

/**
 * @brief
 *      FIFO 0 data input and PLX DMA request source
 */

#define DM7820_BAR2_FIFO0_IN_DATA_DREQ  0x00C8

/**
 * @brief
 *      FIFO 0 control/status
 */

#define DM7820_BAR2_FIFO0_CON_STAT  0x00CA

/**
 * @brief
 *      FIFO 0 PCI read/write port
 */

#define DM7820_BAR2_FIFO0_RW_PORT   0x00CC

/**
 * @} DM7820_Register_FIFO_0
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FIFO channel 1 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_FIFO_1 DM7820 register FIFO 1 offsets
 * @{
 */

/**
 * @brief
 *      FIFO 1 block identifier
 */

#define DM7820_BAR2_FIFO1_ID        0x00D0

/**
 * @brief
 *      FIFO 1 interrupt control/status
 */

#define DM7820_BAR2_FIFO1_INT       0x00D2

/**
 * @brief
 *      FIFO 1 input clock
 */

#define DM7820_BAR2_FIFO1_IN_CLK    0x00D4

/**
 * @brief
 *      FIFO 1 output clock
 */

#define DM7820_BAR2_FIFO1_OUT_CLK   0x00D6

/**
 * @brief
 *      FIFO 1 data input and PLX DMA request source
 */

#define DM7820_BAR2_FIFO1_IN_DATA_DREQ  0x00D8

/**
 * @brief
 *      FIFO 1 control/status
 */

#define DM7820_BAR2_FIFO1_CON_STAT  0x00DA

/**
 * @brief
 *      FIFO 1 PCI read/write port
 */

#define DM7820_BAR2_FIFO1_RW_PORT   0x00DC

/**
 * @} DM7820_Register_FIFO_1
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Programmable clock 0 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_PrgClk_0 DM7820 register programmable clock 0 offsets
 * @{
 */

/**
 * @brief
 *      Programmable clock 0 block identifier
 */

#define DM7820_BAR2_PRGCLK0_ID      0x0100

/**
 * @brief
 *      Programmable clock 0 operating mode
 */

#define DM7820_BAR2_PRGCLK0_MODE    0x0102

/**
 * @brief
 *      Programmable clock 0 master clock
 */

#define DM7820_BAR2_PRGCLK0_CLK     0x0104

/**
 * @brief
 *      Programmable clock 0 start/stop triggers
 */

#define DM7820_BAR2_PRGCLK0_START_STOP  0x0106

/**
 * @brief
 *      Programmable clock 0 period
 */

#define DM7820_BAR2_PRGCLK0_PERIOD  0x0108

/**
 * @} DM7820_Register_PrgClk_0
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Programmable clock 1 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_PrgClk_1 DM7820 register programmable clock 1 offsets
 * @{
 */

/**
 * @brief
 *      Programmable clock 1 block identifier
 */

#define DM7820_BAR2_PRGCLK1_ID      0x0140

/**
 * @brief
 *      Programmable clock 1 operating mode
 */

#define DM7820_BAR2_PRGCLK1_MODE    0x0142

/**
 * @brief
 *      Programmable clock 1 master clock
 */

#define DM7820_BAR2_PRGCLK1_CLK     0x0144

/**
 * @brief
 *      Programmable clock 1 start/stop triggers
 */

#define DM7820_BAR2_PRGCLK1_START_STOP  0x0146

/**
 * @brief
 *      Programmable clock 1 period
 */

#define DM7820_BAR2_PRGCLK1_PERIOD  0x0148

/**
 * @} DM7820_Register_PrgClk_1
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Programmable clock 2 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_PrgClk_2 DM7820 register programmable clock 2 offsets
 * @{
 */

/**
 * @brief
 *      Programmable clock 2 block identifier
 */

#define DM7820_BAR2_PRGCLK2_ID      0x0180

/**
 * @brief
 *      Programmable clock 2 operating mode
 */

#define DM7820_BAR2_PRGCLK2_MODE    0x0182

/**
 * @brief
 *      Programmable clock 2 master clock
 */

#define DM7820_BAR2_PRGCLK2_CLK     0x0184

/**
 * @brief
 *      Programmable clock 2 start/stop triggers
 */

#define DM7820_BAR2_PRGCLK2_START_STOP  0x0186

/**
 * @brief
 *      Programmable clock 2 period
 */

#define DM7820_BAR2_PRGCLK2_PERIOD  0x0188

/**
 * @} DM7820_Register_PrgClk_2
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Programmable clock 3 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_PrgClk_3 DM7820 register programmable clock 3 offsets
 * @{
 */

/**
 * @brief
 *      Programmable clock 3 block identifier
 */

#define DM7820_BAR2_PRGCLK3_ID      0x01C0

/**
 * @brief
 *      Programmable clock 3 operating mode
 */

#define DM7820_BAR2_PRGCLK3_MODE    0x01C2

/**
 * @brief
 *      Programmable clock 3 master clock
 */

#define DM7820_BAR2_PRGCLK3_CLK     0x01C4

/**
 * @brief
 *      Programmable clock 3 start/stop triggers
 */

#define DM7820_BAR2_PRGCLK3_START_STOP  0x01C6

/**
 * @brief
 *      Programmable clock 3 period
 */

#define DM7820_BAR2_PRGCLK3_PERIOD  0x01C8

/**
 * @} DM7820_Register_PrgClk_3
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Advanced interrupt 0 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_AdvInt_0 DM7820 register advanced interrupt 0 offsets
 * @{
 */

/**
 * @brief
 *      Advanced interrupt 0 block identifier
 */

#define DM7820_BAR2_ADVINT0_ID      0x0200

/**
 * @brief
 *      Advanced interrupt 0 mode
 */

#define DM7820_BAR2_ADVINT0_INT_MODE    0x0202

/**
 * @brief
 *      Advanced interrupt 0 master clock
 */

#define DM7820_BAR2_ADVINT0_CLK     0x0204

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 0 mask
 */

#define DM7820_BAR2_ADVINT0_PORT0_MASK  0x0208

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 1 mask
 */

#define DM7820_BAR2_ADVINT0_PORT1_MASK  0x020A

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 2 mask
 */

#define DM7820_BAR2_ADVINT0_PORT2_MASK  0x020C

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 0 event mode compare
 */

#define DM7820_BAR2_ADVINT0_PORT0_CMP   0x0210

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 1 event mode compare
 */

#define DM7820_BAR2_ADVINT0_PORT1_CMP   0x0212

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 2 event mode compare
 */

#define DM7820_BAR2_ADVINT0_PORT2_CMP   0x0214

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 0 value capture
 */

#define DM7820_BAR2_ADVINT0_PORT0_CAPT  0x0218

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 1 value capture
 */

#define DM7820_BAR2_ADVINT0_PORT1_CAPT  0x021A

/**
 * @brief
 *      Advanced interrupt 0 standard I/O port 2 value capture
 */

#define DM7820_BAR2_ADVINT0_PORT2_CAPT  0x021C

/**
 * @} DM7820_Register_AdvInt_0
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Advanced interrupt 1 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_AdvInt_1 DM7820 register advanced interrupt 1 offsets
 * @{
 */

/**
 * @brief
 *      Advanced interrupt 1 block identifier
 */

#define DM7820_BAR2_ADVINT1_ID      0x0240

/**
 * @brief
 *      Advanced interrupt 1 mode
 */

#define DM7820_BAR2_ADVINT1_INT_MODE    0x0242

/**
 * @brief
 *      Advanced interrupt 1 master clock
 */

#define DM7820_BAR2_ADVINT1_CLK     0x0244

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 0 mask
 */

#define DM7820_BAR2_ADVINT1_PORT0_MASK  0x0248

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 1 mask
 */

#define DM7820_BAR2_ADVINT1_PORT1_MASK  0x024A

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 2 mask
 */

#define DM7820_BAR2_ADVINT1_PORT2_MASK  0x024C

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 0 event mode compare
 */

#define DM7820_BAR2_ADVINT1_PORT0_CMP   0x0250

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 1 event mode compare
 */

#define DM7820_BAR2_ADVINT1_PORT1_CMP   0x0252

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 2 event mode compare
 */

#define DM7820_BAR2_ADVINT1_PORT2_CMP   0x0254

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 0 value capture
 */

#define DM7820_BAR2_ADVINT1_PORT0_CAPT  0x0258

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 1 value capture
 */

#define DM7820_BAR2_ADVINT1_PORT1_CAPT  0x025A

/**
 * @brief
 *      Advanced interrupt 1 standard I/O port 2 value capture
 */

#define DM7820_BAR2_ADVINT1_PORT2_CAPT  0x025C

/**
 * @} DM7820_Register_AdvInt_1
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Dual incremental encoder 0 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_IncEnc_0 DM7820 register incremental encoder 0 offsets
 * @{
 */

/**
 * @brief
 *      Incremental encoder 0 block identifier
 */

#define DM7820_BAR2_INCENC0_ID      0x0280

/**
 * @brief
 *      Incremental encoder 0 interrupt control/status
 */

#define DM7820_BAR2_INCENC0_INT     0x0282

/**
 * @brief
 *      Incremental encoder 0 master clock
 */

#define DM7820_BAR2_INCENC0_CLOCK   0x0284

/**
 * @brief
 *      Incremental encoder 0 operating mode
 */

#define DM7820_BAR2_INCENC0_MODE    0x0286

/**
 * @brief
 *      Incremental encoder 0 channel A value
 */

#define DM7820_BAR2_INCENC0_VALUEA  0x0288

/**
 * @brief
 *      Incremental encoder 0 channel B value
 */

#define DM7820_BAR2_INCENC0_VALUEB  0x028A

/**
 * @} DM7820_Register_IncEnc_0
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Dual incremental encoder 1 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_IncEnc_1 DM7820 register incremental encoder 1 offsets
 * @{
 */

/**
 * @brief
 *      Incremental encoder 1 block identifier
 */

#define DM7820_BAR2_INCENC1_ID      0x02C0

/**
 * @brief
 *      Incremental encoder 1 interrupt control/status
 */

#define DM7820_BAR2_INCENC1_INT     0x02C2

/**
 * @brief
 *      Incremental encoder 1 master clock
 */

#define DM7820_BAR2_INCENC1_CLOCK   0x02C4

/**
 * @brief
 *      Incremental encoder 1 operating mode
 */

#define DM7820_BAR2_INCENC1_MODE    0x02C6

/**
 * @brief
 *      Incremental encoder 1 channel A value
 */

#define DM7820_BAR2_INCENC1_VALUEA  0x02C8

/**
 * @brief
 *      Incremental encoder 1 channel B value
 */

#define DM7820_BAR2_INCENC1_VALUEB  0x02CA

/**
 * @} DM7820_Register_IncEnc_1
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Pulse width modulator 0 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_PWM_0 DM7820 register pulse width modulator 0 offsets
 * @{
 */

/**
 * @brief
 *      Pulse width modulator 0 block identifier
 */

#define DM7820_BAR2_PWM0_ID     0x0300

/**
 * @brief
 *      Pulse width modulator 0 mode
 */

#define DM7820_BAR2_PWM0_MODE       0x0302

/**
 * @brief
 *      Pulse width modulator 0 period/width clocks
 */

#define DM7820_BAR2_PWM0_CLK        0x0304

/**
 * @brief
 *      Pulse width modulator 0 period
 */

#define DM7820_BAR2_PWM0_PERIOD     0x0308

/**
 * @brief
 *      Pulse width modulator 0 output A width
 */

#define DM7820_BAR2_PWM0_WIDTHA     0x0310

/**
 * @brief
 *      Pulse width modulator 0 output B width
 */

#define DM7820_BAR2_PWM0_WIDTHB     0x0314

/**
 * @brief
 *      Pulse width modulator 0 output C width
 */

#define DM7820_BAR2_PWM0_WIDTHC     0x0318

/**
 * @brief
 *      Pulse width modulator 0 output D width
 */

#define DM7820_BAR2_PWM0_WIDTHD     0x031C

/**
 * @} DM7820_Register_PWM_0
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Pulse width modulator 1 register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_PWM_1 DM7820 register pulse width modulator 1 offsets
 * @{
 */

/**
 * @brief
 *      Pulse width modulator 1 block identifier
 */

#define DM7820_BAR2_PWM1_ID     0x0340

/**
 * @brief
 *      Pulse width modulator 1 mode
 */

#define DM7820_BAR2_PWM1_MODE       0x0342

/**
 * @brief
 *      Pulse width modulator 1 period/width clocks
 */

#define DM7820_BAR2_PWM1_CLK        0x0344

/**
 * @brief
 *      Pulse width modulator 1 period
 */

#define DM7820_BAR2_PWM1_PERIOD     0x0348

/**
 * @brief
 *      Pulse width modulator 1 output A width
 */

#define DM7820_BAR2_PWM1_WIDTHA     0x0350

/**
 * @brief
 *      Pulse width modulator 1 output B width
 */

#define DM7820_BAR2_PWM1_WIDTHB     0x0354

/**
 * @brief
 *      Pulse width modulator 1 output C width
 */

#define DM7820_BAR2_PWM1_WIDTHC     0x0358

/**
 * @brief
 *      Pulse width modulator 1 output D width
 */

#define DM7820_BAR2_PWM1_WIDTHD     0x035C

/**
 * @} DM7820_Register_PWM_1
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
8254 timer/counter A register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_TmrCtr_A DM7820 register 8254 timer/counter A offsets
 * @{
 */

/**
 * @brief
 *      8254 timer/counter A timer 0 value
 */

#define DM7820_BAR2_TCA_COUNTER_0   0x1000

/**
 * @brief
 *      8254 timer/counter A timer 1 value
 */

#define DM7820_BAR2_TCA_COUNTER_1   0x1004

/**
 * @brief
 *      8254 timer/counter A timer 2 value
 */

#define DM7820_BAR2_TCA_COUNTER_2   0x1008

/**
 * @brief
 *      8254 timer/counter A control word
 */

#define DM7820_BAR2_TCA_CON_WORD    0x100C

/**
 * @} DM7820_Register_TmrCtr_A
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
8254 timer/counter B register offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_TmrCtr_B DM7820 register 8254 timer/counter B offsets
 * @{
 */

/**
 * @brief
 *      8254 timer/counter B timer 0 value
 */

#define DM7820_BAR2_TCB_COUNTER_0   0x1010

/**
 * @brief
 *      8254 timer/counter B timer 1 value
 */

#define DM7820_BAR2_TCB_COUNTER_1   0x1014

/**
 * @brief
 *      8254 timer/counter B timer 2 value
 */

#define DM7820_BAR2_TCB_COUNTER_2   0x1018

/**
 * @brief
 *      8254 timer/counter B control word
 */

#define DM7820_BAR2_TCB_CON_WORD    0x101C

/**
 * @} DM7820_Register_TmrCtr_B
 */

/**
 * @} DM7820_Register_BAR2_Offsets
 */

/*=============================================================================
Length in bytes of each PCI region
 =============================================================================*/

/**
 * @defgroup DM7820_Register_PCI_Region_Lengths DM7820 register PCI region lengths
 * @{
 */

/**
 * @brief
 *      Length in bytes of BAR0 (memory-mapped PLX registers)
 */

#define DM7820_BAR0_LENGTH      0x200

/**
 * @brief
 *      Length in bytes of BAR1 (I/O-mapped PLX registers)
 */

#define DM7820_BAR1_LENGTH      0x100

/**
 * @brief
 *      Length in bytes of BAR2 (memory-mapped FPGA registers)
 */

#define DM7820_BAR2_LENGTH      0x2000

/**
 * @} DM7820_Register_PCI_Region_Lengths
 */

/*=============================================================================
Functional block IDs
 =============================================================================*/

/**
 * @defgroup DM7820_Register_Block_IDs DM7820 register functional block identifiers
 * @{
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Identifier values
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_Block_ID_Values DM7820 register functional block identifier values
 * @{
 */

/**
 * @brief
 *      8254 timer/counter block identifier
 */

#define DM7820_ID_TIMER_COUNTER     0x1001

/**
 * @brief
 *      FIFO block identifier
 */

#define DM7820_ID_FIFO          0x2011

/**
 * @brief
 *      Programmable clock block identifier
 */

#define DM7820_ID_PROGRAMMABLE_CLOCK    0x1000

/**
 * @brief
 *      Advanced interrupt block identifier
 */

#define DM7820_ID_ADVANCED_INTERRUPT    0x0001

/**
 * @brief
 *      Incremental encoder block identifier
 */

#define DM7820_ID_INCREMENTAL_ENCODER   0x0002

/**
 * @brief
 *      Pulse width modulator block identifier
 */

#define DM7820_ID_PULSE_WIDTH_MODULATOR 0x0003

/**
 * @brief
 *      Empty block identifier
 */

#define DM7820_ID_NONE          0x0000

/**
 * @} DM7820_Register_Block_ID_Values
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Functional block ID offsets
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_Block_ID_Offsets DM7820 register functional block identifier offsets
 * @{
 */

/**
 * @brief
 *      Offset of first possible block identifier
 */

#define DM7820_FIRST_ID_OFFSET      0x0080

/**
 * @brief
 *      Offset of last possible block identifier
 */

#define DM7820_LAST_ID_OFFSET       0x03C0

/**
 * @} DM7820_Register_Block_ID_Offsets
 */

/**
 * @} DM7820_Register_Block_IDs
 */

/*=============================================================================
Definitions for BAR2 registers
 =============================================================================*/

/**
 * @defgroup DM7820_Register_BAR2_Macros DM7820 register BAR2 macros
 * @{
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FPGA Version Register at offset 0x0000
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_BAR2_FPGA_Version DM7820 register BAR2 FPGA Version Register
 * @{
 */

/**
 * @brief
 *      Bit mask to extract FPGA type identifier
 */

#define DM7820_FPGA_VERSION_TYPE_ID_MASK    0xFF00

/**
 * @brief
 *      Bit mask to extract FPGA version identifier
 */

#define DM7820_FPGA_VERSION_VERSION_MASK    0x00FF

/**
 * @} DM7820_Register_BAR2_FPGA_Version
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Board Reset Register at offset 0x0004
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Register_BAR2_Board_Reset DM7820 register BAR2 Board Reset Register
 * @{
 */

/**
 * @brief
 *      Value to write to cause a board reset
 */

#define DM7820_BOARD_RESET_DO_RESET     0xA5A5

/**
 * @} DM7820_Register_BAR2_Board_Reset
 */

/**
 * @} DM7820_Register_BAR2_Macros
 */

/**
 * @} DM7820_Register_Header
 */

#ifdef __cplusplus
}
#endif
#endif				/* __dm7820_registers_h__ */
