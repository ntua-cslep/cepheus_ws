/**
    @file

    @brief
        DM7820 user library definitions

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

    $Id: dm7820_library.h 86275 2015-03-04 15:53:23Z rgroner $
*/

#ifndef __dm7820_library_h__
#define __dm7820_library_h__

#include <stdint.h>

#include "dm7820_types.h"
#include <pthread.h>
#include <sys/wait.h>

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @defgroup DM7820_Library_Header DM7820 user library header file
 * @{
 */

/*=============================================================================
Macros
 =============================================================================*/

/**
 * @defgroup DM7820_Library_Macros DM7820 user library macros
 * @{
 */

    /**
     * Check library function return status
     */

#define    DM7820_Return_Status(status,string) \
        if(status != 0) { error(EXIT_FAILURE,errno, "ERROR: %s FAILED\n",string); }

    /**
     * Demand Mode Off, PCI to DM7820
     */

#define    DM7820_DMA_DEMAND_OFF_PCI_TO_DM7820 0x00

     /**
     * Demand Mode Off, DM7820 to PCI
     */

#define    DM7820_DMA_DEMAND_OFF_DM7820_TO_PCI 0x01

    /**
     * Demand Mode On (DREQ Signal Controls DMA), DM7820 to PCI
     */

#define    DM7820_DMA_DEMAND_ON_PCI_TO_DM7820 0x02

    /**
     * Demand Mode On (DREQ Signal Controls DMA), PCI to DM7820
     */

#define    DM7820_DMA_DEMAND_ON_DM7820_TO_PCI 0x03

/**
 * @brief
 *      Configure an incremental encoder phase filter so that counter value
 *      update is disabled for the given transition.
 *
 * @param
 *      filter
 *
 *      The phase filter to disable transition counter update in.
 *
 * @param
 *      transition
 *
 *      The transition to disable counter update for.
 */

#define DM7820_INCENC_DISABLE_PHASE_FILTER_TRANSITION(filter, transition) \
    ((filter) |= (1 << (transition)))

/**
 * @brief
 *      Configure an incremental encoder phase filter so that counter value
 *      update is enabled for the given transition.
 *
 * @param
 *      filter
 *
 *      The phase filter to enable transition counter update in.
 *
 * @param
 *      transition
 *
 *      The transition to enable counter update for.
 */

#define DM7820_INCENC_ENABLE_PHASE_FILTER_TRANSITION(filter, transition) \
    ((filter) &= ~(1 << (transition)))

/**
 * @brief
 *      Reset an incremental encoder phase filter.
 *
 * @param
 *      filter
 *
 *      The phase filter to reset.
 *
 * @note
 *      This macro sets the phase filter to the default phase filter after a
 *      device reset, i.e. enables counter update for all transitions in the
 *      filter.
 */

#define DM7820_INCENC_RESET_PHASE_FILTER(filter) \
    ((filter) = 0x00)

/**
 * @brief
 *      Determine whether or not the specified interrupt source is pending in
 *      the interrupt status obtained via DM7820_General_Get_Interrupt_Status().
 *
 * @param
 *      status
 *
 *      Interrupt status to examine.
 *
 * @param
 *      source
 *
 *      Interrupt source to determine state of.
 *
 * @retval
 *      0x00
 *
 *      The specified interrupt source is not pending.
 *
 * @retval
 *      0xFF
 *
 *      The specified interrupt source is pending.
 */

#define DM7820_INTERRUPT_STATUS_IS_SOURCE_PENDING(status, source) \
    (((status) & (0x1LL << (source))) ? 0xFF : 0x00)

/**
 * @} DM7820_Library_Macros
 */

/*=============================================================================
Type definitions
 =============================================================================*/

/**
 * @defgroup DM7820_Library_Types DM7820 user library type definitions
 * @{
 */

/**
 * @brief
 *      DM7820 user library error code type
 */

	typedef int DM7820_Error;

/**
 * @brief
 *      Incremental encoder phase filter type
 */

	typedef uint8_t dm7820_incenc_phase_filter;

/**
 * @} DM7820_Library_Types
 */

/*=============================================================================
Structures
 =============================================================================*/

/**
 * @defgroup DM7820_Library_Structures DM7820 user library structures
 * @{
 */

/**
 * @brief
 *      DM7820 board descriptor.  This structure holds information about a
 *      device needed by the library.
 */

	struct DM7820_Board_Descriptor {

    /**
     * File descriptor for device returned from open()
     */

		int file_descriptor;

    /**
     * Function pointer to the user ISR callback function.
     */

		void (*isr) (dm7820_interrupt_info status);

    /**
     * Process ID of the child process which will monitor DMA done interrupts.
     */

		pthread_t pid;
	};

/**
 * DM7820 board descriptor type
 */

	typedef struct DM7820_Board_Descriptor DM7820_Board_Descriptor;

/**
 * @} DM7820_Library_Structures
 */

/*=============================================================================
Public library functions
 =============================================================================*/

/**
 * @defgroup DM7820_Library_Functions DM7820 user library functions
 * @{
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Advanced interrupt block functions
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Library_AdvInt_Functions DM7820 user library advanced interrupt functions
 * @{
 */

/**
*******************************************************************************
@brief
    Determine whether or not a status condition has occurred for the given
    advanced interrupt.

@param
    handle

    Address of device's library board descriptor.

@param
    interrupt

    The advanced interrupt to get status of.

@param
    occurred

    Address where occurrence flag should be stored.  Zero will be stored here if
    no status condition has occurred for the interrupt.  A non-zero value will
    be stored here if a status condition has occurred for the interrupt.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      interrupt is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@warning
    If you are using interrupts, the information returned from this function is
    unreliable because the driver's interrupt handler clears all advanced
    interrupt status flags during interrupt acknowledgment.

@note
    This function reads the advanced interrupt status and then clears the
    board's advanced interrupt status flag if it is set.  The hardware will not
    reassert the flag until the next time the condition occurs for the advanced
    interrupt.
 *******************************************************************************
 */

	DM7820_Error DM7820_AdvInt_Get_Status(DM7820_Board_Descriptor * handle,
					      dm7820_advint_interrupt interrupt,
					      uint8_t * occurred);

/**
*******************************************************************************
@brief
    Read the capture register value for the given advanced interrupt and
    standard I/O port.

@param
    handle

    Address of device's library board descriptor.

@param
    interrupt

    The advanced interrupt to read capture register for.

@param
    port

    The standard I/O port to read capture register for.

@param
    value

    The address where capture register value should be stored.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      interrupt is not valid.

        @arg \c
            EINVAL      port is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_AdvInt_Read_Capture(DM7820_Board_Descriptor *
						handle,
						dm7820_advint_interrupt
						interrupt,
						DM7820_StdIO_Port port,
						uint16_t * value);

/**
*******************************************************************************
@brief
    Load the compare register for the given advanced interrupt and standard I/O
    port.

@param
    handle

    Address of device's library board descriptor.

@param
    interrupt

    The advanced interrupt to load compare register for.

@param
    port

    The standard I/O port to load compare register value for.

@param
    value

    The value to load into the compare register.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      interrupt is not valid.

        @arg \c
            EINVAL      port is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_AdvInt_Set_Compare(DM7820_Board_Descriptor * handle,
					       dm7820_advint_interrupt
					       interrupt,
					       DM7820_StdIO_Port port,
					       uint16_t value);

/**
*******************************************************************************
@brief
    Load the mask register for the given advanced interrupt and standard I/O
    port.

@param
    handle

    Address of device's library board descriptor.

@param
    interrupt

    The advanced interrupt to load mask register for.

@param
    port

    The standard I/O port to load mask register value for.

@param
    value

    The value to load into the mask register.  A zero in a bit position means
    that the corresponding port bit can generate an event or match interrupt.
    A one in a bit position means that the corresponding port bit cannot
    generate an event or match interrupt.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      interrupt is not valid.

        @arg \c
            EINVAL      port is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@warning
    For advanced interrupts to work properly, any port and/or bits not being
    used to generate an event or match interrupt must be programmed via the mask
    register to be ignored.
 *******************************************************************************
 */

	DM7820_Error DM7820_AdvInt_Set_Mask(DM7820_Board_Descriptor * handle,
					    dm7820_advint_interrupt interrupt,
					    DM7820_StdIO_Port port,
					    uint16_t value);

/**
*******************************************************************************
@brief
    Select the master clock for the given advanced interrupt.

@param
    handle

    Address of device's library board descriptor.

@param
    interrupt

    The advanced interrupt to select master clock for.

@param
    master

    The master clock to select.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      interrupt is not valid.

        @arg \c
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_ADVINT_MASTER_RESERVED.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    This function yields different results depending upon what interrupt mode
    you are using.  When using event or match mode, this function selects the
    sampling clock source.  When using strobe mode, this function selects the
    strobe signal source.  This behavior is determined by the hardware.

@warning
    When using strobe mode, do not select the 25 MHz clock as the strobe signal
    source because strobe interrupts will be generated every 40 nanoseconds.
 *******************************************************************************
 */

	DM7820_Error DM7820_AdvInt_Set_Master(DM7820_Board_Descriptor * handle,
					      dm7820_advint_interrupt interrupt,
					      dm7820_advint_master_clock
					      master);

/**
*******************************************************************************
@brief
    Set the mode for the given advanced interrupt.

@param
    handle

    Address of device's library board descriptor.

@param
    interrupt

    The advanced interrupt to set mode for.

@param
    mode

    The advanced interrupt mode to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      interrupt is not valid.

        @arg \c
            EINVAL      mode is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_AdvInt_Set_Mode(DM7820_Board_Descriptor * handle,
					    dm7820_advint_interrupt interrupt,
					    dm7820_advint_mode mode);

/**
 * @} DM7820_Library_AdvInt_Functions
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
FIFO block functions
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Library_FIFO_Functions DM7820 user library FIFO functions
 * @{
 */

/**
*******************************************************************************
@brief
    Enable or disable the given FIFO.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to enable or disable.

@param
    enable

    Flag indicating whether or not the FIFO should be enabled.  A value of zero
    means disable the FIFO.  Any other value means enable the FIFO.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The hardware clears a FIFO when it is disabled.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_Enable(DM7820_Board_Descriptor * handle,
					dm7820_fifo_queue fifo, uint8_t enable);

/**
*******************************************************************************
@brief
    Determine whether or not the specified status condition has occurred for the
    given FIFO.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to determine status of.

@param
    condition

    The status condition to check for.

@param
    occurred

    Address where occurrence flag should be stored.  Zero will be stored here if
    the specified condition has not occurred.  A non-zero value will be stored
    here if the specified condition occurred.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

        @arg \c
            EINVAL      condition is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@warning
    If you are using interrupts, the information returned from this function is
    unreliable because the driver's interrupt handler clears all FIFO status
    flags during interrupt acknowledgment.

@note
    If condition is DM7820_FIFO_STATUS_FULL or DM7820_FIFO_STATUS_EMPTY, this
    function clears the board's corresponding FIFO status flag before reading
    the FIFO status to get accurate status.  If the condition is still pending
    after the clear, the hardware will reassert the status flag.

@note
    If condition is DM7820_FIFO_STATUS_READ_REQUEST,
    DM7820_FIFO_STATUS_WRITE_REQUEST, DM7820_FIFO_STATUS_OVERFLOW, or
    DM7820_FIFO_STATUS_UNDERFLOW, this function reads the FIFO status and then
    clears the board's corresponding FIFO status flag if it is set.  The
    hardware will not reassert the flag until the next time the condition
    occurs.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_Get_Status(DM7820_Board_Descriptor * handle,
					    dm7820_fifo_queue fifo,
					    dm7820_fifo_status_condition
					    condition, uint8_t * occurred);

/**
*******************************************************************************
@brief
    Set up direct memory access (DMA) for the given DMA/FIFO channel.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to set up DMA for.

@param
    buffer_count

    The number of DMA buffers to allocate for the DMA/FIFO channel.

@param
    buffer_size

    The size of the DMA buffer to allocate for the DMA/FIFO channel.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EAGAIN      DMA has already been initialized for fifo.

        @arg \c
            EINVAL      fifo is not valid.

        @arg \c
            EINVAL      buffer_count is zero.

        @arg \c
            EINVAL      buffer_count exceeds the default value
                        DM7820_MAX_DMA_BUFFER_COUNT.

        @arg \c
            EINVAL      buffer_size is zero.

        @arg \c
            EINVAL      buffer_values exceeds the default value
                        DM7820_MAX_DMA_BUFFER_SIZE.

        @arg \c
            ENOMEM      Kernel memory allocation failed.

        @arg \c
            EOPNOTSUPP  The device is not PCI master capable.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    Since a single DMA buffer must exist in physically contiguous memory, the
    probability of DMA buffer allocation failure increases as both the number of
    buffers to allocate and the size of each buffer increase.

@note
    Factors beyond the number and size of DMA buffers affect the probability of
    DMA buffer allocation failure.  These factors include the number of
    processes on the system, how much system memory is already in use, and the
    presence of processes (such as the X server) which use a lot of memory.

@note
    System memory can be a scarce resource.  Every system entity needs some
    amount of memory.  Memory is being allocated and released all the time.

@note
    The default value for DM7820_MAX_DMA_BUFFER_COUNT is 16.  If you need to
    change this, edit include/dm7820_driver.h, save the changes, recompile the
    driver, and reload the driver.

@note
    The default value for DM7820_MAX_DMA_BUFFER_SIZE is 262,144 bytes (256
    kilobytes).  If you need to change this, edit include/dm7820_driver.h, save
    the changes, recompile the driver, and reload the driver.

@note
    As the application designer, you have some flexibility to configure DMA as
    as your purpose suits.  However, if this function fails with errno ENOMEM,
    you need to decrease the number of buffers requested or ask for smaller
    buffers.  In extreme cases, you may need to do both.

@note
    The values you specify here are used for all DMA transfers afterward.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_DMA_Initialize(DM7820_Board_Descriptor *
						handle, dm7820_fifo_queue fifo,
						uint32_t buffer_count,
						uint32_t buffer_size);

/**
********************************************************************************
@brief
    Creates a user space DMA buffer

@param
    buf

    Pointer to the buffer to be set by malloc.

@param
    size

    The size of the buffer to allocate.

@retval
    0

    Success

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            ENOMEM      Memory allocation for the buffer failed, or memory was
                        not mapped correctly for lock/unlock, or an attempt was
                        made to lock more memory than is allowed.

        @arg \c
            ENOSYS      The implementation does not support memory locking.

        @arg \c
            EAGAIN      Some or all of the memory could not be locked.

        @arg \c
            EINVAL      The memory size was not a multiple of (PAGESIZE).

        @arg \c
            EPERM       The calling process did not have approprate priveleges
                        to perform memory locking.

********************************************************************************
*/

	DM7820_Error DM7820_FIFO_DMA_Create_Buffer(uint16_t ** buf,
						   uint32_t size);

/**
********************************************************************************
@brief
    Frees a previously created user space buffer

@param
    buf

    A pointer to the buffer to be released.

@param
    size

    Size of the buffer to be released.

@retval
    0

    Success

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            ENOMEM      Memory allocation for the buffer failed, or memory was
                        not mapped correctly for lock/unlock.

        @arg \c
            ENOSYS      The implementation does not support memory locking/unlocking.

        @arg \c
            EAGAIN      Some or all of the memory could not be unlocked.

        @arg \c
            EINVAL      The memory size was not a multiple of (PAGESIZE).

        @arg \c
            EPERM       The calling process did not have approprate priveleges
                        to perform memory unlocking.
********************************************************************************
*/
	DM7820_Error DM7820_FIFO_DMA_Free_Buffer(uint16_t ** buf,
						 uint32_t size);

/**
********************************************************************************
 @brief
    Reads the DMA buffers in the driver

 @param
    handle

    Address of device's library board descriptor.

 @param
    fifo

    The FIFO to determine status of.

 @param
    user_buffer

    Virtual address of the buffer used to copy the DMA buffers back to userspace

 @param
    num_bufs

    The number of buffers to copy from the devices linked list of DMA buffers.

 @retval
    0

    Success

 @retval
    -1

    Failure

 *******************************************************************************
 */

	 DM7820_Error
	    DM7820_FIFO_DMA_Read(DM7820_Board_Descriptor * handle,
				 dm7820_fifo_queue fifo, void *user_buffer,
				 uint32_t num_bufs);

/**
********************************************************************************
 @brief
    Copies a user buffer to DMA buffers to be sent into a FIFO

 @param
    handle

    Address of device's library board descriptor.

 @param
    fifo

    The FIFO to determine status of.

 @param
    user_buffer

    Virtual address of the userspace buffer used to fill DMA buffers

 @param
    num_bufs

    The number of buffers to copy to the devices linked list of DMA buffers.
 @retval
    0

    Success

 @retval
    -1

    Failure

 *******************************************************************************
 */

	 DM7820_Error
	    DM7820_FIFO_DMA_Write(DM7820_Board_Descriptor * handle,
				  dm7820_fifo_queue fifo, void *user_buffer,
				  uint32_t num_bufs);

/**
********************************************************************************
 @brief

    Aborts a DMA transfer on a given channel

 @param
    handle

    Address of device's library board descriptor.

 @param
    fifo

    The specified DMA/FIFO channel to configure.

 @retval
    0

    Success

 @retval
    -1

    Failure

 *******************************************************************************
 */

	 DM7820_Error
	    DM7820_Stop_DMA(DM7820_Board_Descriptor * handle,
			    dm7820_fifo_queue fifo);

/**
*******************************************************************************
@brief
    Configure the specified DMA channel

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The specified DMA/FIFO channel to configure.

@param
    direction

    The direction of the DMA transfer to/from PCI

@param
    transfer_size

    The size of the DMA transfer

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

        @arg \c
            EINVAL      direction is not valid.

        @arg \c
            ENOTTY      ioctl call is invalid.

        @arg \c
            EFAULT      could get ioctl_arguments from userspace.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	 DM7820_Error
	    DM7820_FIFO_DMA_Configure(DM7820_Board_Descriptor * handle,
				      dm7820_fifo_queue fifo,
				      uint8_t direction,
				      uint32_t transfer_size);

/**
*******************************************************************************
@brief
    Enable and/or Start a DMA channel.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The DMA channel to enable.

@param
    enable

    0x00 to disable, 0xFF to enable

@param
    start

    0xFF to start

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_DMA_Enable(DM7820_Board_Descriptor * handle,
					    dm7820_fifo_queue fifo,
					    uint8_t enable, uint8_t start);

/**
*******************************************************************************
@brief
    Read a single value from the given FIFO.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to read.

@param
    data

    Address where value read from FIFO should be stored.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_Read(DM7820_Board_Descriptor * handle,
				      dm7820_fifo_queue fifo, uint16_t * data);

/**
*******************************************************************************
@brief
    Set DMA request source for the given FIFO.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to set DMA request source for.

@param
    source

    The DMA request source to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

        @arg \c
            EINVAL      source is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_Set_DMA_Request(DM7820_Board_Descriptor *
						 handle, dm7820_fifo_queue fifo,
						 dm7820_fifo_dma_request
						 source);

/**
*******************************************************************************
@brief
    Set data input for the given FIFO.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to set data input for.

@param
    input

    The data input to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

        @arg \c
            EINVAL      input is not valid.

        @arg \c
            EOPNOTSUPP  input is not supported by fifo.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_Set_Data_Input(DM7820_Board_Descriptor *
						handle, dm7820_fifo_queue fifo,
						dm7820_fifo_data_input input);

/**
*******************************************************************************
@brief
    Set input clock for the given FIFO.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to set input clock for.

@param
    clock

    The input clock to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

        @arg \c
            EINVAL      clock is not valid.

        @arg \c
            EOPNOTSUPP  clock is equal to DM7820_FIFO_INPUT_CLOCK_RESERVED_1,
                        DM7820_FIFO_INPUT_CLOCK_RESERVED_2,
                        DM7820_FIFO_INPUT_CLOCK_RESERVED_3, or
                        DM7820_FIFO_INPUT_CLOCK_RESERVED_4.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_Set_Input_Clock(DM7820_Board_Descriptor *
						 handle, dm7820_fifo_queue fifo,
						 dm7820_fifo_input_clock clock);

/**
*******************************************************************************
@brief
    Set output clock for the given FIFO.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to set output clock for.

@param
    clock

    The output clock to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

        @arg \c
            EINVAL      clock is not valid.

        @arg \c
            EOPNOTSUPP  clock is equal to DM7820_FIFO_OUTPUT_CLOCK_RESERVED_1,
                        DM7820_FIFO_OUTPUT_CLOCK_RESERVED_2,
                        DM7820_FIFO_OUTPUT_CLOCK_RESERVED_3, or
                        DM7820_FIFO_OUTPUT_CLOCK_RESERVED_4.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_Set_Output_Clock(DM7820_Board_Descriptor *
						  handle,
						  dm7820_fifo_queue fifo,
						  dm7820_fifo_output_clock
						  clock);

/**
*******************************************************************************
@brief
    Write a single value to the given FIFO.

@param
    handle

    Address of device's library board descriptor.

@param
    fifo

    The FIFO to write.

@param
    data

    Data to write to FIFO.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      fifo is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_FIFO_Write(DM7820_Board_Descriptor * handle,
				       dm7820_fifo_queue fifo, uint16_t data);

/**
 * @} DM7820_Library_FIFO_Functions
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
General functions
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Library_General_Functions DM7820 user library general functions
 * @{
 */

/**
*******************************************************************************
@brief
    Close a DM7820 device file.

@param
    handle

    Address of device's library board descriptor.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            ENODATA     handle is NULL.

    Please see the close(2) man page for information on other possible values
    errno may have in this case.

@note
    This function frees the memory allocated for the library board descriptor.

@note
    When processing the close request, the driver disables PLX PCI interrupts,
    disables PLX local interrupt input, and disables PLX DMA channel 0/1
    interrupts.

@warning
    Whether or not this function succeeds, the library board descriptor must
    not be referenced in any way after the function returns.
 *******************************************************************************
 */

	DM7820_Error DM7820_General_Close_Board(DM7820_Board_Descriptor *
						handle);

/**
*******************************************************************************
@brief
    Enable or disable the given interrupt source.

@param
    handle

    Address of device's library board descriptor.

@param
    source

    The interrupt source to change state of.

@param
    enable

    Flag indicating whether or not the interrupt source should be enabled.  A
    value of zero means disable the interrupt source.  Any other value means
    enable the interrupt source.

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

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    Some interrupts are controlled by an enable register secondary to the
    Interrupt Enable Register.  When disabling such an interrupt, if all
    interrupts in the second register are disabled then this function will also
    disable the interrupt in the Interrupt Enable Register.  For example,
    suppose only the empty interrupt is enabled for FIFO 0.  In this case, the
    FIFO 0 interrupt would be enabled in the Interrupt Enable Register and the
    empty interrupt would be enabled in the FIFO 0 Interrupt Register.  When the
    FIFO 0 empty interrupt is disabled, there are no more FIFO 0 interrupts
    enabled so this function would also disable FIFO 0 interrupts.

@note
    When enabling an interrupt source, this function clears all applicable
    interrupt status flags.

@note
    The driver's interrupt handler disables each FIFO interrupt after it occurs
    to prevent possible flooding by these interrupts.  If you are using a FIFO
    interrupt, it must be reenabled before it can be utilized again.
 *******************************************************************************
 */

	DM7820_Error DM7820_General_Enable_Interrupt(DM7820_Board_Descriptor *
						     handle,
						     dm7820_interrupt_source
						     source, uint8_t enable);

/**
*******************************************************************************
@brief
    Get a device's interrupt status, optionally waiting for an interrupt to
    occur.

@param
    handle

    Address of device's library board descriptor.

@param
    interrupt_info

    Address where interrupt information should be stored.

@param
    wait_for_interrupt

    Flag indicating whether or not to wait for an interrupt to occur.  A value
    of zero means do not wait for an interrupt and return whatever interrupt
    status is available.  Any other value means wait for an interrupt to occur
    and then return its status.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINTR       The process was waiting for an interrupt but received a
                        signal before an interrupt occurred.  This is not a
                        fatal error but rather means the wait should be retried.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    If this function is being used to wait for interrupts, signals can wake up
    the process before an interrupt occurs.  If a signal is delivered to the
    process during a wait, the application is responsible for dealing with the
    premature awakening in a reasonable manner.

@note
    When this function is being used to wait for interrupts, it can be woken up
    by a signal before an interrupt occurs and an interrupt may be missed if
    signals are delivered rapidly enough or at inopportune times.  To decrease
    the chances of this, it is strongly suggested that you 1) do not use signals
    or 2) minimize their use in your application.

@note
    This function disables all interrupts for a very brief time to obtain
    accurate status information.  If you call the function repeatedly in a loop
    (such as when busy-waiting for an interrupt to occur), this can interfere
    with system interrupts.  It is strongly suggested that you do not busy-wait
    for interrupts.
 *******************************************************************************
 */

	DM7820_Error DM7820_General_Get_Interrupt_Status(DM7820_Board_Descriptor
							 * handle,
							 dm7820_interrupt_info *
							 interrupt_info,
							 uint8_t
							 wait_for_interrupt);

/**
*******************************************************************************
@brief
    Open a DM7820 device file.

@param
    dev_num

    Minor number of DM7820 device file.

@param
    handle

    Address where address of memory allocated for library device descriptor
    should be stored.  If the first open of a device file fails, then NULL will
    be stored here.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EBUSY       The DM7820 device file with minor number dev_num is
                        already open.
        @arg \c
            ENODEV      dev_num is not a valid DM7820 minor number; 2.4 kernel
                        only.

        @arg \c
            ENOMEM      Library device descriptor memory allocation failed.

        @arg \c
            ENXIO       dev_num is not a valid DM7820 minor number; 2.6 kernel
                        only.

    Please see the open(2) man page for information on other possible values
    errno may have in this case.

@note
    Once a device file is open, it cannot be opened again until it is closed.

@note
    When processing the open request, the driver disables & clears all device
    interrupts, enables PLX PCI interrupts, enables PLX local interrupt input,
    and enables PLX DMA channel 0/1 interrupts.
 *******************************************************************************
 */

	DM7820_Error DM7820_General_Open_Board(uint8_t dev_num,
					       DM7820_Board_Descriptor **
					       handle);

/**
*******************************************************************************
@brief
    Read a device's FPGA and source code revision control versions.

@param
    handle

    Address of device's library board descriptor.

@param
    fpga_type_id

    Address where FPGA version information type identifier field should be
    stored.

@param
    fpga_version

    Address where FPGA version information version identifier field should be
    stored.

@param
    svn_version

    Address where source code revision control version identifier should be
    stored.

@retval
    0

    Success.

@retval
    -1

    Failure.

    Please see the ioctl(2) man page for information on possible values errno
    may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_General_Get_Version_Info(DM7820_Board_Descriptor *
						     handle,
						     uint8_t * fpga_type_id,
						     uint8_t * fpga_version,
						     uint16_t * svn_version);

/**
*******************************************************************************
@brief
    Determine whether or not a device is PCI master capable.

@param
    handle

    Address of device's library board descriptor.

@param
    pci_master

    Address where PCI master capable flag should be stored.  Zero will be stored
    here if the device is not PCI master capable.  A non-zero value will be
    stored here if the device is PCI master capable.

@retval
    0

    Success.

@retval
    -1

    Failure.

    Please see the ioctl(2) man page for information on possible values errno
    may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_General_Is_PCI_Master(DM7820_Board_Descriptor *
						  handle, uint8_t * pci_master);

/**
*******************************************************************************
@brief
    Reset a DM7820 device.

@param
    handle

    Address of device's library board descriptor.

@retval
    0

    Success.

@retval
    -1

    Failure.

    Please see the ioctl(2) man page for information on possible values errno
    may have in this case.

@note
    This function does not reset the PLX chip.
 *******************************************************************************
 */

	DM7820_Error DM7820_General_Reset(DM7820_Board_Descriptor * handle);

	DM7820_Error DM7820_General_InstallISR(DM7820_Board_Descriptor * handle,
					       void (*isr_fnct));

/**
********************************************************************************
@brief
    Uninstall userspace ISR

@param
    handle

    Address of the device's library board descriptor.

@retval
    0

    Success

@retval
    -1

    Failure

********************************************************************************
*/

	DM7820_Error DM7820_General_RemoveISR(DM7820_Board_Descriptor * handle);

/**
********************************************************************************
@brief
    Creates thread to watch for interrupts and call userspace ISR.

@internal

@param
    fnct

    Function that will be run by the new thread.

@param
    data

    Arguments to send to the function in the thread.

@retval
    0

    Success

@retval
    -1

    Failure

********************************************************************************
*/

	DM7820_Error DM7820_General_StartThread(int (*fnct) (void *),
						void *data);

/**
********************************************************************************
@brief
    Waits for DMA Done interrupts.

@internal

@param
    ptr

    Pointer to be typecasted to the device handle.

@retval
    0

    Success

@retval
    -1

    Failure.

@note

    This function is run in a seperate thread.
********************************************************************************
*/

	void *DM7820_General_WaitForInterrupt(void *ptr);

/**
********************************************************************************
@brief
    Changes the Priority for the ISR thread.

@param
    handle

    Address of the device's library board descriptor.

@param
    priority

    Value to change the thread's priority to. (99 highest priority, 1 lowest priority)

@retval
    0

    Success

********************************************************************************
*/

	 DM7820_Error
	    DM7820_General_SetISRPriority(DM7820_Board_Descriptor * handle,
					  int priority);

/**
 * @} DM7820_Library_General_Functions
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Incremental encoder block functions
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Library_IncEnc_Functions DM7820 user library incremental encoder functions
 * @{
 */

/**
*******************************************************************************
@brief
    Configure the given incremental encoder.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to configure.

@param
    phase_filter

    Mask for allowing/disallowing input state transitions from changing the
    counter.

    The DM7820_INCENC_DISABLE_PHASE_FILTER_TRANSITION,
    DM7820_INCENC_ENABLE_PHASE_FILTER_TRANSITION, and
    DM7820_INCENC_RESET_PHASE_FILTER macros should be used modify transitions in
    the phase filter.

@param
    input_mode

    Incremental encoder input mode.

@param
    enable_input_filter

    Flag indicating whether or not the input filter should be enabled.  A value
    of zero means disable the input filter.  Any other value means enable the
    input filter.

@param
    channel_mode

    Incremental encoder channel mode.

@param
    enable_index

    Flag indicating whether or not the index input should be enabled.  A value
    of zero means disable the index input.  Any other value means enable the
    index input.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

        @arg \c
            EINVAL      input_mode is not valid.

        @arg \c
            EINVAL      channel_mode is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The incremental encoder should be disabled before calling this function.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Configure(DM7820_Board_Descriptor * handle,
					     dm7820_incenc_encoder encoder,
					     dm7820_incenc_phase_filter
					     phase_filter,
					     dm7820_incenc_input_mode
					     input_mode,
					     uint8_t enable_input_filter,
					     dm7820_incenc_channel_mode
					     channel_mode,
					     uint8_t enable_index);

/**
*******************************************************************************
@brief
    Enable or disable the given incremental encoder.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to change state of.

@param
    enable

    Flag indicating whether or not the incremental encoder should be enabled.  A
    value of zero means disable the incremental encoder.  Any other value means
    enable the incremental encoder.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Enable(DM7820_Board_Descriptor * handle,
					  dm7820_incenc_encoder encoder,
					  uint8_t enable);

/**
*******************************************************************************
@brief
    Enable or disable value register hold for the given incremental encoder.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to change value register hold state of.

@param
    enable

    Flag indicating whether or not value register hold should be enabled.  A
    value of zero means disable value register hold.  Any other value means
    enable value register hold.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Enable_Hold(DM7820_Board_Descriptor * handle,
					       dm7820_incenc_encoder encoder,
					       uint8_t enable);

/**
*******************************************************************************
@brief
    Get 16-bit counter value of the given independent incremental encoder
    channel.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to get counter value of.

@param
    channel

    The incremental encoder channel to get counter value of.

@param
    value

    Address where counter value should be stored.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

        @arg \c
            EINVAL      channel is not valid.

        @arg \c
            EOPNOTSUPP  Incremental encoder channels A and B are joined.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Get_Independent_Value(DM7820_Board_Descriptor
							 * handle,
							 dm7820_incenc_encoder
							 encoder,
							 dm7820_incenc_channel
							 channel,
							 uint16_t * value);

/**
*******************************************************************************
@brief
    Get 32-bit counter value of the given independent incremental encoder whose
    channels are joined.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to get counter value of.

@param
    value

    Address where counter value should be stored.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

        @arg \c
            EINVAL      channel is not valid.

        @arg \c
            EOPNOTSUPP  Incremental encoder channels A and B are independent.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Get_Joined_Value(DM7820_Board_Descriptor *
						    handle,
						    dm7820_incenc_encoder
						    encoder, uint32_t * value);

/**
*******************************************************************************
@brief
    Determine whether or not the specified status condition has occurred for the
    given incremental encoder.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to get status of.

@param
    condition

    The status condition to check for.

@param
    occurred

    Address where occurrence flag should be stored.  Zero will be stored here if
    the specified condition has not occurred.  A non-zero value will be stored
    here if the specified condition occurred.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

        @arg \c
            EINVAL      condition is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@warning
    If you are using interrupts, the information returned from this function is
    unreliable because the driver's interrupt handler clears all incremental
    encoder interrupt status flags during interrupt acknowledgment.

@note
    This function reads the incremental encoder status and then clears the
    board's specified encoder status flag if it is set.  The hardware will not
    reassert the flag until the next time the specified condition occurs for the
    encoder.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Get_Status(DM7820_Board_Descriptor * handle,
					      dm7820_incenc_encoder encoder,
					      dm7820_incenc_status_condition
					      condition, uint8_t * occurred);

/**
*******************************************************************************
@brief
    Set 16-bit counter value for the given independent incremental encoder
    channel.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to set counter value for.

@param
    channel

    The incremental encoder channel to set counter value for.

@param
    value

    The counter value to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

        @arg \c
            EINVAL      channel is not valid.

        @arg \c
            EOPNOTSUPP  Incremental encoder channels A and B are joined.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Set_Independent_Value(DM7820_Board_Descriptor
							 * handle,
							 dm7820_incenc_encoder
							 encoder,
							 dm7820_incenc_channel
							 channel,
							 uint16_t value);

/**
*******************************************************************************
@brief
    Set 32-bit counter value for the given incremental encoder whose channels
    are joined.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to set counter value for.

@param
    value

    The counter value to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

        @arg \c
            EINVAL      channel is not valid.

        @arg \c
            EOPNOTSUPP  Incremental encoder channels A and B are independent.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Set_Joined_Value(DM7820_Board_Descriptor *
						    handle,
						    dm7820_incenc_encoder
						    encoder, uint32_t value);

/**
*******************************************************************************
@brief
    Set the master clock for the given incremental encoder.

@param
    handle

    Address of device's library board descriptor.

@param
    encoder

    The incremental encoder to set master clock for.

@param
    master

    The master clock to select.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      encoder is not valid.

        @arg \c
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_INCENC_MASTER_RESERVED.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The incremental encoder should be disabled before calling this function.
 *******************************************************************************
 */

	DM7820_Error DM7820_IncEnc_Set_Master(DM7820_Board_Descriptor * handle,
					      dm7820_incenc_encoder encoder,
					      dm7820_incenc_master_clock
					      master);

/**
 * @} DM7820_Library_IncEnc_Functions
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Pulse width modulator block functions
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Library_PWM_Functions DM7820 user library pulse width modulator functions
 * @{
 */

/**
*******************************************************************************
@brief
    Enable or disable the given pulse width modulator (PWM).

@param
    handle

    Address of device's library board descriptor.

@param
    pwm

    The pulse width modulator to change state of.

@param
    enable

    Flag indicating whether or not the pulse width modulator should be enabled.
    A value of zero means disable the PWM.  Any other value means enable the
    PWM.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      pwm is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_PWM_Enable(DM7820_Board_Descriptor * handle,
				       dm7820_pwm_modulator pwm,
				       uint8_t enable);

/**
*******************************************************************************
@brief
    Set the period for the given pulse width modulator (PWM).

@param
    handle

    Address of device's library board descriptor.

@param
    pwm

    The pulse width modulator to set period for.

@param
    period

    Value to set as the period.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      pwm is not valid.

        @arg \c
            ERANGE      period is greater than 0x10000.

        @arg \c
            ERANGE      period is equal to 0.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The pulse width modulator should be disabled before calling this function.
 *******************************************************************************
 */

	DM7820_Error DM7820_PWM_Set_Period(DM7820_Board_Descriptor * handle,
					   dm7820_pwm_modulator pwm,
					   uint32_t period);

/**
*******************************************************************************
@brief
    Set the period master clock for the given pulse width modulator (PWM).

@param
    handle

    Address of device's library board descriptor.

@param
    pwm

    The pulse width modulator to set period master clock for.

@param
    master

    The period master clock to select.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      pwm is not valid.

        @arg \c
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_PWM_PERIOD_MASTER_RESERVED.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The pulse width modulator should be disabled before calling this function.
 *******************************************************************************
 */

	DM7820_Error DM7820_PWM_Set_Period_Master(DM7820_Board_Descriptor *
						  handle,
						  dm7820_pwm_modulator pwm,
						  dm7820_pwm_period_master_clock
						  master);

/**
*******************************************************************************
@brief
    Set the width for the specified output on the given pulse width modulator
    (PWM).

@param
    handle

    Address of device's library board descriptor.

@param
    pwm

    The pulse width modulator to set output width for.

@param
    output

    The pulse width modulator to set width for.

@param
    width

    Value to set as width.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      pwm is not valid.

        @arg \c
            EINVAL      output is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_PWM_Set_Width(DM7820_Board_Descriptor * handle,
					  dm7820_pwm_modulator pwm,
					  dm7820_pwm_output output,
					  uint16_t width);

/**
*******************************************************************************
@brief
    Set the width master clock for the given pulse width modulator (PWM).

@param
    handle

    Address of device's library board descriptor.

@param
    pwm

    The pulse width modulator to set width master clock for.

@param
    master

    The width master clock to select.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      pwm is not valid.

        @arg \c
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_PWM_WIDTH_MASTER_RESERVED.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The pulse width modulator should be disabled before calling this function.
 *******************************************************************************
 */

	DM7820_Error DM7820_PWM_Set_Width_Master(DM7820_Board_Descriptor *
						 handle,
						 dm7820_pwm_modulator pwm,
						 dm7820_pwm_width_master_clock
						 master);

/**
 * @} DM7820_Library_PWM_Functions
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Programmable clock block functions
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Library_PrgClk_Functions DM7820 user library programmable clock functions
 * @{
 */

/**
*******************************************************************************
@brief
    Select the master clock for the given programmable clock.

@param
    handle

    Address of device's library board descriptor.

@param
    clock

    The programmable clock to select master clock for.

@param
    master

    The master clock to select.

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
            EINVAL      master is not valid.

        @arg \c
            EOPNOTSUPP  master is equal to DM7820_PRGCLK_MASTER_RESERVED.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The clock should be disabled before calling this function.
 *******************************************************************************
 */

	DM7820_Error DM7820_PrgClk_Set_Master(DM7820_Board_Descriptor * handle,
					      dm7820_prgclk_clock clock,
					      dm7820_prgclk_master_clock
					      master);

/**
*******************************************************************************
@brief
    Select the mode for the given programmable clock.

@param
    handle

    Address of device's library board descriptor.

@param
    clock

    The programmable clock to select mode for.

@param
    mode

    The mode to select.

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
            EINVAL      mode is not valid.

        @arg \c
            EOPNOTSUPP  mode is equal to DM7820_PRGCLK_MODE_RESERVED.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    When changing from one-shot to continuous mode or from continuous to
    one-shot mode, the clock must be disabled as an intermediate step.  For
    example, if a programmable clock is in continuous mode and you want to
    change it to one-shot mode, you first disable the clock and then set
    continuous mode.
 *******************************************************************************
 */

	DM7820_Error DM7820_PrgClk_Set_Mode(DM7820_Board_Descriptor * handle,
					    dm7820_prgclk_clock clock,
					    dm7820_prgclk_mode mode);

/**
*******************************************************************************
@brief
    Set the period for the given programmable clock.

@param
    handle

    Address of device's library board descriptor.

@param
    clock

    The programmable clock to set period for.

@param
    period

    Value to set as the period.

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
            ERANGE      period is greater than 0x10000.

        @arg \c
            ERANGE      period is equal to 0.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_PrgClk_Set_Period(DM7820_Board_Descriptor * handle,
					      dm7820_prgclk_clock clock,
					      uint32_t period);

/**
*******************************************************************************
@brief
    Set the start trigger for the given programmable clock.

@param
    handle

    Address of device's library board descriptor.

@param
    clock

    The programmable clock to set start trigger for.

@param
    start

    The start trigger to set.

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
            EINVAL      start is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The clock should be disabled before calling this function.
 *******************************************************************************
 */

	DM7820_Error DM7820_PrgClk_Set_Start_Trigger(DM7820_Board_Descriptor *
						     handle,
						     dm7820_prgclk_clock clock,
						     dm7820_prgclk_start_trigger
						     start);

/**
*******************************************************************************
@brief
    Set the stop trigger for the given programmable clock.

@param
    handle

    Address of device's library board descriptor.

@param
    clock

    The programmable clock to set stop trigger for.

@param
    stop

    The stop trigger to set.

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
            EINVAL      stop is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@note
    The clock should be disabled before calling this function.
 *******************************************************************************
 */

	DM7820_Error DM7820_PrgClk_Set_Stop_Trigger(DM7820_Board_Descriptor *
						    handle,
						    dm7820_prgclk_clock clock,
						    dm7820_prgclk_stop_trigger
						    stop);

/**
 * @} DM7820_Library_PrgClk_Functions
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Standard I/O block functions
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Library_StdIO_Functions DM7820 user library standard I/O functions
 * @{
 */

/**
*******************************************************************************
@brief
    Read a value from the given standard I/O port.

@param
    handle

    Address of device's library board descriptor.

@param
    port

    The port to read.

@param
    value

    Address where port value should be stored.

@note
    Any port bit not set to input has whatever value currently being output on
    that bit.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      port is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_StdIO_Get_Input(DM7820_Board_Descriptor * handle,
					    DM7820_StdIO_Port port,
					    uint16_t * value);

/**
*******************************************************************************
@brief
    Set the mode for specific bits in the given standard I/O port.

@param
    handle

    Address of device's library board descriptor.

@param
    port

    The port to set mode for.

@param
    bits

    Bit mask indicating which port bits should have their mode set.  A zero in
    a bit position means the corresponding port bit mode should not be set.  A
    one in a bit position means the corresponding port bit mode should be set.

@param
    mode

    The operating mode to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      port is not valid.

        @arg \c
            EINVAL      mode is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_StdIO_Set_IO_Mode(DM7820_Board_Descriptor * handle,
					      DM7820_StdIO_Port port,
					      uint16_t bits,
					      DM7820_StdIO_IO_Mode mode);

/**
*******************************************************************************
@brief
    Write a value to the given standard I/O port.

@param
    handle

    Address of device's library board descriptor.

@param
    port

    The port to write.

@param
    value

    Value to write.

@note
    Any port bit not set to output is ignored.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      port is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_StdIO_Set_Output(DM7820_Board_Descriptor * handle,
					     DM7820_StdIO_Port port,
					     uint16_t value);

/**
*******************************************************************************
@brief
    Set the peripheral output mode for specific bits in the given standard I/O
    port.

@param
    handle

    Address of device's library board descriptor.

@param
    port

    The port to set peripheral output mode for.

@param
    bits

    Bit mask indicating which port bits should have their peripheral output mode
    set.  A zero in a bit position means the corresponding port bit peripheral
    output mode should not be set.  A one in a bit position means the
    corresponding port bit peripheral output mode should be set.

@param
    mode

    The peripheral output mode to set.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      port is not valid.

        @arg \c
            EINVAL      mode is not valid.

        @arg \c
            EOPNOTSUPP  port does not support mode.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_StdIO_Set_Periph_Mode(DM7820_Board_Descriptor *
						  handle,
						  DM7820_StdIO_Port port,
						  uint16_t bits,
						  DM7820_StdIO_Periph_Mode
						  mode);

/**
*******************************************************************************
@brief
    Determine state of given strobe signal.

@param
    handle

    Address of device's library board descriptor.

@param
    strobe

    Strobe signal to check state of.

@param
    state

    Address where strobe signal state should be stored.  Zero will be stored
    here if the strobe signal is low.  A non-zero value will be stored here if
    the strobe signal is high.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      strobe is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_StdIO_Strobe_Input(DM7820_Board_Descriptor * handle,
					       DM7820_StdIO_Strobe strobe,
					       uint8_t * state);

/**
*******************************************************************************
@brief
    Set the direction (input or output) for the given strobe signal.

@param
    handle

    Address of device's library board descriptor.

@param
    strobe

    Strobe signal to set direction of.

@param
    output

    Flag indicating whether or not the strobe signal should be set to output.  A
    value of zero means set the signal to input.  Any other value means set the
    signal to output.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      strobe is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_StdIO_Strobe_Mode(DM7820_Board_Descriptor * handle,
					      DM7820_StdIO_Strobe strobe,
					      uint8_t output);

/**
*******************************************************************************
@brief
    Set state of given strobe signal.

@param
    handle

    Address of device's library board descriptor.

@param
    strobe

    Strobe signal to set state of.

@param
    state

    Flag indicating what state the signal should be set to.  A value of zero
    set the signal low.  Any other value sets the signal high.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      strobe is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_StdIO_Strobe_Output(DM7820_Board_Descriptor *
						handle,
						DM7820_StdIO_Strobe strobe,
						uint8_t state);

/**
 * @} DM7820_Library_StdIO_Functions
 */

/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
8254 timer/counter block functions
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%*/

/**
 * @defgroup DM7820_Library_TmrCtr_Functions DM7820 user library 8254 timer/counter functions
 * @{
 */

/**
*******************************************************************************
@brief
    Determine whether or not a status condition has occurred for the given
    timer/counter.

@param
    handle

    Address of device's library board descriptor.

@param
    timer

    8254 timer/counter to get status of.

@param
    occurred

    Address where occurrence flag should be stored.  Zero will be stored here if
    no status condition has occurred for the timer.  A non-zero value will be
    stored here if a status condition has occurred for the timer.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      timer is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.

@warning
    If you are using interrupts, the information returned from this function is
    unreliable because the driver's interrupt handler clears all timer/counter
    interrupt status flags during interrupt acknowledgment.

@note
    This function reads the timer/counter status and then clears the board's
    timer status flag if it is set.  The hardware will not reassert the flag
    until the next condition occurs for the timer.
 *******************************************************************************
 */

	DM7820_Error DM7820_TmrCtr_Get_Status(DM7820_Board_Descriptor * handle,
					      dm7820_tmrctr_timer timer,
					      uint8_t * occurred);

/**
*******************************************************************************
@brief
    Program the given 8254 timer/counter.  This will 1) set the waveform mode,
    2) set the count mode, and 3) load a divisor into the timer.

@param
    handle

    Address of device's library board descriptor.

@param
    timer

    8254 timer/counter to select clock input for.

@param
    waveform

    The waveform mode to program.

@param
    count_mode

    The count mode to program.

@param
    divisor

    16-bit divisor to load into the timer.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      timer is not valid.

        @arg \c
            EINVAL      waveform is not valid.

        @arg \c
            EINVAL      count_mode is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_TmrCtr_Program(DM7820_Board_Descriptor * handle,
					   dm7820_tmrctr_timer timer,
					   dm7820_tmrctr_waveform waveform,
					   dm7820_tmrctr_count_mode count_mode,
					   uint16_t divisor);

/**
*******************************************************************************
@brief
    Read the value of the given 8254 timer/counter.

@param
    handle

    Address of device's library board descriptor.

@param
    timer

    8254 timer/counter to read value of.

@param
    value

    Address where timer/counter value should be stored.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      timer is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_TmrCtr_Read(DM7820_Board_Descriptor * handle,
					dm7820_tmrctr_timer timer,
					uint16_t * value);

/**
*******************************************************************************
@brief
    Select the clock input for the given 8254 timer/counter.

@param
    handle

    Address of device's library board descriptor.

@param
    timer

    8254 timer/counter to select clock input for.

@param
    clock

    Clock input to select.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      timer is not valid.

        @arg \c
            EINVAL      clock is not valid.

        @arg \c
            EOPNOTSUPP  clock is equal to DM7820_TMRCTR_CLOCK_RESERVED.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_TmrCtr_Select_Clock(DM7820_Board_Descriptor *
						handle,
						dm7820_tmrctr_timer timer,
						dm7820_tmrctr_clock clock);

/**
*******************************************************************************
@brief
    Select the gate input for the given 8254 timer/counter.

@param
    handle

    Address of device's library board descriptor.

@param
    timer

    8254 timer/counter to select gate input for.

@param
    gate

    Gate input to select.

@retval
    0

    Success.

@retval
    -1

    Failure.@n@n
    errno may be set as follows:
        @arg \c
            EINVAL      timer is not valid.

        @arg \c
            EINVAL      gate is not valid.

    Please see the ioctl(2) man page for information on other possible values
    errno may have in this case.
 *******************************************************************************
 */

	DM7820_Error DM7820_TmrCtr_Select_Gate(DM7820_Board_Descriptor * handle,
					       dm7820_tmrctr_timer timer,
					       dm7820_tmrctr_gate gate);

/**
 * @} DM7820_Library_TmrCtr_Functions
 */

/**
********************************************************************************
@brief
    Install userspace ISR

@param
    handle

    Address of the device's library board descriptor.

@param
    isr_fnct

    Function pointer to the user ISR taht will be called in the event of an
    interrupt

@retval
    0

    Success

@retval
    -1

    Failure

@note
    Any previously installed ISR will be removed before installing a new ISR

@note
    This function creates another thread that runs DM7820_General_WaitForInterrupt().
    This thread is joined back in DM7820_General_RemoveISR().

********************************************************************************
*/

/**
 * @} DM7820_Library_Functions
 */

/**
 * @} DM7820_Library_Header
 */

#ifdef __cplusplus
}
#endif
#endif				/* __dm7820_library_h__ */
