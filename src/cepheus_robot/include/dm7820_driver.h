/**
    @file

    @brief
        Definitions for the DM7820 driver

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

    $Id: dm7820_driver.h 86294 2015-03-04 21:36:57Z rgroner $
*/

#ifndef __dm7820_driver_h__
#define __dm7820_driver_h__

#include <linux/fs.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#include "dm7820_ioctl.h"
#include "dm7820_types.h"

#ifdef __cplusplus
extern "C" {
#endif
/**
 * @defgroup DM7820_Driver_Header DM7820 driver header file
 * @{
 */

/*=============================================================================
Enumerations
 =============================================================================*/

/**
 * @defgroup DM7820_Driver_Enumerations DM7820 driver enumerations
 * @{
 */

/**
 * @brief
 *      Direction of access to standard PCI region
 */

	enum dm7820_pci_region_access_dir {

    /**
     * Read from the region
     */

		DM7820_PCI_REGION_ACCESS_READ = 0,

    /**
     * Write to the region
     */

		DM7820_PCI_REGION_ACCESS_WRITE
	};

/**
 * @brief
 *      Standard PCI region access direction type
 */

	typedef enum dm7820_pci_region_access_dir
	 dm7820_pci_region_access_dir_t;

/**
 * @} DM7820_Driver_Enumerations
 */

/*=============================================================================
Macros
 =============================================================================*/

/**
 * @defgroup DM7820_Driver_Macros DM7820 driver macros
 * @{
 */

/**
 * @brief
 *      Maximum number of characters in device's name
 */

#define DM7820_DEVICE_NAME_LENGTH   22

/**
 * @brief
 *      DM7820 PCI device ID
 */

#define DM7820_PCI_DEVICE_ID        0x7820

/**
 * @brief
 *      RTD Embedded Technologies PCI vendor ID
 */

#define RTD_PCI_VENDOR_ID       0x1435

/**
 * @brief
 *      Number of standard PCI regions
 */

#define DM7820_PCI_REGIONS      PCI_ROM_RESOURCE

/**
 * @brief
 *      Number of FIFO channels per device
 */

#define DM7820_FIFO_CHANNELS    2

/**
 * @brief
 *      Maximum size in bytes of any DMA buffer
 *
 * @note
 *      Be aware that the probability of DMA buffer allocation failure increases
 *      as the buffer size increases.
 *
 * @note
 *      If this default value does not suit your needs, you can change it and
 *      then recompile the driver.
 */

#define DM7820_MAX_DMA_BUFFER_SIZE  0x40000

/**
 * @brief
 *      Maximum number of DMA buffers per DMA/FIFO channel
 *
 * @note
 *      Be aware that the probability of DMA buffer allocation failure increases
 *      as the number of buffers per DMA/FIFO channel increases.
 *
 * @note
 *      If this default value does not suit your needs, you can change it and
 *      then recompile the driver.
 */

#define DM7820_MAX_DMA_BUFFER_COUNT 16

/**
 * @brief
 *      Maximum number of entries in the interrupt status queue;
 */

#define DM7820_INT_QUEUE_SIZE       0x10

/**
 * @} DM7820_Driver_Macros
 */

/*=============================================================================
Constants
 =============================================================================*/

/**
 * @defgroup DM7820_Driver_Constants DM7820 driver constants
 * @{
 */

	const uint8_t DMACSR_COMPLETE_BIT = 0x10;

	const uint8_t DMACSR_ABORT_BIT = 0x04;

	const uint8_t DMACSR_START_BIT = 0x02;

	const uint8_t DMACSR_CLEAR_INTERRUPT_BIT = 0x08;

	const uint32_t PLXCSR_DMA0_INT_ACTIVE = 0x00200000;

	const uint32_t PLXCSR_DMA1_INT_ACTIVE = 0x00400000;

	const uint16_t FPGA_INTCSR_RESERVED_BITS = 0x00C8;

/**
 * @} DM7820_Driver_Constants
 */

/*=============================================================================
Structures
 =============================================================================*/

/**
 * @defgroup DM7820_Driver_Structures DM7820 driver structures
 * @{
 */

/**
 * @brief
 *      DM7820 PCI region descriptor.  This structure holds information about
 *      one of a device's PCI memory regions.
 */

	struct dm7820_pci_region {

    /**
     * I/O port number if I/O mapped
     */

		unsigned long io_addr;

    /**
     * Length of region in bytes
     */

		unsigned long length;

    /**
     * Region's physical address if memory mapped or I/O port number if I/O
     * mapped
     */

		unsigned long phys_addr;

    /**
     * Address at which region is mapped in kernel virtual address space if
     * memory mapped
     */

		void *virt_addr;

    /**
     * Flag indicating whether or not the I/O-mapped memory ranged was
     * allocated.  A value of zero means the memory range was not allocated.
     * Any other value means the memory range was allocated.
     */

		uint8_t allocated;
	};

/**
 * @brief
 *      DM7820 PCI region descriptor type
 */

	typedef struct dm7820_pci_region dm7820_pci_region_t;

/**
 * @brief
 *      DM7820 DMA buffer descriptor.  This structure holds allocation
 *      information for a single DMA buffer.
 */

	typedef struct {

    /**
     * Bus/physical address
     */

		dma_addr_t bus_address;

    /**
     * Virtual address
     */

		void *virtual_address;
	} dm7820_dma_descriptor_t;

/**
 * @brief
 *      DM7820 DMA buffer list item
 */

	typedef struct {

    /**
     * Linked list management
     */

		struct list_head list;

    /**
     * DMA buffer allocation information
     */

		dm7820_dma_descriptor_t *dma_buffer;
	} dm7820_dma_list_item_t;

/**
 * @brief
 *      DM7820 device descriptor.  This structure holds information about a
 *      device needed by the kernel.
 */

	struct dm7820_device_descriptor {

    /**
     * Device name used when requesting resources; a NUL terminated string of
     * the form rtd-dm7820-x where x is the device minor number.
     */

		char device_name[DM7820_DEVICE_NAME_LENGTH];

    /**
     * Information about each of the standard PCI regions
     */

		dm7820_pci_region_t pci[PCI_ROM_RESOURCE];

    /**
     * Concurrency control
     */

		spinlock_t device_lock;

    /**
     * Number of entities which have the device file open.  Used to enforce
     * single open semantics.
     */

		uint8_t reference_count;

    /**
     * IRQ line number
     */

		unsigned int irq_number;

    /**
     * Flag indicating whether or not an interrupt occurred.  Cleared when
     * interrupt status is read.  Set by interrupt handler.
     */

		uint8_t interrupt_occurred;

    /**
     * Used to assist poll in shutting down the thread waiting for interrupts
     */

		uint8_t remove_isr_flag;

    /**
     * Queue of processes waiting to be woken up when an interrupt occurs
     */

		wait_queue_head_t int_wait_queue;

    /**
     * Queue of processes waiting to be woken up when an interrupt occurs
     */

		wait_queue_head_t dma_wait_queue;

    /**
     * Bit mask indicating status of each interrupt source.  A zero in a bit
     * position means the corresponding interrupt source did not occur.  A one
     * in a bit position means the corresponding interrupt source did occur.
     */

		dm7820_int_source_status_t int_source_status;

    /**
     * Per-FIFO channel flag indicating whether or not DMA was initialized.  A
     * value of zero means DMA was not initialized.  Any other value means DMA
     * was initialized.
     */

		uint8_t dma_initialized[DM7820_FIFO_CHANNELS];

    /**
     * Per-FIFO channel DMA transfer size
     */

		uint32_t dma_size[DM7820_FIFO_CHANNELS];

    /**
     * Per-FIFO channel linked list of DMA buffers
     */

		struct list_head dma_buffers_pre_transfer[DM7820_FIFO_CHANNELS];

    /**
     * Per-FIFO channel linked list of DMA buffers containing data read from
     * FIFO
     */

		struct list_head
		 dma_buffers_post_transfer[DM7820_FIFO_CHANNELS];

    /**
     * Per-FIFO flag indicating direction of DMA, true if in read
     * and false if in write
     */
		uint8_t dma_in_read_direction[DM7820_FIFO_CHANNELS];

    /**
     * Interrupt status queue
     */

		dm7820_interrupt_source int_status[DM7820_INT_QUEUE_SIZE];

    /**
     * Number of entries in the interrupt status queue
     */

		unsigned int int_queue_in;

    /**
     * Number of entries read from the interrupt status queue
     */

		unsigned int int_queue_out;

    /**
     * Number of interrupts missed because of a full queue
     */

		unsigned int int_queue_missed;

    /**
     * Number of interrupts currently in the queue
     */

		unsigned int int_queue_count;
	};

/**
 * @brief
 *      DM7820 device descriptor type
 */

	typedef struct dm7820_device_descriptor dm7820_device_descriptor_t;

/**
 * @brief
 *      Interrupt source information for a single Interrupt Status Register bit
 */

	struct dm7820_interrupt_status_source {

    /**
     * Minor interrupt register.  If there is no minor interrupt register, this
     * will be DM7820_MINOR_INT_REG_NONE.
     */

		dm7820_minor_interrupt_register minor_reg;

    /**
     * Interrupt source for register bit.  If there is a minor interrupt
     * register, this will be DM7820_INTERRUPT_NONE.
     */

		dm7820_interrupt_source source;

    /**
     * Table of interrupt sources for register bit.  If there is no minor
     * interrupt register, this will be NULL.
     */

		dm7820_interrupt_source *source_table;
	};

/**
 * @brief
 *      Interrupt Status Register bit interrupt source information type
 */

	typedef struct dm7820_interrupt_status_source
	 dm7820_interrupt_status_source_t;

/**
 * @} DM7820_Driver_Structures
 */

/*=============================================================================
Forward declarations
 =============================================================================*/

/**
 * @defgroup DM7820_Driver_Forward_Declarations DM7820 driver forward declarations
 * @{
 */

/**
 * @brief
 *      File operations supported by driver
 */

	static struct file_operations dm7820_file_ops;

/**
 * @} DM7820_Driver_Forward_Declarations
 */

/*=============================================================================
Function prototypes
 =============================================================================*/

/**
 * @defgroup DM7820_Driver_Functions DM7820 driver functions
 * @{
 */

/**
*******************************************************************************
@brief
    Read from or write to one of the standard PCI regions.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    pci_request

    Address of access' PCI request descriptor.

@param
    direction

    Direction of access to PCI region (read from or write to).

@warning
    This function performs no validation on its arguments.  All arguments are
    assumed correct.
 *******************************************************************************
 */

	static void dm7820_access_pci_region(const dm7820_device_descriptor_t *
					     dm7820_device,
					     dm7820_pci_access_request_t *
					     pci_request,
					     dm7820_pci_region_access_dir_t
					     direction);

/**
*******************************************************************************
@brief
    Allocate an interrupt line for a DM7820 device.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    pci_device

    Address of kernel's PCI device structure for the current DM7820 device.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EBUSY      The interrupt line is allocated to another device which
                        requested it as unsharable; returned by request_irq().

        @arg \c
            -EINVAL     The interrupt line is not valid; returned by
                        request_irq().

        @arg \c
            -EINVAL     No interrupt handler is to be associated with the
                        requested interrupt line; returned by request_irq().

        @arg \c
            -ENOMEM     Memory for interrupt action descriptor could not be
                        allocated; returned by request_irq().

@note
    On failure, this function will clean up by releasing any resources allocated
    by the driver to this point.
 *******************************************************************************
 */

	static int dm7820_allocate_irq(dm7820_device_descriptor_t *
				       dm7820_device,
				       const struct pci_dev *pci_device);

/**
*******************************************************************************
@brief
    Disable all non-PLX interrupts for the specified DM7820 device.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.
 *******************************************************************************
 */

	static void dm7820_disable_all_interrupts(const
						  dm7820_device_descriptor_t *
						  dm7820_device);

/**
*******************************************************************************
@brief
    Disable or enable PLX interrupts for the specified DM7820 device.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    enable

    Flag indicating whether or not PLX interrupts should be enabled.  A value of
    zero means disable PLX interrupts.  Any other value means enable PLX
    interrupts.
 *******************************************************************************
 */

	static void dm7820_enable_plx_interrupts(const
						 dm7820_device_descriptor_t *
						 dm7820_device, uint8_t enable);

/**
*******************************************************************************
@brief
    Free all coherent/consistent DMA mappings for the given DMA/FIFO channel on
    the specified DM7820 device.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    fifo

    The DMA/FIFO channel to free mappings for.

@note
    This function also frees the memory allocated to manage the DMA buffer
    allocation information and the DMA buffer lists.
 *******************************************************************************
 */

	static void dm7820_free_dma_mappings(dm7820_device_descriptor_t *
					     dm7820_device,
					     dm7820_fifo_queue fifo);

/**
*******************************************************************************
@brief
    Get interrupt status for the specified DM7820 device, optionally waiting for
    an interrupt to occur before returning the status.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EFAULT     ioctl_param is not a valid user address.
 *******************************************************************************
 */

	static int dm7820_get_interrupt_status(dm7820_device_descriptor_t *
					       dm7820_device,
					       unsigned long ioctl_param);

/**
*******************************************************************************
@brief
    Determine whether or not a device is PCI master capable.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    pci_master

    Address where pci master capable flag should be stored.  Zero will be stored
    if the device is not PCI master capable.  A non-zero value will be stored
    here if the device is PCI master capable.
 *******************************************************************************
 */

	static void dm7820_get_pci_master_status(dm7820_device_descriptor_t *
						 dm7820_device,
						 uint8_t * pci_master);

/**
*******************************************************************************
@brief
    Initialize the device descriptor for the specified DM7820 device.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.
 *******************************************************************************
 */

	static void
	 dm7820_initialize_device_descriptor(dm7820_device_descriptor_t *
					     dm7820_device);

/**
*******************************************************************************
@brief
    Initialize DMA for the specified DM7820 device.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    ioctl_argument

    Address of kernel's ioctl() request structure.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EAGAIN     DMA has already been initialized.

        @arg \c
            -EINVAL     The number of DMA buffers to allocate is zero.

        @arg \c
            -EINVAL     The number of DMA buffers to allocate exceeds the
                        default value DM7820_MAX_DMA_BUFFER_COUNT.

        @arg \c
            -EINVAL     The DMA buffer size is zero.

        @arg \c
            -EINVAL     The DMA buffer size is not evenly divisible by two.

        @arg \c
            -EINVAL     The DMA buffer size exceeds the default value
                        DM7820_MAX_DMA_BUFFER_SIZE.

        @arg \c
            -ENOMEM     Kernel memory allocation failed.

        @arg \c
            -EOPNOTSUPP The device is not PCI master capable.

@note
    When initializing DMA, this function: 1) allocates coherent/consistent DMA
    mappings, 2) allocates memory to store DMA buffer allocation information,
    3) allocates memory to link DMA buffers into device's DMA buffer list, 4)
    links all DMA buffers into the device's DMA buffer list, 5) allocates memory
    to link DMA buffers in device's free DMA buffer list, and 6) links all DMA
    buffers into the device's free DMA buffer list.

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
 *******************************************************************************
 */

	static int dm7820_initialize_dma(dm7820_device_descriptor_t *
					 dm7820_device,
					 dm7820_ioctl_argument_t *
					 ioctl_argument);

/**
********************************************************************************
@brief
    Returns the physical address of the next available DMA buffer.

@param
    dm7820_device

    Address of the device's DM7820 device descriptor.

@param
    fifo

    FIFO to get physical address of.

@retval
    0

    Success.

 *******************************************************************************
 */

	 dma_addr_t
	    dm7820_get_buffer_phy_addr(dm7820_device_descriptor_t *
				       dm7820_device, dm7820_fifo_queue fifo);

/**
*******************************************************************************
@brief
    Read from DMA buffer to copy to user

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    ioctl_argument

    Address of kernel's ioctl() request structure.

@retval
    0

    Success.

@note

    This function uses a busy wait while waiting for the expected number of DMA
    transactions to take place.
 *******************************************************************************
 */

	static int
	 dm7820_dma_read(dm7820_device_descriptor_t * dm7820_device,
			 dm7820_ioctl_argument_t * ioctl_argument);

/**
*******************************************************************************
@brief
    Write to DMA buffer.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    ioctl_argument

    Address of kernel's ioctl() request structure.

@retval
    0

    Success.
 *******************************************************************************
 */

	static int
	 dm7820_dma_write(dm7820_device_descriptor_t * dm7820_device,
			  dm7820_ioctl_argument_t * ioctl_argument);

/**
*******************************************************************************
@brief
    Stops a DMA transfer on the specified channel if one is currently running.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    fifo

    The specific FIFO/DMA channel on which to abort the transfer.

@retval
    0

    Success.
 *******************************************************************************
 */
	static int dm7820_dma_stop(dm7820_device_descriptor_t * dm7820_device,
				   dm7820_fifo_queue fifo);

/**
*******************************************************************************
@brief
    Pause DMA -- Used by the STOP_DMA function

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    fifo

    The specified DMA/FIFO channel

@retval
    0

    Success.
 *******************************************************************************
 */

	static int
	 dm7820_dma_pause(dm7820_device_descriptor_t * dm7820_device,
			  dm7820_fifo_queue fifo);

/**
*******************************************************************************
@brief
    Checks if there is a transfer currently underway for the specified
    DMA/FIFO channel.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    fifo

    The specified DMA/FIFO channel.

@retval
    0

    No transfer currently underway.  This could also denote that a transfer
    has finished.

@retval
    1

    There is a transfer currently underway..
 *******************************************************************************
 */

	static int dm7820_dma_check_xfer(dm7820_device_descriptor_t *
					 dm7820_device, dm7820_fifo_queue fifo);

/**
*******************************************************************************
@brief
    Initialize the specified DM7820 device.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@note
    When initializing a device, the driver: 1) resets the board, 2) disables PLX
    PCI interrupts, 3) disables PLX local interrupt input, 4) disables PLX DMA
    channel 0/1 interrupts, 5) sets up PLX DMA Channel 0/1 Mode Registers, and
    6) sets up PLX DMA Channel 0/1 Local Address Registers.
 *******************************************************************************
 */

	static void dm7820_initialize_hardware(const dm7820_device_descriptor_t
					       * dm7820_device);

/**
*******************************************************************************
@brief
    Add an interrupt source to the queue

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    source

    The source of the interrupt

@note
    None
 *******************************************************************************
 */

	static void
	 dm7820_int_queue_add(dm7820_device_descriptor_t * dm7820_device,
			      dm7820_interrupt_source source);

/**
*******************************************************************************
@brief
    Remove an interrupt from the front of the queue

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@retval
    dm7820_interrupt_info

    Information about the interrupt and the queue

@note
    None
 *******************************************************************************
 */

	static dm7820_interrupt_info
	    dm7820_dequeue_interrupt(dm7820_device_descriptor_t *
				     dm7820_device);

/**
*******************************************************************************
@brief
    DM7820 device interrupt handler.

@param
    irq_number

    Interrupt line number.

@param
    device_id

    Address of device's DM7820 device descriptor.  This is set on request_irq()
    call.

@retval
    IRQ_HANDLED

    Interrupt successfully processed; 2.6 kernel only.

@retval
    IRQ_NONE

    Interrupt could not be processed; 2.6 kernel only.

@note
    This function does not return a value on 2.4 kernels.
 *******************************************************************************
 */
	static irqreturn_t dm7820_interrupt_handler(int irq_number,
						    void *device_id);

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
/**
*******************************************************************************
@brief
    Process ioctl(2) system calls directed toward a DM7820 device file.

@param
    inode

    Address of kernel's inode descriptor for the device file.  Unused.

@param
    file

    Address of kernel's file descriptor for the device file.

@param
    request_code

    The service being requested.

@param
    ioctl_param

    Third parameter given on ioctl() call.  Depending upon request_code,
    ioctl_param may or may not be used.  Also based upon request_code,
    ioctl_param may be an actual value or may be an address.  If the third
    parameter is not given on the ioctl() call, then ioctl_param has some
    undefined value.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EINVAL     request_code is not valid.

    Please see the descriptions of dm7820_validate_device(),
    dm7820_read_pci_region(), dm7820_write_pci_region(), and
    dm7820_modify_pci_region() for information on other possible values
    returned in this case.
 *******************************************************************************
 */

	static int dm7820_ioctl(struct inode *inode,
				struct file *file,
				unsigned int request_code,
				unsigned long ioctl_param);
#else

/**
*******************************************************************************
@brief
    Process ioctl(2) system calls directed toward a DM7820 device file.

@param
    file

    Address of kernel's file descriptor for the device file.

@param
    request_code

    The service being requested.

@param
    ioctl_param

    Third parameter given on ioctl() call.  Depending upon request_code,
    ioctl_param may or may not be used.  Also based upon request_code,
    ioctl_param may be an actual value or may be an address.  If the third
    parameter is not given on the ioctl() call, then ioctl_param has some
    undefined value.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EINVAL     request_code is not valid.

    Please see the descriptions of dm7820_validate_device(),
    dm7820_read_pci_region(), dm7820_write_pci_region(), and
    dm7820_modify_pci_region() for information on other possible values
    returned in this case.
 *******************************************************************************
 */

	static long dm7820_ioctl(struct file *file,
				 unsigned int request_code,
				 unsigned long ioctl_param);
#endif

/**
*******************************************************************************
@brief
    Perform all actions necessary to initialize the DM7820 driver and devices.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -ENOMEM     /proc entry creation failed.

    Please see the descriptions of dm7820_probe_devices() and
    dm7820_register_char_device() for information on other possible values
    returned in this case.

@note
    On failure, this function will clean up by releasing any resources allocated
    by the driver.

@note
    When loaded, the driver performs a board reset, disables PLX PCI interrupts,
    disables PLX local interrupt input, and disables PLX DMA channel 0/1
    interrupts.
 *******************************************************************************
 */

	int dm7820_load(void);

/**
*******************************************************************************
@brief
    Read an unsigned value from one of a device's PCI regions, modify certain
    bits in the value, and then write it back to the region.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EFAULT     ioctl_param is not a valid user address.

    Please see the description of dm7820_validate_pci_access() for information
    on other possible values returned in this case.
 *******************************************************************************
 */

	static int
	 dm7820_modify_pci_region(dm7820_device_descriptor_t * dm7820_device,
				  unsigned long ioctl_param);

/**
*******************************************************************************
@brief
    Prepare a DM7820 device file to be opened and used.

@param
    inode

    Address of kernel's inode descriptor for the device file.

@param
    file

    Address of kernel's file descriptor for the device file.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EBUSY      The device file is already open.

        @arg \c
            -ENODEV     The device's inode does not refer to a valid DM7820
                        device; 2.4 kernel only.

@note
    When a device is opened, the driver disables & clears all device interrupts,
    enables PLX PCI interrupts, enables PLX local interrupt input, and enables
    PLX DMA channel 0/1 interrupts.
 *******************************************************************************
 */

	static int dm7820_open(struct inode *inode, struct file *file);

/**
*******************************************************************************
@brief
    Determine whether or not a DM7820 device is readable.  This function
    supports the poll(2) and select(2) system calls.

@param
    file

    Address of kernel's file descriptor for the device file.

@param
    poll_table

    Address of kernel's poll table descriptor.  This keeps track of all event
    queues on which the process can wait.

@retval
    status mask

    Bit mask describing the status of the device.@n@n
    The following bits may be set in the mask:
        @arg \c
            POLLPRI will be set if the file descriptor contains an invalid
            device descriptor.

        @arg \c
            POLLIN will be set if an interrupt occurred since the last time the
            interrupt status was read.

        @arg \c
            POLLRDNORM will be set if an interrupt occurred since the last time
            the interrupt status was read.

@note
    A DM7820 device is readable if and only if an interrupt just occurred on the
    device and a process has not yet obtained the interrupt status from it.

@note
    This function is used in the process of waiting until an interrupt occurs on
    a device.

@note
    This function can be executed before an interrupt occurs, which happens if
    something sends a signal to the process.
 *******************************************************************************
 */

	static unsigned int dm7820_poll(struct file *file,
					struct poll_table_struct *poll_table);

/**
*******************************************************************************
@brief
    Probe and set up all functional blocks on a device.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EIO        Unknown or unexpected block ID found.

        @arg \c
            -EBADSLT    Internal driver error.

@note
    At the present time, this function does not do much.  Without additional
    firmware support for determining what blocks exist at driver load time and
    what resources the blocks use, the best that can be done is to assume a
    fixed layout of functional blocks.  This function serves as a placeholder
    for when functional block information can be probed when the driver is
    loaded.
 *******************************************************************************
 */

	static int dm7820_probe_device_blocks(dm7820_device_descriptor_t *
					      dm7820_device);

/**
*******************************************************************************
@brief
    Probe and set up all DM7820 devices.

@param
    device_count

    Address where DM7820 device count should be stored.  The content of this
    this memory is undefined if the function fails.

@param
    device_descriptors

    Address where address of device descriptor memory should be stored.  The
    content of this memory is undefined if the function fails.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -ENAMETOOLONG       Device name creation failed.

        @arg \c
            -ENODEV             No DM7820 devices found.

        @arg \c
            -ENOMEM             Device descriptor memory allocation failed.

    Please see the descriptions of dm7820_process_pci_regions(),
    dm7820_allocate_irq()
    ... for information on other possible values returned in this case.

@note
    If set up of any device fails, then all device set up fails.

@note
    This function allocates memory for the DM7820 device descriptors based upon
    the number of devices found.

@note
    On failure, this function will clean up by releasing any resources allocated
    by the driver to this point.
 *******************************************************************************
 */

	static int dm7820_probe_devices(uint32_t * device_count,
					dm7820_device_descriptor_t **
					device_descriptors);

/**
*******************************************************************************
@brief
    For each of the standard PCI regions, get the region's base address and
    length from kernel PCI resource information set up at boot.  Also, remap
    any memory-mapped region into the kernel's virtual address space.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    pci_device

    Address of kernel's PCI device structure for the current DM7820 device.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EBUSY      I/O port or I/O memory range allocation failed.

        @arg \c
            -EIO        A region's resource flags are not valid.

        @arg \c
            -ENOMEM     Remapping a memory-mapped region into the kernel's
                        virtual address space failed.

@note
    Currently, only BAR0 through BAR2 are used.  BAR0 is the memory-mapped PLX
    DMA register region.  BAR1 is the I/O-mapped PLX DMA register region.  BAR2
    is the memory-mapped FPGA register region.

@note
    On failure, this function will clean up by releasing any resources allocated
    by the driver to this point.
 *******************************************************************************
 */

	static int dm7820_process_pci_regions(dm7820_device_descriptor_t *
					      dm7820_device,
					      const struct pci_dev *pci_device);

/**
*******************************************************************************
@brief
    Read an unsigned value from one of a device's PCI regions.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EFAULT     ioctl_param is not a valid user address.

    Please see the description of dm7820_validate_pci_access() for information
    on other possible values returned in this case.
 *******************************************************************************
 */

	static int
	 dm7820_read_pci_region(dm7820_device_descriptor_t * dm7820_device,
				unsigned long ioctl_param);

/**
*******************************************************************************
@brief
    Register the DM7820 character device and request dynamic allocation of a
    character device major number.

@param
    major

    Address where character device major number should be stored.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EBUSY      A character device major number could not be allocated;
                        returned by alloc_chrdev_region().

        @arg \c
            -EBUSY      All character device major numbers are in use; returned
                        by register_chrdev().

        @arg \c
            -ENOMEM     Memory allocation failed; returned by
                        alloc_chrdev_region().

@note
    This function hides the character device interface differences between 2.4
    and 2.6 kernels.
 *******************************************************************************
 */

	static int dm7820_register_char_device(int *major);

/**
*******************************************************************************
@brief
    Do all processing necessary after the last reference to a DM7820 device
    file is released elsewhere in the kernel.

@param
    inode

    Address of kernel's inode descriptor for the device file.  Unused.

@param
    file

    Address of kernel's file descriptor for the device file.

@retval
    0

    Success.

@retval
    < 0

    Failure.  Please see the description of dm7820_validate_device() for
    information on possible values returned in this case.

@note
    When a device is released, the driver disables PLX PCI interrupts, disables
    PLX local interrupt input, and disables PLX DMA channel 0/1 interrupts.
 *******************************************************************************
 */

	static int dm7820_release(struct inode *inode, struct file *file);

/**
*******************************************************************************
@brief
    Release any resources allocated by the driver.

@note
    This function is called both at module unload time and when the driver is
    cleaning up after some error occurred.
 *******************************************************************************
 */

	static void dm7820_release_resources(void
	    );

/**
*******************************************************************************
@brief
    Process user space DMA function request.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EFAULT     ioctl_param is not a valid user address.

        @arg \c
            -EINVAL     DMA/FIFO channel to operate upon is not valid.

        @arg \c
            -ENOSYS     DMA function request is not valid.

    Please see the descriptions of dm7820_initialize_dma(), and ????????
    for information on other possible values returned in this case.
 *******************************************************************************
 */

	static int dm7820_service_dma_function(dm7820_device_descriptor_t *
					       dm7820_device,
					       unsigned long ioctl_param);

/**
*******************************************************************************
@brief
    Perform all actions necessary to deinitialize the DM7820 driver and devices.
 *******************************************************************************
 */

	void dm7820_unload(void
	    );

/**
*******************************************************************************
@brief
    Unregister the DM7820 character device and free the character device major
    number.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EINVAL     Character major number is not valid; returned by
                        unregister_chrdev(); 2.4 kernel only.

            -EINVAL     Character major number has no file operations registered
                        for it; returned by unregister_chrdev(); 2.4 kernel
                        only.

            -EINVAL     Device name specified when character major number was
                        registered does not match the name being unregistered;
                        returned by unregister_chrdev(); 2.4 kernel only.

@note
    This function hides the character device interface differences between 2.4
    and 2.6 kernels.

@note
    This function does not fail on 2.6 kernels.
 *******************************************************************************
 */

	static int dm7820_unregister_char_device(void
	    );

/**
*******************************************************************************
@brief
    Given what is assumed to be the address of a DM7820 device descriptor, make
    sure it corresponds to a valid DM7820 device descriptor.

@param
    dm7820_device

    Address of device descriptor to be verified.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EBADFD     dm7820_device is not a valid DM7820 device descriptor
                        address.
 *******************************************************************************
 */

	static int dm7820_validate_device(const dm7820_device_descriptor_t *
					  dm7820_device);

/**
*******************************************************************************
@brief
    Validate a user-space access to one of a device's PCI regions.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    pci_request

    Address of PCI region access request descriptor.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EINVAL         The PCI region is not valid.

        @arg \c
            -EMSGSIZE       The access size is not valid.

        @arg \c
            -EOPNOTSUPP     The PCI region offset is valid but is not suitably
                            aligned for the number of bytes to be accessed.

        @arg \c
            -ERANGE         The PCI region offset is not valid.

@note
    This function accesses information in the device descriptor.  Therefore, the
    device descriptor spin lock should be held when this function is called.
 *******************************************************************************
 */

	static int dm7820_validate_pci_access(const dm7820_device_descriptor_t *
					      dm7820_device,
					      const dm7820_pci_access_request_t
					      * pci_request);

/**
*******************************************************************************
@brief
    Write an unsigned value to one of a device's PCI regions.

@param
    dm7820_device

    Address of device's DM7820 device descriptor.

@param
    ioctl_param

    Third parameter given on ioctl() call.  This is the user space address of
    the structure used to pass in the arguments.

@retval
    0

    Success.

@retval
    < 0

    Failure.@n@n
    The following values may be returned:
        @arg \c
            -EFAULT     ioctl_param is not a valid user address.

    Please see the description of dm7820_validate_pci_access() for information
    on other possible values returned in this case.
 *******************************************************************************
 */

	static int
	 dm7820_write_pci_region(dm7820_device_descriptor_t * dm7820_device,
				 unsigned long ioctl_param);

/**
 * @} DM7820_Driver_Functions
 */

/**
 * @} DM7820_Library_Header
 */

#ifdef __cplusplus
}
#endif
#endif				/* __dm7820_driver_h__ */
