/**
    @file

    @brief
        Low level ioctl() request descriptor structure and request code
        definitions

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

    $Id: dm7820_ioctl.h 86294 2015-03-04 21:36:57Z rgroner $
*/

#ifndef __dm7820_ioctl_h__
#define __dm7820_ioctl_h__

#include <linux/ioctl.h>
#include <linux/types.h>

#include "dm7820_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup DM7820_Ioctl_Header DM7820 ioctl header file
 * @{
 */

/*=============================================================================
Enumerations
 =============================================================================*/

/**
 * @defgroup DM7820_Ioctl_Enumerations DM7820 ioctl enumerations
 * @{
 */

/**
 * @brief
 *      Functions supported by driver DMA management system
 */

	typedef enum dm7820_dma_manage_function {

    /**
     * DMA initialization
     */

		DM7820_DMA_FUNCTION_INITIALIZE = 0,

     /**
     * DMA stop
     */

		DM7820_DMA_FUNCTION_STOP,

    /**
     * DMA read
     */

		DM7820_DMA_FUNCTION_READ,

    /**
     * DMA write
     */

		DM7820_DMA_FUNCTION_WRITE,

    /**
     * Get Address of DMA Buffer
     */
		DM7820_DMA_GET_BUFFER_ADDR
	} dm7820_dma_manage_function_t;

/**
 * @} DM7820_Ioctl_Enumerations
 */

/*=============================================================================
Structures
 =============================================================================*/

/**
 * @defgroup DM7820_Ioctl_Structures DM7820 ioctl structures
 * @{
 */

/**
 * @brief
 *      ioctl() request structure for read from or write to PCI region
 */

	struct dm7820_ioctl_region_readwrite {

    /**
     * PCI region access request
     */

		dm7820_pci_access_request_t access;
	};

/**
* typedef for the PCI region access request type
*/
	typedef struct dm7820_ioctl_region_readwrite
	 dm7820_ioctl_region_readwrite_t;

/**
 * @brief
 *      ioctl() request structure for PCI region read/modify/write
 */

	struct dm7820_ioctl_region_modify {

    /**
     * PCI region access request
     */

		dm7820_pci_access_request_t access;

    /**
     * Bit mask that controls which bits can be modified.  A zero in a bit
     * position means that the corresponding register bit should not be
     * modified.  A one in a bit position means that the corresponding register
     * bit should be modified.
     *
     * Note that it's possible to set bits outside of the mask depending upon
     * the register value before modification.  When processing the associated
     * request code, the driver will silently prevent this from happening but
     * will not return an indication that the mask or new value was incorrect.
     */

		union {

    /**
     * Mask for 8-bit operations
     */

			uint8_t mask8;

    /**
     * Mask for 16-bit operations
     */

			uint16_t mask16;

    /**
     * Mask for 32-bit operations
     */

			uint32_t mask32;
		} mask;
	};

/**
 * @brief
 *      ioctl() PCI region read/modify/write request descriptor type
 */

	typedef struct dm7820_ioctl_region_modify dm7820_ioctl_region_modify_t;

/**
 * @brief
 *      ioctl() request structure for getting interrupt status and waiting for
 *      an interrupt to occur
 */

	struct dm7820_ioctl_interrupt_status {

    /**
     * Flag indicating whether or not to wait for an interrupt to occur before
     * returning status.  A value of zero means do not wait for an interrupt and
     * just return whatever status is currently available.  Any other value
     * means wait for an interrupt to occur before returning status.
     */

		uint8_t wait_for_interrupt;

    /**
     * Bit mask indicating status of each interrupt source.  A zero in a bit
     * position means the corresponding interrupt source did not occur.  A one
     * in a bit position means the corresponding interrupt source did occur.
     */

		dm7820_interrupt_info int_source_info;
	};

/**
 * @brief
 *      ioctl() interrupt status request descriptor type
 */

	typedef struct dm7820_ioctl_interrupt_status
	 dm7820_ioctl_interrupt_status_t;

/**
 * @brief
 *      Arguments for DMA initialization function
 */

	typedef struct dm7820_dma_initialize_arguments {

    /**
     * Number of DMA buffers to allocate
     */

		uint32_t buffer_count;

    /**
     * DMA buffer size in bytes
     */

		uint32_t buffer_size;
	} dm7820_dma_initialize_arguments_t;

/**
 * @brief
 *      Structure encapsulating arguments to all possible DMA functions
 */

	typedef union dm7820_dma_function_arguments {

    /**
     * DMA initialization
     */

		dm7820_dma_initialize_arguments_t dma_init;
	} dm7820_dma_function_arguments_t;

/**
 * @brief
 *      ioctl() request structure for performing a DMA function
 */

	typedef struct dm7820_ioctl_dma_function {

    /**
     * Buffer for data coming from user.
     */

		void *user_buffer;

    /**
     * enumeration of the direction mode for the DMA channel
     */

		uint8_t direction;

    /**
     * contains the transfer size for the DMA channel
     */

		uint32_t DMA_Transfer_Size;

    /**
     * The address of a DMA buffer
     */

		uint32_t buffer_address;

    /**
     * Size of the DMA transfer
     */

		uint32_t transfer_size;

    /**
     * DMA/FIFO channel to operate upon
     */

		dm7820_fifo_queue fifo;

    /**
     * DMA function to perform
     */

		dm7820_dma_manage_function_t function;

    /**
     * Arguments required by function
     */

		dm7820_dma_function_arguments_t arguments;
	} dm7820_ioctl_dma_function_t;

/**
 * @brief
 *      ioctl() request structure encapsulating all possible requests.  This is
 *      what gets passed into the kernel from user space on the ioctl() call.
 */

	union dm7820_ioctl_argument {

    /**
     * PCI region read and write
     */

		dm7820_ioctl_region_readwrite_t readwrite;

    /**
     * PCI region read/modify/write
     */

		dm7820_ioctl_region_modify_t modify;

    /**
     * Get interrupt status
     */

		dm7820_ioctl_interrupt_status_t int_status;

    /**
     * DMA management function
     */

		dm7820_ioctl_dma_function_t dma_function;
	};

/**
 * @brief
 *      ioctl() request descriptor type
 */

	typedef union dm7820_ioctl_argument dm7820_ioctl_argument_t;

/**
 * @} DM7820_Ioctl_Structures
 */

/*=============================================================================
Macros
 =============================================================================*/

/**
 * @defgroup DM7820_Ioctl_Macros DM7820 ioctl macros
 * @{
 */

/**
 * @brief
 *      Unique 8-bit value used to generate unique ioctl() request codes
 */

#define DM7820_IOCTL_MAGIC      'D'

/**
 * @brief
 *      First ioctl() request number
 */

#define DM7820_IOCTL_REQUEST_BASE   0x00

/**
 * @brief
 *      ioctl() request code for reading from a PCI region
 */

#define DM7820_IOCTL_REGION_READ \
    _IOWR( \
    DM7820_IOCTL_MAGIC, \
    (DM7820_IOCTL_REQUEST_BASE + 1), \
    dm7820_ioctl_argument_t \
    )

/**
 * @brief
 *      ioctl() request code for writing to a PCI region
 */

#define DM7820_IOCTL_REGION_WRITE \
    _IOW( \
    DM7820_IOCTL_MAGIC, \
    (DM7820_IOCTL_REQUEST_BASE + 2), \
    dm7820_ioctl_argument_t \
    )

/**
 * @brief
 *      ioctl() request code for PCI region read/modify/write
 */

#define DM7820_IOCTL_REGION_MODIFY \
    _IOW( \
    DM7820_IOCTL_MAGIC, \
    (DM7820_IOCTL_REQUEST_BASE + 3), \
    dm7820_ioctl_argument_t \
    )

/**
 * @brief
 *      ioctl() request code for getting interrupt status and waiting for an
 *      interrupt to occur
 */

#define DM7820_IOCTL_GET_INTERRUPT_STATUS \
    _IOWR( \
    DM7820_IOCTL_MAGIC, \
    (DM7820_IOCTL_REQUEST_BASE + 4), \
    dm7820_ioctl_argument_t \
    )

/**
 * @brief
 *      ioctl() request code for DMA function
 */

#define DM7820_IOCTL_DMA_FUNCTION \
    _IOW( \
    DM7820_IOCTL_MAGIC, \
    (DM7820_IOCTL_REQUEST_BASE + 5), \
    dm7820_ioctl_argument_t \
    )

/**
 * @brief
 *      ioctl() request code for User ISR thread wake up
 */

#define DM7820_IOCTL_WAKEUP \
    _IOW( \
    DM7820_IOCTL_MAGIC, \
    (DM7820_IOCTL_REQUEST_BASE + 6), \
    dm7820_ioctl_argument_t \
    )

/**
 * @brief
 *      ioctl() request code to retrieve interrupt status information
 */

#define DM7820_IOCTL_INTERRUPT_INFO \
    _IOWR( \
    DM7820_IOCTL_MAGIC, \
    (DM7820_IOCTL_REQUEST_BASE + 7), \
    dm7820_interrupt_info \
    )

/**
 * @} DM7820_Ioctl_Macros
 */

/**
 * @} DM7820_Ioctl_Header
 */

#ifdef __cplusplus
}
#endif
#endif				/* __dm7820_ioctl_h__ */
