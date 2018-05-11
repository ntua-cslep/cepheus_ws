/**
    @file

    @brief
        DM7820 driver source code

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

    $Id: rtd-dm7820.c 89872 2015-07-08 16:05:17Z rgroner $
*/

#include <linux/sched/signal.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/kernel.h>

#include <asm/io.h>
#include <asm/uaccess.h>
#include <linux/fs.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/cdev.h>
#include <linux/interrupt.h>
#include <linux/wait.h>

#include "dm7820_driver.h"
#include "dm7820_globals.h"
#include "dm7820_ioctl.h"
#include "dm7820_registers.h"
#include "dm7820_types.h"
#include "dm7820_version.h"

/*=============================================================================
Driver documentation
 =============================================================================*/

#define DRIVER_COPYRIGHT RTD_COPYRIGHT_STRING

#define DRIVER_DESCRIPTION "DM7820/DM9820/DM35820 device driver"

#define DRIVER_NAME "rtd-dm7820"

MODULE_AUTHOR(DRIVER_COPYRIGHT);
MODULE_DESCRIPTION(DRIVER_DESCRIPTION);
MODULE_LICENSE("GPL");

/*=============================================================================
Global variables
 =============================================================================*/

/**
 * Character device major number; dynamically assigned
 */

static int dm7820_major;

/**
 * DM7820 device descriptors
 */

static dm7820_device_descriptor_t *dm7820_devices;

/**
 * Character device descriptor for 2.6 kernels.  This is used when registering
 * the DM7820 character device with the kernel.
 */

static struct cdev dm7820_cdev;

/**
 * Number of devices detected during probe
 */

static uint32_t dm7820_device_count;

static struct class *dev_class = NULL;

/**
 * Table of devices supported by the driver.  This array is used by
 * dm7820_probe_devices() to walk the entire PCI device list looking for DM7820
 * devices.  The table is terminated by a "NULL" entry.
 *
 * The individual structures in this array are set up using ANSI C standard
 * format initialization (which is the preferred method in 2.6 kernels) instead
 * of tagged initialization (which is the preferred method in 2.4 kernels).
 */

static const struct pci_device_id dm7820_pci_device_table[] = {
	{
	 .vendor = RTD_PCI_VENDOR_ID,
	 .device = DM7820_PCI_DEVICE_ID,
	 .subvendor = PCI_ANY_ID,
	 .subdevice = PCI_ANY_ID,
	 .class = 0,
	 .class_mask = 0,
	 .driver_data = 0},
	{
	 .vendor = 0,
	 .device = 0,
	 .subvendor = 0,
	 .subdevice = 0,
	 .class = 0,
	 .class_mask = 0,
	 .driver_data = 0}
};

MODULE_DEVICE_TABLE(pci, dm7820_pci_device_table);
/**
 * FIFO 0 block interrupt sources
 */

dm7820_interrupt_source dm7820_fifo_0_int_source[] = {
	DM7820_INTERRUPT_FIFO_0_EMPTY,
	DM7820_INTERRUPT_FIFO_0_FULL,
	DM7820_INTERRUPT_FIFO_0_OVERFLOW,
	DM7820_INTERRUPT_FIFO_0_READ_REQUEST,
	DM7820_INTERRUPT_FIFO_0_UNDERFLOW,
	DM7820_INTERRUPT_FIFO_0_WRITE_REQUEST,
	DM7820_INTERRUPT_NONE
};

/**
 * FIFO 1 block interrupt sources
 */

dm7820_interrupt_source dm7820_fifo_1_int_source[] = {
	DM7820_INTERRUPT_FIFO_1_EMPTY,
	DM7820_INTERRUPT_FIFO_1_FULL,
	DM7820_INTERRUPT_FIFO_1_OVERFLOW,
	DM7820_INTERRUPT_FIFO_1_READ_REQUEST,
	DM7820_INTERRUPT_FIFO_1_UNDERFLOW,
	DM7820_INTERRUPT_FIFO_1_WRITE_REQUEST,
	DM7820_INTERRUPT_NONE
};

/**
 * Incremental encoder 0 block interrupt sources
 */

dm7820_interrupt_source dm7820_incenc_0_int_source[] = {
	DM7820_INTERRUPT_INCENC_0_CHANNEL_A_NEGATIVE_ROLLOVER,
	DM7820_INTERRUPT_INCENC_0_CHANNEL_A_POSITIVE_ROLLOVER,
	DM7820_INTERRUPT_INCENC_0_CHANNEL_B_NEGATIVE_ROLLOVER,
	DM7820_INTERRUPT_INCENC_0_CHANNEL_B_POSITIVE_ROLLOVER,
	DM7820_INTERRUPT_NONE
};

/**
 * Incremental encoder 1 block interrupt sources
 */

dm7820_interrupt_source dm7820_incenc_1_int_source[] = {
	DM7820_INTERRUPT_INCENC_1_CHANNEL_A_NEGATIVE_ROLLOVER,
	DM7820_INTERRUPT_INCENC_1_CHANNEL_A_POSITIVE_ROLLOVER,
	DM7820_INTERRUPT_INCENC_1_CHANNEL_B_NEGATIVE_ROLLOVER,
	DM7820_INTERRUPT_INCENC_1_CHANNEL_B_POSITIVE_ROLLOVER,
	DM7820_INTERRUPT_NONE
};

/**
 * 8254 timer/counter block interrupt sources
 */

dm7820_interrupt_source dm7820_tmrctr_int_source[] = {
	DM7820_INTERRUPT_TMRCTR_A_0,
	DM7820_INTERRUPT_TMRCTR_A_1,
	DM7820_INTERRUPT_TMRCTR_A_2,
	DM7820_INTERRUPT_TMRCTR_B_0,
	DM7820_INTERRUPT_TMRCTR_B_1,
	DM7820_INTERRUPT_TMRCTR_B_2,
	DM7820_INTERRUPT_NONE
};

/**
 * Table of interrupt sources for each Interrupt Status Register bit
 */

dm7820_interrupt_status_source_t dm7820_interrupt_status_source[] = {

    /**
     * Bit 0; Advanced interrupt 0
     */

	[0] = {
	       .minor_reg = DM7820_MINOR_INT_REG_NONE,
	       .source = DM7820_INTERRUPT_ADVINT_0,
	       .source_table = NULL},

    /**
     * Bit 1; Advanced interrupt 1
     */

	[1] = {
	       .minor_reg = DM7820_MINOR_INT_REG_NONE,
	       .source = DM7820_INTERRUPT_ADVINT_1,
	       .source_table = NULL},

    /**
     * Bit 2; 8254 timer/counter interrupt
     */

	[2] = {
	       .minor_reg = DM7820_MINOR_INT_REG_TMRCTR_INT,
	       .source = DM7820_INTERRUPT_NONE,
	       .source_table = &(dm7820_tmrctr_int_source[0])
	       },

    /**
     * Bit 3; Reserved
     */

	[3] = {
	       .minor_reg = DM7820_MINOR_INT_REG_NONE,
	       .source = DM7820_INTERRUPT_NONE,
	       .source_table = NULL},

    /**
     * Bit 4; Incremental encoder 0 interrupt
     */

	[4] = {
	       .minor_reg = DM7820_MINOR_INT_REG_INCENC_0_INT,
	       .source = DM7820_INTERRUPT_NONE,
	       .source_table = &(dm7820_incenc_0_int_source[0])
	       },

    /**
     * Bit 5; Incremental encoder 1 interrupt
     */

	[5] = {
	       .minor_reg = DM7820_MINOR_INT_REG_INCENC_1_INT,
	       .source = DM7820_INTERRUPT_NONE,
	       .source_table = &(dm7820_incenc_1_int_source[0])
	       },

    /**
     * Bit 6; Reserved
     */

	[6] = {
	       .minor_reg = DM7820_MINOR_INT_REG_NONE,
	       .source = DM7820_INTERRUPT_NONE,
	       .source_table = NULL},

    /**
     * Bit 7; Reserved
     */

	[7] = {
	       .minor_reg = DM7820_MINOR_INT_REG_NONE,
	       .source = DM7820_INTERRUPT_NONE,
	       .source_table = NULL},

    /**
     * Bit 8; Pulse width modulator 0 interrupt
     */

	[8] = {
	       .minor_reg = DM7820_MINOR_INT_REG_NONE,
	       .source = DM7820_INTERRUPT_PWM_0,
	       .source_table = NULL},

    /**
     * Bit 9; Pulse width modulator 1 interrupt
     */

	[9] = {
	       .minor_reg = DM7820_MINOR_INT_REG_NONE,
	       .source = DM7820_INTERRUPT_PWM_1,
	       .source_table = NULL},

    /**
     * Bit 10; Programmable clock 0 interrupt
     */

	[10] = {
		.minor_reg = DM7820_MINOR_INT_REG_NONE,
		.source = DM7820_INTERRUPT_PRGCLK_0,
		.source_table = NULL},

    /**
     * Bit 11; Programmable clock 1 interrupt
     */

	[11] = {
		.minor_reg = DM7820_MINOR_INT_REG_NONE,
		.source = DM7820_INTERRUPT_PRGCLK_1,
		.source_table = NULL},

    /**
     * Bit 12; Programmable clock 2 interrupt
     */

	[12] = {
		.minor_reg = DM7820_MINOR_INT_REG_NONE,
		.source = DM7820_INTERRUPT_PRGCLK_2,
		.source_table = NULL},

    /**
     * Bit 13; Programmable clock 3 interrupt
     */

	[13] = {
		.minor_reg = DM7820_MINOR_INT_REG_NONE,
		.source = DM7820_INTERRUPT_PRGCLK_3,
		.source_table = NULL},

    /**
     * Bit 14; FIFO 0 interrupt
     */

	[14] = {
		.minor_reg = DM7820_MINOR_INT_REG_FIFO_0_INT,
		.source = DM7820_INTERRUPT_NONE,
		.source_table = &(dm7820_fifo_0_int_source[0])
		},

    /**
     * Bit 15; FIFO 1 interrupt
     */

	[15] = {
		.minor_reg = DM7820_MINOR_INT_REG_FIFO_1_INT,
		.source = DM7820_INTERRUPT_NONE,
		.source_table = &(dm7820_fifo_1_int_source[0])
		}
};

/*=============================================================================
Driver functions
 =============================================================================*/

/******************************************************************************
Access a standard PCI region
 ******************************************************************************/

static void
dm7820_access_pci_region(const dm7820_device_descriptor_t * dm7820_device,
			 dm7820_pci_access_request_t * pci_request,
			 dm7820_pci_region_access_dir_t direction)
{
	unsigned long address;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Compute the address to be accessed
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	address = pci_request->offset;

	if (dm7820_device->pci[pci_request->region].virt_addr != NULL) {
		address += (unsigned long)
		    dm7820_device->pci[pci_request->region].virt_addr;
	} else {
		address += dm7820_device->pci[pci_request->region].io_addr;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine whether access is a read or write
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (direction == DM7820_PCI_REGION_ACCESS_READ) {

		/*
		 * Region is to be read
		 */

		/*#####################################################################
		   Determine whether the region is memory or I/O mapped
		   ################################################################## */

		if (dm7820_device->pci[pci_request->region].virt_addr != NULL) {

			/*
			 * Region is memory mapped
			 */

			/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			   Determine how many bits are to be accessed
			   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

			switch (pci_request->size) {
			case DM7820_PCI_REGION_ACCESS_8:
				pci_request->data.data8 =
				    ioread8((unsigned long *)address);
				break;

			case DM7820_PCI_REGION_ACCESS_16:
				pci_request->data.data16 =
				    ioread16((unsigned long *)address);
				break;

			case DM7820_PCI_REGION_ACCESS_32:
				pci_request->data.data32 =
				    ioread32((unsigned long *)address);
				break;
			}
		} else {

			/*
			 * Region is I/O mapped
			 */

			/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			   Determine how many bits are to be accessed
			   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

			switch (pci_request->size) {
			case DM7820_PCI_REGION_ACCESS_8:
				pci_request->data.data8 =
				    (uint8_t) inb(address);
				break;

			case DM7820_PCI_REGION_ACCESS_16:
				pci_request->data.data16 =
				    (uint16_t) inw(address);
				break;

			case DM7820_PCI_REGION_ACCESS_32:
				pci_request->data.data32 =
				    (uint32_t) inl(address);
				break;
			}
		}
	} else {

		/*
		 * Region is to be written
		 */

		/*#####################################################################
		   Determine whether the region is memory or I/O mapped
		   ################################################################## */

		if (dm7820_device->pci[pci_request->region].virt_addr != NULL) {

			/*
			 * Region is memory mapped
			 */

			/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			   Determine how many bits are to be accessed
			   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

			switch (pci_request->size) {
			case DM7820_PCI_REGION_ACCESS_8:
				iowrite8(pci_request->data.data8,
					 (unsigned long *)address);
				break;

			case DM7820_PCI_REGION_ACCESS_16:
				iowrite16(pci_request->data.data16,
					  (unsigned long *)address);
				break;

			case DM7820_PCI_REGION_ACCESS_32:
				iowrite32(pci_request->data.data32,
					  (unsigned long *)address);
				break;
			}
		} else {

			/*
			 * Region is I/O mapped
			 */

			/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
			   Determine how many bits are to be accessed
			   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

			switch (pci_request->size) {
			case DM7820_PCI_REGION_ACCESS_8:
				outb(pci_request->data.data8, address);
				break;

			case DM7820_PCI_REGION_ACCESS_16:
				outw(pci_request->data.data16, address);
				break;

			case DM7820_PCI_REGION_ACCESS_32:
				outl(pci_request->data.data32, address);
				break;
			}
		}
	}
}

/******************************************************************************
IRQ line allocation
 ******************************************************************************/

static int
dm7820_allocate_irq(dm7820_device_descriptor_t * dm7820_device,
		    const struct pci_dev *pci_device)
{
	int status;

	/*
	 * The fourth request_irq() argument MUST refer to memory which will remain
	 * valid until the driver is unloaded.  request_irq() simply stores this
	 * address in a structure rather than making a copy of the string it points
	 * to.
	 */

	status = request_irq(pci_device->irq,
			     (irq_handler_t) dm7820_interrupt_handler,
			     IRQF_SHARED,
			     &((dm7820_device->device_name)[0]), dm7820_device);
	if (status != 0) {
		printk(KERN_ERR
		       "%s: ERROR: Unable to allocate IRQ %u (error = %u)\n",
		       &((dm7820_device->device_name)[0]), pci_device->irq,
		       -status);
		dm7820_release_resources();
		return status;
	}

	dm7820_device->irq_number = pci_device->irq;

	printk(KERN_INFO "%s: Allocated IRQ %u\n",
	       &((dm7820_device->device_name)[0]), pci_device->irq);

	return 0;
}

/******************************************************************************
Disable all non-PLX interrupts
 ******************************************************************************/

static void
dm7820_disable_all_interrupts(const dm7820_device_descriptor_t * dm7820_device)
{
	dm7820_pci_access_request_t pci_request;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Disable all local interrupts in Interrupt Enable Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_INTERRUPT_ENABLE;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;
	pci_request.data.data16 = 0x0000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_INTERRUPT_ENABLE;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;

	/*
	 * Preserve reserved bits (3, 6, and 7) and disable all interrupts (bits 0
	 * through 2, 4, 5, and 8 through 15)
	 */

	pci_request.data.data16 &= 0x00C8;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Disable and clear FIFO block interrupts
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable/clear FIFO 0 interrupts
	 */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_FIFO0_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;
	pci_request.data.data16 = 0x0000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	/*
	 * Clear all status bits (8 through 15) and disable all interrupts (bits 0
	 * through 7)
	 */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_FIFO0_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;

	pci_request.data.data16 = 0xFF00;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*
	 * Disable/clear FIFO 1 interrupts
	 */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_FIFO1_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;
	pci_request.data.data16 = 0x0000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	/*
	 * Clear all status bits (8 through 15) and disable all interrupts (bits 0
	 * through 7)
	 */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_FIFO1_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;

	pci_request.data.data16 = 0xFF00;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Disable and clear incremental encoder block 0 interrupts
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_INCENC0_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;
	pci_request.data.data16 = 0x0000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_INCENC0_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;

	/*
	 * Preserve reserved bits (4 through 7 and 12 through 15) and disable all
	 * interrupts (bits 0 through 3)
	 */

	pci_request.data.data16 &= 0xF0F0;

	/*
	 * Clear all status bits (bits 8 through 11)
	 */

	pci_request.data.data16 |= 0x0F00;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Disable and clear incremental encoder block 1 interrupts
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_INCENC1_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;
	pci_request.data.data16 = 0x0000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_INCENC1_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;

	/*
	 * Preserve reserved bits (4 through 7 and 12 through 15) and disable all
	 * interrupts (bits 0 through 3)
	 */

	pci_request.data.data16 &= 0xF0F0;

	/*
	 * Clear all status bits (bits 8 through 11)
	 */

	pci_request.data.data16 |= 0x0F00;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Disable and clear timer/counter block interrupts
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_TC_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;
	pci_request.data.data16 = 0x0000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_TC_INT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;

	/*
	 * Preserve reserved bits (6, 7, 14, and 15) and disable all interrupts
	 * (bits 0 through 5)
	 */

	pci_request.data.data16 &= 0xC0C0;

	/*
	 * Clear all status bits (bits 8 through 13)
	 */

	pci_request.data.data16 |= 0x3F00;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Clear all local interrupts in Interrupt Status Register
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_INTERRUPT_STATUS;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;
	pci_request.data.data16 = 0x0000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_INTERRUPT_STATUS;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;

	/*
	 * Preserve reserved bits (3, 6, and 7)
	 */

	pci_request.data.data16 &= 0x00C8;

	/*
	 * Clear all status bits (bits 0 through 2, 4, 5, and 8 through 15)
	 */

	pci_request.data.data16 |= ~0x00C8;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);
}

/******************************************************************************
Disable/enable PLX interrupts
 ******************************************************************************/

static void
dm7820_enable_plx_interrupts(const dm7820_device_descriptor_t * dm7820_device,
			     uint8_t enable)
{
	dm7820_pci_access_request_t pci_request;

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_INTCSR;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 = 0x00000000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_INTCSR;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;

	/*
	 * Preserve all bits except for 8, 11, 18, & 19 and clear those bits (which
	 * disables PCI interrupts, local interrupt input, and DMA channel 0/1
	 * interrupts)
	 */

	pci_request.data.data32 &= ~0x000C0900;

	/*
	 * If PLX interrupts should be enabled, set bits 8, 11, 18, & 19 (which
	 * enables PCI interrupts, local interrupt input, and DMA channel 0/1
	 * interrupts)
	 */

	if (enable) {
		pci_request.data.data32 |= 0x000C0900;
	}

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

}

/******************************************************************************
Free DMA/FIFO channel DMA mappings
 ******************************************************************************/

static void
dm7820_free_dma_mappings(dm7820_device_descriptor_t * dm7820_device,
			 dm7820_fifo_queue fifo)
{
	unsigned long irq_flags;
	struct list_head *cursor;
	struct list_head *next;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Release DMA mappings, memory allocated for DMA buffer descriptors, and
	   memory allocated for DMA buffer list items
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Iterate through every buffer in device DMA buffer list
	 */

	list_for_each_safe(cursor, next,
			   &((dm7820_device->dma_buffers_pre_transfer)[fifo])) {
		dm7820_dma_list_item_t *list_item;

		/*
		 * Get address of containing structure of this list_head; the structure
		 * is a DMA buffer list item
		 */
		spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);
		list_item = list_entry(cursor, dm7820_dma_list_item_t, list);

		/*
		 * Delete element from list to prevent duplicate memory frees
		 */

		list_del(cursor);

		/*
		 * Free coherent/consistent DMA mapping using information contained in
		 * DMA buffer descriptor referred to by DMA buffer list item
		 */
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
		dma_free_coherent(NULL,
				  (dm7820_device->dma_size)[fifo],
				  (list_item->dma_buffer)->virtual_address,
				  (list_item->dma_buffer)->bus_address);

		/*
		 * Free memory allocated for DMA buffer descriptor and DMA buffer list
		 * item
		 */

		kfree(list_item->dma_buffer);
		kfree(list_item);
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Release Read DMA mappings and buffers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Iterate through every buffer in device DMA read buffer list
	 */

	list_for_each_safe(cursor, next,
			   &((dm7820_device->dma_buffers_post_transfer)[fifo]))
	{
		dm7820_dma_list_item_t *list_item;

		/*
		 * Get address of containing structure of this list_head; the structure
		 * is a DMA buffer list item
		 */
		spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);
		list_item = list_entry(cursor, dm7820_dma_list_item_t, list);

		/*
		 * Delete element from list to prevent duplicate memory frees
		 */

		list_del(cursor);

		/*
		 * Free coherent/consistent DMA mapping using information contained in
		 * DMA buffer descriptor referred to by DMA buffer list item
		 */
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
		dma_free_coherent(NULL,
				  (dm7820_device->dma_size)[fifo],
				  (list_item->dma_buffer)->virtual_address,
				  (list_item->dma_buffer)->bus_address);

		/*
		 * Free memory allocated for DMA buffer descriptor and DMA buffer list
		 * item
		 */

		kfree(list_item->dma_buffer);
		kfree(list_item);
	}
}

/******************************************************************************
Get device interrupt status
 ******************************************************************************/

static int
dm7820_get_interrupt_status(dm7820_device_descriptor_t * dm7820_device,
			    unsigned long ioctl_param)
{
	dm7820_ioctl_argument_t ioctl_argument;
	int return_value = 0;
	uint8_t interrupt_occurred;
	uint8_t status_available;
	wait_queue_entry_t wait;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Copy arguments in from user space and validate them
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (copy_from_user(&ioctl_argument,
			   (dm7820_ioctl_argument_t *) ioctl_param,
			   sizeof(dm7820_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Wait initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Initialize status wait queue entry and add it to the device's interrupt
	 * wait queue.  This prepares the process for going to sleep waiting for an
	 * interrupt.  This is done whether or not the process will ultimately
	 * sleep to avoid a race condition where an interrupt may be missed if it
	 * occurs after reading cached interrupt status and before the process
	 * inserts itself on the interrupt wait queue.
	 */

	init_waitqueue_entry(&wait, current);
	add_wait_queue(&(dm7820_device->int_wait_queue), &wait);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Get device interrupt status
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * If the process ultimately sleeps, a signal can prematurely wake the
	 * process without an interrupt occurring.  Therefore, we need to loop here
	 * to accurately see an interrupt.
	 */

	for (;;) {
		unsigned long irq_flags;
		dm7820_interrupt_info interrupt_info;

		/*#####################################################################
		   Initialization
		   ################################################################## */

		/*
		 * Set process' state to indicate to rest of system that it is asleep;
		 * more preparation for putting the process to sleep.  This is done
		 * whether or not the process will ultimately sleep to avoid a race
		 * condition where an interrupt may be missed if it occurs after
		 * reading cached interrupt status and before the process inserts itself
		 * on the interrupt wait queue.
		 */

		set_current_state(TASK_INTERRUPTIBLE);

		/*
		 * Initially assume that no status is available
		 */

		status_available = 0x00;

		/*#####################################################################
		   Grab device's cached interrupt status and interrupt occurrence flag
		   ################################################################## */

		/*
		 * Prevent a race condition with the interrupt handler
		 */

		spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

		/*
		 * Get the interrupt at the top of the queue, if there is one
		 */

		interrupt_info = dm7820_dequeue_interrupt(dm7820_device);

		ioctl_argument.int_status.int_source_info = interrupt_info;
		interrupt_occurred = interrupt_info.int_remaining >= 0;

		/*
		 * Clear interrupt occurrence flag.  This
		 * must be done in the critical section.
		 */

		dm7820_device->interrupt_occurred = 0x00;

		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);

		/*#####################################################################
		   Local copies of interrupt status and interrupt occurrence flag made,
		   so examine them
		   ################################################################## */

		/*
		 * If interrupt occurred flag is not cleared, then status is available
		 */

		if (interrupt_occurred != 0x00) {
			status_available = 0xFF;
			break;
		}

		/*
		 * An interrupt has not occurred since the last time interrupt status
		 * was read
		 */

		/*
		 * If process does not want to wait, then return whatever status exists
		 */

		if (ioctl_argument.int_status.wait_for_interrupt == 0x00) {
			status_available = 0xFF;
			break;
		}

		/*
		 * At this point, no status is available and the process wants to wait
		 * for an interrupt
		 */

		/*
		 * If there is a signal pending for the process, inform file system
		 * layer that a signal is pending and let it decide what to do about it
		 */

		if (signal_pending(current)) {
			return_value = -ERESTARTSYS;
			break;
		}

		/*
		 * Switch this process away from the CPU, thus finally putting it to
		 * sleep
		 */

		schedule();
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Interrupt status has been determined
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * If status is available, copy interrupt status back to user space
	 */

	if (status_available) {
		if (copy_to_user((dm7820_ioctl_argument_t *) ioctl_param,
				 &ioctl_argument,
				 sizeof(dm7820_ioctl_argument_t)
		    )
		    ) {
			return_value = -EFAULT;
		}
	}

	/*
	 * Mark process as runnable again which undoes previously setting the state
	 * to sleeping
	 */

	set_current_state(TASK_RUNNING);

	/*
	 * Remove entry from device's interrupt wait queue since process is either
	 * not going to sleep or has already slept
	 */

	remove_wait_queue(&(dm7820_device->int_wait_queue), &wait);

	return return_value;
}

/******************************************************************************
Determine PCI master status
 ******************************************************************************/

static void
dm7820_get_pci_master_status(dm7820_device_descriptor_t * dm7820_device,
			     uint8_t * pci_master)
{
	dm7820_pci_access_request_t pci_request;

	pci_request.data.data16 = 0x0000;
	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_BRD_STAT;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	if (pci_request.data.data16 & 0x0001) {
		*pci_master = 0x00;
	} else {
		*pci_master = 0xFF;
	}
}

/******************************************************************************
DM7820 device descriptor initialization
 ******************************************************************************/

static void
dm7820_initialize_device_descriptor(dm7820_device_descriptor_t * dm7820_device)
{

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Interrupt status initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	dm7820_device->interrupt_occurred = 0x00;
	dm7820_device->int_queue_in = 0;
	dm7820_device->int_queue_out = 0;
	dm7820_device->int_queue_missed = 0;
	dm7820_device->int_queue_count = 0;
	dm7820_device->remove_isr_flag = 0x00;
	init_waitqueue_head(&(dm7820_device->int_wait_queue));

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   DMA initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	dm7820_device->dma_initialized[DM7820_FIFO_QUEUE_0] = 0x00;
	dm7820_device->dma_initialized[DM7820_FIFO_QUEUE_1] = 0x00;

	dm7820_device->dma_size[DM7820_FIFO_QUEUE_0] = 0;
	dm7820_device->dma_size[DM7820_FIFO_QUEUE_1] = 0;

	INIT_LIST_HEAD(&
		       (dm7820_device->dma_buffers_pre_transfer
			[DM7820_FIFO_QUEUE_0]));
	INIT_LIST_HEAD(&
		       (dm7820_device->dma_buffers_pre_transfer
			[DM7820_FIFO_QUEUE_1]));

	INIT_LIST_HEAD(&
		       (dm7820_device->dma_buffers_post_transfer
			[DM7820_FIFO_QUEUE_0]));
	INIT_LIST_HEAD(&
		       (dm7820_device->dma_buffers_post_transfer
			[DM7820_FIFO_QUEUE_1]));
}

/******************************************************************************
Pauses the DMA  for DMA/FIFO channel
 ******************************************************************************/

static int
dm7820_dma_pause(dm7820_device_descriptor_t * dm7820_device,
		 dm7820_fifo_queue fifo)
{
	unsigned long irq_flags;
	dm7820_pci_access_request_t pci_request;
	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_DMACSR0 + fifo;
	pci_request.size = DM7820_PCI_REGION_ACCESS_8;
	pci_request.data.data8 = 0x00;

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	/*
	 * writing a 0 to bit DMACSRn[0] to pause the DMA channel
	 * preserve bits 2-16
	 */

	pci_request.data.data8 &= 0xFE;

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_DMACSR0 + fifo;
	pci_request.size = DM7820_PCI_REGION_ACCESS_8;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	return 0;
}

/******************************************************************************
Checks to see if there is a current DMA transfer for DMA/FIFO channel
 ******************************************************************************/

static int
dm7820_dma_check_xfer(dm7820_device_descriptor_t * dm7820_device,
		      dm7820_fifo_queue fifo)
{
	dm7820_pci_access_request_t pci_request;

	/*
	 * read DMASCRn[4] to see if there is currently a transfer underway
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;

	if (fifo == DM7820_FIFO_QUEUE_0)
		pci_request.offset = DM7820_BAR0_DMACSR0;
	else if (fifo == DM7820_FIFO_QUEUE_1)
		pci_request.offset = DM7820_BAR0_DMACSR1;
	else
		return -EINVAL;
	pci_request.size = DM7820_PCI_REGION_ACCESS_8;
	pci_request.data.data8 = 0x00;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	if (pci_request.data.data8 & 0x10) {
		/*
		 * DMA Done
		 */
		return 1;
	} else {
		/*
		 * Transfer underway
		 */
		return 0;
	}
}

/******************************************************************************
Aborts current DMA transfer for DMA/FIFO channel
 ******************************************************************************/
static int
dm7820_dma_abort(dm7820_device_descriptor_t * dm7820_device,
		 dm7820_fifo_queue channel)
{
	dm7820_pci_access_request_t pci_request;
	unsigned long irq_flags;
	uint16_t status_offset;
	uint16_t mode_offset;
	unsigned long timeout = 0;

	switch (channel) {
	case DM7820_FIFO_QUEUE_0:
		status_offset = DM7820_BAR0_DMACSR0;
		mode_offset = DM7820_BAR0_DMAMODE0;
		break;
	case DM7820_FIFO_QUEUE_1:
		status_offset = DM7820_BAR0_DMACSR1;
		mode_offset = DM7820_BAR0_DMAMODE1;
		break;
	default:
		return -EINVAL;
		break;
	}

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	/*
	 * Disable DMA Done interrupt as we are killing the transfer here
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = mode_offset;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 = 0x00000000;
	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.data.data32 &= ~0x00000400;

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = mode_offset;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*
	 * Set the abort bit and clear the enable bit.
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = status_offset;
	pci_request.size = DM7820_PCI_REGION_ACCESS_8;
	pci_request.data.data8 = 0x00;
	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.data.data8 &= ~0x05;
	pci_request.data.data8 |= 0x04;

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = status_offset;
	pci_request.size = DM7820_PCI_REGION_ACCESS_8;
	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*
	 * Wait for the Done bit
	 */

	do {

		pci_request.region = DM7820_PCI_REGION_PLX_MEM;
		pci_request.offset = status_offset;
		pci_request.size = DM7820_PCI_REGION_ACCESS_8;
		pci_request.data.data8 = 0x00;
		dm7820_access_pci_region(dm7820_device, &pci_request,
					 DM7820_PCI_REGION_ACCESS_READ);

		if (timeout++ > 1000000) {
			printk(KERN_ERR "%s: DMA abort timed out!\n",
			       &((dm7820_device->device_name)[0]));
			break;
		}

	} while (!(pci_request.data.data8 & 0x10));

	/*
	 * Enable DMA Done interrupts
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = mode_offset;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 = 0x00000000;
	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	pci_request.data.data32 &= ~0x00000400;
	pci_request.data.data32 |= 0x00000400;

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = mode_offset;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	return 0;

}

/******************************************************************************
Stops current DMA transfer for DMA/FIFO channel
 ******************************************************************************/

static int
dm7820_dma_stop(dm7820_device_descriptor_t * dm7820_device,
		dm7820_fifo_queue fifo)
{

	int status;

	status = dm7820_dma_pause(dm7820_device, fifo);

	if (status != 0) {
		return (-ENOSYS);
	}

	status = dm7820_dma_check_xfer(dm7820_device, fifo);

	/*
	 * If transfer is in progress, abort it
	 */
	if (status == 0) {
		dm7820_dma_abort(dm7820_device, fifo);
	}
	return (0);
}

/******************************************************************************
Read from DMA for DMA/FIFO channel
 ******************************************************************************/
static int
dm7820_dma_read(dm7820_device_descriptor_t * dm7820_device,
		dm7820_ioctl_argument_t * ioctl_argument)
{
	uint32_t buffer_index, num_bufs;
	uint16_t status;
	unsigned long irq_flags;

#if defined(DM7820_DEBUG_DMA)
	uint32_t num_post_bufs = 0;
#endif

	dm7820_fifo_queue fifo = ioctl_argument->dma_function.fifo;
	struct list_head *cursor, *next;

	num_bufs = ioctl_argument->dma_function.transfer_size;
	status = 0;
	buffer_index = 0;

#if defined(DM7820_DEBUG_DMA)
	list_for_each(cursor,
		      (&(dm7820_device->dma_buffers_post_transfer)[fifo])) {
		num_post_bufs++;
	}

	printk(KERN_DEBUG "Reading %d dma%d buffers to %d user buffers\n",
	       num_post_bufs, fifo, num_bufs);

#endif
	/*
	 * Send the filled buffers back to userspace while waiting for
	 * the rest of the data.  This shouldn't do anything if the list
	 * of buffers is empty.
	 */

	list_for_each_safe(cursor, next,
			   (&(dm7820_device->dma_buffers_post_transfer)[fifo]))
	{

		dm7820_dma_list_item_t *dma_item;
		dma_item = list_entry(cursor, dm7820_dma_list_item_t, list);

		/*
		 * Copy any filled buffers back to userspace.
		 */

		status =
		    copy_to_user(ioctl_argument->dma_function.user_buffer
				 +
				 ((dm7820_device->dma_size)[fifo]) *
				 buffer_index,
				 (dma_item->dma_buffer)->virtual_address,
				 (dm7820_device->dma_size)[fifo]);

		if (status != 0) {

			printk(KERN_ERR "ERROR: read_dma failed!\n");
			return -EFAULT;
		}
		/*
		 * After the buffer is copied to userspace, put it back on the
		 * list of usable buffers.  list_move_tail is not used here
		 * because it isn't 2.4 kernel compatable.
		 */

		spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

		list_del(cursor);
		list_add_tail(cursor,
			      (&(dm7820_device->dma_buffers_pre_transfer)
			       [fifo]));

		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);

		/*
		 * used to keep track of our insertion point in the user buffer.
		 */

		buffer_index++;

		if (buffer_index >= num_bufs) {
			break;
		}
	}

	if (buffer_index == 0) {
		printk(KERN_WARNING
		       "%s: WARNING: Read DMA - No buffers were available to be read from.",
		       &((dm7820_device->device_name)[0]));
	}
	return (status);
}

/******************************************************************************
Write to DMA for DMA/FIFO channel
 ******************************************************************************/
static int
dm7820_dma_write(dm7820_device_descriptor_t * dm7820_device,
		 dm7820_ioctl_argument_t * ioctl_argument)
{
	uint32_t status, buffer_index, num_bufs;
	unsigned long irq_flags;
#if defined(DM7820_DEBUG_DMA)
	uint32_t num_post_bufs = 0;
#endif

	dm7820_fifo_queue fifo = ioctl_argument->dma_function.fifo;
	struct list_head *cursor, *next;

	num_bufs = ioctl_argument->dma_function.transfer_size;
	status = 0;
	buffer_index = 0;

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	/*
	 * We must first check to see if the DMA has been initialized in a READ
	 * direction.  If it has, then we need to move all of the buffers
	 * from pre_transfer to post_transfer, and only this one time.
	 * This works because there is no reason to call write_dma if you are
	 * configured to read.  So calling this function tells us to configure
	 * for writing instead.
	 */
	if (dm7820_device->dma_in_read_direction[fifo]) {
		list_for_each_safe(cursor, next,
				   (&(dm7820_device->dma_buffers_pre_transfer)
				    [fifo])) {
			list_del(cursor);
			list_add_tail(cursor,
				      (&
				       (dm7820_device->
					dma_buffers_post_transfer)
[fifo]));
		}

		dm7820_device->dma_in_read_direction[fifo] = 0x00;
#if defined(DM7820_DEBUG_DMA)
		printk(KERN_DEBUG "%s: Switched buffers from read to write.\n",
		       &((dm7820_device->device_name)[0]));
#endif

	}

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);
	/*
	 * Iterate through the list of DMA buffers and copy
	 * data from the user buffer
	 */
#if defined(DM7820_DEBUG_DMA)

	list_for_each(cursor,
		      (&(dm7820_device->dma_buffers_post_transfer)[fifo])) {
		num_post_bufs++;
	}
	printk(KERN_DEBUG "Writing %d user buffers to %d dma%d buffers\n",
	       num_bufs, num_post_bufs, fifo);

#endif
	list_for_each_safe(cursor, next,
			   (&(dm7820_device->dma_buffers_post_transfer)[fifo]))
	{

		dm7820_dma_list_item_t *dma_item;
		dma_item = list_entry(cursor, dm7820_dma_list_item_t, list);

		/*
		 * This cannot be called within spinlocks, as it may sleep
		 */
		status = copy_from_user((dma_item->dma_buffer)->virtual_address,
					ioctl_argument->dma_function.
					user_buffer +
					(dm7820_device->dma_size)[fifo] *
					buffer_index,
					(dm7820_device->dma_size)[fifo]);

		if (status != 0) {
			printk(KERN_ERR "ERROR: write_dma failed!\n");
			return -EFAULT;
		}

		spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);
		list_del(cursor);
		list_add_tail(cursor,
			      (&(dm7820_device->dma_buffers_pre_transfer)
			       [fifo]));
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);

		/*
		 * used to keep track of our insertion point in the user buffer.
		 */

		buffer_index++;

		if (buffer_index >= num_bufs) {
			break;
		}
	}

	if (buffer_index == 0) {
		printk(KERN_WARNING
		       "%s: WARNING: Write DMA - No buffers were available to be written into.",
		       &((dm7820_device->device_name)[0]));
	}
	return (status);
}

/******************************************************************************
Return the physical address of a DMA buffer
 ******************************************************************************/

dma_addr_t
dm7820_get_buffer_phy_addr(dm7820_device_descriptor_t * dm7820_device,
			   dm7820_fifo_queue fifo)
{

	struct list_head *cursor;
	dm7820_dma_list_item_t *dma_item;

	cursor = &dm7820_device->dma_buffers_pre_transfer[fifo];

	/*
	 * Grab the list entry we want from the list of buffers.
	 */

	dma_item = list_entry(cursor->next, dm7820_dma_list_item_t, list);

	/*
	 * Return the physical address of the selected entry
	 */

	return (dma_item->dma_buffer)->bus_address;
}

/******************************************************************************
Initialize DMA for DMA/FIFO channel
 ******************************************************************************/

static int
dm7820_initialize_dma(dm7820_device_descriptor_t * dm7820_device,
		      dm7820_ioctl_argument_t * ioctl_argument)
{
	dm7820_fifo_queue fifo;
	uint32_t buffer;
	uint32_t buffer_count;
	uint32_t buffer_size;
	uint8_t pci_master;
	unsigned long irq_flags;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Validate user arguments
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	fifo = ioctl_argument->dma_function.fifo;

	/*
	 * DMA buffer count must be greater than zero and not more than default
	 * buffer count limit
	 */

	buffer_count =
	    ioctl_argument->dma_function.arguments.dma_init.buffer_count;

	if ((buffer_count <= 0) || (buffer_count > DM7820_MAX_DMA_BUFFER_COUNT)) {
		return -EINVAL;
	}

	/*
	 * DMA buffer size must be 1) greater than zero, 2) evenly divisible by 2,
	 * and 3) not more than default buffer size limit
	 */

	buffer_size =
	    ioctl_argument->dma_function.arguments.dma_init.buffer_size;

	if ((buffer_size <= 0)
	    || (buffer_size & 0x00000001)
	    || (buffer_size > DM7820_MAX_DMA_BUFFER_SIZE)
	    ) {
		return -EINVAL;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Device-level error checking
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Protect against interference from interrupts and other processors
	 */

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	/*
	 * To perform DMA, device must be PCI master capable
	 */

	dm7820_get_pci_master_status(dm7820_device, &pci_master);
	if (!pci_master) {
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
		return -EOPNOTSUPP;
	}

	/*
	 * Do not allow multiple initializations
	 */

	if (dm7820_device->dma_initialized[fifo]) {
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
		return -EAGAIN;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Allocate DMA buffers
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Save DMA/FIFO channel buffer size in device descriptor.  This must be
	 * done before allocating any DMA buffers because this information is used
	 * in the process of cleaning up after a failed allocation.
	 */

	dm7820_device->dma_size[fifo] = buffer_size;
	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);
	/*
	 * Allocate the requested number of DMA buffers
	 */

	for (buffer = 0; buffer < buffer_count; buffer++) {
		dm7820_dma_descriptor_t *dma_descriptor;
		dm7820_dma_list_item_t *dma_item;
		dma_addr_t bus_address;
		void *virtual_address;

		/*#####################################################################
		   DMA mapping and memory allocations
		   ################################################################## */

		/*
		 * Allocate coherent/consistent DMA mapping
		 */

		virtual_address =
		    dma_alloc_coherent(NULL, buffer_size,
				       &bus_address, (GFP_ATOMIC | __GFP_NOWARN)
		    );
		if (virtual_address == NULL) {

			dm7820_free_dma_mappings(dm7820_device, fifo);
			return -ENOMEM;
		}

		/*
		 * Allocate DMA buffer descriptor
		 */

		dma_descriptor =
		    kmalloc(sizeof(dm7820_dma_descriptor_t), GFP_ATOMIC);

		if (dma_descriptor == NULL) {
			dma_free_coherent(NULL, buffer_size,
					  virtual_address, bus_address);
			dm7820_free_dma_mappings(dm7820_device, fifo);
			return -ENOMEM;
		}

		/*
		 * Allocate device DMA buffer list item
		 */

		dma_item = kmalloc(sizeof(dm7820_dma_list_item_t), GFP_ATOMIC);

		if (dma_item == NULL) {
			kfree(dma_descriptor);
			dma_free_coherent(NULL, buffer_size,
					  virtual_address, bus_address);
			dm7820_free_dma_mappings(dm7820_device, fifo);
			return -ENOMEM;
		}

		/*@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
		   Add DMA buffer to device's DMA buffer list
		   @@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@ */

		/*
		 * Save buffer bus and virtual addresses in DMA buffer descriptor
		 */
		spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);
		dma_descriptor->virtual_address = virtual_address;
		dma_descriptor->bus_address = bus_address;

		/*
		 * Associate DMA buffer descriptor with DMA buffer list item
		 */

		dma_item->dma_buffer = dma_descriptor;

		/*
		 * Add DMA buffer list item at end of DMA buffer list
		 */

		list_add_tail(&(dma_item->list),
			      &(dm7820_device->dma_buffers_pre_transfer[fifo]));

		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
	}

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	dm7820_device->dma_initialized[fifo] = 0xFF;
	dm7820_device->dma_in_read_direction[fifo] = 0xFF;

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	return 0;
}

/******************************************************************************
DM7820 device hardware initialization
 ******************************************************************************/

static void
dm7820_initialize_hardware(const dm7820_device_descriptor_t * dm7820_device)
{
	dm7820_pci_access_request_t pci_request;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Generic device initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Tell the FPGA to reset the board
	 */

	pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
	pci_request.offset = DM7820_BAR2_BOARD_RESET;
	pci_request.size = DM7820_PCI_REGION_ACCESS_16;
	pci_request.data.data16 = DM7820_BOARD_RESET_DO_RESET;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   PLX interrupt initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Disable PLX interrupts.  Normally this function would need locking, but
	 * because we're initializing, we won't worry about concurrent access.
	 */

	dm7820_enable_plx_interrupts(dm7820_device, 0x00);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   PLX DMA initialization
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*#########################################################################
	   Set up DMA Channel 0 Mode Register
	   ###################################################################### */

	/*
	 * Read register
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_DMAMODE0;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 = 0x00000000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	/*
	 * Save reserved bits and mask everything else off
	 */

	pci_request.data.data32 &= ~0x003FFFFF;

	/*
	 * Set desired bits and write back to register
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_DMAMODE0;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 |= 0x00021DC1;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*#########################################################################
	   Set up DMA Channel 1 Mode Register
	   ###################################################################### */

	/*
	 * Read register
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_DMAMODE1;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 = 0x00000000;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	/*
	 * Save reserved bits and mask everything else off
	 */

	pci_request.data.data32 &= ~0x003FFFFF;

	/*
	 * Set desired bits and write back to register
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_DMAMODE1;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 |= 0x00021DC1;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*#########################################################################
	   Set up DMA Channel 0 Local Address Register
	   ###################################################################### */

	/*
	 * Set register to FPGA FIFO 0 PCI Read/Write Port Register offset
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_DMALADR0;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 = DM7820_BAR2_FIFO0_RW_PORT;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	/*#########################################################################
	   Set up DMA Channel 1 Local Address Register
	   ###################################################################### */

	/*
	 * Set register to FPGA FIFO 1 PCI Read/Write Port Register offset
	 */

	pci_request.region = DM7820_PCI_REGION_PLX_MEM;
	pci_request.offset = DM7820_BAR0_DMALADR1;
	pci_request.size = DM7820_PCI_REGION_ACCESS_32;
	pci_request.data.data32 = DM7820_BAR2_FIFO1_RW_PORT;

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);
}

/******************************************************************************
DM7820 interrupt handler
 ******************************************************************************/

static irqreturn_t dm7820_interrupt_handler(int irq_number, void *device_id)
{

	dm7820_device_descriptor_t *dm7820_device;
	uint16_t int_enable;
	uint16_t int_status;
	uint16_t local_disable_mask;
	uint16_t new_int_enable;
	uint16_t new_int_status;
	uint32_t plx_intcsr;
	uint8_t status_bit;
	uint8_t data8;
	uint32_t data32;

	struct list_head *cursor;
	struct list_head *next;
	dm7820_dma_list_item_t *dma_item;

	dm7820_device = (dm7820_device_descriptor_t *) device_id;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Sanity checking
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Make sure we have a valid DM7820 device descriptor
	 */

	if (dm7820_validate_device(dm7820_device) != 0) {
		printk(KERN_ERR
		       "%s: ERROR: Invalid device descriptor in interrupt\n",
		       &((dm7820_device->device_name)[0])
		    );
		return IRQ_NONE;
	}

	/*
	 * We have a valid device descriptor, so lock it up for multiprocessor
	 * protection.  Local processor interrupts are not disabled.
	 */

	spin_lock(&(dm7820_device->device_lock));

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Get PLX Interrupt Control/Status Register value and determine if the
	   device actually generated an interrupt
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read PLX Interrupt Control/Status Register
	 */

	plx_intcsr =
	    ioread32(dm7820_device->pci[DM7820_PCI_REGION_PLX_MEM].virt_addr +
		     DM7820_BAR0_INTCSR);

	/*
	 * If neither a PLX local interrupt nor a PLX DMA channel 0 nor a PLX DMA
	 * channel 1 interrupt is pending, some other device must have interrupted
	 */

	if (!(plx_intcsr & 0x00608000)) {
		spin_unlock(&(dm7820_device->device_lock));

		return IRQ_NONE;
	}

	/*
	 * Make sure kernel's interrupt line number matches the one allocated for
	 * this device
	 */

	if (irq_number != dm7820_device->irq_number) {
		spin_unlock(&(dm7820_device->device_lock));
		printk(KERN_ERR "%s: ERROR: Interrupt on wrong IRQ line\n",
		       &((dm7820_device->device_name)[0])
		    );
		return IRQ_NONE;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   ISR for DMA 0
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
	if (plx_intcsr & PLXCSR_DMA0_INT_ACTIVE) {
#if defined(DM7820_DEBUG_DMA)
		int pre_buff = 0, post_buff = 0;
#endif
		data8 =
		    ioread8(dm7820_device->pci
			    [DM7820_PCI_REGION_PLX_MEM].virt_addr +
			    DM7820_BAR0_DMACSR0);

		/*
		 * Did we just finish a transfer, and not because
		 * of a DMA abort?
		 */

		if ((data8 & DMACSR_COMPLETE_BIT)
		    && !(data8 & DMACSR_ABORT_BIT)) {

			/*
			 * We know the last DMA transfer is complete.  Prepare the next
			 * transfer.
			 */

			data32 = ioread32(dm7820_device->pci
					  [DM7820_PCI_REGION_PLX_MEM].virt_addr
					  + DM7820_BAR0_DMADPR0);

			cursor =
			    &dm7820_device->dma_buffers_pre_transfer
			    [DM7820_FIFO_QUEUE_0];

			/*
			 * Check that the buffer that was just transferred isn't gone,
			 * as that would be bad.
			 */

			if (list_empty(cursor) == 0) {
				next = cursor->next;
				list_del(next);
				list_add_tail(next,
					      &dm7820_device->dma_buffers_post_transfer
					      [DM7820_FIFO_QUEUE_0]);
			} else {

				/*
				 * This is an error, since we just finished transferring a buffer,
				 * so that buffer should still be in the pre-transfer list.
				 */
				printk(KERN_ERR "%s: ERROR: A DMA0 buffer was transferred, \
                 but is missing from the list.\n",
				       &((dm7820_device->device_name)[0]));
			}

#if defined(DM7820_DEBUG_DMA)
			list_for_each(cursor,
				      (&
				       (dm7820_device->dma_buffers_post_transfer)
[DM7820_FIFO_QUEUE_0])) {
				post_buff++;
			}
			list_for_each(cursor,
				      (&
				       (dm7820_device->dma_buffers_pre_transfer)
[DM7820_FIFO_QUEUE_0])) {
				pre_buff++;
			}

			printk(KERN_DEBUG
			       "DMA0 Transfer complete: %d buffers waiting, %d buffers completed\n",
			       pre_buff, post_buff);

#endif
			/*
			 * if there is another DMA buffer available get it ready for DMA
			 */

			if (list_empty(cursor) == 0) {

				/*
				 * Get the address of the a new empty buffer and write its
				 * physical address to the correct register
				 */

				dma_item = list_entry(cursor->next,
						      dm7820_dma_list_item_t,
						      list);

				iowrite32((dma_item->dma_buffer)->bus_address,
					  dm7820_device->pci
					  [DM7820_PCI_REGION_PLX_MEM].virt_addr
					  + DM7820_BAR0_DMAPADR0);

				/*
				 * Write DMACSRx[1] high to restart the DMA
				 */

				data8 |= DMACSR_START_BIT;
			}
		}

		/*
		 * Write DMACSRx[3] high to clear the interrupt.
		 */

		data8 |= DMACSR_CLEAR_INTERRUPT_BIT;

		/*
		 * Write to DMA Control/Status register to clear interrupt and possibly
		 * restart the next DMA transfer
		 */

		iowrite8(data8,
			 dm7820_device->pci[DM7820_PCI_REGION_PLX_MEM].
			 virt_addr + DM7820_BAR0_DMACSR0);

		dm7820_int_queue_add(dm7820_device,
				     DM7820_INTERRUPT_FIFO_0_DMA_DONE);

		spin_unlock(&(dm7820_device->device_lock));

		wake_up_interruptible(&(dm7820_device->int_wait_queue));

		return IRQ_HANDLED;
	}
	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   ISR for DMA 1
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */
	if (plx_intcsr & PLXCSR_DMA1_INT_ACTIVE) {
#if defined(DM7820_DEBUG_DMA)
		int pre_buff = 0, post_buff = 0;
#endif
		data8 =
		    ioread8(dm7820_device->pci
			    [DM7820_PCI_REGION_PLX_MEM].virt_addr +
			    DM7820_BAR0_DMACSR1);

		/*
		 * Did we just finish a transfer, and not because
		 * of a DMA abort?
		 */

		if ((data8 & DMACSR_COMPLETE_BIT)
		    && !(data8 & DMACSR_ABORT_BIT)) {

			/*
			 * We know the last DMA transfer is complete.  Prepare the next
			 * transfer.
			 */

			data32 = ioread32(dm7820_device->pci
					  [DM7820_PCI_REGION_PLX_MEM].virt_addr
					  + DM7820_BAR0_DMADPR1);

			cursor =
			    &dm7820_device->dma_buffers_pre_transfer
			    [DM7820_FIFO_QUEUE_1];

			/*
			 * Check that the buffer that was just transferred isn't gone,
			 * as that would be bad.
			 */

			if (list_empty(cursor) == 0) {
				next = cursor->next;
				list_del(next);
				list_add_tail(next,
					      &dm7820_device->dma_buffers_post_transfer
					      [DM7820_FIFO_QUEUE_1]);
			} else {

				/*
				 * This is an error, since we just finished transferring a buffer,
				 * so that buffer should still be in the pre-transfer list.
				 */
				printk(KERN_ERR "%s: ERROR: A DMA1 buffer was transferred, \
                 but is missing from the list.\n",
				       &((dm7820_device->device_name)[0]));
			}

#if defined(DM7820_DEBUG_DMA)
			list_for_each(cursor,
				      (&
				       (dm7820_device->
					dma_buffers_post_transfer)
[DM7820_FIFO_QUEUE_1])) {
				post_buff++;
			}
			list_for_each(cursor,
				      (&
				       (dm7820_device->dma_buffers_pre_transfer)
[DM7820_FIFO_QUEUE_1])) {
				pre_buff++;
			}

			printk(KERN_DEBUG
			       "DMA1 Transfer complete: %d buffers waiting, %d buffers completed\n",
			       pre_buff, post_buff);

#endif

			/*
			 * if there is another DMA buffer available get it ready for DMA
			 */

			if (list_empty(cursor) == 0) {

				/*
				 * Get the address of the a new empty buffer and write its
				 * physical address to the correct register
				 */

				dma_item = list_entry(cursor->next,
						      dm7820_dma_list_item_t,
						      list);

				iowrite32((dma_item->dma_buffer)->bus_address,
					  dm7820_device->pci
					  [DM7820_PCI_REGION_PLX_MEM].virt_addr
					  + DM7820_BAR0_DMAPADR1);

				/*
				 * Write DMACSRx[1] high to restart the DMA
				 */

				data8 |= DMACSR_START_BIT;
			}
		}

		/*
		 * Write DMACSRx[3] high to clear the interrupt.
		 */

		data8 |= DMACSR_CLEAR_INTERRUPT_BIT;

		/*
		 * Write to DMA Control/Status register to clear interrupt and possibly
		 * restart the next DMA transfer
		 */

		iowrite8(data8,
			 dm7820_device->pci[DM7820_PCI_REGION_PLX_MEM].
			 virt_addr + DM7820_BAR0_DMACSR1);

		dm7820_int_queue_add(dm7820_device,
				     DM7820_INTERRUPT_FIFO_1_DMA_DONE);

		spin_unlock(&(dm7820_device->device_lock));

		wake_up_interruptible(&(dm7820_device->int_wait_queue));

		return IRQ_HANDLED;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Get Interrupt Enable Register value to see which interrupts are enabled
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read Interrupt Enable Register
	 */

	int_enable =
	    ioread16(dm7820_device->pci[DM7820_PCI_REGION_FPGA_MEM].virt_addr +
		     DM7820_BAR2_INTERRUPT_ENABLE);

	/*
	 * Save Interrupt Enable Register value
	 */

	new_int_enable = int_enable;

	/*
	 * Mask out reserved bits
	 */

	int_enable &= ~FPGA_INTCSR_RESERVED_BITS;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Get Interrupt Status Register value to see which interrupts are pending
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Read Interrupt Status Register
	 */

	int_status =
	    ioread16(dm7820_device->pci[DM7820_PCI_REGION_FPGA_MEM].virt_addr +
		     DM7820_BAR2_INTERRUPT_STATUS);

	/*
	 * Save Interrupt Status Register value but save reserved bits and mask off
	 * all status bits
	 */

	new_int_status = (int_status & FPGA_INTCSR_RESERVED_BITS);

	/*
	 * Mask out reserved bits
	 */

	int_status &= ~FPGA_INTCSR_RESERVED_BITS;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Determine if device actually generated an interrupt
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * If no enabled interrupts are pending, some other device must have
	 * interrupted
	 */

	if (!(int_enable & int_status)) {
		spin_unlock(&(dm7820_device->device_lock));

		return IRQ_NONE;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Process each enabled and pending local interrupt
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Initially assume no interrupts will be disabled locally
	 */

	local_disable_mask = 0x0000;

	/*
	 * Iterate over all Interrupt Status Register bits
	 */

	for (status_bit = 0; status_bit < 16; status_bit++) {
		dm7820_interrupt_source *source_table;
		dm7820_interrupt_status_source_t *status_source;
		dm7820_minor_int_reg_layout_t *minor_reg;
		uint16_t clear_mask;
		uint16_t disable_mask;
		uint16_t minor_value;
		uint8_t source;

		/*#####################################################################
		   Determine which interrupts need further processing
		   ################################################################## */

		/*
		 * Only process enabled pending interrupts
		 */

		if (!(int_enable & int_status & (1 << status_bit))) {

			continue;
		}

		/*
		 * Clear interrupt in Interrupt Status Register
		 */

		new_int_status |= (1 << status_bit);

		/*
		 * No further processing needed for source with no minor interrupt
		 * register
		 */

		if (dm7820_interrupt_status_source[status_bit].minor_reg
		    == DM7820_MINOR_INT_REG_NONE) {

#if defined(DM7820_DEBUG_INTERRUPTS)

			printk(KERN_INFO "%s: Local source: %u\n",
			       &((dm7820_device->device_name)[0]),
			       dm7820_interrupt_status_source
			       [status_bit].source);

#endif

			/*
			 * Add the interrupt to the int queue for later processing
			 */

			dm7820_int_queue_add(dm7820_device,
					     dm7820_interrupt_status_source
					     [status_bit].source);
			continue;
		}

		/*#####################################################################
		   Process minor interrupt register
		   ################################################################## */

		/*
		 * Get shortcuts to status bit source and minor interrupt register
		 * information
		 */

		status_source = &(dm7820_interrupt_status_source[status_bit]);
		minor_reg =
		    &(dm7820_minor_int_reg_layout[status_source->minor_reg]);

		/*
		 * Read minor interrupt register to see which interrupts are enabled and
		 * pending
		 */

		minor_value =
		    ioread16((dm7820_device->pci
			      [DM7820_PCI_REGION_FPGA_MEM].virt_addr +
			      minor_reg->offset)
		    );

		/*
		 * Only process enabled pending minor interrupts
		 */

		if (!(((minor_value & minor_reg->enable_mask) << 8)
		      & (minor_value & minor_reg->status_mask)
		    )
		    ) {
			printk(KERN_ERR
			       "%s: ERROR: No minor enabled interrupts pending\n",
			       &((dm7820_device->device_name)[0])
			    );
			continue;
		}

		/*
		 * Initially assume no status bits to clear in minor interrupt register
		 */

		clear_mask = 0x0000;

		/*
		 * Initially assume no interrupts to disable in minor interrupt register
		 */

		disable_mask = 0x0000;

		/*
		 * Loop over all interrupt sources for the status bit
		 */

		for (source = 0, source_table = status_source->source_table;
		     source_table[source] != DM7820_INTERRUPT_NONE; source++) {
			dm7820_interrupt_control_t *int_control;

			/*
			 * Get shortcut to interrupt control information
			 */

			int_control =
			    &(dm7820_interrupt_control[source_table[source]]);

			/*
			 * Ignore any source not enabled and pending
			 */

			if (!(((minor_value & int_control->minor_enable) << 8)
			      & (minor_value & int_control->minor_status)
			    )
			    ) {

				continue;
			}
#if defined(DM7820_DEBUG_INTERRUPTS)

			printk(KERN_INFO "%s: Minor source: %u\n",
			       &((dm7820_device->device_name)[0]),
			       source_table[source]);

#endif

			/*
			 * Add the interrupt to the int queue for later processing
			 */

			dm7820_int_queue_add(dm7820_device,
					     source_table[source]);

			/*
			 * Minor interrupt register status flag needs to be cleared
			 */

			clear_mask |= int_control->minor_status;

			/*
			 * Disable any FIFO interrupt in minor interrupt register to prevent
			 * possible flooding by these interrupts
			 */

			if ((minor_reg->offset == DM7820_BAR2_FIFO0_INT)
			    || (minor_reg->offset == DM7820_BAR2_FIFO1_INT)
			    ) {
				disable_mask |= int_control->minor_enable;
			}
		}

		/*
		 * Preserve interrupt enable & reserved bits and mask off all interrupt
		 * status bits
		 */

		minor_value &=
		    (minor_reg->enable_mask | minor_reg->reserved_mask);

		/*
		 * Clear the necessary interrupt status flags
		 */

		minor_value |= clear_mask;

		/*
		 * Disable indicated interrupts; only FIFO interrupts will be disabled
		 * here
		 */

		minor_value &= ~disable_mask;

		/*
		 * If no enabled interrupts are left in minor register, disable the
		 * associated local interrupt
		 */

		if ((minor_value & minor_reg->enable_mask) == 0x0000) {
			local_disable_mask |= (1 << status_bit);
		}

		/*
		 * Write to minor interrupt register to clear the interrupts
		 */

		iowrite16(minor_value,
			  (dm7820_device->pci
			   [DM7820_PCI_REGION_FPGA_MEM].virt_addr +
			   minor_reg->offset)
		    );
	}

	/*
	 * Clear each local enabled and pending interrupt
	 */

	iowrite16(new_int_status,
		  (dm7820_device->pci[DM7820_PCI_REGION_FPGA_MEM].virt_addr +
		   DM7820_BAR2_INTERRUPT_STATUS)
	    );

	/*
	 * Disable any interrupts which should be turned off locally; only FIFO
	 * interrupts will be disabled here
	 */

	if (local_disable_mask) {
		iowrite16((new_int_enable & ~local_disable_mask),
			  (dm7820_device->pci
			   [DM7820_PCI_REGION_FPGA_MEM].virt_addr +
			   DM7820_BAR2_INTERRUPT_STATUS)
		    );

#if defined(DM7820_DEBUG_INTERRUPTS)

		printk(KERN_INFO "%s: Local disable mask: 0x%04x\n",
		       &((dm7820_device->device_name)[0]), local_disable_mask);

#endif

	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Post interrupt acknowledgment processing
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Indicate that interrupt occurred and therefore status is available
	 */

	dm7820_device->interrupt_occurred = 0xFF;

	/*
	 * All done, release the multiprocessor protection lock
	 */

	spin_unlock(&(dm7820_device->device_lock));

	/*
	 * Wake up any process waiting for an interrupt to occur
	 */

	wake_up_interruptible(&(dm7820_device->int_wait_queue));

	return IRQ_HANDLED;
}

/******************************************************************************
 Add an interrupt to the interrupt queue
 ******************************************************************************/
static void
dm7820_int_queue_add(dm7820_device_descriptor_t * dm7820_device,
		     dm7820_interrupt_source int_source)
{

	/*
	 * Make sure we have a valid int source
	 */
	if (int_source < 0 || int_source >= DM7820_INTERRUPT_NONE) {
		printk(KERN_ERR
		       "%s: Invalid or no interrupt source added to queue: %d\n",
		       &((dm7820_device->device_name)[0]), int_source);
	}

	/*
	 * This is where the information is added to the queue if there is room
	 * otherwise we indicate queue overflow and log a missed interrupt
	 */

	if (dm7820_device->int_queue_count < DM7820_INT_QUEUE_SIZE) {
		/*
		 * Collect interrupt data and store in the device structure
		 */
		dm7820_device->int_status[dm7820_device->int_queue_in] =
		    int_source;

		dm7820_device->int_queue_in++;

		if (dm7820_device->int_queue_in == (DM7820_INT_QUEUE_SIZE)) {
			/*
			 * Wrap around to the front of the queue
			 */
			dm7820_device->int_queue_in = 0;

		}

		dm7820_device->int_queue_count++;
#if defined(DM7820_DEBUG_INTERRUPTS)

		printk(KERN_DEBUG "%s: Adding interrupt: %d (Count now: %d)\n",
		       &((dm7820_device->device_name)[0]), int_source,
		       dm7820_device->int_queue_count);
#endif

	} else {
		/*
		 * Indicate interrupt status queue overflow
		 */
		printk(KERN_WARNING
		       "%s: WARNING: Missed interrupt info because queue is full\n",
		       &((dm7820_device->device_name)[0]));

		dm7820_device->int_queue_missed++;

	}
}

/******************************************************************************
Pull the next interrupt off the queue (if there is one)
This function assumes the caller has a spinlock
*******************************************************************************/
static dm7820_interrupt_info
dm7820_dequeue_interrupt(dm7820_device_descriptor_t * dm7820_device)
{
	dm7820_interrupt_info interrupt_status;

	/*
	 * If there is an interrupt in the queue then retrieve the data.
	 */

	if (dm7820_device->int_queue_count > 0) {

		/*
		 * Cache local copies of the interrupt status
		 */

		interrupt_status.source =
		    dm7820_device->int_status[dm7820_device->int_queue_out];

		/*
		 * Make copy of the calculated number of interrupts in the queue and
		 * return this value -1 to signify how many more interrupts the
		 * reading device needs to receive
		 */

		dm7820_device->int_queue_count--;
		interrupt_status.int_remaining = dm7820_device->int_queue_count;

		/*
		 * Increment the number of interrupt statuses we have sent to the user
		 */

		dm7820_device->int_queue_out++;

		if (dm7820_device->int_queue_out == (DM7820_INT_QUEUE_SIZE)) {

			/*
			 * wrap around if we have to
			 */

			dm7820_device->int_queue_out = 0;

		}
#if defined(DM7820_DEBUG_INTERRUPTS)

		printk(KERN_DEBUG
		       "%s: Removing interrupt: %d (Remaining: %d)\n",
		       &((dm7820_device->device_name)[0]),
		       interrupt_status.source, dm7820_device->int_queue_count);
#endif

	} else {

		/*
		 * Indicate that there are no interrupts in the queue
		 */

		interrupt_status.int_remaining = -1;
		interrupt_status.source = DM7820_INTERRUPT_NONE;
	}

	/*
	 * Pass back the number of missed interrupts regardless of its value
	 */

	interrupt_status.int_missed = dm7820_device->int_queue_missed;
	interrupt_status.error = 0;

	return interrupt_status;
}

/******************************************************************************
Send interrupt status information back to user
 ******************************************************************************/

static int
dm7820_get_interrupt_info(dm7820_device_descriptor_t * dm7820_device,
			  unsigned long ioctl_param)
{
	dm7820_interrupt_info interrupt_status;

	unsigned long irq_flags;

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	interrupt_status = dm7820_dequeue_interrupt(dm7820_device);

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	/*
	 * Copy the interrupt status back to user space
	 */
	if (copy_to_user((dm7820_interrupt_info *) ioctl_param,
			 &interrupt_status, sizeof(dm7820_interrupt_info))) {
		return -EFAULT;
	}

	return 0;
}

/******************************************************************************
Handle ioctl(2) system calls
 ******************************************************************************/
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static int
dm7820_ioctl(struct inode *inode,
	     struct file *file,
	     unsigned int request_code, unsigned long ioctl_param)
#else
static long
dm7820_ioctl(struct file *file, unsigned int request_code,
	     unsigned long ioctl_param)
#endif
{
	dm7820_device_descriptor_t *dm7820_device;
	int status;

	status = dm7820_validate_device(file->private_data);
	if (status != 0) {
		return status;
	}

	dm7820_device = (dm7820_device_descriptor_t *) file->private_data;

	switch (request_code) {
	case DM7820_IOCTL_REGION_READ:
		status = dm7820_read_pci_region(dm7820_device, ioctl_param);
		break;

	case DM7820_IOCTL_REGION_WRITE:
		status = dm7820_write_pci_region(dm7820_device, ioctl_param);
		break;

	case DM7820_IOCTL_REGION_MODIFY:
		status = dm7820_modify_pci_region(dm7820_device, ioctl_param);
		break;

	case DM7820_IOCTL_GET_INTERRUPT_STATUS:
		status =
		    dm7820_get_interrupt_status(dm7820_device, ioctl_param);
		break;

	case DM7820_IOCTL_DMA_FUNCTION:
		status =
		    dm7820_service_dma_function(dm7820_device, ioctl_param);
		break;

	case DM7820_IOCTL_WAKEUP:
		{
			unsigned long irq_flags;
			spin_lock_irqsave(&(dm7820_device->device_lock),
					  irq_flags);
			dm7820_device->remove_isr_flag = 0xFF;
			spin_unlock_irqrestore(&(dm7820_device->device_lock),
					       irq_flags);
			wake_up_interruptible(&(dm7820_device->int_wait_queue));
			status = 0;
			break;
		}
	case DM7820_IOCTL_INTERRUPT_INFO:
		status = dm7820_get_interrupt_info(dm7820_device, ioctl_param);
		break;

	default:
		status = -EINVAL;
		break;
	}

	return status;
}

/******************************************************************************
Initialize DM7820 driver and devices
 ******************************************************************************/

int dm7820_load(void)
{
	int status;

	printk(KERN_INFO "%s: Initializing module (version %s).\n",
	       DRIVER_NAME, DRIVER_RELEASE);

	printk(KERN_INFO "%s: %s\n", DRIVER_NAME, DRIVER_DESCRIPTION);
	printk(KERN_INFO "%s: %s\n", DRIVER_NAME, DRIVER_COPYRIGHT);

	dm7820_devices = NULL;
	dm7820_major = 0;

	status = dm7820_probe_devices(&dm7820_device_count, &dm7820_devices);
	if (status != 0) {
		return status;
	}

	status = dm7820_register_char_device(&dm7820_major);
	if (status != 0) {
		printk(KERN_ERR
		       "%s: ERROR: Dynamic character device registration FAILED (errno "
		       "= %d)\n", DRIVER_NAME, -status);
		dm7820_release_resources();
		return status;
	}

	printk(KERN_INFO
	       "%s: Driver registered using character major number %d\n",
	       DRIVER_NAME, dm7820_major);

	return 0;
}

/******************************************************************************
Read from a PCI region, modify bits in the value, and write the new value back
to the region
 ******************************************************************************/

static int
dm7820_modify_pci_region(dm7820_device_descriptor_t * dm7820_device,
			 unsigned long ioctl_param)
{
	dm7820_ioctl_argument_t ioctl_argument;
	dm7820_pci_access_request_t pci_request;
	int status;
	unsigned long irq_flags;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Copy arguments in from user space and validate them
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (copy_from_user(&ioctl_argument,
			   (dm7820_ioctl_argument_t *) ioctl_param,
			   sizeof(dm7820_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	status =
	    dm7820_validate_pci_access(dm7820_device,
				       &(ioctl_argument.modify.access)
	    );
	if (status != 0) {
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
		return status;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Do the actual read/modify/write
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Make a copy of user arguments to keep from overwriting them
	 */

	pci_request = ioctl_argument.modify.access;

	/*
	 * Read current value
	 */

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_READ);

	/*
	 * Modify the value based upon mask
	 */

	switch (ioctl_argument.modify.access.size) {
	case DM7820_PCI_REGION_ACCESS_8:

		/*
		 * Preserve bits which are not to be changed and clear bits which
		 * will be changed
		 */

		pci_request.data.data8 &= ~ioctl_argument.modify.mask.mask8;

		/*
		 * Fold in the new value but don't allow bits to be set which are
		 * are not modifiable according to the mask
		 */

		pci_request.data.data8 |=
		    (ioctl_argument.modify.access.data.data8 & ioctl_argument.
		     modify.mask.mask8);

		break;

	case DM7820_PCI_REGION_ACCESS_16:

		/*
		 * Preserve bits which are not to be changed and clear bits which
		 * will be changed
		 */

		pci_request.data.data16 &= ~ioctl_argument.modify.mask.mask16;

		/*
		 * Fold in the new value but don't allow bits to be set which are
		 * are not modifiable according to the mask
		 */

		pci_request.data.data16 |=
		    (ioctl_argument.modify.access.data.data16 & ioctl_argument.
		     modify.mask.mask16);

		break;

	case DM7820_PCI_REGION_ACCESS_32:

		/*
		 * Preserve bits which are not to be changed and clear bits which
		 * will be changed
		 */

		pci_request.data.data32 &= ~ioctl_argument.modify.mask.mask32;

		/*
		 * Fold in the new value but don't allow bits to be set which are
		 * are not modifiable according to the mask
		 */

		pci_request.data.data32 |=
		    (ioctl_argument.modify.access.data.data32 & ioctl_argument.
		     modify.mask.mask32);

		break;
	}

	/*
	 * Write new value
	 */

	dm7820_access_pci_region(dm7820_device, &pci_request,
				 DM7820_PCI_REGION_ACCESS_WRITE);

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	return 0;
}

/******************************************************************************
Open a DM7820 device
 ******************************************************************************/

static int dm7820_open(struct inode *inode, struct file *file)
{
	dm7820_device_descriptor_t *dm7820_device;
	unsigned int minor_number;
	unsigned long irq_flags;

	minor_number = iminor(inode);
	if (minor_number >= dm7820_device_count) {

		/*
		 * This will never be returned on a 2.6 kernel.  A file system layer
		 * above this one checks for invalid character device minor numbers and
		 * returns -ENXIO before this function is ever invoked.
		 */

		return -ENODEV;
	}

	dm7820_device = &(dm7820_devices[minor_number]);

	spin_lock_init(&(dm7820_device->device_lock));

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	if (dm7820_device->reference_count) {
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
		return -EBUSY;
	}

	file->private_data = dm7820_device;
	dm7820_device->reference_count++;

	dm7820_disable_all_interrupts(dm7820_device);

	/*
	 * Disable any previous DMA transfers in progress
	 */
	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	dm7820_dma_stop(dm7820_device, DM7820_FIFO_QUEUE_0);
	dm7820_dma_stop(dm7820_device, DM7820_FIFO_QUEUE_1);

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);
	/*
	 * Enable PLX interrupts
	 */

	dm7820_enable_plx_interrupts(dm7820_device, 0xFF);

	dm7820_initialize_device_descriptor(dm7820_device);

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	return 0;
}

/******************************************************************************
Determine whether or not a device is readable
 ******************************************************************************/

static unsigned int
dm7820_poll(struct file *file, struct poll_table_struct *poll_table)
{
	dm7820_device_descriptor_t *dm7820_device;
	unsigned int interrupts_in_queue;
	unsigned int status_mask = 0;
	unsigned long irq_flags;

	/*
	 * If we don't have a valid DM7820 device descriptor, no status is available
	 */

	if (dm7820_validate_device(file->private_data) != 0) {

		/*
		 * This value causes select(2) to indicate that a file descriptor is
		 * present in its file descriptor sets but it will be in the exception
		 * set rather than in the input set.
		 */

		return POLLPRI;
	}

	dm7820_device = (dm7820_device_descriptor_t *) file->private_data;

	/*
	 * Register with the file system layer so that it can wait on and check for
	 * DM7820 events
	 */

	poll_wait(file, &(dm7820_device->int_wait_queue), poll_table);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Waiting is done interruptibly, which means that a signal could have been
	   delivered.  Thus we might have been woken up by a signal before an
	   interrupt occurred.  Therefore, the process needs to examine the device's
	   interrupt flag.
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Prevent a race condition with the interrupt handler and make a local copy
	 * of the interrupt flag
	 */

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	interrupts_in_queue = dm7820_device->int_queue_count;

	if (dm7820_device->remove_isr_flag) {
		status_mask = (POLLIN | POLLRDNORM);
	}

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Interpret interrupt flag
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * The flag is cleared after reading.  See if it is clear or not.
	 */

	if (interrupts_in_queue > 0) {

		status_mask |= (POLLIN | POLLRDNORM);

	}

	return status_mask;
}

/******************************************************************************
Probe and configure all functional blocks on a device
 ******************************************************************************/

static int
dm7820_probe_device_blocks(dm7820_device_descriptor_t * dm7820_device)
{
	uint16_t offset;

	/*
	 * Iterate over all possible functional block ID register offsets
	 */

	offset = DM7820_FIRST_ID_OFFSET;

	while (offset <= DM7820_LAST_ID_OFFSET) {
		char *block_name;
		uint16_t block_id;
		uint16_t expected_id;
		uint16_t offset_increment;

		block_id =
		    ioread16(dm7820_device->pci
			     [DM7820_PCI_REGION_FPGA_MEM].virt_addr + offset);

		/*
		 * Validate block ID and associate a name with it
		 */

		switch (block_id) {
		case DM7820_ID_ADVANCED_INTERRUPT:
			block_name = "Advanced interrupt";
			break;

		case DM7820_ID_FIFO:
			block_name = "FIFO";
			break;

		case DM7820_ID_INCREMENTAL_ENCODER:
			block_name = "Incremental encoder";
			break;

		case DM7820_ID_PROGRAMMABLE_CLOCK:
			block_name = "Programmable clock";
			break;

		case DM7820_ID_PULSE_WIDTH_MODULATOR:
			block_name = "Pulse width modulator";
			break;

		case DM7820_ID_TIMER_COUNTER:
			block_name = "Timer/counter";
			break;

		case DM7820_ID_NONE:
			block_name = "No";
			break;

		default:
			printk(KERN_ERR
			       "%s: ERROR: Unknown block ID %#x at offset %#x\n",
			       &((dm7820_device->device_name)[0]), block_id,
			       offset);
			return -EIO;
			break;
		}

		/*
		 * Validate block offset, indicate what block ID should have been read,
		 * and set offset increment for next block ID register
		 */

		switch (offset) {
		case 0x0080:
			expected_id = DM7820_ID_TIMER_COUNTER;
			offset_increment = 0x0040;
			break;

		case 0x00C0:
			expected_id = DM7820_ID_FIFO;
			offset_increment = 0x0010;
			break;

		case 0x00D0:
			expected_id = DM7820_ID_FIFO;
			offset_increment = 0x0030;
			break;

		case 0x0100:
			expected_id = DM7820_ID_PROGRAMMABLE_CLOCK;
			offset_increment = 0x0040;
			break;

		case 0x0140:
			expected_id = DM7820_ID_PROGRAMMABLE_CLOCK;
			offset_increment = 0x0040;
			break;

		case 0x0180:
			expected_id = DM7820_ID_PROGRAMMABLE_CLOCK;
			offset_increment = 0x0040;
			break;

		case 0x01C0:
			expected_id = DM7820_ID_PROGRAMMABLE_CLOCK;
			offset_increment = 0x0040;
			break;

		case 0x0200:
			expected_id = DM7820_ID_ADVANCED_INTERRUPT;
			offset_increment = 0x0040;
			break;

		case 0x0240:
			expected_id = DM7820_ID_ADVANCED_INTERRUPT;
			offset_increment = 0x0040;
			break;

		case 0x0280:
			expected_id = DM7820_ID_INCREMENTAL_ENCODER;
			offset_increment = 0x0040;
			break;

		case 0x02C0:
			expected_id = DM7820_ID_INCREMENTAL_ENCODER;
			offset_increment = 0x0040;
			break;

		case 0x0300:
			expected_id = DM7820_ID_PULSE_WIDTH_MODULATOR;
			offset_increment = 0x0040;
			break;

		case 0x0340:
			expected_id = DM7820_ID_PULSE_WIDTH_MODULATOR;
			offset_increment = 0x0040;
			break;

		case 0x0380:
			expected_id = DM7820_ID_NONE;
			offset_increment = 0x0040;
			break;

		case 0x03C0:
			expected_id = DM7820_ID_NONE;
			offset_increment = 0x0040;
			break;

		default:
			printk(KERN_ERR "%s: ERROR: Invalid block offset %#x\n",
			       &((dm7820_device->device_name)[0]), offset);
			return -EBADSLT;
		}

		/*
		 * Verify the correct block ID was read
		 */

		if (block_id != expected_id) {
			printk(KERN_ERR
			       "%s: ERROR: Unexpected block ID %#x at offset %#x\n",
			       &((dm7820_device->device_name)[0]), block_id,
			       offset);
			return -EIO;
		}

		printk(KERN_INFO "%s: %s block at offset %#x\n",
		       &((dm7820_device->device_name)[0]), block_name, offset);

		offset += offset_increment;
	}

	return 0;
}

/******************************************************************************
Probe and configure all DM7820 devices
 ******************************************************************************/

static int
dm7820_probe_devices(uint32_t * device_count,
		     dm7820_device_descriptor_t ** device_descriptors)
{
	struct pci_dev *pci_device;
	uint32_t minor_number;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Count the number of supported devices in the kernel's PCI device list
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	*device_count = 0;
	pci_device = NULL;

	while ((pci_device = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_device))
	       != NULL) {

		if (pci_match_id(dm7820_pci_device_table, pci_device) == NULL) {

			continue;
		}

		(*device_count)++;
	}

	if (*device_count == 0) {
		printk(KERN_ERR "%s: ERROR: No devices found\n", DRIVER_NAME);
		return -ENODEV;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Allocate memory for the device descriptors
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	*device_descriptors = kmalloc((*device_count *
				       sizeof(dm7820_device_descriptor_t)),
				      GFP_KERNEL);
	if (*device_descriptors == NULL) {
		printk(KERN_ERR
		       "%s: ERROR: Device descriptor memory allocation FAILED\n",
		       DRIVER_NAME);
		return -ENOMEM;
	}

	memset(*device_descriptors,
	       0, (*device_count * sizeof(dm7820_device_descriptor_t))
	    );

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Set up all DM7820 devices
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	pci_device = NULL;
	minor_number = 0;

	while ((pci_device = pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_device))
	       != NULL) {
		dm7820_device_descriptor_t *dm7820_device;
		dm7820_pci_access_request_t pci_request;
		int status;
		uint8_t pci_master;

		/*
		 * See if the current PCI device is in the table of devices supported
		 * by the driver.  If not, just ignore it and go to next PCI device.
		 */

		if (pci_match_id(dm7820_pci_device_table, pci_device) == NULL) {

			continue;
		}

		printk(KERN_INFO
		       "%s: Minor %u: DM7820 found at bus %u, slot %02X, function %02X\n",
		       DRIVER_NAME,
		       minor_number,
		       pci_device->bus->number,
		       PCI_SLOT(pci_device->devfn), PCI_FUNC(pci_device->devfn)
		    );

		dm7820_device = &((*device_descriptors)[minor_number]);

		spin_lock_init(&(dm7820_device->device_lock));

		dm7820_initialize_device_descriptor(dm7820_device);

		/*
		 * Create the full device name
		 */

		status = snprintf(&((dm7820_device->device_name)[0]),
				  DM7820_DEVICE_NAME_LENGTH,
				  "%s-%u", DRIVER_NAME, minor_number);
		if (status >= DM7820_DEVICE_NAME_LENGTH) {
			printk(KERN_ERR
			       "%s-%u> ERROR: Device name creation FAILED\n",
			       DRIVER_NAME, minor_number);
			dm7820_release_resources();
			pci_dev_put(pci_device);
			return -ENAMETOOLONG;
		}

		status = pci_enable_device(pci_device);

		if (status < 0) {

			pci_dev_put(pci_device);

		}

		/*
		 * Determine 1) how many standard PCI regions are present, 2) how the
		 * regions are mapped, and 3) how many bytes are in each region.  Also,
		 * remap any memory-mapped region into the kernel's address space.
		 */

		status = dm7820_process_pci_regions(dm7820_device, pci_device);
		if (status != 0) {
			pci_dev_put(pci_device);
			return status;
		}

		/*
		 * Associate device IRQ line with device in kernel
		 */

		status = dm7820_allocate_irq(dm7820_device, pci_device);
		if (status != 0) {
			pci_dev_put(pci_device);
			return status;
		}

		/*
		 * Determine which functional blocks are present on the device and how
		 * they are layed out
		 */

		status = dm7820_probe_device_blocks(dm7820_device);
		if (status != 0) {
			dm7820_release_resources();
			pci_dev_put(pci_device);
			return status;
		}

		/*
		 * Read and print FPGA version information
		 */

		pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
		pci_request.offset = DM7820_BAR2_FPGA_VERSION;
		pci_request.size = DM7820_PCI_REGION_ACCESS_16;

		dm7820_access_pci_region(dm7820_device, &pci_request,
					 DM7820_PCI_REGION_ACCESS_READ);

		printk(KERN_INFO "%s: FPGA version: %#x\n",
		       &((dm7820_device->device_name)[0]),
		       pci_request.data.data16);

		/*
		 * Read and print source code revision control version information
		 */

		pci_request.region = DM7820_PCI_REGION_FPGA_MEM;
		pci_request.offset = DM7820_BAR2_SVN_VERSION;
		pci_request.size = DM7820_PCI_REGION_ACCESS_16;

		dm7820_access_pci_region(dm7820_device, &pci_request,
					 DM7820_PCI_REGION_ACCESS_READ);

		printk(KERN_INFO "%s: Svn version: %#x\n",
		       &((dm7820_device->device_name)[0]),
		       pci_request.data.data16);

		/*
		 * Print a warning message if device is not PCI master capable
		 */

		dm7820_get_pci_master_status(dm7820_device, &pci_master);
		if (!pci_master) {
			printk(KERN_WARNING
			       "%s: WARNING: Device does not support DMA\n",
			       &((dm7820_device->device_name)[0])
			    );
		}

		dm7820_initialize_hardware(dm7820_device);

		pci_set_master(pci_device);

		minor_number++;
	}

	printk(KERN_INFO "%s: Found %u DM7820 device(s)\n", DRIVER_NAME,
	       *device_count);

	return 0;
}

/******************************************************************************
Set up standard PCI regions
 ******************************************************************************/

static int
dm7820_process_pci_regions(dm7820_device_descriptor_t * dm7820_device,
			   const struct pci_dev *pci_device)
{
	uint8_t region;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Process each standard PCI region
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	for (region = 0; region < DM7820_PCI_REGIONS; region++) {
		unsigned long address = 0;
		unsigned long length;
		unsigned long flags;

		/*#####################################################################
		   Get region's physical address and length in bytes.  If either is zero,
		   the region is unused and should be ignored.
		   ################################################################## */

		address = pci_resource_start(pci_device, region);
		if (address == 0) {
			continue;
		}

		length = pci_resource_len(pci_device, region);
		if (length == 0) {
			continue;
		}

		/*#####################################################################
		   Save information in PCI region descriptor
		   ################################################################## */

		dm7820_device->pci[region].phys_addr = address;
		dm7820_device->pci[region].length = length;

		/*#####################################################################
		   Determine how the region is mapped
		   ################################################################## */

		flags = pci_resource_flags(pci_device, region);

		if (flags & IORESOURCE_IO) {

			/*
			 * The region is I/O mapped
			 */

			/*
			 * Allocate the I/O port range
			 */

			if (request_region
			    (address, length, &((dm7820_device->device_name)[0])
			    )
			    == NULL) {
				printk(KERN_ERR
				       "%s: ERROR: I/O port range %#lx-%#lx allocation FAILED\n",
				       &((dm7820_device->device_name)[0]),
				       address, (address + length - 1)
				    );
				dm7820_release_resources();
				return -EBUSY;
			}

			dm7820_device->pci[region].io_addr = address;

			printk(KERN_INFO
			       "%s: Allocated I/O port range %#lx-%#lx\n",
			       &((dm7820_device->device_name)[0]), address,
			       (address + length - 1)
			    );
		} else if (flags & IORESOURCE_MEM) {

			/*
			 * The region is memory mapped
			 */

			/*
			 * Remap the region's physical address into the kernel's virtual
			 * address space and allocate the memory range
			 */

			dm7820_device->pci[region].virt_addr =
			    ioremap_nocache(address, length);
			if (dm7820_device->pci[region].virt_addr == NULL) {
				printk(KERN_ERR
				       "%s: ERROR: BAR%u remapping FAILED\n",
				       &((dm7820_device->device_name)[0]),
				       region);
				dm7820_release_resources();
				return -ENOMEM;
			}

			if (request_mem_region
			    (address, length, &((dm7820_device->device_name)[0])
			    )
			    == NULL) {
				printk(KERN_ERR
				       "%s: ERROR: I/O memory range %#lx-%#lx allocation FAILED\n",
				       &((dm7820_device->device_name)[0]),
				       address, (address + length - 1)
				    );
				dm7820_release_resources();
				return -EBUSY;
			}

			dm7820_device->pci[region].allocated = 0xFF;

			printk(KERN_INFO
			       "%s: Allocated I/O memory range %#lx-%#lx\n",
			       &((dm7820_device->device_name)[0]), address,
			       (address + length - 1)
			    );
		} else {

			/*
			 * The region has invalid resource flags
			 */

			printk(KERN_ERR "%s: ERROR: Invalid PCI region flags\n",
			       &((dm7820_device->device_name)[0])
			    );
			dm7820_release_resources();
			return -EIO;
		}

		/*#####################################################################
		   Print information about the region
		   ################################################################## */

		printk(KERN_INFO "%s: BAR%u Region:\n",
		       &((dm7820_device->device_name)[0]), region);

		if (dm7820_device->pci[region].io_addr != 0) {
			printk(KERN_INFO "    Address: %#lx (I/O mapped)\n",
			       dm7820_device->pci[region].io_addr);
		} else {
			printk(KERN_INFO "    Address: %#lx (memory mapped)\n",
			       (unsigned long)dm7820_device->pci[region].
			       virt_addr);
			printk(KERN_INFO "    Address: %#lx (physical)\n",
			       dm7820_device->pci[region].phys_addr);
		}

		printk(KERN_INFO "    Length:  %#lx\n",
		       dm7820_device->pci[region].length);
	}

	return 0;
}

/******************************************************************************
Read from a PCI region
 ******************************************************************************/

static int
dm7820_read_pci_region(dm7820_device_descriptor_t * dm7820_device,
		       unsigned long ioctl_param)
{
	dm7820_ioctl_argument_t ioctl_argument;
	int status;
	unsigned long irq_flags;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Copy arguments in from user space and validate them
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (copy_from_user(&ioctl_argument,
			   (dm7820_ioctl_argument_t *) ioctl_param,
			   sizeof(dm7820_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	status =
	    dm7820_validate_pci_access(dm7820_device,
				       &(ioctl_argument.readwrite.access)
	    );
	if (status != 0) {
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
		return status;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Do the actual read
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	dm7820_access_pci_region(dm7820_device,
				 &(ioctl_argument.readwrite.access),
				 DM7820_PCI_REGION_ACCESS_READ);

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Copy results back to user space
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (copy_to_user((dm7820_ioctl_argument_t *) ioctl_param,
			 &ioctl_argument, sizeof(dm7820_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	return 0;
}

/******************************************************************************
Register DM7820 character device with kernel
 ******************************************************************************/

static int dm7820_register_char_device(int *major)
{
	dev_t device, devno;
	int status;
	struct device *dev = NULL;
	char dev_file_name[30];
	int minor = 0;

	status =
	    alloc_chrdev_region(&device, 0, dm7820_device_count, DRIVER_NAME);
	if (status < 0) {
		return status;
	}

	cdev_init(&dm7820_cdev, &dm7820_file_ops);
	dm7820_cdev.owner = THIS_MODULE;

	status = cdev_add(&dm7820_cdev, device, dm7820_device_count);
	if (status < 0) {
		unregister_chrdev_region(device, dm7820_device_count);
		return status;
	}

	*major = MAJOR(device);

	dev_class = class_create(THIS_MODULE, DRIVER_NAME);

	if (dev_class == NULL) {
		unregister_chrdev_region(device, dm7820_device_count);
		return -ENODEV;
	}

	for (minor = 0; minor < dm7820_device_count; minor++) {
		sprintf(dev_file_name, "%s-%u", DRIVER_NAME, minor);
		devno = MKDEV(*major, minor);
		dev = device_create(dev_class,
				    NULL, devno, NULL, dev_file_name, 0);

		if (dev == NULL) {
			return -ENODEV;
		}
		printk(KERN_INFO "%s: Created device file %s", DRIVER_NAME,
		       dev_file_name);
	}

	return 0;
}

/******************************************************************************
Close a DM7820 device file
 ******************************************************************************/

static int dm7820_release(struct inode *inode, struct file *file)
{
	dm7820_device_descriptor_t *dm7820_device;
	int status;
	unsigned long irq_flags;

	status = dm7820_validate_device(file->private_data);
	if (status != 0) {
		return status;
	}

	dm7820_device = (dm7820_device_descriptor_t *) file->private_data;

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	dm7820_device->reference_count--;
	file->private_data = NULL;

	/*
	 * Disable PLX interrupts
	 */

	dm7820_enable_plx_interrupts(dm7820_device, 0x00);

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	dm7820_dma_abort(dm7820_device, DM7820_FIFO_QUEUE_0);
	dm7820_dma_abort(dm7820_device, DM7820_FIFO_QUEUE_1);

	dm7820_free_dma_mappings(dm7820_device, DM7820_FIFO_QUEUE_0);
	dm7820_free_dma_mappings(dm7820_device, DM7820_FIFO_QUEUE_1);

	return 0;
}

/******************************************************************************
Release resources allocated by driver
 ******************************************************************************/

static void dm7820_release_resources(void)
{
	uint32_t minor_number;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Release device-level resources
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	for (minor_number = 0; minor_number < dm7820_device_count;
	     minor_number++) {
		dm7820_device_descriptor_t *dm7820_device;
		uint8_t region;

		dm7820_device = &((dm7820_devices)[minor_number]);

		/*
		 * Free any allocated IRQ
		 */

		if (dm7820_device->irq_number != 0) {
			free_irq(dm7820_device->irq_number, dm7820_device);
			printk(KERN_INFO "%s: Freed IRQ %u\n",
			       &((dm7820_device->device_name)[0]),
			       dm7820_device->irq_number);
		}

		/*
		 * Free any resources allocated for the PCI regions
		 */

		for (region = 0; region < DM7820_PCI_REGIONS; region++) {

			/*
			 * Determine how region is mapped
			 */

			if (dm7820_device->pci[region].virt_addr != NULL) {

				/*
				 * Region is memory-mapped
				 */

				/*
				 * If memory range allocation succeeded, free the range
				 */

				if (dm7820_device->pci[region].allocated !=
				    0x00) {
					release_mem_region(dm7820_device->pci
							   [region].phys_addr,
							   dm7820_device->pci
							   [region].length);

					printk(KERN_INFO
					       "%s: Released I/O memory range %#lx-%#lx\n",
					       &((dm7820_device->device_name)
						 [0]), (unsigned long)
					       dm7820_device->
					       pci[region].phys_addr,
					       ((unsigned long)
						dm7820_device->pci[region].
						phys_addr +
						dm7820_device->pci[region].
						length - 1)
					    );
				}

				/*
				 * Unmap region from kernel's address space
				 */

				iounmap(dm7820_device->pci[region].virt_addr);

				printk(KERN_INFO
				       "%s: Unmapped kernel mapping at %#lx\n",
				       &((dm7820_device->device_name)[0]),
				       (unsigned long)
				       dm7820_device->pci[region].virt_addr);
			} else if (dm7820_device->pci[region].io_addr != 0) {

				/*
				 * Region is I/O-mapped
				 */

				/*
				 * Free I/O port range
				 */

				release_region(dm7820_device->pci[region].
					       phys_addr,
					       dm7820_device->pci[region].
					       length);

				printk(KERN_INFO
				       "%s: Released I/O port range %#lx-%#lx\n",
				       &((dm7820_device->device_name)[0]),
				       (unsigned long)
				       dm7820_device->pci[region].phys_addr,
				       ((unsigned long)
					dm7820_device->pci[region].phys_addr +
					dm7820_device->pci[region].length - 1)
				    );
			}
		}
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Release driver-level resources
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Free device descriptor memory after all references to it are finished
	 */

	kfree(dm7820_devices);
}

/******************************************************************************
Service DMA function request
 ******************************************************************************/

static int
dm7820_service_dma_function(dm7820_device_descriptor_t * dm7820_device,
			    unsigned long ioctl_param)
{
	dm7820_ioctl_argument_t ioctl_argument;
	int status = 0;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Copy arguments in from user space and validate them
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (copy_from_user(&ioctl_argument,
			   (dm7820_ioctl_argument_t *) ioctl_param,
			   sizeof(dm7820_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	switch (ioctl_argument.dma_function.fifo) {
	case DM7820_FIFO_QUEUE_0:
	case DM7820_FIFO_QUEUE_1:
		break;

	default:
		return -EINVAL;
		break;
	}

	switch (ioctl_argument.dma_function.function) {
	case DM7820_DMA_FUNCTION_INITIALIZE:
		status = dm7820_initialize_dma(dm7820_device, &ioctl_argument);
		break;

	case DM7820_DMA_FUNCTION_STOP:
		status = dm7820_dma_stop(dm7820_device,
					 ioctl_argument.dma_function.fifo);
		break;

	case DM7820_DMA_FUNCTION_READ:
		status = dm7820_dma_read(dm7820_device, &ioctl_argument);
		break;

	case DM7820_DMA_FUNCTION_WRITE:
		status = dm7820_dma_write(dm7820_device, &ioctl_argument);
		break;

	case DM7820_DMA_GET_BUFFER_ADDR:
		status = dm7820_get_buffer_phy_addr(dm7820_device,
						    ioctl_argument.dma_function.
						    fifo);
		break;

	default:
		status = -EINVAL;
		break;
	}

	return status;
}

/******************************************************************************
Deinitialize DM7820 driver and devices
 ******************************************************************************/

void dm7820_unload(void)
{
	int status;
	struct pci_dev *pci_device = NULL;

	dm7820_release_resources();

	while ((pci_device =
		pci_get_device(PCI_ANY_ID, PCI_ANY_ID, pci_device)) != NULL) {

		/*
		 * See if the current PCI device is in the table of devices supported
		 * by the driver.  If not, just ignore it and go to next PCI device.
		 */

		if (pci_match_id(dm7820_pci_device_table, pci_device) == NULL) {

			continue;
		}

		pci_clear_master(pci_device);

	}

	status = dm7820_unregister_char_device();
	if (status != 0) {
		printk(KERN_ERR
		       "%s: ERROR: Character device unregistration FAILED (errno "
		       " = %d)!\n", DRIVER_NAME, -status);
		printk(KERN_ERR
		       "%s: ERROR: A system reboot should be performed\n",
		       DRIVER_NAME);
	}

	printk(KERN_INFO "%s: Character device %d unregistered\n",
	       DRIVER_NAME, dm7820_major);

	printk(KERN_INFO "%s: Module unloaded.\n", DRIVER_NAME);
}

/******************************************************************************
Unregister DM7820 character device with kernel
 ******************************************************************************/

static int dm7820_unregister_char_device(void)
{
	unsigned int minor;

	cdev_del(&dm7820_cdev);

	for (minor = 0; minor < dm7820_device_count; minor++) {
		device_destroy(dev_class, MKDEV(dm7820_major, minor));

	}

	class_unregister(dev_class);

	class_destroy(dev_class);

	unregister_chrdev_region(MKDEV(dm7820_major, 0), dm7820_device_count);
	return 0;
}

/******************************************************************************
Validate a DM7820 device descriptor
 ******************************************************************************/

static int
dm7820_validate_device(const dm7820_device_descriptor_t * dm7820_device)
{
	uint32_t minor_number;

	for (minor_number = 0; minor_number < dm7820_device_count;
	     minor_number++) {
		if (dm7820_device == &((dm7820_devices)[minor_number])) {
			return 0;
		}
	}

	return -EBADFD;
}

/******************************************************************************
Validate user-space PCI region access
 ******************************************************************************/

static int
dm7820_validate_pci_access(const dm7820_device_descriptor_t * dm7820_device,
			   const dm7820_pci_access_request_t * pci_request)
{
	uint16_t align_mask;
	uint8_t access_bytes;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Validate the data size
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * Verify the data size in bits.  Set the number of bytes being accessed;
	 * this is used to determine whether or not the region offset actually lies
	 * within the region.  Set the offset alignment bit mask; this is used to
	 * determine whether the region offset is suitably aligned for the access.
	 */

	switch (pci_request->size) {
	case DM7820_PCI_REGION_ACCESS_8:
		access_bytes = 1;
		align_mask = 0x0;
		break;

	case DM7820_PCI_REGION_ACCESS_16:
		access_bytes = 2;
		align_mask = 0x1;
		break;

	case DM7820_PCI_REGION_ACCESS_32:
		access_bytes = 4;
		align_mask = 0x3;
		break;

	default:
		return -EMSGSIZE;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Validate the PCI region
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	switch (pci_request->region) {
	case DM7820_PCI_REGION_PLX_MEM:
	case DM7820_PCI_REGION_PLX_IO:
	case DM7820_PCI_REGION_FPGA_MEM:
		break;

	default:
		return -EINVAL;
		break;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Validate the PCI region offset
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	/*
	 * All bytes being accessed must lie entirely within the PCI region
	 */

	if (pci_request->offset
	    > (dm7820_device->pci[pci_request->region].length - access_bytes)
	    ) {
		return -ERANGE;
	}

	/*
	 * Offset where access will occur must be suitably aligned for the size of
	 * access
	 */

	if (pci_request->offset & align_mask) {
		return -EOPNOTSUPP;
	}

	return 0;
}

/******************************************************************************
Write to a PCI region
 ******************************************************************************/

static int
dm7820_write_pci_region(dm7820_device_descriptor_t * dm7820_device,
			unsigned long ioctl_param)
{
	dm7820_ioctl_argument_t ioctl_argument;
	int status;
	unsigned long irq_flags;

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Copy arguments in from user space and validate them
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	if (copy_from_user(&ioctl_argument,
			   (dm7820_ioctl_argument_t *) ioctl_param,
			   sizeof(dm7820_ioctl_argument_t)
	    )
	    ) {
		return -EFAULT;
	}

	spin_lock_irqsave(&(dm7820_device->device_lock), irq_flags);

	status =
	    dm7820_validate_pci_access(dm7820_device,
				       &(ioctl_argument.readwrite.access)
	    );
	if (status != 0) {
		spin_unlock_irqrestore(&(dm7820_device->device_lock),
				       irq_flags);
		return status;
	}

	/*%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	   Do the actual write
	   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% */

	dm7820_access_pci_region(dm7820_device,
				 &(ioctl_argument.readwrite.access),
				 DM7820_PCI_REGION_ACCESS_WRITE);

	spin_unlock_irqrestore(&(dm7820_device->device_lock), irq_flags);

	return 0;
}

/*=============================================================================
Module entry point definitions
 =============================================================================*/

module_init(dm7820_load);
module_exit(dm7820_unload);

/*=============================================================================
Operations supported on DM7820 device files.

NOTE:
        The individual structures in this array are set up using ANSI C
        standard format initialization (which is the preferred method in 2.6
        kernels) instead of tagged initialization (which is the preferred
        method in 2.4 kernels).
 =============================================================================*/

static struct file_operations dm7820_file_ops = {
	.owner = THIS_MODULE,
	.poll = dm7820_poll,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
	.ioctl = dm7820_ioctl,
#else
	.unlocked_ioctl = dm7820_ioctl,
#endif
	.open = dm7820_open,
	.release = dm7820_release,
};
