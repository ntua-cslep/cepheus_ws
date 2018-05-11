Linux Software (DM7820/DM9820)

SWP-700010101 rev D

Version v03.00.00.89870

Copyright (C) RTD Embedded Technologies, Inc.  All Rights Reserved.

This software is licensed as described in the RTD End-User Software License
Agreement.  For a copy of this agreement, refer to the file LICENSE.TXT (which
should be included with this software) or contact RTD Embedded Technologies,
Inc.



------------------
Supported Hardware
------------------

This software supports the RTD DM7820, DM9820, and DM35820 dataModules.



-------------------------
Supported Kernel Versions
-------------------------

This software has been tested with the following Linux distributions and kernel
versions:

    * openSUSE 11.4 (unmodified kernel 2.6.37)
    * openSUSE 13.1 (unmodified kernel 3.11)
    * openSUSE 13.1 (kernel 3.14 from kernel.org)

 Due to API differences between kernel versions, RTD cannot guarantee compatibility
 with kernels and distributions not listed above.  If a user wishes to use an 
 unsupported kernel/distribution, it may be necessary to modify the driver module 
 code and/or Makefiles for the specific Linux environment.
 
--------------------------
Supported CPU Architecture
--------------------------
  
 This software has been validated on the following CPU architectures.
 
    * i586 (32-bit) single-core
    * i586 (32-bit) dual-core
    * x86-64 (64-bit) single-core
    * x86-64 (64-bit) dual-core 


-------------------
Supported Compilers
-------------------

The driver software and example programs were compiled using the GNU gcc 
compiler under openSUSE 11.4 and openSUSE 13.1



------
Driver
------

The directory driver/ contains source code related to the driver.


In order to use a driver, one must first compile it, load it into the kernel,
and create device files for the board(s).  To do this, issue the following
commands while sitting in the driver/ directory:

    * make
    * make load


The driver module must be loaded before running any program which accesses a
DM7820 device.



-----------------
Library Interface
-----------------

The directory lib/ contains source code related to the user library.


The DM7820 library is created with a file name of librtd-dm7820.a and is
statically linked.


Please refer to the software manual for details on using the user level library
functions.  These functions are prototyped in the file include/dm7820_library.h;
this header file must be included in any code which wishes to call library
functions.


The library must be built before compiling the example programs or your
application.


To build the library, issue the command "make" within lib/.



----------------
Example Programs
----------------

The directory examples/ contains source code related to the example programs,
which demonstrate how to use features of the DM7820 boards, test the driver, or
test the library.  In addition to source files, examples/ holds other files as
well; the purpose of these files will be explained below.


The following files are provided in examples/:

    * Makefile
	    Make description file for building example programs.
    * advint_event_interrupt.c
	    Example program which demonstrates how to use advanced interrupt block
	    event mode interrupts.
    * advint_match_interrupt.c
	    Example program which demonstrates how to use advanced interrupt block
	    match mode interrupts.
    * advint_status.c
	    Example program which demonstrates how to get advanced interrupt status
	    when not using interrupts.
    * advint_strobe_interrupt.c
	    Example program which demonstrates how to use advanced interrupt block
	    strobe mode interrupts.
    * basic_test.c
	    Program which tests the basic functionality of the driver.
    * brdctl_device_info.c
	    Example program which demonstrates how to obtain version information and
	    PCI master capable status from a device.
    * dma_capture_buffers.c
        Example program which demonstrates the ability to continuously capture
        DMA data to large user buffers (16MB).
    * dma_capture_file.c
        Example program which demonstrates the ability to continuously capture
        DMA data to a file.
    * fifo_cascaded.c
	    Example program which demonstrates how to use the FIFO block FIFOs such
	    that the output of one FIFO serves as the input of another FIFO.
    * fifo_interrupt.c
	    Example program which demonstrates how to use the FIFO block interrupts.
    * fifo_pci_access.c
	    Example program which demonstrates how to use the FIFO block FIFOs with
	    PCI read requests clocking data out of the FIFO and PCI write requests
	    clocking data into the FIFO.
    * incenc_encoders.c
	    Example program which demonstrates how to use the incremental encoder
	    block encoders.
    * incenc_interrupt.c
	    Example program which demonstrates how to use the incremental encoder
	    block interrupts.
    * incenc_status.c
	    Example program which demonstrates how to get incremental encoder status
	    when not using interrupts.
    * library_test.c
        Program which tests the basic functionality of the user library.        
    * loopback.c
    	Example program which demonstrates how to execute a FIFO loopback with DMA.
    * prgclk_clocks.c
	    Example program which demonstrates how to use the programmable clock
	    block clocks.
    * pwm_interrupt.c
	    Example program which demonstrates how to use the pulse width modulator
	    block interrupts.
    * pwm_measure.c
        Example program which demonstrates how an incoming pulse width can be measured
        using the DM7820.        
    * pwm_modulators.c
	    Example program which demonstrates how to use the pulse width modulator
	    block modulators.
    * stdio_digital_io.c
	    Example program which demonstrates how to use the standard I/O block
	    digital I/O.
    * stdio_peripheral_output.c
	    Example program which demonstrates how to select peripheral outputs for
	    standard I/O block digital I/O ports.
    * stdio_strobe_signal.c
	    Example program which demonstrates how to use the standard I/O block
	    strobe signals.
    * tmrctr_interrupt.c
	    Example program which demonstrates how to use 8254 timer/counter block
	    timer interrupts.
    * tmrctr_status.c
	    Example program which demonstrates how to get 8254 timer/counter block
	    timer status when not using interrupts.
    * tmrctr_timers.c
	    Example program which demonstrates how to use the 8254 timer/counter
	    block timers.


To build the example programs, issue the command "make" within examples/.



------------
Header Files
------------

The directory include/ contains all header files needed by the driver, example
programs, library, and user applications.



-------------
Driver Design
-------------



        +------------------+        +----------------------------+
        | Example Programs |   OR   | User-Developed Application |
        | Provided by RTD  |        |                            |
        +------------------+        +----------------------------+
                 |                                |
                 |                                |
                 +--------------------------------+
                               |
                               | High-level C Functions
                               |
                          +---------+
                          | Library |
                          +---------+
                               |
                               | Low-level IOCTLs
    User Mode                  |
    ==========================================================================
    Kernel Mode                |
                           +--------+
                           | Driver |
                           +--------+
                               |
                               | Kernel Services
                               |
    ==========================================================================
                            Hardware



-----------------
Known Limitations
-----------------

 1. This software was tested only on little-endian processors.  If you are using
    a big-endian CPU, you will need to examine the driver, example, and library
    source code for endianness issues and resolve them.

 2. Many conditions affect board throughput and interrupt performance.  For a
    discussion of these issues, please see the Application Note SWM-640000021
    (Linux Interrupt Performance) available on our web site.

 3. If you are using the interrupt wait mechanism, be aware that signals
    delivered to the application can cause the sleep to awaken prematurely.
    Interrupts may be missed if signals are delivered rapidly enough or at
    inopportune times.

 4. Be aware that DM7820_General_Get_Interrupt_Status() disables all interrupts
    for a very brief time to obtain accurate status information.  If you call
    this function repeatedly in a loop (such as when busy-waiting for an
    interrupt to occur), this can interfere with system interrupts.  It is
    strongly suggested that you do not busy-wait for interrupts.

 5. The driver's interrupt handler disables each FIFO interrupt after it occurs
    to prevent possible flooding by these interrupts.  If you are using a FIFO
    interrupt, it must be reenabled before it can be utilized again.

 6. User applications cannot control the DMA 0 and 1 done interrupts.  The
    driver manages these interrupts to ensure proper DMA operation.
    
 7. The driver can be configured to use fairly large amounts of memory for DMA.
    This memory must be contiguous for DMA operations and as such can be a hard
    to come by.  Systems that have been running for a while tend to have
    fragmented memory and can increase the chance of DMA memory allocation to
    result in a failure.  The system should have at least 128MB to better ensure
    larger DMA buffer allocation.


-------------------------
Getting Technical Support
-------------------------

If you require additional support with this product, or any other products from
RTD Embedded Technologies, contact us using the information below:

	RTD Embedded Technologies, Inc.
	103 Innovation Boulevard
	State College, PA 16803 USA

	Telephone: (814) 234-8087
	Fax: (814) 234-5218
	Sales Information and Quotes: sales@rtd.com
	Technical Assistance: techsupport@rtd.com
	Web Site: http://www.rtd.com
