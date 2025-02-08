/**
 * File              : drvEmEvr.c
 * Author            : chengsn <chengsn@ihep.ac.cn>
 * Date              : 2025-02-08
 * Last Modified Date: 2025-02-08
 * Last Modified By  : chengsn <chengsn@ihep.ac.cn>
 * Description       : event receiver driver
 *
 * Copyright (c) 2025 chengsn <chengsn@ihep.ac.cn>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MemEvrCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDemEvrS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHemEvr
 * LIABILITY, WHETHemEvr IN AN ACTION OF CONTRACT, TORT OR OTHemEvrWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHemEvr DEALINGS IN THE
 * SOFTWARE.
 */

/**********************************************************************
 *                       Imported Header Files                        *
 **********************************************************************/
/* Standard C library */
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <ctype.h>

/* EPICS Standard library */
#include <iocsh.h>
/* #include <epicsInterrupt.h> */
#include <epicsStdio.h>
/* #include <epicsStdlib.h> */
#include <epicsThread.h>
/* #include <epicsTime.h> */
#include <epicsTypes.h>

/* EPICS device support */
#include <devLib.h>

/* EPICS driver support */
#include <drvSup.h>

/* EPICS Symbol exporting macro definitions */
#include <epicsExport.h>

/* EPICS generaltime module*/
#include <generalTimeSup.h>
#include <epicsGeneralTime.h>

/* header files for event receiver */
#include "drvEmEvr.h"

/**********************************************************************
 *                         Macro Definitions                          *
 **********************************************************************/

/* the default memory page size of the Linux Kernel is 4KB */
#define PAGE_SIZE 4096UL
/* page mask */
#define PAGE_MASK (PAGE_SIZE - 1)
/* bse address of the register map */
#define BASE_ADDR 0
/* masked address */
#define MASK_ADDR (BASE_ADDR & PAGE_MASK)
/* print error message */
#define FATAL                                                                                                          \
    do {                                                                                                           \
        fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", __LINE__, __FILE__, errno, strerror(errno));  \
        return -1;                                                                                             \
    } while (0)
/* initialize emEvr structure */
#define EMEVR_INIT                                                                                                        \
{                                                                                                              \
    NULL, NULL, 0, 0, NULL, 0, {0}, {0}                                                               \
}
/* number of event codes */
#define EVENT_NUM 256

/* timestamp clock nanosecond conversion factor */
#define NANO_CONV 10.0

/* one second */
#define ONE_SECOND 1000000000.0

/**********************************************************************
 *                        Hardware Definitions                        *
 **********************************************************************/

/*     Register Address Base Offset Numbers     */
#define FRONT_PANEL 0x0000
#define TRIG_CTRL 0x0400
#define TIME_STAMP 0x1000

/**********************************************************************
 *                       Common IO Definitions                        *
 **********************************************************************/
/* read 32-bit date from offset */
#define READ_32(BASE, OFFSET) *(volatile epicsUInt32 *)((epicsUInt8 *)BASE + OFFSET)

/* write 32-bit date from offset */
#define WRITE_32(BASE, OFFSET, VALUE) *(volatile epicsUInt32 *)((epicsUInt8 *)BASE + OFFSET) = VALUE

/**********************************************************************
 *                          Static Structure                          *
 **********************************************************************/

static EmEvrStruct emEvr = EMEVR_INIT;

/**********************************************************************
 *                   Function Type Definitions                        *
 **********************************************************************/
/* user-defined event function */
typedef void (*EVENT_FUNC)(void);

/**********************************************************************
 *                  Prototype Function Declarations                   *
 **********************************************************************/
int open_emEvr(char *, epicsUInt32, epicsUInt32 **);
epicsStatus configure_emEvr(char *, epicsUInt32, epicsUInt32 **);
epicsThreadId init_emEvr_irq();
void *emEvr_irq_handler(void *);
epicsStatus get_current_time(epicsTimeStamp *);
epicsStatus get_event_time(epicsTimeStamp *, int);
epicsStatus get_emEvr_time(EmEvrStruct *, epicsUInt32, epicsTimeStamp *);
epicsStatus init_event_time();

/**********************************************************************
 *                        Function Definitions                        *
 **********************************************************************/

/*-----------------------------------------------
 * get_emEvr: retrieve a pointer to the emEvr structure
 *-----------------------------------------------
 * output:
 *   emEvr: EmEvr pointer
 * return: status
 */
EmEvrStruct *get_emEvr()
{
    return &emEvr;
}

/*---------------------------------------------
 * open_emEvr: open an emEvr device
 *---------------------------------------------
 * input:
 *   device: the device name in /dev/ directory
 *   offset: accessible address for the device
 * output:
 *   vir_addr: the mapped virtual address
 * return: emEvr device fd
 */
int open_emEvr(char *device, epicsUInt32 offset, epicsUInt32 **vir_addr)
{
    int fd;
    void *page_addr;
    char device_name[50];
    /* device name handle */
    strcpy(device_name, "/dev/");
    strcat(device_name, device);
    /* open an image of device */
    if ((fd = open(device_name, O_RDWR | O_SYNC)) == -1)
        FATAL;
    /* map one page for device spacename */
    page_addr = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, offset & ~PAGE_MASK);
    if (page_addr == (void *)-1)
        FATAL;
    /* printf("One page mapped at address %p.\n", page_addr); */
    *vir_addr = page_addr + MASK_ADDR;
    return fd;
}

/*---------------------------------------------------------------
 * configure_emEvr: open and configure the emEvr device before IOC Init
 *---------------------------------------------------------------
 * input:
 *   device: the device name in /dev/directory
 *   offset: accessible address for the device
 */
epicsStatus configure_emEvr(char *device, epicsUInt32 offset)
{
    epicsThreadId tid;
    int fd;
    /* open the emEvr device and store the virtual address in *pEr */
    fd = open_emEvr(device, offset, (epicsUInt32 **)&emEvr.pEr);
    // printf("fd1:%d\n",fd);
    if (fd < 0)
        FATAL;
    /* store the fd into emEvr structure */
    emEvr.fd = fd;

    /* initialize the interrupt thread */
    tid = init_emEvr_irq();
    if (tid == 0)
        FATAL;
    /* store the tid into emEvr structure */
    emEvr.tid = tid;

    /* init event time */
    init_event_time();

    return 0;
}

/*-------------------------------------------
 * init_emEvr_irq: initialize the interrupt handler
 *-------------------------------------------
 * return: thread tid
 */
epicsThreadId init_emEvr_irq()
{
    /* the default sched is SCHED_FIFO */
    epicsThreadOpts opts = EPICS_THREAD_OPTS_INIT;
    opts.priority = epicsThreadPriorityHigh;
    /* creat a new thread */
    return epicsThreadCreateOpt("irq_handler", (EPICSTHREADFUNC)irq_handler, (void *)&emEvr.fd, &opts);
}

/*----------------------------------------------
 * irq_handler: handle interrupts from ER device
 *----------------------------------------------
 * input:
 *   argv: parm passed to new thread
 */
void *irq_handler(void *argv)
{
    /* int policy, ret; */
    /* struct sched_param param; */
    /* for main process */
    register epicsUInt32 *pEr = emEvr.pEr;
    int fd = *(int *)argv;
    // printf("fd3:%d\n",fd);
    int irq_count;
    int enable = 1;
    unsigned int mask;
    unsigned int event_code;
    unsigned long seconds;
    unsigned long nanoseconds;

    write(fd, &enable, sizeof(enable));
    while (1) {
        /* unsigned long tmp; */
        if (read(fd, &irq_count, 4) != 4) {
            perror("read uio0");
        }

        for (int i = 0; i < 10; i++) {
            seconds = READ_32(pEr, EVENT_SEC);
            nanoseconds = READ_32(pEr, EVENT_NSEC);
            event_code = READ_32(pEr, EVENT_CODE);

            emEvr.event_ts[event_code].secPastEpoch = seconds;
            emEvr.event_ts[event_code].nsec = nanoseconds;
        }
        write(fd, &enable, sizeof(enable));
    }
    return NULL;
}

