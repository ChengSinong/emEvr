/**
 * File              : drvEmEvr.c
 * Author            : chengsn <chengsn@ihep.ac.cn>
 * Date              : 2025-02-08
 * Last Modified Date: 2025-03-10
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
 * LIABILITY, WHETHemEvr IN AN ACTION OF CONTRACT, TORT OR OTHemEvrWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHemEvr
 * DEALINGS IN THE SOFTWARE.
 */

/**********************************************************************
 *                       Imported Header Files                        *
 **********************************************************************/
/* Standard C library */
#include <errno.h>
#include <sys/fcntl.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <unistd.h>

/* EPICS Standard library */
#include <epicsInterrupt.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <iocsh.h>

/* EPICS device support */
#include <devLib.h>

/* EPICS driver support */
#include <drvSup.h>

/* EPICS Symbol exporting macro definitions */
#include <epicsExport.h>

/* EPICS generaltime module*/
#include <epicsGeneralTime.h>
#include <generalTimeSup.h>

/* header files for event receiver */
#include "drvEmEvr.h"

/**********************************************************************
 *                         Macro Definitions                          *
 **********************************************************************/

/* the default memory page size of the Linux Kernel is 4KB */
#define BASE_PAGE_SIZE 4096UL
/* the size of the device uio0 is 48MB */
#define PAGE_SIZE 5 * 4 * 1024 * BASE_PAGE_SIZE
/* page mask */
#define PAGE_MASK (PAGE_SIZE - 1)
/* bse address of the register map */
#define BASE_ADDR 0
/* masked address */
#define MASK_ADDR (BASE_ADDR & PAGE_MASK)
/* print error message */
#define FATAL                                                                  \
  do {                                                                         \
    fprintf(stderr, "Error at line %d, file %s (%d) [%s]\n", __LINE__,         \
            __FILE__, errno, strerror(errno));                                 \
    return -1;                                                                 \
  } while (0)
/* number of event codes */
#define EVENT_NUM 256

/* The frequency of the timestamp clock is 100 MHz
 * so, the timestamp clock period is 1 s / 100 MHz = 10 ns
 */
/* timestamp clock nanosecond conversion factor */
#define NANO_CONV 10.0

/* one second */
#define ONE_SECOND 1000000000.0

/**********************************************************************
 *                        Hardware Definitions                        *
 **********************************************************************/

/*     Register Address Base Offset Numbers     */

/* UIO0 base address is 0x42000000
 * FRONT_PANEL base address is 0x44000000
 * TRIG_CTRL base address is 0x43000400
 * TIME_STAMP base address is 0x42001000
 * CODE_COUNT base address is 0x42001C00
 * So, the offset of each register is calculated by adding the base address to
 * the offset number.
 */

// Trigger control means: Delay, Width and Code_id, and each control word has
// 32-bit
#define TRIG_CTRL 0x00000400
// Time stamp means: Seconds, NanoSeconds, Event_code, and each word has 32-bit
#define TIME_STAMP 0x01001000
// Front Panel means: Front Panel Control, and each word has 32-bit
#define FRONT_PANEL 0x02000000
// Latch register means: Latch Control, and each word has 32-bit
#define LATCH 0x03000000

// Latch read means: timestamp
#define LATCH_READ 0x03001000

// Code count means: Code_count, and each word has 32-bit
#define CODE_COUNT 0x04001C00

/*     Register Address Offset Numbers     */
// Front Panel Registers
#define FRONT_PANEL_OFFSET(x) (FRONT_PANEL + (x - 1) * 4)
// Trigger Control Registers
#define TRIG_CTRL_DELAY(x) (TRIG_CTRL + (x - 1) * 4 * 3)
#define TRIG_CTRL_WIDTH(x) (TRIG_CTRL + 4 + (x - 1) * 4 * 3)
#define TRIG_CTRL_CODE_ID(x) (TRIG_CTRL + 8 + (x - 1) * 4 * 3)
// Time Stamp Registers
#define TIME_STAMP_SECONDS(x) (TIME_STAMP + (x - 1) * 4 * 3)
#define TIME_STAMP_NANOSECONDS(x) (TIME_STAMP + 4 + (x - 1) * 4 * 3)
#define TIME_STAMP_EVENT_CODE(x) (TIME_STAMP + 8 + (x - 1) * 4 * 3)
// Code Count Registers
#define CODE_COUNT_NUM(x) (CODE_COUNT + (x - 1) * 4)

/**********************************************************************
 *                       Common IO Definitions                        *
 **********************************************************************/
/* read 32-bit date from offset */
#define READ_32(BASE, OFFSET)                                                  \
  *(volatile epicsUInt32 *)((epicsUInt8 *)BASE + OFFSET)

/* write 32-bit date from offset */
#define WRITE_32(BASE, OFFSET, VALUE)                                          \
  *(volatile epicsUInt32 *)((epicsUInt8 *)BASE + OFFSET) = VALUE

/**********************************************************************
 *                          Static Structure                          *
 **********************************************************************/

static EmEvrStruct *emEvr;

/**********************************************************************
 *                   Function Type Definitions                        *
 **********************************************************************/
/* user-defined event function */
typedef void (*EVENT_FUNC)(void);

/**********************************************************************
 *                  Prototype Function Declarations                   *
 **********************************************************************/
void *emEvr_irq_handler(void *);
epicsStatus open_emEvr(epicsInt8 *, epicsUInt32, epicsUInt32 **);
epicsStatus configure_emEvr(epicsInt8 *, epicsUInt32);
epicsThreadId init_emEvr_irq();
epicsStatus get_current_time(epicsTimeStamp *);
epicsStatus get_event_time(epicsTimeStamp *, epicsInt32);
epicsStatus get_emEvr_time(EmEvrStruct *, epicsUInt32, epicsTimeStamp *);
epicsStatus init_event_time();
epicsStatus read_event_code(epicsUInt32 *, epicsInt32);

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
EmEvrStruct *get_emEvr() { return emEvr; }

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
epicsStatus open_emEvr(epicsInt8 *device, epicsUInt32 offset,
                       epicsUInt32 **vir_addr) {
  epicsInt32 fd;
  void *page_addr;
  epicsInt8 device_name[50];
  /* device name handle */
  strcpy(device_name, "/dev/");
  strcat(device_name, device);
  /* open an image of device */
  if ((fd = open(device_name, O_RDWR | O_SYNC)) == -1)
    FATAL;
  /* map one page for device spacename */
  page_addr = mmap(0, PAGE_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd,
                   offset & ~PAGE_MASK);
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
epicsStatus configure_emEvr(epicsInt8 *device, epicsUInt32 offset) {
  epicsThreadId tid;
  epicsInt32 fd;
  /* initialize emEvr structure */
  emEvr = (EmEvrStruct *)calloc(1, sizeof(EmEvrStruct));
  if (emEvr == NULL)
    FATAL;
  /* open the emEvr device and store the virtual address in *pEr */
  fd = open_emEvr(device, offset, (epicsUInt32 **)&emEvr->pEr);
  // printf("fd1:%d\n",fd);
  if (fd < 0) {
    free(emEvr);
    FATAL;
  }
  /* store the fd into emEvr structure */
  emEvr->fd = fd;

  /* initialize the interrupt thread */
  tid = init_emEvr_irq();
  if (tid == 0)
    FATAL;
  /* store the tid into emEvr structure */
  emEvr->tid = tid;

  /* init event time */
  init_event_time();

  return 0;
}

/*-------------------------------------------
 * init_emEvr_irq: initialize the interrupt handler
 *-------------------------------------------
 * return: thread tid
 */
epicsThreadId init_emEvr_irq() {
  /* the default sched is SCHED_FIFO */
  epicsThreadOpts opts = EPICS_THREAD_OPTS_INIT;
  opts.priority = epicsThreadPriorityHigh;
  /* creat a new thread */
  return epicsThreadCreateOpt("emEvr_irq_handler",
                              (EPICSTHREADFUNC)emEvr_irq_handler,
                              (void *)&emEvr->fd, &opts);
}

/*----------------------------------------------
 * emEvr_irq_handler: handle interrupts from emEvr device
 *----------------------------------------------
 * input:
 *   argv: parm passed to new thread
 */
void *emEvr_irq_handler(void *argv) {
  /* epicsInt32 policy, ret; */
  /* struct sched_param param; */
  /* for main process */
  register epicsUInt32 *pEr = emEvr->pEr;
  epicsInt32 fd = *(epicsInt32 *)argv;
  // printf("fd3:%d\n",fd);
  epicsInt32 irq_count;
  epicsInt32 enable = 1;
  epicsUInt32 event_code;
  epicsUInt32 seconds;
  epicsUInt32 nanoseconds;

  write(fd, &enable, sizeof(enable));
  while (1) {
    /* epicsUInt32 tmp; */
    if (read(fd, &irq_count, 4) != 4) {
      perror("read uio0");
    }

    printf("irq_count:%d\n", irq_count);

    for (epicsInt32 i = 1; i <= EVENT_NUM; i++) {
      event_code = read_event_code(pEr, i);
      if (event_code != 0) {
        printf("event_code:%d\n", event_code);
        seconds = READ_32(pEr, TIME_STAMP_SECONDS(event_code));
        nanoseconds = READ_32(pEr, TIME_STAMP_NANOSECONDS(event_code));

        emEvr->event_ts[event_code].secPastEpoch = seconds;
        emEvr->event_ts[event_code].nsec = nanoseconds;
        // Invoke device-support layer event function if registered
        if (emEvr->dev_event_func != NULL) {
          (*emEvr->dev_event_func)(emEvr, event_code,
                                   &emEvr->event_ts[event_code]);
        }
      }
    }
    write(fd, &enable, sizeof(enable));
  }
  return NULL;
}

/*----------------------------------------------
 * process_otw: process the output pulse width
 *----------------------------------------------
 * input:
 *   EmEvr: pointer to emEvr structure
 *   digit: channel of the pulse
 *   value: value of the pulse width
 */
void process_otw(EmEvrStruct *EmEvr, epicsUInt32 digit, epicsFloat64 value) {
  register epicsUInt32 *pEr = EmEvr->pEr;
  epicsUInt32 OFFSET = TRIG_CTRL_WIDTH(digit);
  WRITE_32(pEr, OFFSET, (epicsInt32)value);
  printf("Processing OTW channel %d\n", digit);
}

/*----------------------------------------------
 * process_otw: process the output pulse delay
 *----------------------------------------------
 * input:
 *   EmEvr: pointer to emEvr structure
 *   digit: channel of the pulse
 *   value: value of the pulse delay
 */
void process_otd(EmEvrStruct *EmEvr, epicsUInt32 digit, epicsFloat64 value) {
  register epicsUInt32 *pEr = EmEvr->pEr;
  epicsUInt32 OFFSET = TRIG_CTRL_DELAY(digit);
  WRITE_32(pEr, OFFSET, (epicsInt32)value);
  printf("Processing OTD channel %d\n", digit);
}

/*----------------------------------------------
 * process_fps: process the front panel switch
 *----------------------------------------------
 * input:
 *   EmEvr: pointer to emEvr structure
 *   digit: channel of the fps
 *   value: value of the switch
 */
void process_fps(EmEvrStruct *EmEvr, epicsUInt32 digit, epicsFloat64 value) {
  register epicsUInt32 *pEr = EmEvr->pEr;
  epicsUInt32 OFFSET = FRONT_PANEL_OFFSET(digit);
  WRITE_32(pEr, OFFSET, (epicsInt32)value);
  printf("Processing FPS channel %d\n", digit);
}

/*-----------------------------------------------
 * init_event_time: Register event time to generaltime module
 *-----------------------------------------------
 */
epicsStatus init_event_time() {
  int ret = 0;
  ret |= generalTimeCurrentTpRegister("Event", 70, get_current_time);
  ret |= generalTimeEventTpRegister("Event", 70, get_event_time);

  if (ret) {
    printf("init_event_time failed.\n");
    FATAL;
  }
  return ret;
}

/*-----------------------------------------------------------
 * get_current_time: get current time without event code
 *-----------------------------------------------------------
 * input:
 *   pDest: pointer to EPICS timestamp structure
 */
epicsStatus get_current_time(epicsTimeStamp *pDest) {
  int ret;
  ret = get_event_time(pDest, 0);
  // printf("current_time:%d\n", ret);
  return ret;
}

/*-----------------------------------------------------------------
 * get_event_time: get event time for the specified event code
 *-----------------------------------------------------------------
 * input:
 *   pDest: pointer to EPICS timestamp structure
 *   event: event code
 */
epicsStatus get_event_time(epicsTimeStamp *pDest, epicsInt32 event_code) {
  EmEvrStruct *emEvr = get_emEvr();
  epicsStatus ret;
  ret = get_emEvr_time(emEvr, event_code, pDest);
  // printf("event_time:%d\n", ret);
  return ret;
}

/*---------------------------------------
 * get_emEvr_time: get timestamp from EmEvr
 *---------------------------------------
 * input:
 *   emEvr: pointer to emEvr structure
 *   event: event code
 *   timestamp: epics timestamp structure
 */
epicsStatus get_emEvr_time(EmEvrStruct *emEvr, epicsUInt32 event_code,
                           epicsTimeStamp *timestamp) {
  epicsTimeStamp localtime;
  epicsUInt32 overflow;
  epicsFloat64 nanoseconds;
  register epicsUInt32 *pEr = emEvr->pEr;

  /* clear timestamp structure */
  timestamp->secPastEpoch = 0;
  timestamp->nsec = 0;

  /* clear localtime structure */
  localtime.secPastEpoch = 631152000;
  localtime.nsec = 0;

  /* get timestamp from emEvr */
  if ((event_code > 0) && (event_code < EVENT_NUM)) {
    localtime = emEvr->event_ts[event_code];
  } else {
    /* read the latched timestamp */
    //        localtime.secPastEpoch = READ_32(pEr, LATCH_SEC);
    //        localtime.nsec = READ_32(pEr, LATCH_NSEC);
  }

  /* convert the timestamp into EPICS timestamp */
  nanoseconds = (epicsFloat64)localtime.nsec;

  nanoseconds = (nanoseconds * NANO_CONV);

  if (nanoseconds >= ONE_SECOND) {
    overflow = (epicsUInt32)(nanoseconds / ONE_SECOND);
    localtime.secPastEpoch += overflow;
    nanoseconds -= ((epicsFloat64)overflow * ONE_SECOND);
  }

  /* store the converted timestamp */
  localtime.nsec = (epicsUInt32)(nanoseconds + 0.5);

  *timestamp = localtime;

  return 0;
}

/*----------------------------------------------
 * read_event_code: read the event code from emEvr
 *-
 * input:
 *   pEr: pointer to emEvr structure
 *   digit: channel of the event code
 * output:
 *   event_code: the event code
 */
epicsStatus read_event_code(epicsUInt32 *pEr, epicsInt32 digit) {
  epicsUInt32 counter = READ_32(pEr, CODE_COUNT_NUM(digit));
  if (counter == 150) {
    return 0;
  }
  return digit;
}

/*----------------------------------------------
 * register_device_event_handler: Register a Device-Support Level Event Handler
 *----------------------------------------------
 * input:
 *   emEvr: pointer to emEvr structure
 *   dev_event_func: Address of the device-support layer event function.
 */
void register_device_event_handler(EmEvrStruct *emEvr,
                                   DEV_EVENT_FUNC dev_event_func) {
  emEvr->dev_event_func = dev_event_func;
}

/**********************************************************************
 *                      register IOC Shell                            *
 **********************************************************************/
const iocshArg configure_emEvrArg0 = {"device", iocshArgString};
const iocshArg configure_emEvrArg1 = {"offset", iocshArgInt};
const iocshArg *const configure_emEvrArgs[2] = {&configure_emEvrArg0,
                                                &configure_emEvrArg1};
const iocshFuncDef configure_emEvrDef = {
    "configure_emEvr", 2, configure_emEvrArgs, "Configure emEvr\n"};
void configure_emEvrCall(const iocshArgBuf *args) {
  configure_emEvr(args[0].sval, (epicsUInt32)args[1].ival);
}
static void configure_emEvrRegister(void) {
  iocshRegister(&configure_emEvrDef, configure_emEvrCall);
}
epicsExportRegistrar(configure_emEvrRegister);