/**
 * File              : drvEmEvr.h
 * Author            : chengsn <chengsn@ihep.ac.cn>
 * Date              : 2025-02-08
 * Last Modified Date: 2025-03-10
 * Last Modified By  : chengsn <chengsn@ihep.ac.cn>
 * Description       : Header file for Event Receiver (EmEvr) driver
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
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef DRV_EM_EVR_H
#define DRV_EM_EVR_H
 
 /**********************************************************************
  *                         Required Headers                           *
  **********************************************************************/
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <epicsThread.h>
#include <dbScan.h>
 
/**********************************************************************
 *                         Macro Definitions                          *
 **********************************************************************/
#define EVENT_NUM 256 /* Number of supported event codes */
 
/**********************************************************************
 *          Prototype Structure and Function Declarations             *
 **********************************************************************/
/* Micro TCA Event Receiver card structure prototype */
typedef struct EmEvrStruct EmEvrStruct;
 /* Event handler function prototypes */
typedef void (*EVENT_FUNC)(void);                     /* Generic event handler */
typedef void (*DEV_EVENT_FUNC)(struct EmEvrStruct*, epicsInt16, epicsTimeStamp*); /* Device event handler */
 
/* Event Receiver (EmEvr) structure */
typedef struct EmEvrStruct {
    /* Pointer to the event receiver register map */
    epicsUInt32 *pEr;
 
    /* ER device file descriptor */
    epicsInt32 fd;
 
    /* Interrupt thread ID */
    epicsThreadId tid;
 
    /* Pointer to user-defined event handler */
    EVENT_FUNC event_func;
 
    /* Pointer to device-support event handling routine */
    DEV_EVENT_FUNC dev_event_func;
 
    /* Mapping RAM enable flag */
    bool ram_ena;
 
    /* Event timestamps */
    epicsTimeStamp event_ts[EVENT_NUM];
 
    /* Current view of mapping RAM */
    epicsUInt16 event_map[EVENT_NUM];
 
    /* Record processing structure */
    IOSCANPVT ioscan_pvt;
};
 
/**********************************************************************
 *         Function Prototypes For Driver Support Routines            *
 **********************************************************************/
/**
 * Retrieves a pointer to the EmEvr structure.
 * @return Pointer to the EmEvr structure.
 */
EmEvrStruct* get_emEvr();

/**
 * Registers a device-support level event handler.
 * @param emEvr        Pointer to the EmEvr structure.
 * @param dev_event_func Device-support event handler function.
 */
void register_device_event_handler(EmEvrStruct *emEvr, DEV_EVENT_FUNC dev_event_func);
 
/**
 * Processes the output pulse width.
 * @param emEvr Pointer to the EmEvr structure.
 * @param digit Channel of the pulse.
 * @param value Value of the pulse width.
 */
void process_otw(EmEvrStruct *emEvr, epicsUInt32 digit, epicsFloat64 value);

/**
 * Processes the output pulse delay.
 * @param emEvr Pointer to the EmEvr structure.
 * @param digit Channel of the pulse.
 * @param value Value of the pulse delay.
 */
void process_otd(EmEvrStruct *emEvr, epicsUInt32 digit, epicsFloat64 value);
 
/**
 * Processes the front panel switch.
 * @param emEvr Pointer to the EmEvr structure.
 * @param digit Channel of the switch.
 * @param value Value of the switch.
 */
void process_fps(EmEvrStruct *emEvr, epicsUInt32 digit, epicsFloat64 value);

#endif /* DRV_EM_EVR_H */