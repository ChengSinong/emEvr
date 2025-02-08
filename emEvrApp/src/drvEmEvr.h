/**
 * File              : drvEmEvr.h
 * Author            : chengsn <chengsn@ihep.ac.cn>
 * Date              : 2025-02-08
 * Last Modified Date: 2025-02-08
 * Last Modified By  : chengsn <chengsn@ihep.ac.cn>
 * Description       : header File              : drvEmEvr.h
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

/**********************************************************************
 *              Other Header Files Required by This File              *
 **********************************************************************/
/* Standard C Library Headers */
#include <stdbool.h>

/* EPICS Base Headers */
#include <epicsTypes.h>  /* EPICS Architecture-independent type definitions */
#include <epicsTime.h>   /* EPICS Timestamp support library */
/* EPICS Database and Record Support Headers */
#include <dbScan.h>      /* EPICS Database Scan Routines and Definitions */
#include <epicsThread.h>   /* EPICS thread library */

/* EPICS Menu Definitions */
/* #include <menuYesNo.h> */ /* Choice menu for "Yes/No" fields */

/**********************************************************************
 *                         Macro Definitions                          *
 **********************************************************************/
#define EVENT_NUM 256

/**********************************************************************
 *                          Device Prototype                          *
 **********************************************************************/
/* ER structure prototype */
typedef struct emEvrStruct {
    /* Pointer to the event receiver record */
    epicsUInt32 *pRec;
    /* Pointer to the event receiver register map */
    epicsUInt32 *pEr;
    /* ER device fd */
    int fd;
    /* irq thread id */
    epicsThreadId tid;
    /* Pointer to user function */
    void* event_func;
    /* Mapping RAM enable */
    bool ram_ena;
    /* Event timestamps */
    epicsTimeStamp event_ts[EVENT_NUM];
    /* Current view of mapping RAM */
    epicsUInt16 event_map[EVENT_NUM];
    /* Record processing struct */
    IOSCANPVT ioscan_pvt[EVENT_NUM];
} EmEvrStruct;

/**********************************************************************
 *         Function Prototypes For Driver Support Routines            *
 **********************************************************************/

EmEvrStruct* get_emEvr();
void extract_ld(const char *input, char *letters, int *digit);
