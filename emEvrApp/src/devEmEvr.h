/**
 * File              : devEmEvr.h
 * Author            : chengsn <chengsn@ihep.ac.cn>
 * Date              : 2025-02-08
 * Last Modified Date: 2025-03-10
 * Last Modified By  : chengsn <chengsn@ihep.ac.cn>
 * Description       : Header file for Event Receiver (EmEvr) device support
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

#ifndef DEV_EM_EVR_H
#define DEV_EM_EVR_H
 
 /**********************************************************************
  *                         Required Headers                           *
  **********************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <ctype.h>
#include <epicsTypes.h>
#include <epicsTime.h>
#include <dbScan.h>
 
/**********************************************************************
 *                         Type Definitions                           *
 **********************************************************************/
/* User event handler function prototype */
typedef void (*USER_EVENT_FUNC)(epicsInt16, epicsTimeStamp*);

/**********************************************************************
 *         Function Prototypes For Device Support Routines            *
 **********************************************************************/
/**
 * Registers a user-defined event handler.
 * @param event_func User-defined event handler function.
 * @return Status of registration (epicsTrue on success, epicsFalse on failure).
 */
epicsStatus register_event_handler(USER_EVENT_FUNC event_func);
 
#endif /* DEV_EM_EVR_H */