/**
 * File              : devEmEvr.c
 * Author            : chengsn <chengsn@ihep.ac.cn>
 * Date              : 2025-02-08
 * Last Modified Date: 2025-02-20
 * Last Modified By  : chengsn <chengsn@ihep.ac.cn>
 * Description       : event receiver device support
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
 *                       Imported Header Files                        *
 **********************************************************************/
/* Standard C library */
#include <stdio.h>

/* EPICS Standard library */
#include <epicsInterrupt.h>
#include <epicsStdio.h>
#include <epicsStdlib.h>
#include <epicsThread.h>
#include <iocsh.h>
/* EPICS device support */
#include <devLib.h>
#include <devSup.h>
#include <recSup.h>
/* EPICS driver support */
#include <drvSup.h>
/* EPICS Symbol exporting macro definitions */
#include <epicsExport.h>
/* EPICS Record Support global routine definitions */
#include <recGbl.h>
/* EPICS aiRecord.h header file */
#include <aiRecord.h>

/* header files for event receiver */
#include "devEmEvr.h"
#include "drvEmEvr.h"

/**********************************************************************
 *                       Prototype Function Declarations                       *
 **********************************************************************/
/* device support functions for ER */
epicsStatus init_ai(aiRecord *);
epicsStatus read_ai(aiRecord *);
epicsStatus process_ai(aiRecord *);
epicsStatus get_ioint_info(int cmd, aiRecord *prec, IOSCANPVT *ppvt);

/**********************************************************************
 *                        Function Definitions                        *
 **********************************************************************/
void extract_ld(epicsInt8 *parm, epicsInt8 *letters, epicsInt32 *digit);
void device_event_func(EmEvrStruct *, epicsInt16, epicsTimeStamp *);
/*---------------------------------------------
 * extract_ld: Extract letters and digits from parm string
 *---------------------------------------------
 * input:
 *   parm: the parameter string containing letters and digits
 * output:
 *   letters: a string to store the extracted uppercase letters
 *   digit: a pointer to an integer to store the extracted digit value
 * return: None
 */
void extract_ld(epicsInt8 *parm, epicsInt8 *letters, epicsInt32 *digit) {
  epicsInt32 letter_index = 0;

  letters[0] = '\0'; // Initialize the letters string to null
  *digit = -1; // Initialize to -1 to indicate that no valid digit was found

  // Extract letter and digit parts
  for (epicsInt32 i = 0; parm[i] != '\0'; i++) {
    epicsInt8 ch = parm[i];
    if (isalpha(ch) && letter_index < 3) {
      // Extract letters (only keep the first three uppercase letters)
      letters[letter_index++] = toupper(ch); // Store uppercase letter
      if (letter_index >= 10) { // Prevent exceeding the array capacity
        break;
      }
    } else if (isdigit(ch)) {
      // Handle digit characters (0-9)
      if (*digit == -1) {
        *digit = (ch - '0'); // First digit found, set initial value
      } else {
        *digit = (*digit * 10) + (ch - '0'); // Accumulate digit
      }
    }
  }
  // Ensure the letters string is null-terminated
  letters[letter_index] = '\0';
  // Restrict valid digits to the range 1-256
  if (*digit < 1 || *digit > 256) {
    *digit = -1; // Set to -1 to indicate an error
  }
}

/*---------------------------------------------
 * register_event_handler: Register the user-defined event handler
 *---------------------------------------------
 * input:
 *   CardNum: the card number of the ER device
 *   event_func: the user-defined event handler function
 * return:
 *   epicsStatus: Status of registration
 */
epicsStatus register_event_handler(USER_EVENT_FUNC event_func) {
  // Pointer to the card structure for the requested card
  EmEvrStruct *EmEvr;

  EmEvr = get_emEvr();
  if (EmEvr == NULL) {
    printf("register_event_handler: EmEvr is NULL.\n");
    return epicsFalse;
  }
  // Abort if another handler is already registered
  if (NULL != EmEvr->event_func) {
    printf("register_event_handler: Handler already registered.\n");
    return epicsFalse;
  }

  // Register the event handler function and return success.
  EmEvr->event_func = (EVENT_FUNC)event_func;
  return epicsTrue;
}

/*---------------------------------------------
 * device_event_func: Device Support Layer Interrupt-Level Event Handling
 *Routine
 *---------------------------------------------
 * input:
 *   EmEvr: pointer to the EmEvr structure for the requested card
 *   event_code: the event code number (1-256)
 *   timestamp: the timestamp of the event
 */
void device_event_func(EmEvrStruct *EmEvr, epicsInt16 event_code,
                       epicsTimeStamp *timestamp) {
  /*---------------------
   * Invoke the user-defined event handler (if one is defined)
   */
  if (EmEvr == NULL) {
    printf("device_event_func: EmEvr is NULL.\n");
    return;
  }
  if (EmEvr->event_func != NULL)
    (*(USER_EVENT_FUNC)EmEvr->event_func)(event_code, timestamp);

  /*---------------------
   * Schedule processing for any event-driven records
   */
  scanIoRequest(EmEvr->ioscan_pvt);
  printf("device_event_func: Event code %d \n", event_code);
}

/*---------------------------------------------
 * configure_event_handler: Configure the event handler for the EmEvr device
 *---------------------------------------------
 */
void configure_event_handler() {

  // Pointer to the card structure for the requested card
  EmEvrStruct *EmEvr = get_emEvr();

  // Register the device-specific event handler function
  register_device_event_handler(EmEvr, (DEV_EVENT_FUNC)device_event_func);
  // Initialize the scan I/O request processing
  scanIoInit(&EmEvr->ioscan_pvt);
}

/*---------------------------------------------
 * process_ai: Process ai record by extracting and handling command
 *---------------------------------------------
 * input:
 *   pai: pointer to ai record structure
 * return:
 *   epicsStatus: Status of processing
 */
epicsStatus process_ai(aiRecord *pai) {
  epicsInt8 *parm;
  epicsInt8 letters[10];
  epicsInt32 digit = 0;
  epicsFloat64 value = 0.0;
  EmEvrStruct *EmEvr = get_emEvr();

  // Extract the letters and digit from the input string
  parm = pai->inp.value.instio.string;
  // Read the current value from the record
  value = pai->val;

  // Verify that the input string is valid
  if (parm == NULL) {
    printf("No parameter provided\n");
    return epicsFalse;
  }

  extract_ld(parm, letters, &digit);

  if (digit == -1) {
    printf("No valid digit found. Number out of range (1-256).\n");
    return epicsFalse;
  } else {
    // Process the letters and digit to generate the output value
    if (strcmp(letters, "OTW") == 0) {
      process_otw(EmEvr, digit, value);
    } else if (strcmp(letters, "OTD") == 0) {
      process_otd(EmEvr, digit, value);
    } else if (strcmp(letters, "FPS") == 0) {
      if (digit > 16 || digit < 1) { // FPS digit out of range
        printf("FPS digit out of range (1-16).\n");
        return epicsFalse;
      }
      process_fps(EmEvr, digit, value);
    } else {
      printf("Unknown command.\n");
      return epicsFalse;
    }
  }

  return epicsTrue;
}

/*---------------------------------------------
 * init_ai: Initialize ai record.
 *---------------------------------------------
 * input:
 *   pai: pointer to ai record structure
 * return:
 *   epicsStatus: Status of initialization
 */
epicsStatus init_ai(aiRecord *pai) {
  // Perform initialization specific tasks here
  epicsStatus status = epicsTrue;

  // Call the processing function to handle initialization logic
  if (process_ai(pai) != epicsTrue) {
    status = epicsFalse;
  }

  return status;
}

/*---------------------------------------------
 * read_ai: Read ai record value
 *---------------------------------------------
 * input:
 *   pai: pointer to ai record structure
 */
epicsStatus read_ai(aiRecord *pai) {
  // Execute processing logic
  epicsStatus status = process_ai(pai);

  if (status != epicsTrue) {
    printf("Error processing ai record.\n");
    return 1;
  }

  return 2;
}

/*---------------------------------------------
 * get_ioint_info: Get I/O interrupt information
 *---------------------------------------------
 * input:
 *   cmd: command to get I/O interrupt information
 *   prec: pointer to ai record structure
 *   ppvt: pointer to I/O scan processing structure
 * return:
 *   epicsStatus: Status of I/O interrupt information
 */
epicsStatus get_ioint_info(int cmd, aiRecord *prec, IOSCANPVT *ppvt) {
  EmEvrStruct *EmEvr = get_emEvr();
  *ppvt = EmEvr->ioscan_pvt;
  return 0;
}

/**********************************************************************
 *                 Device Support Entry Table (DSET)                  *
 **********************************************************************/
/* AI Record */
struct {
  long number;
  DEVSUPFUN report;
  DEVSUPFUN init;
  DEVSUPFUN init_record;
  DEVSUPFUN get_ioint_info;
  DEVSUPFUN read_write;
  DEVSUPFUN special_linconv;
} devReadAI = {6,
               /* -- No device report routine  */
               (DEVSUPFUN)NULL,
               /* -- No device initialization route */
               (DEVSUPFUN)NULL,
               /* Record initialization routine */
               (DEVSUPFUN)init_ai,
               /* -- No I/O interrupt information */
               (DEVSUPFUN)get_ioint_info,
               /* Read record routine */
               (DEVSUPFUN)read_ai,
               /* -- No special linear conversion */
               (DEVSUPFUN)NULL};
epicsExportAddress(dset, devReadAI);

/**********************************************************************
 *                      register IOC Shell                            *
 **********************************************************************/
const iocshFuncDef configure_event_handlerDef = {
    "configure_event_handler", 0, NULL,
    "Configure the event handler for the EmEvr device\n"};
void configure_event_handlerCall(const iocshArgBuf *args) {
  configure_event_handler();
}
static void configure_event_handlerRegister(void) {
  iocshRegister(&configure_event_handlerDef, configure_event_handlerCall);
}
epicsExportRegistrar(configure_event_handlerRegister);