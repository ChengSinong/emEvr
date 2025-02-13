/**
 * File              : devEmEvr.c
 * Author            : chengsn <chengsn@ihep.ac.cn>
 * Date              : 2025-02-08
 * Last Modified Date: 2025-02-08
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
#include <epicsTypes.h>
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
#include "drvEmEvr.h"
#include "devEmEvr.h"

/**********************************************************************
 *                       Prototype Declarations                       *
 **********************************************************************/
/* device support functions for ER */
epicsStatus init_record(aiRecord *);
epicsStatus read_ai(aiRecord *);

/**********************************************************************
 *                        Function Definitions                        *
 **********************************************************************/

/*-----------------------------------------------------------------------------
 * init_record: init ai record.
 *-----------------------------------------------------------------------------
 * input:
 *   pai: pointer to ai record structure
 */
epicsStatus init_record(aiRecord *pai) {
    return 0;
}
 
/*---------------------------------------------
 * extract_ld: Extract letters and digits from parm string
 *---------------------------------------------
 * input:
 *   parm: the parm string containing letters and digits
 * output:
 *   letters: a string to store the extracted uppercase letters
 *   digit: a pointer to an integer to store the extracted digit value
 * return: None 
 */
void extract_ld(epicsInt8 *parm, epicsInt8 *letters, epicsInt32 *digit) {
    epicsInt32 letter_index = 0;
    *digit = -1; // Initialize to -1 to indicate that no valid digit was found

    // Extract letter and digit parts
    for (epicsInt32 i = 0; parm[i] != '\0'; i++) {
        epicsInt8 ch = parm[i];

        if (isalpha(ch) && letter_index < 3) {
            // Extract letters (only keep the first three uppercase letters)
            letters[letter_index++] = toupper(ch); // Store uppercase letter
        } else if (isdigit(ch)) {
            // Handle digit characters (0-9)
            if (*digit == -1) {
                *digit = (ch - '0'); // First digit found, set initial value
            } else {
                *digit = (*digit * 10) + (ch - '0'); // Accumulate digit
            }
        } else if (ch >= 'A' && ch <= 'F') {
            // Handle hexadecimal characters (A-F)
            if (*digit == -1) {
                *digit = 10 + (ch - 'A'); // First hex character found, initialize
            } else {
                *digit = (*digit * 16) + (ch - 'A' + 10); // Accumulate
            }
        } else if (ch >= 'a' && ch <= 'f') {
            // Handle hexadecimal characters (a-f)
            if (*digit == -1) {
                *digit = 10 + (ch - 'a'); // First hex character found, initialize
            } else {
                *digit = (*digit * 16) + (ch - 'a' + 10); // Accumulate
            }
        }
    }

    // Ensure the letters string is null-terminated
    letters[letter_index] = '\0';

    // Restrict valid digits to the range 0-15
    if (*digit == -1 || *digit > 15) {
        *digit = -1; // Set to -1 to indicate an error
    }
}

/*---------------------------------------------
 * read_ai: Read ai record value
 *---------------------------------------------
 * input:
 *   pai: pointer to ai record structure
 */
epicsInt32 read_ai(aiRecord *pai) {
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
    if (parm != NULL) {
        extract_ld(parm, letters, &digit);
        if (digit == -1) {
            printf("No valid digit found. Number out of range (0-15).\n");
        } else {
            // Process the letters and digit to generate the output value
            if (strcmp(letters, "OTW") == 0) {
                process_otw(EmEvr, digit, value);
            } else if (strcmp(letters, "OTD") == 0) {
                process_otd(EmEvr, digit, value);
            } else if (strcmp(letters, "FPS") == 0) {
                process_fps(EmEvr, digit, value);
            } else {
                printf("Unknown command.\n");
                return 1;
            }
        }
    } else {
        printf("No parameter provided\n");
    }
    return 2;
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
                (DEVSUPFUN)init_record,
                /* -- No I/O interrupt information */
                (DEVSUPFUN)NULL, 
                /* Read record routine */
                (DEVSUPFUN)read_ai, 
                /* -- No special linear conversion */
                (DEVSUPFUN)NULL
};
epicsExportAddress(dset, devReadAI);