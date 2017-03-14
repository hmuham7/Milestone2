/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    message_thread.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _MESSAGE_THREAD_H
#define _MESSAGE_THREAD_H

// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "messages.h"
#include "jsmn.h"
#include "json_parser.h"
#include "debug.h"

#ifdef __cplusplus  // Provide C++ Compatibility

extern "C" {

#endif

typedef struct {
    char Destination;
    char MsgCount;
    char Data[SIZE];
} Tx_DataType;

typedef struct {
    float encoderdata;
}RoverData;

typedef enum RequestType_enum { FRtoSR,
                                FRtoTR,
                                FRtoCMR,
                                TRtoSR,
                                TRtoFR,
                                TRtoCMR,
                                CMRtoSR,
                                CMRtoTR,
                                CMRtoFR,
                                SRtoTR,
                                SRtoFR,
                                SRtoCMR
} RequestType;

//------------------------------------------------------------------------------
//You should not need to change anything beyond this point
//------------------------------------------------------------------------------
typedef enum MsgType_enum {SEND_RESPONSE, SEND_REQUEST} MsgType;

typedef struct {
    char Source;
    bool Error;
    char Data[SIZE];
    char MsgCount;
    unsigned int MessagesDropped;
} ExternalObj;

typedef struct {
    MsgType Type;
    ExternalObj External;
    RequestType Request;
} MsgObj;

typedef struct {
    int32_t MessagesDropped;
    int32_t ErrorCount;
    int32_t GoodCount;
    
    unsigned int Request_From_Flagrover;
    unsigned int Request_To_Flagrover;
    unsigned int Response_From_Flagrover;
    unsigned int Response_To_Flagrover;
    
    unsigned int Request_From_Sensorrover;
    unsigned int Request_To_Sensorrover;
    unsigned int Response_From_Sensorrover;
    unsigned int Response_To_Sensorrover;
    
    unsigned int Request_From_Tagrover;
    unsigned int Request_To_Tagrover;
    unsigned int Response_From_Tagrover;
    unsigned int Response_To_Tagrover;
    
    unsigned int Request_From_CMrover;
    unsigned int Request_To_CMrover;
    unsigned int Response_From_CMrover;
    unsigned int Response_To_CMrover;
    
    unsigned int Request_From_Server;
    unsigned int Request_To_Server;
    unsigned int Response_From_Server;
    unsigned int Response_To_Server;
} StatObjectType;
/*******************************************************************************
  Function:
    void MESSAGE_CONTROLLER_THREAD_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the
    application in its initial state and prepares it to run so that its
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MESSAGE_CONTROLLER_THREAD_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void MESSAGE_THREAD_Initialize ( void );


/*******************************************************************************
  Function:
    void MESSAGE_CONTROLLER_THREAD_Tasks ( void )

  Summary:
    MPLAB Harmony Demo application tasks function

  Description:
    This routine is the Harmony Demo application's tasks function.  It
    defines the application's state machine and core logic.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    MESSAGE_CONTROLLER_THREAD_Tasks();
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
 */

void MESSAGE_THREAD_Tasks( void );

void MESSAGE_THREAD_InitializeQueue();

void MESSAGE_THREAD_ReadFromQueue(MsgObj* pvBuffer);

void resetLocalTime();

void addLocalTime();

int getLocalTime();

#endif /* _MESSAGE_CONTROLLER_THREAD_H */

//DOM-IGNORE-BEGIN
#ifdef __cplusplus
}
#endif
//DOM-IGNORE-END

/*******************************************************************************
 End of File
 */
