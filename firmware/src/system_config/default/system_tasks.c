/*******************************************************************************
 System Tasks File

  File Name:
    system_tasks.c

  Summary:
    This file contains source code necessary to maintain system's polled state
    machines.

  Description:
    This file contains source code necessary to maintain system's polled state
    machines.  It implements the "SYS_Tasks" function that calls the individual
    "Tasks" functions for all the MPLAB Harmony modules in the system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    polled in the system.  These handles are passed into the individual module
    "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2015 released Microchip Technology Inc.  All rights reserved.

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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "system_config.h"
#include "system_definitions.h"
#include "motor_thread.h"
#include "control_thread.h"
#include "uart_thread.h"
#include "message_thread.h"


// *****************************************************************************
// *****************************************************************************
// Section: Local Prototypes
// *****************************************************************************
// *****************************************************************************


 
static void _SYS_Tasks ( void );
static void _MOTOR_THREAD_Tasks(void);
static void _CONTROL_THREAD_Tasks(void);
static void _UART_THREAD_Tasks(void);
static void _MESSAGE_THREAD_Tasks(void);


// *****************************************************************************
// *****************************************************************************
// Section: System "Tasks" Routine
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void SYS_Tasks ( void )

  Remarks:
    See prototype in system/common/sys_module.h.
*/

void SYS_Tasks ( void )
{
    /* Create OS Thread for Sys Tasks. */
    xTaskCreate((TaskFunction_t) _SYS_Tasks,
                "Sys Tasks",
                1024, NULL, 0, NULL);

    /* Create OS Thread for MOTOR_THREAD Tasks. */
    xTaskCreate((TaskFunction_t) _MOTOR_THREAD_Tasks,
                "MOTOR_THREAD Tasks",
                1024, NULL, 1, NULL);

    /* Create OS Thread for CONTROL_THREAD Tasks. */
    xTaskCreate((TaskFunction_t) _CONTROL_THREAD_Tasks,
                "CONTROL_THREAD Tasks",
                1024, NULL, 1, NULL);

    /* Create OS Thread for UART_THREAD Tasks. */
    xTaskCreate((TaskFunction_t) _UART_THREAD_Tasks,
                "UART_THREAD Tasks",
                1024, NULL, 1, NULL);

    /* Create OS Thread for MESSAGE_THREAD Tasks. */
    xTaskCreate((TaskFunction_t) _MESSAGE_THREAD_Tasks,
                "MESSAGE_THREAD Tasks",
                1024, NULL, 1, NULL);

    /**************
     * Start RTOS * 
     **************/
    vTaskStartScheduler(); /* This function never returns. */
}


/*******************************************************************************
  Function:
    void _SYS_Tasks ( void )

  Summary:
    Maintains state machines of system modules.
*/
static void _SYS_Tasks ( void)
{
    while(1)
    {
        /* Maintain system services */
        SYS_DEVCON_Tasks(sysObj.sysDevcon);

        /* Maintain Device Drivers */

        /* Maintain Middleware */

        /* Task Delay */
    }
}


/*******************************************************************************
  Function:
    void _MOTOR_THREAD_Tasks ( void )

  Summary:
    Maintains state machine of MOTOR_THREAD.
*/

static void _MOTOR_THREAD_Tasks(void)
{
    while(1)
    {
        MOTOR_THREAD_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _CONTROL_THREAD_Tasks ( void )

  Summary:
    Maintains state machine of CONTROL_THREAD.
*/

static void _CONTROL_THREAD_Tasks(void)
{
    while(1)
    {
        CONTROL_THREAD_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _UART_THREAD_Tasks ( void )

  Summary:
    Maintains state machine of UART_THREAD.
*/

static void _UART_THREAD_Tasks(void)
{
    while(1)
    {
        UART_THREAD_Tasks();
    }
}


/*******************************************************************************
  Function:
    void _MESSAGE_THREAD_Tasks ( void )

  Summary:
    Maintains state machine of MESSAGE_THREAD.
*/

static void _MESSAGE_THREAD_Tasks(void)
{
    while(1)
    {
        MESSAGE_THREAD_Tasks();
    }
}


/*******************************************************************************
 End of File
 */

