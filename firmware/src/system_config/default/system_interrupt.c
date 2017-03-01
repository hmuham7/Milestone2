/*******************************************************************************
 System Interrupts File

  File Name:
    system_interrupt.c

  Summary:
    Raw ISR definitions.

  Description:
    This file contains a definitions of the raw ISRs required to support the
    interrupt sub-system.

  Summary:
    This file contains source code for the interrupt vector functions in the
    system.

  Description:
    This file contains source code for the interrupt vector functions in the
    system.  It implements the system and part specific vector "stub" functions
    from which the individual "Tasks" functions are called for any modules
    executing interrupt-driven in the MPLAB Harmony system.

  Remarks:
    This file requires access to the systemObjects global data structure that
    contains the object handles to all MPLAB Harmony module objects executing
    interrupt-driven in the system.  These handles are passed into the individual
    module "Tasks" functions to identify the instance of the module to maintain.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2011-2014 released Microchip Technology Inc.  All rights reserved.

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

#include <xc.h>
#include <sys/attribs.h>
#include "motor_thread.h"
#include "control_thread.h"
#include "message_thread.h"
#include "system_definitions.h"
#include "debug.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

    
void IntHandlerDrvTmrInstance0(void)
{
    dbgOutputVal(0x94);
    PLIB_USART_TransmitterEnable (USART_ID_1);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}
    
void IntHandlerDrvTmrInstance1(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
    MsgObj obj;
    obj.Type = SEND_REQUEST;
    dbgPinsDirection();
    dbgOutputVal(0x03);
    switch(MYROVER){
        case SENSOR_ROVER:
            break;
        case FLAG_ROVER:
            obj.Request = FRtoSR;
            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
            break;
        case TAG_ROVER:
            obj.Request = TRtoSR;
            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
            break;
        case CM_ROVER:
            obj.Request = CMRtoSR;
            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
            break;
    }
    
    incrementSystemClock();
    
    portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_4);
}
 void IntHandlerDrvUsartInstance0(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    
    // Transmit code
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_TRANSMIT))
    {
        
        SYS_INT_SourceDisable(INT_SOURCE_USART_1_TRANSMIT);
        while (!PLIB_USART_TransmitterBufferIsFull(USART_ID_1)
                && !uart_queue_empty())
        {
            char buf;
//            int i = UART_THREAD_ReadFromQueue(&buf);
            int i = UART_THREAD_ReadFromQueueFromISR(&buf, &pxHigherPriorityTaskWoken);
            if (i)
            {
                PLIB_USART_TransmitterByteSend(USART_ID_1, buf);
            }
        }
        if(!uart_queue_empty())
            SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_TRANSMIT);
    }
    
    // Receive Code
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_RECEIVE))
    {
        uart_wifly_receive(pxHigherPriorityTaskWoken);
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
    }
    portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
} 
/*******************************************************************************
 End of File
*/

