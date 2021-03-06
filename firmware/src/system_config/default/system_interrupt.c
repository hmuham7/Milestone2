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
#include "debug.h"
#include "uart_thread.h"
#include "message_thread.h"
#include "system_definitions.h"

// *****************************************************************************
// *****************************************************************************
// Section: System Interrupt Vector Functions
// *****************************************************************************
// *****************************************************************************

    
void IntHandlerDrvTmrInstance0(void)
{
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    
    dbgOutputLoc(ENTER_TIMER_ISR0);
    dbgOutputLoc(0x29);
//    MsgObj obj;
//    obj.Type = SEND_REQUEST;
//    
//    switch(MYROVER){
//        dbgOutputLoc(ENTER_TIMER_IR0_SWITCH_CASE);
//        case FLAG_ROVER:
//            obj.Request = FRtoSR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            obj.Request = FRtoTR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            obj.Request = FRtoCMR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            break;
//        case TAG_ROVER:
//            obj.Request = TRtoSR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            obj.Request = TRtoFR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            obj.Request = TRtoCMR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            break;
//        case CM_ROVER:
//            obj.Request = CMRtoSR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            obj.Request = CMRtoTR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            obj.Request = CMRtoFR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            break;
//        case SENSOR_ROVER:
//            obj.Request = SRtoTR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            obj.Request = SRtoFR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            obj.Request = SRtoCMR;
//            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
//            break;
//    }
    
    addLocalTime();
    
    portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
    PLIB_INT_SourceFlagClear(INT_ID_0,INT_SOURCE_TIMER_2);
}

void IntHandlerDrvUsartInstance0(void)
{
    dbgOutputLoc(ENTER_UART_ISR);
    BaseType_t pxHigherPriorityTaskWoken = pdFALSE;
    
    ///////////////////// Receive Code //////////////////////////////////////////////////
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_RECEIVE) && !DRV_USART0_ReceiverBufferIsEmpty())
    {
        uart_wifly_receive(pxHigherPriorityTaskWoken);
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_RECEIVE);
    }
    
    ////////////////////////// Transmit code //////////////////////////////////////////////
    if(SYS_INT_SourceStatusGet(INT_SOURCE_USART_1_TRANSMIT) && !(DRV_USART_TRANSFER_STATUS_TRANSMIT_FULL & DRV_USART0_TransferStatus())) 
    {
        dbgOutputLoc(ENTER_UART_TX_IF);
        
        SYS_INT_SourceDisable(INT_SOURCE_USART_1_TRANSMIT);
        while (!PLIB_USART_TransmitterBufferIsFull(USART_ID_1)
                && !uart_queue_empty())
        {
            dbgOutputLoc(ENTER_UART_TX_WHILE);
            dbgOutputLoc(BEFORE_RECEIVE_ISR);
            
            char byte = UART_THREAD_ReadFromQueueFromISR(pxHigherPriorityTaskWoken);
            
            dbgOutputLoc(AFTER_RECEIVE_ISR);
            dbgOutputLoc(BEFORE_SEND_TX_BYTE);
            
            PLIB_USART_TransmitterByteSend(USART_ID_1, byte);
            
            dbgOutputLoc(AFTER_SEND_TX_BYTE);
        }
        if(!uart_queue_empty())
            SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
        SYS_INT_SourceStatusClear(INT_SOURCE_USART_1_TRANSMIT);
    }

    DRV_USART_TasksReceive(sysObj.drvUsart0);
    DRV_USART_TasksError(sysObj.drvUsart0);
    portEND_SWITCHING_ISR(pxHigherPriorityTaskWoken);
    dbgOutputLoc(LEAVE_UART_ISR);
}
/*******************************************************************************
 End of File
*/

