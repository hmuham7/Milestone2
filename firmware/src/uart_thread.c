/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    uart.c

  Summary:
    This file contains the source code for the MPLAB Harmony application.

  Description:
    This file contains the source code for the MPLAB Harmony application.  It
    implements the logic of the application's state machine and it may call
    API routines of other MPLAB Harmony modules in the system, such as drivers,
    system services, and middleware.  However, it does not call any of the
    system interfaces (such as the "Initialize" and "Tasks" functions) of any of
    the modules in the system or make any assumptions about when those functions
    are called.  That is the responsibility of the configuration-specific system
    files.
 *******************************************************************************/

// DOM-IGNORE-BEGIN
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
// DOM-IGNORE-END


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include "debug.h"
//#include "app.h"
#include <stdbool.h>
#include "message_thread.h"


QueueHandle_t uart_queue;

#define QUEUE_TYPE              char
#define QUEUE_SIZE              1024


void UART_THREAD_InitializeQueue() {
    uart_queue = xQueueCreate(QUEUE_SIZE, sizeof(QUEUE_TYPE));
    if(uart_queue == 0) {
        //dbgOutputBlock(pdFALSE);
    }
}
/*******************************************************************************
  Function:
    void UART_THREAD_Initialize ( void )

  Remarks:
    See prototype in uart_thread.h.
 */

void UART_THREAD_Initialize ( void )
{
    UART_THREAD_InitializeQueue();
    dbgOutputLoc(ENTER_UART_THREAD_INITIALIZER);
}



/******************************************************************************
  Function:
    void UART_THREAD_Tasks ( void )

  Remarks:
    See prototype in uart_thread.h.
 */

void UART_THREAD_Tasks ( void )
{
    dbgOutputLoc(ENTER_UART_THREAD_TASKS);
    uart_wifly_init();
    
    while(1){
        dbgOutputLoc(0x88);
    }
}

int UART_THREAD_ReadFromQueue(char* pvBuffer) {
    int ret = xQueueReceive(uart_queue, pvBuffer, portMAX_DELAY);
    return ret;
}

void UART_THREAD_SendToQueue(char buffer) {
    xQueueSendToBack(uart_queue, &buffer, portMAX_DELAY);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
}

char UART_THREAD_ReadFromQueueFromISR( BaseType_t pxHigherPriorityTaskWoken) {
    char buff;
    xQueueReceiveFromISR(uart_queue, &buff, &pxHigherPriorityTaskWoken);
    return buff;
}

/////////////////////////////
bool uart_queue_empty()
{
    return xQueueIsQueueEmptyFromISR(uart_queue);
}

void uart_wifly_init()
{
    DRV_USART0_Open( DRV_USART_INDEX_1, 
        (DRV_IO_INTENT_READWRITE | DRV_IO_INTENT_BLOCKING) );
    SYS_INT_SourceDisable(INT_SOURCE_USART_1_ERROR);
}

void uart_wifly_receive(BaseType_t pxHigherPriorityTaskWoken)
{
    MsgObj obj;
    bool no_error;
    char value;
    obj.Type = SEND_RESPONSE;
    while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
        value = PLIB_USART_ReceiverByteReceive(USART_ID_1);
        no_error = messageparser(value, obj.External.Data, &obj.External.Source, &obj.External.MsgCount, &obj.External.Error, &obj.External.MessagesDropped);
        dbgOutputVal(0x99);
        if(no_error) {
            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
        }
        else if(obj.External.Error) {
            MESSAGE_THREAD_SendToQueueISR(obj, &pxHigherPriorityTaskWoken);
        }
    }
    
}

/*******************************************************************************
 End of File
 */
