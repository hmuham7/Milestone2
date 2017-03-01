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
#include "messages.h"
#include "message_thread.h"


#include <stdbool.h>


QueueHandle_t uart_queue;

#define QUEUE_TYPE char
#define QUEUE_SIZE 4096

/*******************************************************************************
  Function:
    void UART_THREAD_Initialize ( void )

  Remarks:
    See prototype in uart_thread.h.
 */

void UART_THREAD_Initialize ( void )
{
    UART_THREAD_InitializeQueue();
    SYS_INT_SourceEnable(INT_SOURCE_USART_1_RECEIVE);
}

void UART_THREAD_InitializeQueue() {
    uart_queue = xQueueCreate(QUEUE_SIZE, sizeof(QUEUE_TYPE));
    if(uart_queue == 0) {
        dbgOutputBlock(pdFALSE);
    }
}

/******************************************************************************
  Function:
    void UART_THREAD_Tasks ( void )

  Remarks:
    See prototype in uart_thread.h.
 */

void UART_THREAD_Tasks ( void )
{
    dbgPinsDirection();
    uart_wifly_init();
    dbgOutputVal(0x04);
    MsgObj obj;
    obj.Type = SEND_RESPONSE;
    dbgOutputVal(0x03);
    char c;
    while(1){
        
        UART_THREAD_ReadFromQueue(&c);
        
        bool no_error = messageparser(c, obj.External.Data, &obj.External.Source, &obj.External.MsgCount, &obj.External.Error);
        dbgOutputVal(no_error);
        
        if(no_error) {
            MESSAGE_THREAD_SendToQueue(obj);
        }
        else if(obj.External.Error) {
            MESSAGE_THREAD_SendToQueue(obj);
        }
    }
}

void UART_THREAD_SendToQueue(char buffer) {
    xQueueSend(uart_queue, &buffer, portMAX_DELAY);
    PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
}

void UART_THREAD_SendToQueueISR(char buffer, BaseType_t *pxHigherPriorityTaskWoken) {
    xQueueSendFromISR(uart_queue, &buffer, pxHigherPriorityTaskWoken);
}

int UART_THREAD_ReadFromQueue(char* pvBuffer) {
    int ret = xQueueReceive(uart_queue, pvBuffer, portMAX_DELAY);
    return ret;
}

int UART_THREAD_ReadFromQueueFromISR(char* pvBuffer, BaseType_t *pxHigherPriorityTaskWoken) {
    int ret = xQueueReceiveFromISR(uart_queue, pvBuffer, pxHigherPriorityTaskWoken);
    return ret;
}

/////////////////////////////////////////////////////////////////
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
    while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1)){
        uint8_t buff[8] = {0};
        int i = 0;
        while(PLIB_USART_ReceiverDataIsAvailable(USART_ID_1) && (i < 8)){
            buff[i] = PLIB_USART_ReceiverByteReceive(USART_ID_1);
            i++;
        }
        send_uart_byte(i, &buff[0], pxHigherPriorityTaskWoken);
    }
}

void send_uart_byte(uint8_t numBytes, uint8_t *bytes, BaseType_t pxHigherPriorityTaskWoken)
{
    uint8_t buff[10] = {0};
    buff[0] = 'u';
    buff[1] = numBytes;
    
    int i;
    for(i = 0; i < numBytes; i++){
        buff[i+2] = bytes[i];
    }
    
    UART_THREAD_SendToQueueISR(buff[0], &pxHigherPriorityTaskWoken);
}

/*******************************************************************************
 End of File
 */
