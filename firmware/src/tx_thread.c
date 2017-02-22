/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    tx_thread.c

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

#include "tx_thread.h"
#include "tx_thread_public.h"
#include "uart.h"
#include "debug.h"
#include "messages.h"
#include "message_thread.h"
#include "message_thread_public.h"

// *****************************************************************************
// *****************************************************************************
// Section: Global Data Definitions
// *****************************************************************************
// *****************************************************************************

QueueHandle_t tx_queue;

#define QUEUE_TYPE          Tx_DataType
#define QUEUE_SIZE          500

/*******************************************************************************
  Function:
    void TX_THREAD_Initialize ( void )

  Remarks:
    See prototype in tx_thread.h.
 */
void TX_THREAD_InitializeQueue() {
    tx_queue = xQueueCreate(QUEUE_SIZE, sizeof(QUEUE_TYPE));
    if(tx_queue == 0) {
        /*Handle this Error*/
        dbgOutputBlock(pdFALSE);
    }
}

void TX_THREAD_Initialize ( void )
{
    TX_THREAD_InitializeQueue();
}
/******************************************************************************
  Function:
    void TX_THREAD_Tasks ( void )

  Remarks:
    See prototype in tx_thread.h.
 */

void TX_THREAD_Tasks ( void )
{
    Tx_DataType obj;
    char message[SIZE];
    DRV_TMR0_Start();
    dbgPinsDirection();
    dbgOutputVal(0x03);
    while(1){
        
        TX_THREAD_ReadFromQueue(&obj);

        int len = messagecreator(message, obj.Data, obj.Destination, obj.MsgCount);
        int i;
        for(i = 0; i < len; i++) {
            Uart_SendToQueue(message[i]);
        }
        PLIB_USART_TransmitterEnable (USART_ID_1);
        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
        //SYS_INT_SourceEnable(INT_SOURCE_USART_1_TRANSMIT);
    }
}

void TX_THREAD_ReadFromQueue(Tx_DataType *pvBuffer) {
    xQueueReceive(tx_queue, pvBuffer, portMAX_DELAY);
}

void TX_THREAD_SendToQueue(Tx_DataType buffer) {
    xQueueSend(tx_queue, &buffer, portMAX_DELAY);
}

void TX_THREAD_SendToQueueISR(Tx_DataType buffer, BaseType_t *pxHigherPriorityTaskWoken) {
    xQueueSendFromISR(tx_queue, &buffer, pxHigherPriorityTaskWoken);
}
/*******************************************************************************
 End of File
 */
