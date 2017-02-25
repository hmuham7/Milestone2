/*******************************************************************************
  MPLAB Harmony Application Source File

  Company:
    Microchip Technology Inc.

  File Name:
    message_controller_thread.c

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

#include "message_thread.h"
#include "message_thread_public.h"
#include "json_parser.h"
#include "messages.h"

QueueHandle_t _queue;
static int systemClock;

#define QUEUE_TYPE             MsgObj
#define QUEUE_SIZE             500

/*******************************************************************************
  Function:
    void MESSAGE_CONTROLLER_THREAD_Initialize ( void )

  Remarks:
    See prototype in message_controller_thread.h.
 */

void MESSAGE_THREAD_InitializeQueue() {
    _queue = xQueueCreate(QUEUE_SIZE, sizeof(QUEUE_TYPE));
}

void MESSAGE_THREAD_Initialize ( void )
{
    MESSAGE_THREAD_InitializeQueue();
    resetSystemClock();
    dbgPinsDirection();
    dbgOutputVal(0x07);
}


/******************************************************************************
  Function:
    void MESSAGE_CONTROLLER_THREAD_Tasks ( void )

  Remarks:
    See prototype in message_controller_thread.h.
 */

void MESSAGE_THREAD_Tasks ( void )
{
    InternalData internalData;
    memset(&internalData, 0, sizeof(InternalData));
    
    StatObjectType statObject;
    memset(&statObject, 0, sizeof(StatObjectType));
    
    type_t type = strange;
    items_t items[12];
    int numItems;
    dbgPinsDirection();
    dbgOutputVal(0x03);
    
    DRV_TMR0_Start();
    DRV_TMR1_Start();
    dbgPinsDirection();
    dbgOutputVal(0x05);
    while(1) {
        
        initialize_parser();
        MsgObj obj;
        memset(&obj, 0, sizeof(MsgObj));

        Tx_DataType tx_obj;
        memset(&tx_obj, 0, sizeof(Tx_DataType));

        MESSAGE_THREAD_ReadFromQueue(&obj);
        
        switch(obj.Type) {
            case SEND_RESPONSE: {
                
                if(obj.External.Error) {
                    statObject.ErrorCount++;
                    continue;
                }
                
                statObject.GoodCount++;
                
                json_parser(obj.External.Data, &type, items,  &numItems);
                
                switch(type) {
                    case request: {
                        switch(obj.External.Source) {
                            case SENSOR_ROVER:
                                if ((obj.External.MsgCount - statObject.Req_From_Sensorrover) < 0) {
                                    statObject.MessagesDropped = (256 - statObject.Req_From_Sensorrover) + obj.External.MsgCount;
                                } else {
                                    statObject.MessagesDropped = (obj.External.MsgCount - statObject.Req_From_Sensorrover);
                                }
                                statObject.Req_From_Sensorrover++;
                                break;
                            case FLAG_ROVER:
                                if ((obj.External.MsgCount - statObject.Req_From_Flagrover) < 0) {
                                    statObject.MessagesDropped = (256 - statObject.Req_From_Flagrover) + obj.External.MsgCount;
                                } else {
                                    statObject.MessagesDropped = (obj.External.MsgCount - statObject.Req_From_Flagrover);
                                }
                                statObject.Req_From_Flagrover++;
                                break;
                            case TAG_ROVER:
                                if ((obj.External.MsgCount - statObject.Req_From_Tagrover) < 0) {
                                    statObject.MessagesDropped = (256 - statObject.Req_From_Tagrover) + obj.External.MsgCount;
                                } else {
                                    statObject.MessagesDropped = (obj.External.MsgCount - statObject.Req_From_Tagrover);
                                }
                                statObject.Req_From_Tagrover++;
                                break;
                            case CM_ROVER:
                                if ((obj.External.MsgCount - statObject.Req_From_CMrover) < 0) {
                                    statObject.MessagesDropped = (256 - statObject.Req_From_CMrover) + obj.External.MsgCount;
                                } else {
                                    statObject.MessagesDropped = (obj.External.MsgCount - statObject.Req_From_CMrover);
                                }
                                statObject.Req_From_CMrover++;
                                break;
                            case SERVER:
                                break;
                            default:
                                continue;
                        }

                        int i = 0;
                        tx_obj.Destination = obj.External.Source;
                        sprintf(tx_obj.Data, "{\"type\":\"Response\"");
                        for(i = 0; i < numItems; i++) {
                            switch(items[i]) {
                                case CommStats_flag_rover: case CommStats_sensor_rover: case CommStats_tag_rover: case CommStats_cm_rover: {
                                    sprintf(tx_obj.Data+strlen(tx_obj.Data), 
                                        ",\"CommStats%s\":{"
                                        "\"myName\":\"%s\","
                                        "\"numGoodMessagesRecved\":\"%d\","
                                        "\"numCommErrors\":\"%d\","
                                        "\"numMessagesDropped\":\"%d\","    
                                        "\"numJSONRequestsRecved\":\"%d\","
                                        "\"numJSONResponsesRecved\":\"%d\","
                                        "\"numJSONRequestsSent\":\"%d\","
                                        "\"numJSONResponsesSent\":\"%d\"}",
                                        ROVER_NAME,
                                        statObject.GoodCount,
                                        statObject.ErrorCount,
                                        statObject.MessagesDropped,
                                        statObject.Req_From_Sensorrover + statObject.Req_From_Flagrover + statObject.Req_From_Tagrover + statObject.Req_From_CMrover,
                                        statObject.Res_From_Sensorrover + statObject.Res_From_Flagrover + statObject.Res_From_Tagrover + statObject.Res_From_CMrover,
                                        statObject.Req_To_Sensorrover + statObject.Req_To_Flagrover + statObject.Req_To_Tagrover + statObject.Req_To_CMrover,
                                        statObject.Res_To_Sensorrover + statObject.Res_To_Flagrover + statObject.Res_To_Tagrover + statObject.Res_To_CMrover
                                        );
                                    tx_obj.Destination = SERVER;
                                    break;
                                }
                                case SensorData: {
                                    sprintf(tx_obj.Data+strlen(tx_obj.Data), ",\"SensorData\":\"%0.02f\"", internalData.sensordata);
                                    break;
                                }
                                case msLocalTime:{
                                    sprintf(tx_obj.Data+strlen(tx_obj.Data), ",\"msLocalTime\":\"%d\"", getSystemClock() * 10);
                                    tx_obj.Destination = obj.External.Source;
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                        sprintf(tx_obj.Data+strlen(tx_obj.Data),"}");
                        switch(obj.External.Source) {
                            case SENSOR_ROVER: {
                                tx_obj.MsgCount = statObject.Res_To_Sensorrover;
                                statObject.Res_To_Sensorrover++;
                                break;
                            }
                            case FLAG_ROVER: {
                                tx_obj.MsgCount = statObject.Res_To_Flagrover;
                                statObject.Res_To_Flagrover++;
                                break;
                            }
                            case TAG_ROVER: {
                                tx_obj.MsgCount = statObject.Res_To_Tagrover;
                                statObject.Res_To_Tagrover++;
                                break;
                            }
                            case CM_ROVER: {
                                tx_obj.MsgCount = statObject.Res_To_CMrover;
                                statObject.Res_To_CMrover++;
                                break;
                            }
                            default: {
                                break;
                            }
                        }
                        char message[SIZE];
                        int len = messagecreator(message, tx_obj.Data, tx_obj.Destination, tx_obj.MsgCount);
                        int k;
                        for(k = 0; k < len; k++) {
                            UART_THREAD_SendToQueue(message[k]);
                        }
                        PLIB_USART_TransmitterEnable (USART_ID_1);
                        PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
                        break;
                    }
                    case response: {
                        switch(obj.External.Source) {
                            case FLAG_ROVER: {
                                statObject.Res_From_Flagrover++;
                                break;
                            }
                            case TAG_ROVER: {
                                statObject.Res_From_Tagrover++;
                                break;
                            }
                            case CM_ROVER: {
                                statObject.Res_From_CMrover++;
                                break;
                            }
                            default: {
                                break;
                            }
                        }
                        break;
                    }
                    case strange: {
                        break;
                    }
                    default: {
                        break;
                    }
                }
                break;
            }
            case SEND_REQUEST: {
                switch(obj.Request) {
                    case FRtoSR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"Obstacles\",\"YOUR_COORDINATES\",\"YOUR_Orientation\"]}");
                        tx_obj.Destination = SENSOR_ROVER;
                        tx_obj.MsgCount = statObject.Req_To_Sensorrover;
                        statObject.Req_To_Sensorrover++;
                        break;
                    }
                    case TRtoSR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"Obstacles\",\"YOUR_COORDINATES\",\"YOUR_Orientation\"]}");
                        tx_obj.Destination = SENSOR_ROVER;
                        tx_obj.MsgCount = statObject.Req_To_Sensorrover;
                        statObject.Req_To_Sensorrover++;
                        break;
                    }
                    case CMRtoSR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"Obstacles\",\"YOUR_COORDINATES\",\"YOUR_Orientation\"]}");
                        tx_obj.Destination = SENSOR_ROVER;
                        tx_obj.MsgCount = statObject.Req_To_Sensorrover;
                        statObject.Req_To_Sensorrover++;
                        break;
                    }
                    default: {
                        continue;
                    }
                }
                char message[SIZE];
                int len = messagecreator(message, tx_obj.Data, tx_obj.Destination, tx_obj.MsgCount);
                int i;
                for(i = 0; i < len; i++) {
                    UART_THREAD_SendToQueue(message[i]);
                }
                PLIB_USART_TransmitterEnable (USART_ID_1);
                PLIB_INT_SourceEnable(INT_ID_0, INT_SOURCE_USART_1_TRANSMIT);
                break;
            }
            case UPDATE: {
                switch(obj.Update.Type) {
                    case LOCATION:{
                        internalData.location = obj.Update.Data.location;
                        break;
                    }
                    case ORIENTATION: {
                        internalData.orientation = obj.Update.Data.orientation;
                        break;
                    }
                    case SENSORDATA: {
                        internalData.sensordata = obj.Update.Data.sensordata;
                        break;
                    }
                    default: {
                        break;
                    }
                }
                break;
            }
            default: {
                break;
            }
    }
}
}

void MESSAGE_THREAD_ReadFromQueue(MsgObj* pvBuffer) {
    xQueueReceive(_queue, pvBuffer, portMAX_DELAY);
}

void MESSAGE_THREAD_SendToQueue(MsgObj buffer) {
    xQueueSend(_queue, &buffer, portMAX_DELAY);
}

void MESSAGE_THREAD_SendToQueueISR(MsgObj buffer, BaseType_t *pxHigherPriorityTaskWoken) {
    xQueueSendFromISR(_queue, &buffer, pxHigherPriorityTaskWoken);
}

void resetSystemClock(){
    systemClock = 0;
}

void incrementSystemClock(){
    systemClock++;
}

int getSystemClock(){
    return systemClock;
}

/*******************************************************************************
 End of File
 */
