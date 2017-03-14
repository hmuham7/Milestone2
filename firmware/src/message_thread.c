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
#include "json_parser.h"
#include "messages.h"
#include "debug.h"

QueueHandle_t message_queue;
static int LocalTime;

#define QUEUE_TYPE             MsgObj
#define QUEUE_SIZE             100

/*******************************************************************************
  Function:
    void MESSAGE_CONTROLLER_THREAD_Initialize ( void )

  Remarks:
    See prototype in message_controller_thread.h.
 */

void MESSAGE_THREAD_InitializeQueue() {
    message_queue = xQueueCreate(QUEUE_SIZE, sizeof(QUEUE_TYPE));
    if(message_queue == 0) {
        //dbgOutputBlock(pdFALSE);
    }
}

void MESSAGE_THREAD_Initialize ( void )
{
    dbgOutputLoc(ENTER_MESSAGE_THREAD_INITIALIZER);
    MESSAGE_THREAD_InitializeQueue();
    resetLocalTime();
    
    MsgObj obj;
    obj.Type = SEND_REQUEST;
    obj.Request = FRtoCMR;
    MESSAGE_THREAD_SendToQueue(&obj);  
}


/******************************************************************************
  Function:
    void MESSAGE_CONTROLLER_THREAD_Tasks ( void )

  Remarks:
    See prototype in message_controller_thread.h.
 */

void MESSAGE_THREAD_Tasks ( void )
{
    dbgOutputLoc(ENTER_MESSAGE_THREAD_TASKS);
    
    DRV_TMR0_Start(); // timer start for 100ms Interrupt driven Request Generation
    resetLocalTime();
    
    RoverData rData;
    memset(&rData, 0, sizeof(RoverData));
    
    StatObjectType statObject;
    memset(&statObject, 0, sizeof(StatObjectType));
    
    statObject.Request_To_CMrover           = 0;
    statObject.Request_To_Flagrover         = 0;
    statObject.Request_To_Sensorrover       = 0;
    statObject.Request_To_Tagrover          = 0;
    statObject.Request_From_CMrover         = 0;
    statObject.Request_From_Flagrover       = 0;
    statObject.Request_From_Sensorrover     = 0;
    statObject.Request_From_Tagrover        = 0;
    statObject.Request_From_Server          = 0;
    statObject.Response_To_CMrover          = 0;
    statObject.Response_To_Flagrover        = 0;
    statObject.Response_To_Sensorrover      = 0;
    statObject.Response_To_Tagrover         = 0;
    statObject.Response_From_CMrover        = 0;
    statObject.Response_From_Flagrover      = 0;
    statObject.Response_From_Sensorrover    = 0;
    statObject.Response_From_Tagrover       = 0;
    
    statObject.GoodCount = 0x00;
    statObject.ErrorCount = 0x00;
    
    type_t type = strange;
    items_t items[12];
    int numItems;
    while(1) {
        
        dbgOutputLoc(ENTER_MESSAGE_THREAD_WHILE);
        MsgObj obj;
        memset(&obj, 0, sizeof(MsgObj));

        Tx_DataType tx_obj;
        memset(&tx_obj, 0, sizeof(Tx_DataType));

        MESSAGE_THREAD_ReadFromQueue(&obj);
        
        switch(obj.Type) {
            case SEND_RESPONSE: {
                dbgOutputLoc(0x98);
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
                                statObject.Request_From_Sensorrover++;
                                statObject.Response_To_Sensorrover++;
                                break;
                            case FLAG_ROVER:
                                statObject.Request_From_Flagrover++;
                                statObject.Response_To_Flagrover++;
                                break;
                            case TAG_ROVER:
                                statObject.Request_From_Tagrover++;
                                statObject.Response_To_Tagrover++;
                                break;
                            case CM_ROVER:
                                statObject.Request_From_CMrover++;
                                statObject.Response_To_CMrover++;
                                break;
                            case SERVER:
                                statObject.Request_From_Server++;
                                statObject.Response_To_Server++;
                                break;
                            default:
                                continue;
                        }

                        int i = 0;
                        tx_obj.Destination = obj.External.Source;
                        sprintf(tx_obj.Data, "{\"type\":\"Response\"");
                        
                        unsigned int JSONReqRcv     = statObject.Request_From_Sensorrover + statObject.Request_From_Flagrover + statObject.Request_From_Tagrover + statObject.Request_From_CMrover + statObject.Request_From_Server;
                        unsigned int JSONResRcv     = statObject.Response_From_Sensorrover + statObject.Response_From_Flagrover + statObject.Response_From_Tagrover + statObject.Response_From_CMrover + statObject.Response_From_Server;
                        unsigned int JSONReqSent    = statObject.Request_To_Sensorrover + statObject.Request_To_Flagrover + statObject.Request_To_Tagrover + statObject.Request_To_CMrover + statObject.Request_To_Server;
                        unsigned int JSONResSent    = statObject.Response_To_Sensorrover + statObject.Response_To_Flagrover + statObject.Response_To_Tagrover + statObject.Response_To_CMrover + statObject.Response_To_Server;
                        
                        for(i = 0; i < numItems; i++) {
                            switch(items[i]) {
                                case CommStats_flag_rover: case CommStats_sensor_rover: case CommStats_tag_rover: case CommStats_cm_rover: {
                                    sprintf(tx_obj.Data+strlen(tx_obj.Data), 
                                        ",\"CommStats\":{"
                                        "\"Name\":\"%s\","
                                        "\"#GoodMsgs\":\"%d\","
                                        "\"#MsgsDropd\":\"%d\","    
                                        "\"#JReqRcv\":\"%d\","
                                        "\"#JResRcv\":\"%d\","
                                        "\"#JReqSent\":\"%d\","
                                        "\"#JResSent\":\"%d\"}",
                                        ROVER_NAME,
                                        statObject.GoodCount,
                                        statObject.ErrorCount,
                                        JSONReqRcv,
                                        JSONResRcv,
                                        JSONReqSent,
                                        JSONResSent
                                        );
                                    break;
                                }
//                                case EncoderData: {
//                                    sprintf(tx_obj.Data+strlen(tx_obj.Data), ",\"SensorData\":\"%0.02f\"", rData.sensordata);
//                                    break;
//                                }
                                case msLocalTime:{
                                    sprintf(tx_obj.Data+strlen(tx_obj.Data), ",\"msLocalTime\":\"%d ms\"", getLocalTime() * 100);
                                    break;
                                }
                                default:
                                    break;
                            }
                        }
                        sprintf(tx_obj.Data+strlen(tx_obj.Data),"}");
                        dbgOutputLoc(0x96);
                        char message[SIZE];
                        int len = messagecreator(message, tx_obj.Data, tx_obj.Destination, tx_obj.MsgCount);
                        int k;
                        for(k = 0; k < len; k++) {
                            UART_THREAD_SendToQueue(message[k]);
                        }
                        break;
                    }
                    case response: {
                        switch(obj.External.Source) {
                            case FLAG_ROVER: {
                                statObject.Response_From_Flagrover++;
                                break;
                            }
                            case TAG_ROVER: {
                                statObject.Response_From_Tagrover++;
                                break;
                            }
                            case CM_ROVER: {
                                statObject.Response_From_CMrover++;
                                break;
                            }
                            case SENSOR_ROVER: {
                                statObject.Response_From_Sensorrover++;
                                break;
                            }
                            case SERVER: {
                                statObject.Response_From_Server++;
                                break;
                            }
                            default: {
                                break;
                            }
                        }
                        unsigned int ResRcvd = statObject.Response_From_Flagrover + statObject.Response_From_Sensorrover + statObject.Response_From_Tagrover + statObject.Response_From_CMrover + statObject.Response_From_Server;
                        
                        tx_obj.Destination = obj.External.Source;
                        sprintf(tx_obj.Data, "{\"type\":\"UpdatetoResponse\"");
                        sprintf(tx_obj.Data+strlen(tx_obj.Data),
                                ",\"CommStats\":{"
                                "\"Name\":\"%s\","
                                "\"#ResponsesRcvd\":\"%d\",",
                                ROVER_NAME,
                                ResRcvd
                                );
                        tx_obj.MsgCount = (char) ResRcvd;
                        char message[SIZE];
                        int len = messagecreator(message, tx_obj.Data, tx_obj.Destination, tx_obj.MsgCount);
                        dbgOutputLoc(0x97);
                        int k;
                        for(k = 0; k < len; k++) {
                            UART_THREAD_SendToQueue(message[k]);
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
            case SEND_REQUEST: 
            {
                switch(obj.Request) {
                    case FRtoSR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_sensor_rover\"]}");
                        tx_obj.Destination = SENSOR_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Sensorrover;
                        statObject.Request_To_Sensorrover++;
                        break;
                    }
                    case FRtoTR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_tag_rover\"]}");
                        tx_obj.Destination = TAG_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Tagrover;
                        statObject.Request_To_Tagrover++;
                        break;
                    }
                    case FRtoCMR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_cm_rover\"]}");
                        tx_obj.Destination = CM_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_CMrover;
                        statObject.Request_To_CMrover++;
                        break;
                    }
                    case TRtoSR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_sensor_rover\"]}");
                        tx_obj.Destination = SENSOR_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Sensorrover;
                        statObject.Request_To_Sensorrover++;
                        break;
                    }
                    case TRtoFR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_flag_rover\"]}");
                        tx_obj.Destination = FLAG_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Flagrover;
                        statObject.Request_To_Flagrover++;
                        break;
                    }
                    case TRtoCMR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_cm_rover\"]}");
                        tx_obj.Destination = CM_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_CMrover;
                        statObject.Request_To_CMrover++;
                        break;
                    }
                    case CMRtoSR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_sensor_rover\"]}");
                        tx_obj.Destination = SENSOR_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Sensorrover;
                        statObject.Request_To_Sensorrover++;
                        break;
                    }
                    case CMRtoTR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_tag_rover\"]}");
                        tx_obj.Destination = TAG_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Tagrover;
                        statObject.Request_To_Tagrover++;
                        break;
                    }
                    case CMRtoFR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_flag_rover\"]}");
                        tx_obj.Destination = FLAG_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Flagrover;
                        statObject.Request_To_Flagrover++;
                        break;
                    }
                    case SRtoFR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_flag_rover\"]}");
                        tx_obj.Destination = FLAG_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Flagrover;
                        statObject.Request_To_Flagrover++;
                        break;
                    }
                    case SRtoTR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_tag_rover\"]}");
                        tx_obj.Destination = TAG_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_Tagrover;
                        statObject.Request_To_Tagrover++;
                        break;
                    }
                    case SRtoCMR: {
                        sprintf(tx_obj.Data, "{\"type\":\"Request\",\"items\":[\"msLocalTime\",\"CommStats_cm_rover\"]}");
                        tx_obj.Destination = CM_ROVER;
                        tx_obj.MsgCount = (char) statObject.Request_To_CMrover;
                        statObject.Request_To_CMrover++;
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
                break;
            }
            default: {
                break;
            }
    }
}
}

void MESSAGE_THREAD_ReadFromQueue(MsgObj* pvBuffer) {
    xQueueReceive(message_queue, pvBuffer, portMAX_DELAY);
}

void MESSAGE_THREAD_SendToQueue(MsgObj* buffer) {
    xQueueSendToBack(message_queue, buffer, portMAX_DELAY);
}

void MESSAGE_THREAD_SendToQueueISR(MsgObj buffer, BaseType_t *pxHigherPriorityTaskWoken) {
    xQueueSendToBackFromISR(message_queue, &buffer, pxHigherPriorityTaskWoken);
}

void resetLocalTime(){
    LocalTime = 0;
}

void addLocalTime(){
    LocalTime++;
}

int getLocalTime(){
    return LocalTime;
}

/*******************************************************************************
 End of File
 */
