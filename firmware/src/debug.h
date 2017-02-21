/* 
 * File:   debug.h
 * Author: Team8
 * 
 *
 */

#include "system_config.h"
#include "system_definitions.h"

#ifndef DEBUG_H
#define	DEBUG_H

#ifdef	__cplusplus
extern "C" {
#endif
    // Milestone 1 debug values
#define ENTER_TASK     0x01
#define BEFORE_WHILE   0x02
#define BEFORE_SEND    0x03
#define BEFORE_RECEIVE 0x04
#define AFTER_SEND     0x05
#define AFTER_RECEIVE  0x06
#define ENTER_ISR      0x07
#define LEAVE_ISR      0x08
#define UART           0x09
#define ERROR          0xFF
    // Milestone 2 ISR debug values
#define ENTER_USART0_ISR                       0x11
#define LEAVE_USART0_ISR                       0x12
#define BEFORE_SEND_TO_QUEUE_USART0_ISR        0x13
#define AFTER_SEND_TO_QUEUE_USART0_ISR         0x14
#define BEFORE_RECEIVE_FR_QUEUE_USART0_ISR     0x15
#define AFTER_RECEIVE_FR_QUEUE_USART0_ISR      0x16
    // Milestone 2 RX_thread debug values
#define ENTER_RXTHREAD                                  0x21
#define LEAVE_RXTHREAD                                  0x22
#define BEFORE_WHILELOOP_RXTHREAD                       0x23
#define BEFORE_SEND_TO_QUEUE_RXTHREAD                   0x24
#define AFTER_SEND_TO_QUEUE_RXTHREAD                    0x25
#define BEFORE_RECEIVE_FR_QUEUE_RXTHREAD                0x26
#define AFTER_RECEIVE_FR_QUEUE_RXTHREAD                 0x27
#define BEFORE_RECEIVE_FR_QUEUE_READFROMQUEUE_RXTHREAD  0x28
#define AFTER_RECEIVE_FR_QUEUE_READFROMQUEUE_RXTHREAD   0x29
    // Milestone 2 RX_thread debug values
#define ENTER_TXTHREAD                      0x31
#define BEFORE_WHILELOOP_TXTHREAD           0x32
#define BEFORE_SEND_TO_QUEUE_TXTHREAD       0x33
#define AFTER_SEND_TO_QUEUE_TXTHREAD        0x34
#define BEFORE_RECEIVE_FR_QUEUE_TXTHREAD    0x35
#define AFTER_RECEIVE_FR_QUEUE_TXTHREAD     0x36
    // Milestone 2 Message.c debug values
#define ENTER_CHECKSUM_MESSAGE_C                                0x40
#define LEAVE_CHECKSUM_MESSAGE_C                                0x41
#define ENTER_CREATEMESSAGE_MESSAGE_C                           0x42
#define LEAVE_CREATEMESSAGE_MESSAGE_C                           0x43
#define ENTER_PARSEMESSAGE_MESSAGE_C                            0x44
#define LEAVE_PARSEMESSAGE_MESSAGE_C                            0x45
#define CASE_IDLE_STATE_PARSEMESSAGE_MESSAGE_C                  0x46
#define CASE_CHECK_DESTINATION_CHAR_PARSEMESSAGE_MESSAGE_C      0x47
#define CASE_CHECK_SOURCE_CHAR_PARSEMESSAGE_MESSAGE_C           0x48
#define CASE_CHECK_MESSAGE_COUNT_PARSEMESSAGE_MESSAGE_C         0x49
#define CASE_GET_DATALENGTH_UPPER_PARSEMESSAGE_MESSAGE_C        0x50
#define CASE_GET_DATALENGTH_LOWER_PARSEMESSAGE_MESSAGE_C        0x51
#define CASE_GET_DATA_PARSEMESSAGE_MESSAGE_C                    0x52
#define CASE_GET_CHECK_SUM_PARSEMESSAGE_MESSAGE_C               0x53
#define CASE_CHECK_ENDCHAR_PARSEMESSAGE_MESSAGE_C               0x54
#define BEFORE_FIRST_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C         0x55
#define IN_FIRST_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C             0x56
#define IN_SECOND_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C            0x57
#define IN_THIRD_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C             0x58
#define AFTER_THIRD_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C          0x59
    //Milestone2 message_controller.c debug values
#define ENTER_MESSAGE_CONTROLLER_THREAD                             0x60
#define LEAVE_MESSAGE_CONTROLLER_THREAD                             0x61
#define CASE_EXTERNAL_REQUEST_RESPONSE_MESSAGE_CONTROLLER_THREAD    0x62
#define CASE_SEND_REQUEST_MESSAGE_CONTROLLER_THREAD                 0x63
#define CASE_UPDATE_MESSAGE_CONTROLLER_THREAD                       0x64
#define BEFORE_READ_FROM_Q_MESSAGE_CONTROLLER_THREAD                0x65
#define AFTER_READ_FROM_Q_MESSAGE_CONTROLLER_THREAD                 0x66
    
void dbgOutputVal (unsigned char outVal);

void dbgOutputLoc (unsigned char outVal);

void dbgUARTVal( unsigned char outVal);

void dbgPinsDirection();

void dbgOutputBlock(int outVal);

void dbgOutputBlockISR(int outVal);

void error ();

#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */