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
    
#define INITIALIZER                                             0x00
#define ENTER_TASK                                              0x01
#define BEFORE_WHILE                                            0x02
#define BEFORE_RECEIVE                                          0x03
#define AFTER_RECEIVE                                           0x04
#define BEFORE_SEND_UART                                        0x05
#define AFTER_SEND_UART                                         0x06

#define ENTER_UART_ISR                                          0x11
#define LEAVE_UART_ISR                                          0x12
#define ENTER_UART_TX_IF                                        0x23
#define ENTER_UART_TX_WHILE                                     0x14
#define BEFORE_RECEIVE_ISR                                      0x15
#define AFTER_RECEIVE_ISR                                       0x16
#define BEFORE_SEND_TX_BYTE                                     0x17
#define AFTER_SEND_TX_BYTE                                      0x18
#define ENTER_TIMER_ISR0                                        0x19
#define ENTER_TIMER_IR0_SWITCH_CASE                             0X20
    
// Milestone 2 uart_thread.c debug values
#define ENTER_UART_THREAD_INITIALIZER                           0x21
#define ENTER_UART_THREAD_TASKS                                 0x22
    
//  Milestone 2 message_thread.C debug values
#define ENTER_MESSAGE_THREAD_INITIALIZER                        0x31
#define ENTER_MESSAGE_THREAD_TASKS                              0x32
#define ENTER_MESSAGE_THREAD_WHILE                              0X33
    
// Milestone 2 Messages.c debug values
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

#define UART           0x09
#define ERROR          0xFF

void dbgOutputVal (unsigned char outVal);

void dbgOutputLoc (unsigned char outVal);

void dbgUARTVal( unsigned char outVal);

void dbgPinsDirection();

void error ();

#ifdef	__cplusplus
}
#endif

#endif	/* DEBUG_H */