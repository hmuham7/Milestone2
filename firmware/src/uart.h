/* 
 * File:   uart.h
 * Author: Muhammad
 *
 * Created on February 20, 2017, 5:25 PM
 */

#ifndef UART_H
#define	UART_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include "system_config.h"
#include "system_definitions.h"
#include "messages.h"
#include "motor_thread.h"
#include "control_thread.h"
#include "tx_thread.h"
#include "message_thread.h"
#include "rx_thread.h"

#ifdef	__cplusplus
extern "C" {
#endif

void Uart_InitializeQueue();

int Uart_ReadFromQueue(void* pvBuffer);

void Uart_SendToQueue(char buffer);

void Uart_SendToQueueISR(char buffer, BaseType_t *pxHigherPriorityTaskWoken);


#ifdef	__cplusplus
}
#endif

#endif	/* UART_H */

