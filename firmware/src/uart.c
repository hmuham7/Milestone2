#include "uart.h"

#define QUEUE_TYPE          char
#define QUEUE_SIZE          500

QueueHandle_t uartqueue;

void Uart_InitializeQueue() {
    uartqueue = xQueueCreate(QUEUE_SIZE, sizeof(QUEUE_TYPE));
    if(uartqueue == 0) {
        dbgOutputBlock(pdFALSE);
    }
}

void InitializeISRQueues() {
    Uart_InitializeQueue();
}

int Uart_ReadFromQueue(void* pvBuffer) {
    int ret = xQueueReceiveFromISR(uartqueue, pvBuffer, portMAX_DELAY);
    return ret;
}

void Uart_SendToQueue(char buffer) {
    xQueueSendToBack(uartqueue, &buffer, portMAX_DELAY);
}

void Uart_SendToQueueISR(char buffer, BaseType_t *pxHigherPriorityTaskWoken) {
    xQueueSendToBackFromISR(uartqueue, &buffer, pxHigherPriorityTaskWoken);
}
