/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Team08

  @File Name
    messages.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _MESSAGES_H    /* Guard against multiple inclusion */
#define _MESSAGES_H

#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>

#define START               0x35
#define END                 0x42
#define SIZE                500
#define routine_error       0

#define FLAG_ROVER          0x01
#define SENSOR_ROVER        0x02
#define TAG_ROVER           0x03
#define CM_ROVER            0x04
#define SERVER              0x05

#define MYROVER             FLAG_ROVER
#define ROVER_NAME          "Flagrover"

int messagecreator(char buf[], char msgData[], char destination, char msgcount);

bool messageparser(char c, char data[], char* source, char* msgCount, bool *fault);

char checksum(char* cs);

typedef enum {
    IDLE,
    DESTINATION,
    SOURCE,
    MESSAGE_COUNT,
    DATALENGTH_UPPER,
    DATALENGTH_LOWER,
    DATA,
    CHECKSUM,
    ENDCHAR,
} STATES;

#endif /* _MESSAGES_H */

/* *****************************************************************************
 End of File
 */
