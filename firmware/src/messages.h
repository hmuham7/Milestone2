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

#define START               0x3C
#define END                 0x3E
#define SIZE                200
#define routine_error       0

#define FLAG_ROVER      0x46  //F
#define SENSOR_ROVER    0x53  //S
#define TAG_ROVER       0x54  //T
#define CM_ROVER        0x43  //C
#define SERVER          0x73  //s

#define MYROVER         FLAG_ROVER
#define ROVER_NAME      "Flagrover"

bool ParseMessage(char c, char data[], char* source, char* messageCount, bool *err);

int messagecreator(char buf[], char msgData[], char destination, char msgcount);

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
