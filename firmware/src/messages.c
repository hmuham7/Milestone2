/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.c

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "messages.h"
#include "debug.h"

static STATES state = IDLE;
static size_t internalBufferIndex = 0;
static char internalCheckSum = 0;;
static size_t size = 0;

bool messageparser(char c, char data[], char* source, char* msgCount, bool *fault) {
    dbgOutputLoc(ENTER_PARSEMESSAGE_MESSAGE_C);
	switch (state) {
	case IDLE: {
        dbgOutputLoc(CASE_IDLE_STATE_PARSEMESSAGE_MESSAGE_C);
        *fault = false;
		internalBufferIndex = 0;
		internalCheckSum = NULL;
		size = 0;
		memset(data, 0, SIZE);
		if (c == START) {
			state = DESTINATION;
		}
		return false;
	}
	case DESTINATION: {
        *fault = false;
        dbgOutputLoc(CASE_CHECK_DESTINATION_CHAR_PARSEMESSAGE_MESSAGE_C);
        if (c == MYROVER) {
			state = SOURCE;
		}
        else {
            state = IDLE;
        }
		return false;
	}
    case SOURCE: {
        dbgOutputLoc(CASE_CHECK_SOURCE_CHAR_PARSEMESSAGE_MESSAGE_C);
        switch(c) {
            case FLAG_ROVER: {
                *source = FLAG_ROVER;
                break;
            }
            case SENSOR_ROVER: {
                *source = SENSOR_ROVER;
                break;
            }
            case TAG_ROVER: {
                *source = TAG_ROVER;
                break;
            }
            case CM_ROVER: {
                *source = CM_ROVER;
                break;
            }
            case SERVER: {
                *source = SERVER;
                break;
            }
            default: {
                *fault = true;
                state = IDLE;
                return false;
            }
            return false;
        }
        state = MESSAGE_COUNT;
        return false;
    }
	case MESSAGE_COUNT: {
        dbgOutputLoc(CASE_CHECK_MESSAGE_COUNT_PARSEMESSAGE_MESSAGE_C);
        state = DATALENGTH_UPPER;
        *msgCount = c;
		return false;
	}
	case DATALENGTH_UPPER: {
        dbgOutputLoc(CASE_GET_DATALENGTH_UPPER_PARSEMESSAGE_MESSAGE_C);
        size = (c & 0xFF) << 8;
		state = DATALENGTH_LOWER;
		return false;
	}
	case DATALENGTH_LOWER: {
        dbgOutputLoc(CASE_GET_DATALENGTH_LOWER_PARSEMESSAGE_MESSAGE_C);
        size = size | (c & 0x00FF);
        if(size > (SIZE - 8)) {
            state = IDLE;
            *fault = true;
            return false;
        }
		state = DATA;
		return false;
	}
	case DATA: {
        dbgOutputLoc(CASE_GET_DATA_PARSEMESSAGE_MESSAGE_C);
        data[internalBufferIndex] = c;
		internalBufferIndex = internalBufferIndex + 1;
        dbgOutputLoc(BEFORE_FIRST_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C);
        if (internalBufferIndex >= size || internalBufferIndex >= SIZE) {
			state = CHECKSUM;
            internalBufferIndex = 0;
		}
        if(c == START) {
            *fault = true;
            state = DESTINATION;
        }
        if(c == END) {
            *fault = true;
            state = IDLE;
        }
        dbgOutputLoc(AFTER_THIRD_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C);
		return false;
	}
	case CHECKSUM: {
        dbgOutputLoc(CASE_GET_CHECK_SUM_PARSEMESSAGE_MESSAGE_C);
        internalCheckSum = c;
		if (internalCheckSum != checksum(data)) {
            *fault = true;
			state = IDLE;
		}
		else {
            state = ENDCHAR;
        }
		return false;
	}
	case ENDCHAR: {
        dbgOutputLoc(CASE_CHECK_ENDCHAR_PARSEMESSAGE_MESSAGE_C);
        state = IDLE;
        *fault = c != END;
		return !(*fault);
	}
	}
    dbgOutputLoc(LEAVE_PARSEMESSAGE_MESSAGE_C);
}

/**
 * Create a message using our format
 * @param buf destination for message
 * @param messageData data for the message
 * @param destination character for who message is sent to
 * @return length of the message
 */
int messagecreator(char buf[], char msgData[], char destination, char msgcount) {
    dbgOutputLoc(ENTER_CREATEMESSAGE_MESSAGE_C);
	memset(buf, 0, SIZE);
	// Format: start byte, destination, source, message count, data length (upper 8 bits), data length (lower 8 bits), data, checksum
	
	if (!routine_error) {
		size_t len = strlen(msgData);
		return sprintf(buf, "%c%c%c%c%c%c%s%c%c",
			START,
			destination,
			MYROVER,
			msgcount && 0xFF,
	        (len & 0xFF00) >> 8,
			len & 0x00FF,
			msgData,
			checksum(msgData),
			END
			);
	} else {
		char badMessage[512];
        int i;
        int len = strlen(msgData);

        /* remove bytes from message */
        for (i = 0; i < len - 1; i++) {
            badMessage[i] = msgData[i + 5];
        }

	    if(i < len) {
            badMessage[i] = '\0';
        }

        return sprintf(buf,"%c%c%c%c%c%c%s%c%c",
            START,
            destination,
            MYROVER,
            msgcount && 0xFF,
    		(len & 0xFF00) >> 8,
            len & 0x00FF,
            badMessage,
            checksum(msgData),
            END
            );
	}
	
    dbgOutputLoc(LEAVE_CREATEMESSAGE_MESSAGE_C);
}

char checksum(char* cs)
{
    dbgOutputLoc(ENTER_CHECKSUM_MESSAGE_C);
    char* temp = cs;
	signed char sum = -1;
	while (*temp != 0)
	{
		sum += *temp;
		temp++;
	}
    dbgOutputLoc(LEAVE_CHECKSUM_MESSAGE_C);
	return sum;
}

/* *****************************************************************************
 End of File
 */

