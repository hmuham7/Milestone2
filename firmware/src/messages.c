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
#include "message_thread.h"

static STATES state = IDLE;
static size_t BufIndex = 0;
static char internalCheckSum = 0;;
static size_t size = 0;
static unsigned int dropd = 0;

bool messageparser(char ch, char data[], char* source, char* msgCount, bool *err, unsigned int* msgdrop) {
    dbgOutputLoc(ENTER_PARSEMESSAGE_MESSAGE_C);
	switch (state) {
	case IDLE: {
        dbgOutputLoc(CASE_IDLE_STATE_PARSEMESSAGE_MESSAGE_C);
        *err = false;
		BufIndex = 0;
		internalCheckSum = NULL;
		size = 0;
		memset(data, 0, SIZE);
		if (ch == START) {
            dbgOutputLoc(0x27);
			state = DESTINATION;
		}
        return false;
    }
	case DESTINATION: {
        *err = false;
        dbgOutputLoc(CASE_CHECK_DESTINATION_CHAR_PARSEMESSAGE_MESSAGE_C);
        if (ch == MYROVER) {
			state = SOURCE;
		}
        else {
            state = IDLE;
            dropd++;
        }
        return false;
	}
    case SOURCE: {
        dbgOutputLoc(CASE_CHECK_SOURCE_CHAR_PARSEMESSAGE_MESSAGE_C);
        if(ch == SENSOR_ROVER){
            *source = SENSOR_ROVER;
        }
        else if(ch == FLAG_ROVER){
            *source = FLAG_ROVER;
        }
        else if(ch == TAG_ROVER){
            *source = TAG_ROVER;
        }
        else if(ch == CM_ROVER){
            *source = CM_ROVER;
        }
        else if(ch == SERVER){
            *source = SERVER;
        }
        else{
            state = IDLE;
            return false;
        }
        state = MESSAGE_COUNT;
        return false;
       
    }
	case MESSAGE_COUNT: {
        dbgOutputLoc(CASE_CHECK_MESSAGE_COUNT_PARSEMESSAGE_MESSAGE_C);
        state = DATALENGTH_UPPER;
        *msgCount = ch;
        return false;
	}
	case DATALENGTH_UPPER: {
        dbgOutputLoc(CASE_GET_DATALENGTH_UPPER_PARSEMESSAGE_MESSAGE_C);
        size = (ch & 0xFF) << 8;
        state = DATALENGTH_LOWER;
        return false;
	}
	case DATALENGTH_LOWER: {
        dbgOutputLoc(CASE_GET_DATALENGTH_LOWER_PARSEMESSAGE_MESSAGE_C);
        size = size | (ch & 0x00FF);
        if(size > (SIZE - 8)) {
            state = IDLE;
            *err = true;
            return false;
        }    
        state = DATA;  
        return false;
	}
	case DATA: {
        dbgOutputLoc(CASE_GET_DATA_PARSEMESSAGE_MESSAGE_C);
        data[BufIndex] = ch;
		BufIndex = BufIndex + 1;
        dbgOutputLoc(BEFORE_FIRST_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C);
        if (BufIndex >= size || BufIndex >= SIZE) {
			state = CHECKSUM;
            BufIndex = 0;
		}
        if(ch == START) {
            *err = true;
            state = DESTINATION;
        }
        if(ch == END) {
            *err = true;
            state = IDLE;
        }
        dbgOutputLoc(AFTER_THIRD_IF_GET_DATA_PARSEMESSAGE_MESSAGE_C);
		return false;
	}
	case CHECKSUM: {
        dbgOutputLoc(CASE_GET_CHECK_SUM_PARSEMESSAGE_MESSAGE_C);
        internalCheckSum = ch;
        char checkIt = checksum(data);
		if (internalCheckSum != checkIt){
            dropd++;
            *err = true;
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
        *err = ch != END;
        *msgdrop = dropd;
		return !(*err);
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

    size_t len = strlen(msgData);
    return sprintf(buf, "%c%c%c%c%c%c%s%c%c",
        START,
        destination,
        MYROVER,
        msgcount,// && 0xFF,
        (len & 0xFF00) >> 8,
        len & 0x00FF,
        msgData,
        checksum(msgData),
        END
        );
    dbgOutputLoc(LEAVE_CREATEMESSAGE_MESSAGE_C);
}

char checksum(char* cs)
{
    dbgOutputLoc(ENTER_CHECKSUM_MESSAGE_C);
    char* temp = cs;
	char sum = 0;
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


