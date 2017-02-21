/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _JSON_PARSER_H    /* Guard against multiple inclusion */
#define _JSON_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "jsmn.h"

typedef enum {request, response, strange} type_t;

typedef enum {CommStats_flag_rover, CommStats_sensor_rover, CommStats_tag_rover, CommStats_cm_rover, 
        SensorData, msLocalTime} items_t;

typedef struct {
  char stringValue[1024];
  items_t enumValue;
} DictionaryType;

static const DictionaryType Dictionary[] = {
    {"CommStats_flag_rover", CommStats_flag_rover},
    {"CommStats_sensor_rover", CommStats_sensor_rover},
    {"CommStats_tag_rover", CommStats_tag_rover},
    {"CommStats_cm_rover", CommStats_cm_rover},
    {"SensorData", SensorData},
    {"msLocalTime", msLocalTime}
};

static jsmn_parser p;
static jsmntok_t t[512]; /* We expect no more than 128 tokens */
static char buf[255];

void initialize_parser();

int json_tokenizer(const char *json, jsmntok_t *tok, const char *s) ;

void json_parser(const char* JSON_STRING, type_t *type, items_t items[], int *numItems);

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
