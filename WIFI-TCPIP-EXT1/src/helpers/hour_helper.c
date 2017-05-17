/*
 * hour_helper.c
 *
 * Created: 17/05/2017 14:22:16
 *  Author: pedro
 */

#include <stdlib.h>

#include "hour_helper.h"

#define HOUR_DECIMAL_ID   0
#define HOUR_DIGIT_ID     1
#define MINUTE_DECIMAL_ID 3
#define MINUTE_DIGIT_ID   4

#define char_to_int(val) (val - '0')

hour_t *parse_hour(char *hour_str)
{
	hour_t *h = (hour_t *) malloc(sizeof(hour_t));
	h->hour = char_to_int(hour_str[HOUR_DECIMAL_ID]) * 10 +
			  char_to_int(hour_str[HOUR_DIGIT_ID]);
	h->minute = char_to_int(hour_str[MINUTE_DECIMAL_ID]) * 10 +
	            char_to_int(hour_str[MINUTE_DIGIT_ID]);
	return h;
}

