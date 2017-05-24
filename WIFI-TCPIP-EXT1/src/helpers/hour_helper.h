/*
 * hour_helper.h
 *
 * Created: 17/05/2017 14:18:49
 *  Author: pedro
 */


#ifndef HOUR_HELPER_H_
#define HOUR_HELPER_H_


typedef struct {
	char hour;
	char minute;
} hour_t;

hour_t *parse_hour(char *hour_str);

#endif /* HOUR_HELPER_H_ */