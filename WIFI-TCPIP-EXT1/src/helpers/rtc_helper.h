/*
 * rtc_helper.h
 *
 * Created: 17/05/2017 20:12:37
 *  Author: pedro
 */


#ifndef RTC_HELPER_H_
#define RTC_HELPER_H_

#include "hour_helper.h"
#include "rtc.h"
#include "pmc.h"

void rtc_init(void);
void rtc_set_alarm(hour_t *h);


#endif /* RTC_HELPER_H_ */