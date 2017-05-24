/*
 * rtc_helper.h
 *
 * Created: 17/05/2017 20:16:33
 *  Author: pedro
 */

#include "rtc_helper.h"

#define YEAR   2017
#define MONTH  3
#define DAY    27
#define WEEK   13
#define HOUR   9
#define MINUTE 5
#define SECOND 0

void rtc_init(void)
{
	// PMC
	pmc_enable_periph_clk(ID_RTC);

	// Default RTC config for 24hrs mode
	rtc_set_hour_mode(RTC, 0);

	// Configure date and hour manually
	// TODO: Use actual time, not random consts
	//       (gather data from the internet
	//		 once online)
	rtc_set_date(RTC, YEAR, MONTH, DAY, WEEK);
	rtc_set_time(RTC, HOUR, MINUTE, SECOND);

	// Configure RTC interrupts
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
	NVIC_SetPriority(RTC_IRQn, 1);
	NVIC_EnableIRQ(RTC_IRQn);

	// Enable alarm interrupts
	rtc_enable_interrupt(RTC, RTC_IER_ALREN);
}

void rtc_set_alarm(hour_t *h)
{
	uint32_t date_err = rtc_set_date_alarm(RTC,
			1, MONTH, 1, DAY);
	uint32_t time_err = rtc_set_time_alarm(RTC,
			1, h->hour, 1, h->minute, 1, 0);
	if (!time_err && !date_err) {
		printf("Alarm set to: %.2hhd:%.2hhd\n",
				h->hour, h->minute);
		uint32_t pul_hour, pul_minute;
		rtc_get_time(RTC, &pul_hour, &pul_minute,
				NULL);
		printf("Current time: %.2hhd:%.2hhd\n",
				pul_hour, pul_minute);

	} else
		puts("RTC alarm set error");
}