/*
 * RTC.h
 *
 *  Created on: Mar 1, 2018
 *      Author: Jeefo
 */

#ifndef RTC_H_
#define RTC_H_

void incSeconds();

typedef struct{
	int years;
	int months;
	int weeks;
	int days;
	int hours;
	int minutes;
	int seconds;
}rtc;
typedef struct{
	int hours;
	int minutes;
}time_;
rtc getUptime();
void updateTime(char* data);
void rtcClock();
void checkTimer();
#endif /* RTC_H_ */
