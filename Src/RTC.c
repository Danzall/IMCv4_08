/*
 * RTC.c
 *
 *  Created on: Mar 1, 2018
 *      Author: Jeefo
 */

#include "main.h"
#include "stm32f0xx_hal.h"
//#include "cmsis_os.h"
/* Start user code for include. Do not edit comment generated here */
#include "GSM.h"
#include "stdio.h"
#include "stdlib.h"
#include "RTC.h"
#include "string.h"
#include "myString.h"
/* End user code. Do not edit comment generated here */

unsigned int seconds;
unsigned int minutes;
unsigned int hours;
unsigned int days;
unsigned int weeks;
unsigned int years;
rtc uptime;
rtc time;
time_ onTime;
time_ offTime;
char on[10] = "01,05";
char off[10] = "01,10";
char timeR[10] = "01,15";
unsigned int relay = 0;
unsigned int relayTemp = 0;

void incSeconds(){
	//char time[60];
	uptime.seconds++;
	if((uptime.seconds == 20)||(uptime.seconds == 40)||(uptime.seconds == 58)){

	}
	if (uptime.seconds >= 60){
		uptime.seconds = 0;
		uptime.minutes++;
		if((uptime.minutes == 2)||(uptime.minutes == 5)||(uptime.minutes == 10)||(uptime.minutes == 20)||(uptime.minutes == 30)||(uptime.minutes == 40)||(uptime.minutes == 50)||(uptime.minutes == 0)){

		}

	}
	if (uptime.minutes >= 60){
		uptime.minutes = 0;
		uptime.hours++;
		char sms[30];
		//sprintf(sms, "GSM module uptime-%dd:%dh:%dm:%ds", uptime.days,uptime.hours, uptime.minutes, uptime.seconds);
		//strcpy(sms,)
		//sendSMS("0720631005", sms);
		//buildInfo();
	}
	if (uptime.hours >= 24){
		uptime.hours = 0;
		uptime.days++;
	}
	if (uptime.days >= 7){
		uptime.days = 0;
		uptime.weeks++;
	}
	if (uptime.weeks >= 52){
		uptime.weeks = 0;
		uptime.years++;
	}
	//sprintf(time, "years:%.2d weeks:%d days:%d hours:%d minutes:%d seconds:%d", uptime.years, uptime.weeks, uptime.days, uptime.hours, uptime.minutes, uptime.seconds);
	//Debug_Send(time,UART0);
	//Debug_Send("\r\n",UART0);
}

void rtcClock(){
	char time1[60];
	time.seconds++;
	if((time.seconds == 20)||(time.seconds == 40)||(time.seconds == 58)){

	}
	if (time.seconds >= 60){
		time.seconds = 0;
		time.minutes++;
		if((time.minutes == 2)||(time.minutes == 5)||(time.minutes == 10)||(time.minutes == 20)||(uptime.minutes == 30)||(uptime.minutes == 40)||(uptime.minutes == 50)||(uptime.minutes == 0)){

		}

	}
	if (time.minutes >= 60){
		time.minutes = 0;
		time.hours++;
		//char sms[30];
		//sprintf(sms, "GSM module uptime-%dd:%dh:%dm:%ds", uptime.days,uptime.hours, uptime.minutes, uptime.seconds);
		//strcpy(sms,)
		//sendSMS("0720631005", sms);
		//buildInfo();
	}
	if (time.hours >= 24){
		time.hours = 0;
		time.days++;
	}
	if (time.days >= 7){
		time.days = 0;
		time.weeks++;
	}
	if (time.weeks >= 52){
		time.weeks = 0;
		time.years++;
	}
	//sprintf(time1, "years:%.2d weeks:%d days:%d hours:%d minutes:%.2d seconds:%.2d", time.years, time.weeks, time.days, time.hours, time.minutes, time.seconds);
	sprintf(time1, "h:%.2d m:%.2d s:%.2d\r\n", time.hours, time.minutes, time.seconds);
	//Debug_Send(time1);
	//Debug_Send("\r\n",UART0);
	sprintf(timeR,"%.2d,%.2d\r\n", time.hours, time.minutes);
	/*Debug_Send("timeR-",UART0);*/
	//Debug_Send(timeR);
}

rtc getUptime(){
	return uptime;
}

void updateTime(char* data){
	char temp[5];

	Debug_Send("Time updated!!\r\n");
	Debug_Send(data);
	Debug_Send("\r\n");
	myStrSection(data, temp,20,',',0);	//hours
	strcpy(timeR,temp);
	strcat(timeR,",");
	time.hours = myStrLong(temp, 10);
	myStrSection(data, temp,20,',',1);	//mins
	strcat(timeR,temp);
	time.minutes = myStrLong(temp, 10);
	myStrSection(data, temp,20,',',2);	//secs
	time.seconds = myStrLong(temp, 10);
	myStrSection(data, temp,20,',',3);	//on hours
	strcpy(on,temp);
	strcat(on,",");
	onTime.hours = myStrLong(temp, 10);
	myStrSection(data, temp,20,',',4);	//on minutes
	strcat(on,temp);
	onTime.minutes = myStrLong(temp, 10);
	myStrSection(data, temp,20,',',5);	//off hours
	strcpy(off,temp);
	strcat(off,",");
	offTime.hours = myStrLong(temp, 10);
	myStrSection(data, temp,20,',',6);	//off minutes
	strcat(off,temp);
	offTime.minutes = myStrLong(temp, 10);
	Debug_Send(on);
	Debug_Send("\r\n");
	Debug_Send(off);
	Debug_Send("\r\n");
	Debug_Send(timeR);
	Debug_Send("\r\n");
	//if (strcmp(on,off) > 0) Debug_Send("More\r\n",UART0);
	//else Debug_Send("less\r\n",UART0);

}

void checkTimer(){
	//char time[10];
	//strcpy(time,"01,05");
	//strcpy(on,"15,150");
	//strcpy(off,"15,10");
	//if (strcmp(on,off) > 0) Debug_Send("More test\r\n",UART0);	//if 1s arg more returns positive
	//else Debug_Send("less test\r\n",UART0);
	//if ((strcmp(timeR,on) > 0)&&(strcmp(off,timeR) > 0)) Debug_Send("More!!!\r\n",UART0);
	if (strncmp(off,on,5) >= 0){
		//Debug_Send("Same day!!\r\n");
		if ((strncmp(timeR,on,5) >= 0)&&(strncmp(off,timeR,5) > 0)){
			//Debug_Send("ON!!!\r\n");
			relayTemp = 1;
			if (relay != relayTemp){
				relay = 1;
				Debug_Send("ON!!!\r\n");
				RelayOn();
			}

		}
		else{
			//Debug_Send("OFF!!!\r\n");
			relayTemp = 0;
			if (relay != relayTemp){
				relay = 0;
				Debug_Send("OFF!!!\r\n");
				RelayOff();
			}

		}
	}
	else {
		//Debug_Send("Different day!!\r\n");
		if (strncmp(timeR,on,5) > 0){
			//Debug_Send("ON!!!\r\n");
			relayTemp = 1;
			if (relay != relayTemp){
				relay = 1;
				Debug_Send("ON!!!\r\n");
				RelayOn();
			}
		}
		else if (strncmp(off,timeR,5) > 0){
			//Debug_Send("ON!!!\r\n");
			relayTemp = 1;
			if (relay != relayTemp){
				relay = 1;
				Debug_Send("ON!!!\r\n");
				RelayOn();
			}
		}
		else{
			//Debug_Send("OFF!!!\r\n");
			relayTemp = 0;
			if (relay != relayTemp){
				relay = 0;
				Debug_Send("OFF!!!\r\n");
				RelayOff();
			}
		}
	}

}
