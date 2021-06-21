#include<stdio.h>
//#include<conio.h>
#include<math.h>
#include "nrf_log.h"
#define pi 3.414

void senoide (void){
	
 int freq=50;
 int fs=50;//amostragem = sample rate
 int amplitude = 1;
 int cycle = 1;
	
 double angle,increment,sample;
 angle=0;
 increment=((2*pi)/(fs/freq));
	for (cycle=1;cycle<=freq;cycle++)
	{
		while (angle<=(2*pi))
		{
			sample=(amplitude * sin(angle));
			angle=angle+increment;
			NRF_LOG_INFO("sample  : %f",sample);	
		}
	}

}