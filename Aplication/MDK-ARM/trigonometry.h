/*This library aims to make an easier implementation of trigonometric signals to be generated and used by the aplication. It's not limited to trigonometric signals tough, 
here you can also implement other types of signals to be used*/

//INCLUDES============================================================
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include "nrf_log.h"
//DEFINES============================================================
#define M_PI 3.14159265358979323846
//VARIABLES==========================================================
float pi=M_PI;//defining PI
float A=3;//defining Amplitude of the sine wave
float f=1;//defining frequency as 50Hz
float c=0; //defining variable for the auxiliary counter
float max_period=1000;//period for iterations
float radian_factor=M_PI/180;//sin function takes values in radians so this factor allows you to specify your angles in degree, e.g sin ( 30*radian_factor)=0,5
char abuf[8];//we need this because it's the only way  NRF_LOG_INFO will print your values to your terminal program
//FUNCTON PROTOTYPES===========================================================
double seno ();
double cosseno ();
//FUNCTIONS=====================================================================
double seno (){

double result;
	
 if (true)
 {
	
	result=A*sin ( (2*radian_factor*pi*f*c));
//	uint8_t len=sprintf(abuf, "%f",result);
//	NRF_LOG_INFO(" a : %s",abuf);
	 c++;
	
}
 
else {
	c=0;
}
return result; 
}
//===============================================================================
double cosseno (){
	
double result;
	
 if (true)
 {
	
	result=A*cos( (2*radian_factor*pi*f*c));
//	uint8_t len=sprintf(abuf, "%f",result);
//	NRF_LOG_INFO(" a : %s",abuf);
	 c++;
	
}
 
else {
	c=0;
}
return result; 
}


