#include <stdint.h>
#include <stdio.h>
#include "nrf_error.h"
#include "nrf_log_ctrl.h"
#include "nrf_log.h"
#include "app_timer.h"

//#define PRECISION 1; /* define this only if you're going
//#define SHOWMEASURES/*use this to print the mean of adc acquisition in ADC.h*/
//#define SHOWADC_SAMPLE /* define this if you want to see the adc value without the mean in ADC.h*/
//#define MAUROTESTE/* use this to enable slot of test 1 in main.c*/
#define MAUROTESTE_2/* define this to enable slot of test 2* in main.c*/
//#define ADV_ALL/* define this if you want to use all available data on function HR_advdata_manuf_data_update inside main.c*/
//#define CAL_SET_SHOW /*define this to print all the output status of  ADS018_Cal_Set in main.c*/
//#define TESTADV /* define this if you want to test  stop and restart of the advertising functionality in main.c*/
//#define SHOWSTATS//define this if you wanna show load power and rpm in heritage.h
//#define SHOWFILTER_OUT4 /*define this if you want to see the parameter out4 of function filter in heritage.h*/
//#define SHOWDIRECTION /*define this if you want to see the direction outuput of function get_direction in heritage.h*/

/**
@brief this function allows you to print one float number and one string concatenated in the same
line in NRF_LOG_INFO
*/
void SUPER_LOG(float f, char  string[40])
{
	NRF_LOG_FINAL_FLUSH();
	char output[50]; //for storing the converted string
  sprintf(output, "%f", f);
  NRF_LOG_INFO("%s %s",string, output);
//    int intPart = (int) f;
//    int decimalPart = (f - intPart) * PRECISION;
	  //NRF_LOG_INFO("%s : %d.%d",string,intPart,decimalPart);
}


//easier way to create a timer, you just have to remember that you need to use 
//APP_TIMER_DEF to create the "name" before calling this function.
void simple_timer( app_timer_id_t const * name,app_timer_timeout_handler_t function){
	
	
	ret_code_t err_code;
	    err_code = app_timer_create(name,
                                APP_TIMER_MODE_REPEATED,
                                function);
    APP_ERROR_CHECK(err_code);
	
}
