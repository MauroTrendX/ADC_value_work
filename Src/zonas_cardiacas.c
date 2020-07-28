/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
/*inclusao do arquivo contendo as configurações e enumerações das cores e LEDs*/
#include "Controle_LEDs.h"

/* Exportação das rotinas para medição e configuração das zonas cardíacas */
#include "zonas_cardiacas.h"
#include <stdbool.h>
/*inclusao do arquivo contendo as configurações da camada HAL do microcontrolador utilizado*/
//#include "stm32l4xx_hal.h"
#include "buffer_services.h"


/******************************************************************************/
/******************************************************************************/
#define MAX_HR_RHR           	      220
#define MAX_HR_MB2									208
#define AGE_DEFAULT_VALUE						23

#define SPIVI_ZONE_1_THRESHOLD  		50
#define SPIVI_ZONE_2_THRESHOLD  		60
#define SPIVI_ZONE_3_THRESHOLD  		70
#define SPIVI_ZONE_4_THRESHOLD  		80
#define SPIVI_ZONE_5_THRESHOLD			90

#define RHR_RESTING_ZONE_THRESHOLD  54
#define RHR_ZONE_1_THRESHOLD  			28
#define RHR_ZONE_2_THRESHOLD  			42
#define RHR_ZONE_3_THRESHOLD  			56
#define RHR_ZONE_4_THRESHOLD  			70
#define RHR_ZONE_5_THRESHOLD				83

#define MYBEAT_V2_ZONE_1_THRESHOLD 50
#define MYBEAT_V2_ZONE_2_THRESHOLD 58
#define MYBEAT_V2_ZONE_3_THRESHOLD 66
#define MYBEAT_V2_ZONE_4_THRESHOLD 76
#define MYBEAT_V2_ZONE_5_THRESHOLD 84
#define MYBEAT_V2_ZONE_6_THRESHOLD 92

#define CIA_ATHLETICA_ZONE_1_THRESHOLD 50
#define CIA_ATHLETICA_ZONE_2_THRESHOLD 61
#define CIA_ATHLETICA_ZONE_3_THRESHOLD 71
#define CIA_ATHLETICA_ZONE_4_THRESHOLD 84
#define CIA_ATHLETICA_ZONE_5_THRESHOLD 92


/******************************************************************************/
/******************************************************************************/
typedef struct{
	hr_zone_select_t				hr_zone_calc_select;
	uint8_t									age;
	uint8_t									resting_hr;
	uint8_t									percentage_SPIVI_zone_limits[END_ZONE];
	uint8_t									percentage_rhr_zone_limits[END_ZONE];
	uint8_t									percentage_CIA_ATHLETICA_zone_limits[END_ZONE];
	uint8_t									percentage_MYBEAT_V2_zone_limits[7];
  float 									hr_zone_limits[END_ZONE];
} hr_nv_buf_t;

extern volatile bool flagEstadoLed;
extern volatile bool avisoBateriaFraca;

static hr_nv_buf_t user_metrics_info;

const static hr_nv_buf_t metric_default = {
	hr_zone_select_SPIVI,
	AGE_DEFAULT_VALUE,
	60,
	{ 
		SPIVI_ZONE_1_THRESHOLD,
		SPIVI_ZONE_2_THRESHOLD,
		SPIVI_ZONE_3_THRESHOLD,
		SPIVI_ZONE_4_THRESHOLD,
		SPIVI_ZONE_5_THRESHOLD
	},
	{ 
		RHR_ZONE_1_THRESHOLD,
		RHR_ZONE_2_THRESHOLD,
		RHR_ZONE_3_THRESHOLD,
		RHR_ZONE_4_THRESHOLD,
		RHR_ZONE_5_THRESHOLD
	},
	{ 
		CIA_ATHLETICA_ZONE_1_THRESHOLD, 
		CIA_ATHLETICA_ZONE_2_THRESHOLD,
		CIA_ATHLETICA_ZONE_3_THRESHOLD, 
		CIA_ATHLETICA_ZONE_4_THRESHOLD, 
		CIA_ATHLETICA_ZONE_5_THRESHOLD 
	},
	{ 
		MYBEAT_V2_ZONE_1_THRESHOLD, 
		MYBEAT_V2_ZONE_2_THRESHOLD, 
		MYBEAT_V2_ZONE_3_THRESHOLD, 
		MYBEAT_V2_ZONE_4_THRESHOLD, 
		MYBEAT_V2_ZONE_5_THRESHOLD, 
		MYBEAT_V2_ZONE_6_THRESHOLD 
	},
	{ 
		0,
		0,
		0,
		0,
		0
	}
};

/******************************************************************************/
/******************************************************************************/
uint8_t obter_idade(void){
	return user_metrics_info.age;
}

/******************************************************************************/
void definir_idade(uint8_t value){
	user_metrics_info.age = value;
}

/******************************************************************************/
/******************************************************************************/
hr_zone_select_t get_hr_zone_select(void){
	if( user_metrics_info.hr_zone_calc_select > hr_zone_select_MYBEAT_V2 ){
		user_metrics_info.hr_zone_calc_select =  hr_zone_select_MYBEAT_V2;
	}
	return user_metrics_info.hr_zone_calc_select;
}

/******************************************************************************/
/******************************************************************************/
void set_hr_zone_select(hr_zone_select_t value){
	user_metrics_info.hr_zone_calc_select = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_resting_heart_rate_value(void){
	if( user_metrics_info.resting_hr == 0xFF ){
		user_metrics_info.resting_hr = metric_default.resting_hr;
	}

	return user_metrics_info.resting_hr;
}

/******************************************************************************/
void set_resting_heart_rate_value(uint8_t value){
	user_metrics_info.resting_hr = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t max_hr_get_rhr(void){
	return (MAX_HR_RHR-obter_idade());
}
/******************************************************************************/
/******************************************************************************/
float max_hr_get_mb2(void){
	return (MAX_HR_MB2-(0.7*obter_idade()));
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_SPIVI_percentage_threshold( zonesEnum zone ){
	if( user_metrics_info.percentage_SPIVI_zone_limits[zone] == 0 || user_metrics_info.percentage_SPIVI_zone_limits[zone] == 0xFF ){
		return  metric_default.percentage_SPIVI_zone_limits[zone];
	}
	else{
		return user_metrics_info.percentage_SPIVI_zone_limits[zone];
	}
}

/******************************************************************************/
void set_SPIVI_percentage_threshold( zonesEnum zone, uint8_t value ){
	user_metrics_info.percentage_SPIVI_zone_limits[zone] = value;
}

/******************************************************************************/
/******************************************************************************/
float  get_hr_SPIVI_info( zonesEnum zone ){
    return (float)((((float)MAX_HR_RHR - (float)obter_idade())*(float)get_SPIVI_percentage_threshold(zone))/(float)100);
}

/******************************************************************************/
/*
 * Paramêtros para calcular levando em consideração a zona cardíaca de repouso.
 */
/******************************************************************************/
uint8_t get_rhr_percentage_threshold( zonesEnum zone ){
	if( user_metrics_info.percentage_rhr_zone_limits[zone] == 0 || user_metrics_info.percentage_rhr_zone_limits[zone] == 0xFF ){
		return metric_default.percentage_rhr_zone_limits[zone];
	}
	else{
		return user_metrics_info.percentage_rhr_zone_limits[zone];
	}
}

/******************************************************************************/
void set_rhr_percentage_threshold( zonesEnum zone, uint8_t value ){
	user_metrics_info.percentage_rhr_zone_limits[zone] = value;
}

/******************************************************************************/
/******************************************************************************/
float get_hr_rhr_info( zonesEnum zone ){
    return (float)((((float)max_hr_get_rhr() - (float)get_resting_heart_rate_value())*(float)get_rhr_percentage_threshold(zone))/(float)100+(float)get_resting_heart_rate_value());
}
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/*
 * Paramêtros para calcular baseado no protocolo da CIA ATHLETICA.
 */
/******************************************************************************/

uint8_t get_CIA_ATHLETICA_percentage_threshold( zonesEnum zone ){
	if( user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[zone] == 0 || user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[zone] == 0xFF ){
		return  metric_default.percentage_CIA_ATHLETICA_zone_limits[zone];
	}
	else{
		return user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[zone];
	}
}

/******************************************************************************/
void set_CIA_ATHLETICA_percentage_threshold( zonesEnum zone, uint8_t value ){
	user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[zone] = value;
}

/******************************************************************************/
float get_hr_CIA_ATHLETICA_info( zonesEnum zone ){
    return (float)(((float)max_hr_get_mb2()*(float)get_CIA_ATHLETICA_percentage_threshold(zone))/(float)100);
}
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/*
 * Paramêtros para calcular baseado no protocolo MyBeat V2.
 */
/******************************************************************************/

uint8_t get_MYBEAT_V2_percentage_threshold( zonesEnum zone ){
	if( user_metrics_info.percentage_MYBEAT_V2_zone_limits[zone] == 0 || user_metrics_info.percentage_MYBEAT_V2_zone_limits[zone] == 0xFF ){
		return  metric_default.percentage_MYBEAT_V2_zone_limits[zone];
	}
	else{
		return user_metrics_info.percentage_MYBEAT_V2_zone_limits[zone];
	}
}

/******************************************************************************/
void set_MYBEAT_V2_percentage_threshold( zonesEnum zone, uint8_t value ){
	user_metrics_info.percentage_MYBEAT_V2_zone_limits[zone] = value;
}

/******************************************************************************/
float get_hr_MYBEAT_V2_info( zonesEnum zone ){
    return (float)(((float)max_hr_get_mb2()*(float)get_MYBEAT_V2_percentage_threshold(zone))/(float)100);
}
/******************************************************************************/
/******************************************************************************/



void zones_info_init( void )
{
		zonesEnum loop;
		
	if(get_hr_zone_select() == hr_zone_select_RHR )
	{
		for( loop=ZONE_1; loop<ZONE_6; loop++ )
		{
			user_metrics_info.hr_zone_limits[loop] = get_hr_rhr_info( loop );
		}
	}
	else if( get_hr_zone_select() == hr_zone_select_SPIVI )
	{
		for( loop=ZONE_1; loop<ZONE_6; loop++ )
		{
			user_metrics_info.hr_zone_limits[loop] = get_hr_SPIVI_info( loop );
		}
	}
	else if( get_hr_zone_select() == hr_zone_select_CIA_ATHLETICA )
	{
		for( loop=ZONE_1; loop<ZONE_6; loop++ )
		{
			user_metrics_info.hr_zone_limits[loop] = get_hr_CIA_ATHLETICA_info( loop );
		}
	}
	else //if( get_hr_zone_select() == hr_zone_select_CIA_ATHLETICA )
	{
		for( loop=ZONE_1; loop<RESTING_ZONE; loop++ )
		{
			user_metrics_info.hr_zone_limits[loop] = get_hr_MYBEAT_V2_info( loop );
		}
	}
}

/******************************************************************************/
/******************************************************************************/

void load_user_info_metrics(void){
    
		user_metrics_info.age=get_age_metric();
		user_metrics_info.hr_zone_calc_select=get_hr_zone_preference_calc();
	
		user_metrics_info.percentage_SPIVI_zone_limits[ZONE_1]=get_SPIVI_zone1_percentage_threshold();
		user_metrics_info.percentage_SPIVI_zone_limits[ZONE_2]=get_SPIVI_zone2_percentage_threshold();
		user_metrics_info.percentage_SPIVI_zone_limits[ZONE_3]=get_SPIVI_zone3_percentage_threshold();
		user_metrics_info.percentage_SPIVI_zone_limits[ZONE_4]=get_SPIVI_zone4_percentage_threshold();
		user_metrics_info.percentage_SPIVI_zone_limits[ZONE_5]=get_SPIVI_zone5_percentage_threshold();
		
		user_metrics_info.percentage_rhr_zone_limits[ZONE_1]=get_rhr_zone1_percentage_threshold();
		user_metrics_info.percentage_rhr_zone_limits[ZONE_2]=get_rhr_zone2_percentage_threshold();
		user_metrics_info.percentage_rhr_zone_limits[ZONE_3]=get_rhr_zone3_percentage_threshold();
		user_metrics_info.percentage_rhr_zone_limits[ZONE_4]=get_rhr_zone4_percentage_threshold();
		user_metrics_info.percentage_rhr_zone_limits[ZONE_5]=get_rhr_zone5_percentage_threshold();

		user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[ZONE_1]=metric_default.percentage_CIA_ATHLETICA_zone_limits[ZONE_1];
		user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[ZONE_2]=metric_default.percentage_CIA_ATHLETICA_zone_limits[ZONE_2];
		user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[ZONE_3]=metric_default.percentage_CIA_ATHLETICA_zone_limits[ZONE_3];
		user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[ZONE_4]=metric_default.percentage_CIA_ATHLETICA_zone_limits[ZONE_4];
		user_metrics_info.percentage_CIA_ATHLETICA_zone_limits[ZONE_5]=metric_default.percentage_CIA_ATHLETICA_zone_limits[ZONE_5];

		user_metrics_info.percentage_MYBEAT_V2_zone_limits[ZONE_1]=metric_default.percentage_MYBEAT_V2_zone_limits[ZONE_1];
		user_metrics_info.percentage_MYBEAT_V2_zone_limits[ZONE_2]=metric_default.percentage_MYBEAT_V2_zone_limits[ZONE_2];
		user_metrics_info.percentage_MYBEAT_V2_zone_limits[ZONE_3]=metric_default.percentage_MYBEAT_V2_zone_limits[ZONE_3];
		user_metrics_info.percentage_MYBEAT_V2_zone_limits[ZONE_4]=metric_default.percentage_MYBEAT_V2_zone_limits[ZONE_4];
		user_metrics_info.percentage_MYBEAT_V2_zone_limits[ZONE_5]=metric_default.percentage_MYBEAT_V2_zone_limits[ZONE_5];
		user_metrics_info.percentage_MYBEAT_V2_zone_limits[ZONE_6]=metric_default.percentage_MYBEAT_V2_zone_limits[ZONE_6];

		user_metrics_info.resting_hr=get_resting_heart_rate_metric();
		
		zones_info_init();	

}

/******************************************************************************/
/******************************************************************************/
zonesEnum get_current_zone ( float heart_rate )
{
		if(user_metrics_info.hr_zone_calc_select==hr_zone_select_MYBEAT_V2)
		{
				if( heart_rate < user_metrics_info.hr_zone_limits[ZONE_1] )
				{
						return RESTING_ZONE;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_1] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_2] )
				{
						return ZONE_1;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_2] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_3] )
				{
						return ZONE_2;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_3] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_4] )
				{
						return ZONE_3;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_4] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_5] )
				{
						return ZONE_4;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_5] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_6] )
				{
						return ZONE_5;
				}
				return ZONE_6;			
		}
		else
		{
				if( heart_rate < user_metrics_info.hr_zone_limits[ZONE_1] )
				{
						return RESTING_ZONE;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_1] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_2] )
				{
						return ZONE_1;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_2] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_3] )
				{
						return ZONE_2;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_3] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_4] )
				{
						return ZONE_3;
				}
				else if( heart_rate >= user_metrics_info.hr_zone_limits[ZONE_4] && heart_rate < user_metrics_info.hr_zone_limits[ZONE_5] )
				{
						return ZONE_4;
				}
		
				return ZONE_5;
		}	
}

/******************************************************************************/
/******************************************************************************/
float get_hr_zone_limits(zonesEnum zone){ 
	return user_metrics_info.hr_zone_limits[zone];
}

/******************************************************************************/
/******************************************************************************/
zonesEnum zonaAtual;
uint8_t perc_hr_max;

void calcular_zona_atual(uint8_t BPM)/*,selecionarCores *queroEstaCor, selecionarLED *queroEsteLED)*/{
	
		float fracao;		
		selecionarCores queroEstaCor;
		selecionarLED queroEsteLED;
//		zonesEnum temp;
		zonaAtual=get_current_zone(BPM);
		

		if(user_metrics_info.hr_zone_calc_select==hr_zone_select_MYBEAT_V2||user_metrics_info.hr_zone_calc_select==hr_zone_select_CIA_ATHLETICA)
				perc_hr_max=(BPM*100)/max_hr_get_mb2();

		else
				perc_hr_max=(BPM*100)/max_hr_get_rhr();

				
		if(avisoBateriaFraca){
				queroEstaCor=vermelho;//*queroEstaCor=verde;
						
				if(flagEstadoLed) {
						queroEsteLED=LED_BLINK;//*queroEsteLED=LED1;
				}
				else{
						queroEsteLED=LEDS_DESLIGADOS;//*queroEsteLED=LEDS_DESLIGADOS;
				}	
		}
		else{
	
				if(user_metrics_info.hr_zone_calc_select==hr_zone_select_MYBEAT_V2)
				{
						if( zonaAtual == RESTING_ZONE ){
								queroEstaCor=verde;//*queroEstaCor=verde;
								
								if(flagEstadoLed) {
										queroEsteLED=LED_BLINK;//*queroEsteLED=LED1;
								}
								else{
										queroEsteLED=LEDS_DESLIGADOS;//*queroEsteLED=LEDS_DESLIGADOS;
								}				
						}
				
						if( zonaAtual == ZONE_1 ){
								queroEstaCor=branco;//*queroEstaCor=branco;
					
								fracao = (get_hr_zone_limits(ZONE_2) - get_hr_zone_limits(ZONE_1) )/4;
					
								if(BPM < get_hr_zone_limits(ZONE_1) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
					
								else if(BPM < get_hr_zone_limits(ZONE_1) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
					
								else if(BPM < get_hr_zone_limits(ZONE_1) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
					
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}
						if( zonaAtual == ZONE_2 ){
								queroEstaCor=azul;//*queroEstaCor=azul;
				
								fracao = (get_hr_zone_limits(ZONE_3) - get_hr_zone_limits(ZONE_2))/4;
					
								if(BPM < get_hr_zone_limits(ZONE_2) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
					
								else if(BPM < get_hr_zone_limits(ZONE_2) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
					
								else if(BPM < get_hr_zone_limits(ZONE_2) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
					
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}		
						}
				
						if( zonaAtual == ZONE_3 ){
								queroEstaCor=verde;//*queroEstaCor=verde;
						
								fracao = (get_hr_zone_limits(ZONE_4) - get_hr_zone_limits(ZONE_3))/4;
							
								if(BPM < get_hr_zone_limits(ZONE_3) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_3) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_3) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
							
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}
				
						if( zonaAtual == ZONE_4){
								queroEstaCor=amarelo;//*queroEstaCor=amarelo;
						
								fracao = (get_hr_zone_limits(ZONE_5) - get_hr_zone_limits(ZONE_4))/4;
							
								if(BPM < get_hr_zone_limits(ZONE_4) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_4) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_4) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
								
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}
				
						if( zonaAtual == ZONE_5){
								queroEstaCor=laranja;//*queroEstaCor=vermelho;
						
								fracao = (get_hr_zone_limits(ZONE_6) - get_hr_zone_limits(ZONE_5))/4;
							
								if(BPM < get_hr_zone_limits(ZONE_5) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_5) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_5) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
							
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}	
						
						if( zonaAtual == ZONE_6){
								queroEstaCor=vermelho;//*queroEstaCor=vermelho;
						
								fracao = ((max_hr_get_mb2()) - get_hr_zone_limits(ZONE_6))/4;
							
								if(BPM < get_hr_zone_limits(ZONE_6) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_6) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_6) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
							
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}		
				}
				else
				{
						if( zonaAtual == RESTING_ZONE ){
								queroEstaCor=verde;//*queroEstaCor=verde;
								
								if(flagEstadoLed) {
										queroEsteLED=LED_BLINK;//*queroEsteLED=LED1;
								}
								else{
										queroEsteLED=LEDS_DESLIGADOS;//*queroEsteLED=LEDS_DESLIGADOS;
								}				
						}
				
						if( zonaAtual == ZONE_1 ){
								if(user_metrics_info.hr_zone_calc_select==hr_zone_select_CIA_ATHLETICA)
										queroEstaCor=azul;
								else
										queroEstaCor=branco;//*queroEstaCor=branco;
					
								fracao = (get_hr_zone_limits(ZONE_2) - get_hr_zone_limits(ZONE_1) )/4;
					
								if(BPM < get_hr_zone_limits(ZONE_1) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
					
								else if(BPM < get_hr_zone_limits(ZONE_1) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
					
								else if(BPM < get_hr_zone_limits(ZONE_1) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
					
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}
						if( zonaAtual == ZONE_2 ){
								
								if(user_metrics_info.hr_zone_calc_select==hr_zone_select_CIA_ATHLETICA)
										queroEstaCor=verde;
								else
										queroEstaCor=azul;//*queroEstaCor=azul;
				
								fracao = (get_hr_zone_limits(ZONE_3) - get_hr_zone_limits(ZONE_2))/4;
					
								if(BPM < get_hr_zone_limits(ZONE_2) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
					
								else if(BPM < get_hr_zone_limits(ZONE_2) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
					
								else if(BPM < get_hr_zone_limits(ZONE_2) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
					
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}		
						}
				
						if( zonaAtual == ZONE_3 ){
							
								if(user_metrics_info.hr_zone_calc_select==hr_zone_select_CIA_ATHLETICA)
										queroEstaCor=amarelo;
								else
										queroEstaCor=verde;//*queroEstaCor=verde;
						
								fracao = (get_hr_zone_limits(ZONE_4) - get_hr_zone_limits(ZONE_3))/4;
							
								if(BPM < get_hr_zone_limits(ZONE_3) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_3) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_3) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
							
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}
				
						if( zonaAtual == ZONE_4){
								if(user_metrics_info.hr_zone_calc_select==hr_zone_select_CIA_ATHLETICA)
										queroEstaCor=laranja;
								else
										queroEstaCor=amarelo;//*queroEstaCor=amarelo;
						
								fracao = (get_hr_zone_limits(ZONE_5) - get_hr_zone_limits(ZONE_4))/4;
							
								if(BPM < get_hr_zone_limits(ZONE_4) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_4) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_4) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
								
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}
				
						if( zonaAtual == ZONE_5){
										queroEstaCor=vermelho;//*queroEstaCor=vermelho;
						
								fracao = ((220-obter_idade()) - get_hr_zone_limits(ZONE_5))/4;
							
								if(BPM < get_hr_zone_limits(ZONE_5) + fracao){
										queroEsteLED=LED1;//*queroEsteLED=LED1;
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_5) + 2*fracao){
										queroEsteLED=LED2;//*queroEsteLED=LED2;	
								}
							
								else if(BPM < get_hr_zone_limits(ZONE_5) + 3*fracao){
										queroEsteLED=LED3;//*queroEsteLED=LED3;	
								}
							
								else {
										queroEsteLED=LED4;//*queroEsteLED=LED4;		
								}				
						}			
				}
		}		
		set_led(queroEsteLED);
		set_cor(queroEstaCor);	
//		set_led(LED4);
//		set_cor(branco);
		
		
}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
uint8_t get_color(void){				

		if(zonaAtual==RESTING_ZONE) {
				return 7;
		}
		else{
				return get_current_color();
		}
}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
uint8_t get_hr_percent(void){

		return perc_hr_max;

}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
