#include <stdint.h>
#include <stdio.h>
#include "nrf_log.h"
#include <math.h>

//STRUCTS, UNIONS and other struct like types====================================
typedef struct {
	int32_t input;
	int32_t output;
	int32_t m1;
	int32_t m2;
	int32_t A;
	int32_t B;
	int32_t E;
	int32_t W;
	int32_t D;
	int32_t P1;
	int32_t P2;
	int32_t Q1;
	int32_t Q2;
	int32_t L;
	int16_t mem2;
} ADS018_IIR_2ORDER_Type;
//VARIABLES====================================================
volatile int32_t filter_type = 0;//0:Low Pass 2.5Hz- fs:25Hz - 1:Low Pass 2.5Hz- fs:50Hz
//volatile  int16_t positivo = 300;
//volatile  int16_t negativo = -300;
//volatile  int16_t atual = 0;
//volatile	int16_t i=-0;
//volatile	int16_t j=0;
const int32_t ADS018_IIR_Coef[2][6]={
		{-27938, 16175, 3436, -28916, 6259, 6883},//1ª coluna
		{-31356, 24477, 5871, -31538, 17071, 12765}//2ª coluna
};//m1 m2 lambda
volatile ADS018_IIR_2ORDER_Type ADS018_IIR_E1={0,0,-27938,16175,0,0,0,0,0,0,0,0,0,1000,0};
volatile ADS018_IIR_2ORDER_Type ADS018_IIR_E2={0,0,-28916,6259,0,0,0,0,0,0,0,0,0,1000,0};
//high pass filter
#define FILTER_HPF_CF     0.3
volatile int32_t filter_lastyi = 0;
volatile int32_t filter_lastyo = 0;
volatile int16_t filter_coef_num = 2470;
volatile int16_t filter_coef_den = 2500;
//FUNCTION PROTOTYPES==================================================================
void filter_init(int16_t inputy, int16_t input2);//protótipo
void ADS018_IIR_Update(int16_t in, int16_t in2, ADS018_IIR_2ORDER_Type *p, int16_t *pout, int16_t *pout2);// protótipo
void filter(int16_t inputy, int16_t input2, int16_t *outputy, int16_t *output2);
//FUNCTIONS=================================================================
/*function for initialization of the filter
Parameter:
1) inputy variable to be filtered
2) input2 2nd variable to be filtered
*/
void filter_init(int16_t inputy,int16_t input2)//coloca uma variável que deseja medir e outra que é dummy
{
	int16_t out1;
	int16_t out2;
	int16_t out3;
	int16_t out4;

	ADS018_IIR_E1.m1 = ADS018_IIR_Coef[filter_type][0];
	ADS018_IIR_E1.m2 = ADS018_IIR_Coef[filter_type][1];
	ADS018_IIR_E1.L  = ADS018_IIR_Coef[filter_type][2];
	ADS018_IIR_E2.m1 = ADS018_IIR_Coef[filter_type][3];
	ADS018_IIR_E2.m2 = ADS018_IIR_Coef[filter_type][4];
	ADS018_IIR_E2.L  = ADS018_IIR_Coef[filter_type][5];
	ADS018_IIR_E1.Q1 = 0;
	ADS018_IIR_E1.Q2 = 0;
	ADS018_IIR_E2.Q1 = 0;
	ADS018_IIR_E2.Q2 = 0;
	ADS018_IIR_Update(inputy, input2, (ADS018_IIR_2ORDER_Type *)&ADS018_IIR_E1, (int16_t *)&out1, (int16_t *)&out2);
	ADS018_IIR_Update(out1, out2, (ADS018_IIR_2ORDER_Type *)&ADS018_IIR_E2, (int16_t *)&out3, (int16_t *)&out4);
	filter_lastyi = inputy;
	filter_lastyo = inputy;
}


void ADS018_IIR_Update(int16_t in, int16_t in2, ADS018_IIR_2ORDER_Type *p, int16_t *pout, int16_t *pout2)
{
   *pout2 = p->mem2;
   p->mem2 = in2;

   p->input = (in * 1000) / p->L;
   p->A = p->input + p->Q1;
   p->B = p->Q2;
   p->E = (p->A  * p->m1) / 32768;
   p->W = p->E + ((p->B * p->m2) / 32768);
   p->P1 = p->input - (p->B + p->W);
   p->P2 = p->A + p->W;
   p->D  = ((p->P2 + p->B) * p->L) / 2000;
   p->Q1 = p->P1;
   p->Q2 = p->P2;
   *pout = (int16_t)p->D;
}
/*
funtion to filter the values
parameters

1) inputy: variable to be filtered
2) input2: 2nd variable to be filtered
3) outputy: pointer to a struct to hold the first filtered output variable
4) output2: pointer to a struct to hold the 2nd filtered output variable,
            in this implementation it'll be a dummy variable.
*/


void filter(int16_t inputy, int16_t input2, int16_t *outputy, int16_t *output2)
{
	int16_t out1;
	int16_t out2;
	int16_t out3;
	int16_t out4;
	int32_t dy;

	//low pass filter
	ADS018_IIR_Update(inputy, input2, (ADS018_IIR_2ORDER_Type *)&ADS018_IIR_E1, (int16_t *)&out1, (int16_t *)&out2);
	ADS018_IIR_Update(out1, out2, (ADS018_IIR_2ORDER_Type *)&ADS018_IIR_E2, (int16_t *)&out3, (int16_t *)&out4);

	//high pass filter - 0.3Hz (25*100-0.3*100)/(25*100)
	dy = out3 - filter_lastyi;
	filter_lastyi = out3;
	out3 = (filter_coef_num*filter_lastyo/filter_coef_den)+dy;
	filter_lastyo = out3;

	(*outputy) = (int16_t)out3;
	(*output2) = (int16_t)out4;
}




