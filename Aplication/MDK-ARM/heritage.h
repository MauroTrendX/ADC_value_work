/*
05/03/2021-Mauro da Silva
This file contains functions inherited from MY WATT responsible for bringing more capabilities to the application.
In case of further development, please, try to keep the relevant code in this file as this will result in a more clear an easy
software documentation.
*/

#include <stdint.h>
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
#include <stdio.h>
#include "nrf_log.h"
#include <math.h>
#include "ADC.h"
#include "lis2dw12_reg.h"
#include "ble_gap.h"
//DEFINES=======================================================================================================
//Cycle******************************************************
#define ADS018_S_N 2
#define ADS018_S_N_MAX 10
#define ADS018_F_LEN_MIN 3     /**< min samples on advertising filter */
#define ADS018_F_N 8
#define ADS018_USE_FN 1
#define ADS1120_MAX_N_CAL 2
#define ADS018_ROTATION_MIN 10   /**< min rpm */
#define ADS018_ROTATION_MAX 160  /**< max rpm */
//filter************************************************************
#define FILTER_HPF_CF     0.3
//Update_Factors********************************************************************
#define ADS018_PIX2      6.283185307
#define ADS018_PI        3.141592654
#define ADS018_HALF_PI   1.570796327
#define ADS018_KGFM_TO_KCAL 0.0023404781
#define ADS018_KGFM_TO_CAL  2.3404780884
#define ADS018_KGF_TO_N     9.80665
#define ADS018_ENERGY_EFF_FACTOR     4
//CALIBRATION
#define ADS018_LOAD_REF_DEFAULT    10582  /**< Load calibration reference in gf */
#define ADS018_AZ_N_MAX 15
#define ADS018_AZ_FACTOR 3.0
#define ADS018_POS_BOT   0
#define ADS018_POS_TOP   1
#define ADS018_POS_FRONT 2
#define ADS018_POS_REAR  3
#define ADS018_AZ_DEBOUNCE 8
#define ADS018_AZ_Z_TOL 3640
#define ADS018_AZ_Z_POS 0
#define ADS018_AZ_Z_LI  (ADS018_AZ_Z_POS-ADS018_AZ_Z_TOL)
#define ADS018_AZ_Z_LS  (ADS018_AZ_Z_POS+ADS018_AZ_Z_TOL)
//STRUCTS, UNIONS and other struct like types====================================
//Cycle****************************************************************************
typedef struct {
	int32_t f;
	int32_t fp;
	int32_t fn;
	int32_t n;
} ADS018_mem_data_Type;
typedef struct {
	uint32_t n;
	int32_t rotation;
	float load;//int32_t altered for a better precision mauro
	int32_t torque;
	int32_t energy;
	float power;//int32_t altered for a better precision mauro
} ADS018_res_data_Type;
typedef struct {
	int32_t adc_value[ADS1120_MAX_N_CAL];
	int32_t uV_value[ADS1120_MAX_N_CAL];
	int32_t eng_value[ADS1120_MAX_N_CAL];
	int32_t n_point;
} ADS018_cal_Type;
typedef struct {
	uint32_t Sig;
	uint32_t Id;
	uint32_t Adv_Time;
	uint32_t Mean_Time;
	uint32_t Aslp_Count;
	uint32_t Load_Unit;
	uint32_t Arm;
	ADS018_cal_Type Cal;
	int32_t Kp; 
	int32_t Kn; 
	uint32_t Serial_Number;
	uint32_t Mode;
} ADS018_NV_Type;
typedef struct {
	uint16_t Cmd;
	uint8_t Minutes;
	uint8_t Seconds;
	uint32_t Cycle;
	uint16_t Rotation;
	uint16_t HR;
	uint16_t Energy;
	uint16_t Power;
	int16_t Load;
	uint16_t Torque;
} ADS018_cycle_data_Type;

//filter****************************************************************
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

typedef struct {
	int16_t Xaccel;
	int16_t Yaccel;
	int16_t Zaccel;
	int16_t Orientation;
	int16_t Load;
	int16_t Torque;
} ADS018_raw_data_Type;

typedef union{
  int16_t i16bit[3];//y,x,z
  uint8_t u8bit[6];//yh,yl,xh,xl,zh,zl
} axis3bit16_t;
//VARIABLES=====================================================================================================================
// Cycle**********************************************************
volatile uint16_t glob_var;
volatile int16_t ADS018_raw_Yaccel_F = 0;
volatile int16_t ADS018_y=0;
volatile int16_t ADS018_cyle_bac_limit_y = 700;//originally was 500
volatile int16_t ADS018_cyle_fro_limit_y =-700;//originally was -500
volatile uint16_t ADS018_last_flag_y=0;
volatile uint16_t ADS018_flag_y=0;
volatile uint16_t ADS018_stt_flag_y = 0;

volatile uint16_t ADS018_SCycle_Stt = 0;       // !< state of sector state machine
volatile uint32_t ADS018_SCycle_Tout_Count = 0;          // !< sample count for cyle timeout   
volatile uint32_t ADS018_SCycle_Tout_Limit = 75;         // !< limit count for cyle timeout
volatile uint32_t ADS018_SCycle_Tout_Num = 3;            // !< number times update zero adv, maximum allowed timeuout events.
volatile uint32_t ADS018_SCycle_Tout_Cmd = 0;            // !< command update to filter

volatile uint32_t ADS018_Cycle_Last_S = 0;     // !< 0..1: semi-sphere number, 0:front, 1:rear
volatile uint32_t ADS018_Cycle_Stt = 0;        // !< state of cycle state machine
volatile uint16_t ADS018_Cycle_Flag = 0;       // !< 1: turn complete
volatile uint32_t ADS018_Cycle_Dir = 0;        // !< 0:CW, 1:CCW
volatile uint32_t ADS018_Cycle_S = 0;          // !< 0..1: semi-sphere number, 0:front, 1:rear
volatile uint32_t ADS018_Cycle_S_Cmd = 0;      // !< 0..2: command
volatile uint32_t ADS018_Cycle_Result = 0;     // !< avaliacao do ciclo:

volatile uint32_t ADS018_CyleCounter = 0;
volatile float ADS018_Energy = 0;
volatile uint32_t ADS018_Cycle_S_n = ADS018_S_N;  // !< number of sectors where angular speed is constant

volatile ADS018_mem_data_Type ADS018_mem_data_s[ADS018_S_N_MAX]={{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},
		{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};
volatile ADS018_mem_data_Type ADS018_mem_data_c={0,0,0,0};
volatile ADS018_res_data_Type ADS018_res_data_c={0,0,0,0,0};
volatile ADS018_res_data_Type ADS018_res_data={0,0,0,0,0};

volatile uint32_t ADS018_res_data_f_len = ADS018_F_LEN_MIN;
volatile ADS018_res_data_Type ADS018_res_data_m = {0,0,0,0,0};
volatile uint32_t ADS018_res_data_f_in  = 0;
volatile uint32_t ADS018_res_data_f_out = 0;
volatile uint32_t ADS018_res_data_f_stt = 0;
volatile ADS018_res_data_Type ADS018_res_data_f[ADS018_F_N];

volatile uint32_t ADS018_res_data_cmd   = 0;
volatile uint32_t nnnn=0;
volatile int16_t ads1120_ADC  = 0;               // !< ads1120 ADC binary
volatile int16_t ads1120_ADCB = 0;               // !< ads1120 ADC binary balanced
volatile int16_t ads1120_ADCT = 0;               // !< ads1120 ADC binary balanced and tared must be declared in calibration.h, for now
volatile float ADS018_load_factor_n[ADS018_F_N];
volatile float ADS018_energy_factor_n[ADS018_F_N];
volatile float ADS018_power_factor_n[ADS018_F_N];
// ROTATION
volatile int16_t ADS018_rotation_factor =60*50; // !< rotation in [RPM/10]//originally was 10*60*50 now its only RPM
volatile int16_t ADS018_Rotation_Min_N = 12;  // !< min rotation number of samples
volatile int16_t ADS018_Rotation_Max_N = 150; // !< max rotation number of samples
// ADVERTISING
volatile uint32_t ADS018_SampleCounter  = 0;
volatile uint32_t ADS018_ShowCounter    = 0;
volatile uint32_t ADS018_ShowLimit      = 49;
volatile uint32_t ADS018_Sec_Prescaler  = 0;
volatile uint32_t ADS018_Sec_Limit      = 50;
volatile uint8_t ADS018_Sec_Counter     = 0;
volatile uint8_t ADS018_Min_Counter     = 0;

volatile int16_t teste_lib=16;

volatile ADS018_cycle_data_Type *pADS018_transfer;
volatile ADS018_cycle_data_Type ADS018_data = {0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0.0};
volatile ADS018_cycle_data_Type *pADS018_data = (ADS018_cycle_data_Type *)&ADS018_data;
volatile uint32_t	adv = 0;


volatile ADS018_raw_data_Type ADS018_raw    = {0, 0, 0, 0, 0.0};  //raw aquisition buffer
volatile float ADS018_AZ_load = 0.0;
volatile float ADS018_AZ_mean = 0.0;
volatile float ADS018_AZ_sdev = 0.0;
volatile uint32_t ADS018_AZ_counter = 0;
volatile uint32_t ADS018_AZ_n = ADS018_AZ_N_MAX;
volatile float ADS018_AZ_tol = 0.0;
volatile float ADS018_AZ_absmean = 0.0;
int8_t session=0;
lis2dw12_all_sources_t all_source;
ADS018_NV_Type *pADS018_NV_buf;
volatile  float load_mauro;//teste 15/06
//update_Factors***************************************************
int ADS018_sample_frequency=50;
volatile float ADS018_load_factor;
volatile float ADS018_energy_factor;
volatile float ADS018_power_factor;
volatile float ADS018_CalA_n[ADS018_F_N];
//volatile float ADS018_load_factor_n[ADS018_F_N];
volatile float ADS018_torque_factor_n[ADS018_F_N];
volatile float ADS018_energy_factor_n[ADS018_F_N];
volatile uint8_t  mma8x5x_pl = 0;              // !< PORTRAIT/LANDSCAPE code
volatile int16_t ADS018_AZ_Tare __attribute__ ((section(".noinit"))); //era inicializado com 0
volatile float ADS018_tare = 0.0;
volatile uint32_t ADS018_AZ_tdebounce = ADS018_AZ_DEBOUNCE;
volatile uint32_t ADS018_AZ_flag = 0;
//Load convertion
volatile float ADS018_Cal_A=1;
volatile float ADS018_Cal_B=0;
volatile int16_t ADS018_Cal_ADC_Zero = 0;
volatile int16_t ADS018_Cal_ADC_Delta = 1;
static ADS018_NV_Type ADS018_NV_buf __attribute__ ((section(".noinit"))); /**< Setup data in RAM */
volatile uint32_t ADS018_AZ_stt     = 0;
volatile int16_t global_mixer_z ;
//filter*********************************************************
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
volatile int32_t filter_lastyi = 0;
volatile int32_t filter_lastyo = 0;
volatile int16_t filter_coef_num = 2470;
volatile int16_t filter_coef_den = 2500;
stmdev_ctx_t dev_ctx;
static axis3bit16_t z_acceleration;
volatile uint8_t ADS018_AtualGear = 0;
// FUNCTION PROTOTYPES =============================================================================================================================================================================================================================================================================================
//Cycle****************************************************
void ADS018_Time_Update(void);//prototype
void ADS018_Set_Result_C(ADS018_mem_data_Type *pin, ADS018_res_data_Type *pout);//prototype
void ADS018_Set_Result(ADS018_res_data_Type *pin, ADS018_res_data_Type *pout, int len);//prototype
void ADS018_Cycle(int16_t vel);//prototype
void ADS018_res_data_clear(ADS018_res_data_Type *p);//prototype
void init_vars();//prototype
void init_varc();//prototype
void init_f(void);//prototype
void remove_f(void);//prototype
void acc_f(void);//prototype
void insert_f(void);//prototype
void insert_zero(void);//prototype
void update_s(void);//prototype
void update_c(void);//prototype
void init_var(void);//prototype
void ADS018_Time_Update();//prototype
void ADS018_Update_SCycle(void);//prototype
void ADS018_Set_Mean_Data(void);//prototype
void ADS018_Update_Factors(void);//prototype
void get_load (int16_t adc_measure);//prototype
void get_direction(void);//prototype
void ADS018_BT_Exec(void);//prototype
void advertising_stop(void);//prototype
void get_session(int16_t accel);//prototype
//filter ********************************************************
void filter_init(int16_t inputy, int16_t input2);//prototype
void ADS018_IIR_Update(int16_t in, int16_t in2, ADS018_IIR_2ORDER_Type *p, int16_t *pout, int16_t *pout2);//prototype
void filter(int16_t inputy, int16_t input2, int16_t *outputy, int16_t *output2);//prototype
//ADC
void ADS018_Set_Cal_Adc_Zero(int16_t new_zero);//prototype
void ADS018_Cal_Set(ADS018_cal_Type *ptab, float *pa, float *pb, int16_t *pix, int16_t *pd);//prototype
void ADS018_Set_Cal_ADC_Value_Zero(int16_t new_adc_zero);//prototype
//void ADS018_AZ(void);//prototype
int16_t ADS018_Load_Balanced(int16_t load_adc);
//FUNCTIONS=============================================================================================================================================================================================================================
 //Cycle **********************************************
 void ADS018_Update_Factors()
{
	int i;
	pADS018_NV_buf->Arm =166;//166mm ou 178mm
	float arm = pADS018_NV_buf->Arm / 1000.0;
	//NRF_LOG_INFO("pADS018_NV_buf->Arm %d",pADS018_NV_buf->Arm);
	float f = 2 * ADS018_ENERGY_EFF_FACTOR * ADS018_KGFM_TO_CAL * ADS018_PIX2 * arm; /* fator 4x para 25% de eficiencia estimada */
	float f1 = 2 * ADS018_sample_frequency * ADS018_KGF_TO_N * ADS018_PIX2 * arm;
	float f2;
//mudado348
	  ADS018_energy_factor = f * ADS018_Cal_A;
    ADS018_power_factor  = f1 * ADS018_Cal_A;//entenda como 2 *omega *  torque  //( o  ____o)  
	  ADS018_load_factor = 10 * ADS018_Cal_A;
    ADS018_CalA_n[0] = ADS018_Cal_A; 
    ADS018_load_factor_n[0]           = ADS018_load_factor;
    ADS018_torque_factor_n[0]        = ADS018_load_factor * arm;
    ADS018_energy_factor_n[0]       = ADS018_energy_factor;
    ADS018_power_factor_n[0]    =     ADS018_power_factor;
	  for (i=1;i<ADS018_F_N;i++){
		     f2 = (1.0*i);
		    ADS018_CalA_n[i] = ADS018_load_factor/f2;
        ADS018_load_factor_n[i]    = ADS018_load_factor_n[0]/f2;
        ADS018_torque_factor_n[i]  = ADS018_torque_factor_n[0]/f2;
        ADS018_energy_factor_n[i]  = ADS018_energy_factor_n[0]/f2;
        ADS018_power_factor_n[i]   = ADS018_power_factor_n[0]/f2;
			  
			}
}
 
                            //pin=&ADS018_mem_data_c
void ADS018_Set_Result_C(ADS018_mem_data_Type *pin, ADS018_res_data_Type *pout)//function declaration
{
//NRF_LOG_INFO("ADCT %d",ads1120_ADCT);
#ifdef ADS018_USE_FN
	// calcula carga corrigida pela eficiencia usando carga positiva e negativa medida
	pADS018_NV_buf->Kp=500;
	int32_t loadc = ((pADS018_NV_buf->Kp*(pin->fp)) + (pADS018_NV_buf->Kn*(pin->fn)))/1000;
	//NRF_LOG_INFO("loadc %d",loadc);
	//NRF_LOG_INFO("load %d",loadc);
#else
	// calcula carga corrigida pela eficiencia usando carga total medida
	int32_t loadc = (pADS018_NV_buf->Kp*(pin->f)/1000);//ignore
#endif

//limita carga corrigida para valores positivos
	if (loadc<0){
		loadc = -1*loadc;
//	    pout->load     = ADS018_load_factor_n[ADS018_Cycle_S_n] * loadc;
//        if (ADS018_uart_on){=================================================================================================================
//            pout->torque   = ADS018_torque_factor_n[ADS018_Cycle_S_n] * loadc;
//        }
		loadc = 0;
		ADS018_Cycle_Result = 2;  // carga negativa forcada para zero mas considerada normal
		
	}
	else{
//	    pout->load     = ADS018_load_factor_n[ADS018_Cycle_S_n] * loadc;
//        if (ADS018_uart_on){================================================================================================================
//            pout->torque   = ADS018_torque_factor_n[ADS018_Cycle_S_n] * loadc;
//        }

	}
	int32_t load = (pin->f);
	pout->load = (ADS018_load_factor_n[ADS018_Cycle_S_n] * load);
//	pout->load=ADC_sample*ADS018_Cal_A+ADS018_Cal_B;

#ifdef SHOWSTATS
SUPER_LOG(pout->load,"pout>load");
//SUPER_LOG(load_mauro,"carga final para calculo %d");
#endif
//	
 //inicia supondo n=0
	pout->rotation = 0;
	pout->energy   = 0;
	pout->power    = 0;
//trata para n<>0
	if ((pin->n)>0){
		if((pin->n) > ADS018_Rotation_Max_N){
			pout->n = 0;
	        ADS018_Cycle_Result = 4;// rotacao abaixo do minimo 12 rpm
	    }
		else{
			if((pin->n) < ADS018_Rotation_Min_N){
						pout->n = 0;
		        ADS018_Cycle_Result = 2; // rotacao acima do maximo 125 rpm

			}
			else{
			        pout->n =(pin->n);
	            pout->rotation = ADS018_rotation_factor / (pin->n);
				#ifdef SHOWSTATS
				      NRF_LOG_INFO(" pout->rotation  %d", pout->rotation);
				#endif
		          pout->energy   = ADS018_energy_factor_n[ADS018_Cycle_S_n] * loadc;
				      pout->power    = ADS018_power_factor_n[ADS018_Cycle_S_n] * loadc / (pin->n);
				#ifdef  SHOWSTATS
				      SUPER_LOG(pout->power," pout->power");
				#endif
	            if (ADS018_Cycle_Result == 0) ADS018_Cycle_Result = 1; // valor normal, preserva =2 definido antes
			}
		}
	}
}

void ADS018_Set_Result(ADS018_res_data_Type *pin, ADS018_res_data_Type *pout, int len)
{
	if (len>0){
		  pout->rotation = pin->rotation/len;
      pout->load     = pin->load/len;
      pout->torque   = pin->torque/len;
	    pout->energy   = pin->energy/len;
	    pout->power    = pin->power/len;
	}
}




void ADS018_Cycle(int16_t vel)
{
	
	// count cycles and set direction
 // y transitions
	ADS018_y = vel;
	switch (ADS018_stt_flag_y){
        case  0: // wait top transition
                 if (ADS018_y > 0){
                	 ADS018_stt_flag_y = 1;//flag state 1=waiting top transition
                 }
        	     break;
        case  1: // confirm top transition
                 if (ADS018_y > ADS018_cyle_bac_limit_y){
	                 ADS018_last_flag_y = ADS018_flag_y;
                	 ADS018_flag_y = 1;
                	 ADS018_stt_flag_y = 2;//flag state 2=top transition confirmed
                 }
                 else{
                	 ADS018_stt_flag_y = 0;//set to zero to go back to waiting top transition
                 }
        	     break;
        case  2: // wait botton transition
                 if (ADS018_y < 0){
           	         ADS018_stt_flag_y = 3;
                 }
        	     break;
        case  3: // confirm botton transition
                 if (ADS018_y < ADS018_cyle_fro_limit_y){
                     ADS018_last_flag_y = ADS018_flag_y;
           	         ADS018_flag_y = 0;
           	         ADS018_stt_flag_y = 0;
                 }
                 else{
           	         ADS018_stt_flag_y = 2;
                 }
        	     break;
        default: break;
	}

    if(ADS018_last_flag_y != ADS018_flag_y){ //y orientation changed 
    	ADS018_last_flag_y = ADS018_flag_y;
    	if(ADS018_cyle_fro_limit_y != 0){
    		ADS018_SCycle_Tout_Count = 0;
    		ADS018_SCycle_Tout_Num = 3;
    	}
        switch (ADS018_Cycle_Stt){
        case  0: //init
        	     if(ADS018_flag_y == 1){ // wait transition (-) -> (+),y against gravity
       	    	     ADS018_Cycle_Flag = 0;
       	    	     ADS018_Cycle_Dir = 1; // direction counter clock wise
        	         ADS018_Cycle_Stt = 1;
                   ADS018_Cycle_Last_S = 0;
                   ADS018_Cycle_S = 1;
        	    	 ADS018_Cycle_S_Cmd = 1; //signal initial transition
        	     }
    	         break;
        case  1: //on top
             	 if(ADS018_flag_y == 0){
             		 ADS018_Cycle_Last_S = 1;
             		 ADS018_Cycle_S = 0;
             		 ADS018_Cycle_S_Cmd = 2;
             		 ADS018_Cycle_Stt = 2;
        	     }
    	         break;
        case  2: //on botton
             	 if(ADS018_flag_y == 1){
             		ADS018_Cycle_Last_S = 0;
             		ADS018_Cycle_S = 1;
    	    		ADS018_Cycle_S_Cmd = 2;
             		ADS018_Cycle_Stt = 1;
             		ADS018_Cycle_Flag = 1;
             		ADS018_CyleCounter++;
        	     }
    	         break;
        default: break;
        }
    }
    else{
    	if(ADS018_Cycle_Stt != 0){
    	    ADS018_SCycle_Tout_Count++;
    	    if (ADS018_SCycle_Tout_Count > ADS018_SCycle_Tout_Limit){
    		    ADS018_SCycle_Tout_Count = 0;
    	        ADS018_Cycle_Stt = 0;
    		    ADS018_SCycle_Stt = 0;
    		    ADS018_SCycle_Tout_Num--;
    		    ADS018_SCycle_Tout_Cmd = 1;
    	    }
    	}
    	else{
    		if (ADS018_SCycle_Tout_Num != 0){
        	    ADS018_SCycle_Tout_Count++;
        	    if (ADS018_SCycle_Tout_Count > ADS018_SCycle_Tout_Limit){
    			    ADS018_SCycle_Tout_Count = 0;
        	        ADS018_Cycle_Stt = 0;
        		    ADS018_SCycle_Stt = 0;
    			    ADS018_SCycle_Tout_Num--;
    			    ADS018_SCycle_Tout_Cmd = 1;
        	    }
    		}
    	}

    }

}



void ADS018_res_data_clear(ADS018_res_data_Type *p){
    p->n = 0;
    p->rotation = 0;
    p->energy = 0;
    p->power = 0;
    p->load = 0;
    p->torque = 0;
}


void init_vars(){
		int i;
		for (i=0;i<ADS018_Cycle_S_n;i++){
			
			ADS018_mem_data_s[i].n = 0;
			ADS018_mem_data_s[i].f = 0;
#ifdef ADS018_USE_FN
			ADS018_mem_data_s[i].fp = 0;
			ADS018_mem_data_s[i].fn = 0;
#endif
		}
	}

	
	void init_varc(){
		ADS018_mem_data_c.f = 0;
		ADS018_mem_data_c.n = 0;
#ifdef ADS018_USE_FN
		ADS018_mem_data_c.fp = 0;
		ADS018_mem_data_c.fn = 0;
#endif
	}
	
	
void init_f(void){

//		int i;

		ADS018_res_data_f_len = 0; 
		ADS018_res_data_clear((ADS018_res_data_Type *)&ADS018_res_data_m);

//		for (i=0; i<ADS018_res_data_f_len; i++){
//			ADS018_res_data_f[i].n = 0;
//			ADS018_res_data_f[i].rotation = 0;
//			ADS018_res_data_f[i].load = 0;
//			ADS018_res_data_f[i].torque = 0;
//			ADS018_res_data_f[i].energy = 0;
//			ADS018_res_data_f[i].power = 0;
//		}
	
		ADS018_res_data_f_in  = ADS018_res_data_f_len;
		ADS018_res_data_f_out = 0;
		ADS018_res_data_f_stt = 1;
	}

void remove_f(void){
		if (ADS018_res_data_f_len > ADS018_F_LEN_MIN){
		    ADS018_res_data_m.n -= ADS018_res_data_f[ADS018_res_data_f_out].n;
		    ADS018_res_data_m.rotation -= ADS018_res_data_f[ADS018_res_data_f_out].rotation;
		    ADS018_res_data_m.load -= ADS018_res_data_f[ADS018_res_data_f_out].load;
		    ADS018_res_data_m.torque -= ADS018_res_data_f[ADS018_res_data_f_out].torque;
		    ADS018_res_data_m.energy -= ADS018_res_data_f[ADS018_res_data_f_out].energy;
		    ADS018_res_data_m.power -= ADS018_res_data_f[ADS018_res_data_f_out].power;
		    ADS018_res_data_f_out++;
		    if (ADS018_res_data_f_out >= ADS018_F_N) ADS018_res_data_f_out = 0;
		    ADS018_res_data_f_len--;
		}
	}

	
void acc_f(void){
//#define DEBUG_FILA
#if defined (DEBUG_FILA)
		ADS018_res_data_c.n = 3;
		ADS018_res_data_c.rotation = 55;
		ADS018_res_data_c.load = 20;
		ADS018_res_data_c.torque = 100;
		ADS018_res_data_c.energy = 200;
		ADS018_res_data_c.power = 185;
#endif
		if(ADS018_res_data_f_len < ADS018_F_N){
		    ADS018_res_data_f[ADS018_res_data_f_in].n = ADS018_res_data_c.n;
		    ADS018_res_data_f[ADS018_res_data_f_in].rotation = ADS018_res_data_c.rotation;
		    ADS018_res_data_f[ADS018_res_data_f_in].load = ADS018_res_data_c.load;
		    ADS018_res_data_f[ADS018_res_data_f_in].torque = ADS018_res_data_c.torque;
		    ADS018_res_data_f[ADS018_res_data_f_in].energy = ADS018_res_data_c.energy;
		    ADS018_res_data_f[ADS018_res_data_f_in].power = ADS018_res_data_c.power;
		    ADS018_res_data_m.n += ADS018_res_data_c.n;
		    ADS018_res_data_m.rotation += ADS018_res_data_c.rotation;
		    ADS018_res_data_m.load += ADS018_res_data_c.load;
		    ADS018_res_data_m.torque += ADS018_res_data_c.torque;
		    ADS018_res_data_m.energy += ADS018_res_data_c.energy;
		    ADS018_res_data_m.power += ADS018_res_data_c.power;
		    ADS018_res_data_f_in++;
		    if (ADS018_res_data_f_in >= ADS018_F_N) ADS018_res_data_f_in = 0;
		    ADS018_res_data_f_len++;
		}
	}
	
	
	void insert_f(void){
		remove_f();
		acc_f();
	}
	
	
	
void insert_zero(void){

		remove_f();

		ADS018_res_data_f[ADS018_res_data_f_in].n = 0;
		ADS018_res_data_f[ADS018_res_data_f_in].rotation = 0;
		ADS018_res_data_f[ADS018_res_data_f_in].load = 0;
		ADS018_res_data_f[ADS018_res_data_f_in].torque = 0;
		ADS018_res_data_f[ADS018_res_data_f_in].energy = 0;
		ADS018_res_data_f[ADS018_res_data_f_in].power = 0;
	    ADS018_res_data_f_in++;
		if (ADS018_res_data_f_in >= ADS018_F_N) ADS018_res_data_f_in = 0;
		ADS018_res_data_f_len++;
	}

void update_s(void){
		if (ADS018_mem_data_s[ADS018_Cycle_Last_S].n>0){
			ADS018_mem_data_s[ADS018_Cycle_Last_S].f = ADS018_mem_data_s[ADS018_Cycle_Last_S].f / ADS018_mem_data_s[ADS018_Cycle_Last_S].n;
			ADS018_mem_data_c.n += ADS018_mem_data_s[ADS018_Cycle_Last_S].n;
			ADS018_mem_data_c.f += ADS018_mem_data_s[ADS018_Cycle_Last_S].f;
#ifdef ADS018_USE_FN
			ADS018_mem_data_s[ADS018_Cycle_Last_S].fp = ADS018_mem_data_s[ADS018_Cycle_Last_S].fp / ADS018_mem_data_s[ADS018_Cycle_Last_S].n;
			ADS018_mem_data_s[ADS018_Cycle_Last_S].fn = ADS018_mem_data_s[ADS018_Cycle_Last_S].fn / ADS018_mem_data_s[ADS018_Cycle_Last_S].n;
			ADS018_mem_data_c.fp += ADS018_mem_data_s[ADS018_Cycle_Last_S].fp;
			ADS018_mem_data_c.fn += ADS018_mem_data_s[ADS018_Cycle_Last_S].fn;
			ADS018_mem_data_s[ADS018_Cycle_Last_S].fp = 0;
			ADS018_mem_data_s[ADS018_Cycle_Last_S].fn = 0;
#endif
			ADS018_mem_data_s[ADS018_Cycle_Last_S].n = 0;
			ADS018_mem_data_s[ADS018_Cycle_Last_S].f = 0;
		}
	}

void update_c(void){
	
	     if(ADS018_Cycle_Flag == 1){ //on sector==0
				 ADS018_Update_Factors();
	    	 ADS018_Set_Result_C((ADS018_mem_data_Type *)&ADS018_mem_data_c, (ADS018_res_data_Type *)&ADS018_res_data_c);
				 ADS018_Energy += ADS018_res_data_c.energy;
				 if (ADS018_res_data_f_stt == 0) init_f(); //init mean
    		 else{
	    	     //update mean
				 
    			 switch(ADS018_Cycle_Result){
    			 case  0: break; // undefined, ignore
    			 case  1: // normal - positive load
    			 case  2: // normal - negative load forced to zero
//    				      if(ADS018_res_data_m.n == 0){ // filled with zeros
//    				    	  insert_f(); // remove old and add sample to mean
//    				      }
//    				      else{
    				          // adjust len
//	    		              while(((ADS018_res_data_m.n + ADS018_res_data_c.n) > ADS018_f_limit) && (ADS018_res_data_f_len > ADS018_F_LEN_MIN)){
//	    		    		      remove_f(); // remove old sample
//	    		              }

	    		    	      if(ADS018_res_data_f_len == ADS018_F_N){
	    		    		      insert_f(); // remove old and add sample to mean
												acc_f();
	    		    	      }
	    		    	      else{
	    		    		      acc_f(); // add sample to mean
	    		    	      }
//    				      }
   				          break;
    			 case  3: break; // rpm > max rpm, ignore
    			 case  4: break; // rpm < min rpm, ignore
    			 default: break;
    			 }
    		 }
   		     nnnn = ADS018_res_data_c.n;
 	    	 ADS018_res_data_cmd = 1; // update advertising data
   		     init_varc();
   		     ADS018_Cycle_Result = 0; // reinicia avaliacao do ciclo
	    }
    }

		
void init_var(void){
		init_vars();
		init_varc();
	}


void ADS018_Update_SCycle(void)
{			
/* In the original code, taking it was made in eclipse,the following functions were declared inside ADS018_Update_SCycle. Since we're making
  all our projects in keil, they were declared outside this function but left commented here for clarity.*/
//  init_vars();
//  init_varc();
//  init_f();
//  remove_f();
//  acc_f();
//	insert_zero();
//	update_s();//
//	update_c();
//init_var();

	if (ADS018_SCycle_Tout_Cmd == 1){
	    //sector timeout
        ADS018_SCycle_Tout_Cmd = 0;
//    	ADS018_Print_Str("tout2\n\r");
    	  init_var();
        ADS018_res_data_clear((ADS018_res_data_Type *)&ADS018_res_data_c);
/*    	if (ADS018_res_data_f_stt == 0) */ init_f(); //init mean
//        insert_zero();
        ADS018_Cycle_Result  = 0;  // reinicia avaliacao do ciclo
        ADS018_res_data_cmd  = 1;  // update advertising data
    }
    else{
        //q change
        switch(ADS018_Cycle_S_Cmd){
        case  0: break;
        case  1: init_var();     
	             ADS018_SCycle_Stt = 1;
	             ADS018_res_data_f_stt = 0;
                 break;
        case  2: //change q on load
					     update_s();
    	         update_c();
    		       break;
	    default: break;
	    }
	    ADS018_Cycle_S_Cmd = 0;
    }
	//update on sector
    switch(ADS018_SCycle_Stt){
	case  0: break; // sync init
	case  1: // run load
	         ADS018_mem_data_s[ADS018_Cycle_S].n++;
	         ADS018_mem_data_s[ADS018_Cycle_S].f += ads1120_ADCT;
#ifdef ADS018_USE_FN
	         if (ads1120_ADCT>0){
	        	 ADS018_mem_data_s[ADS018_Cycle_S].fp += ads1120_ADCT;//AQUIIIIIIIIIIII
	         }
	         else{
	        	 ADS018_mem_data_s[ADS018_Cycle_S].fn += ads1120_ADCT;
	         }
#endif
		     break;
    case  2: // run unload
			ADS018_mem_data_s[ADS018_Cycle_S].n++;//state never reached,optimize ?
		 	 break;
	default: break;
	}
}


void ADS018_Time_Update(void)
{
	if (ADS018_SampleCounter == 0xffffffff) ADS018_SampleCounter=0;
	else ADS018_SampleCounter++;
	ADS018_ShowCounter++;
	if (ADS018_ShowCounter > ADS018_ShowLimit) ADS018_ShowCounter=0;
	ADS018_Sec_Prescaler++;
	if (ADS018_Sec_Prescaler >= ADS018_Sec_Limit){
		ADS018_Sec_Prescaler = 0;
		ADS018_Sec_Counter++;
		if (ADS018_Sec_Counter >= 60){
			ADS018_Sec_Counter = 0;
			ADS018_Min_Counter++;
			if (ADS018_Min_Counter >= 60) ADS018_Min_Counter = 0;
		}
	}
}



//only to advertise, result should be ready even without this function.
void ADS018_Update_Advertising_Data(void)//only to advertise, result should be
{
    pADS018_transfer->Cycle    = ADS018_CyleCounter;
    pADS018_transfer->Minutes  = ADS018_Min_Counter;
    pADS018_transfer->Seconds  = ADS018_Sec_Counter;
    pADS018_transfer->Energy   = (uint16_t)(ADS018_Energy/1000);
    if( ADS018_res_data_f_len ) {
		pADS018_transfer->Power    = ADS018_res_data_m.power/ADS018_res_data_f_len;
		pADS018_transfer->HR       = nnnn*10;
		pADS018_transfer->Torque   = (uint16_t)ADS018_res_data_m.torque/ADS018_res_data_f_len;
		pADS018_transfer->Rotation = (uint16_t)ADS018_res_data_c.rotation;
		pADS018_transfer->Load     = (uint16_t)ADS018_res_data_c.load;
    }
    else {
		pADS018_transfer->Power    = 0;
		pADS018_transfer->HR       = 0;
		pADS018_transfer->Rotation = 0;
		pADS018_transfer->Load     = 0;
		pADS018_transfer->Torque   = 0;
    }
}


void ADS018_Set_Mean_Data(void)
{
	if (ADS018_res_data_cmd  == 1){
	    ADS018_res_data_cmd  = 0;
	    ADS018_Set_Result((ADS018_res_data_Type *)&ADS018_res_data_c, (ADS018_res_data_Type *)&ADS018_res_data, ADS018_res_data_f_len);//alterado de data m para data c
	}
		
}

//filter***********************************

//Parameter:
//1) inputy variable to be filtered
//2) input2 2nd variable to be filtered
//*/
void filter_init(int16_t inputy,int16_t input2)//two variables, one for each signal intended to be filtered.
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
#ifdef SHOWFILTER_OUT4
	NRF_LOG_INFO("out 4 %d",out4);
#endif
}


//after this function we're completely capable to adjust the values read by the ADC
void ADS018_Cal_Set(ADS018_cal_Type *ptab, float *pa, float *pb, int16_t *pix, int16_t *pd)
{

	//Aqui eh importante lembrar do ioio mixoxo y=m(x-x0)+y0
	volatile float a;//
 	volatile float b;

    if (     (    ptab->adc_value[1] == ptab->adc_value[0]    )  )
			{
    	ptab->adc_value[1] = ptab->adc_value[0] + 200; // fix to default
    }

    a = (ptab->eng_value[1]-ptab->eng_value[0]);//y-y0
		//aqui dava 8
    if ((a>-1.0) && (a<1.0)){
    	ptab->eng_value[0] = 0.0;  // fix to default
    	ptab->adc_value[1] = ADS018_LOAD_REF_DEFAULT;     // fix to default
    	a = (ptab->eng_value[1]-ptab->eng_value[0]);
    }

    a = a/(1000.0*(ptab->adc_value[1]-ptab->adc_value[0]));//m//aqui dava 0,008 OU SEJA m/1000 que eh devido estar em gf e eu querer em kgf
		b = (ptab->eng_value[0]/1000.0) - (ptab->adc_value[0]*a);//b esta dando 0.013 OU SEJA y0/1000
			
    *pa = a*1000;//alterado 14/06
  	*pb = b*1000;

	if (a!= 0.0){
		 *pix = -b/a;
		
	}
	else *pix = 0;

	int16_t d = (ptab->adc_value[1]-ptab->adc_value[0]);
	if (ptab->adc_value[1] < ptab->adc_value[0]) d = -1*d;

	*pd = d;

}

void ADS018_Set_Cal_Adc_Zero(int16_t new_zero){
	ADS018_Set_Cal_ADC_Value_Zero(new_zero);
	//updates all the values used for calculations after updating via ble
	ADS018_Cal_Set((ADS018_cal_Type *)&(pADS018_NV_buf->Cal), (float *)&ADS018_Cal_A, (float *)&ADS018_Cal_B, (int16_t *)&ADS018_Cal_ADC_Zero, (int16_t *)&ADS018_Cal_ADC_Delta);
}

void ADS018_Set_Cal_ADC_Value_Zero(int16_t new_adc_zero)
{
	if(new_adc_zero != ADS018_NV_buf.Cal.adc_value[1]){
	    ADS018_NV_buf.Cal.adc_value[0] = ((int32_t)new_adc_zero);
	  //  if (ADS018_Save_Setup() == NRF_SUCCESS){};
    }
}


void get_direction(void)
{
//	
//	lis2dw12_reg_t int_route;
//  lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
//  int_route.ctrl4_int1_pad_ctrl.int1_6d = PROPERTY_ENABLE;
//  lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);

				lis2dw12_all_sources_get(&dev_ctx, &all_source);
// if (all_source.sixd_src._6d_ia) {

      if (all_source.sixd_src.xh) {
				#ifdef SHOWDIRECTION
 			 NRF_LOG_INFO("XH");
        #endif 
      }

      if (all_source.sixd_src.xl) {
				#ifdef SHOWDIRECTION
				 NRF_LOG_INFO("XL");
				#endif

      }

      if (all_source.sixd_src.yh) {
				mma8x5x_pl = ADS018_POS_BOT;
				#ifdef SHOWDIRECTION
				 NRF_LOG_INFO("YH");
				#endif
      }

      if (all_source.sixd_src.yl) {
				mma8x5x_pl = ADS018_POS_TOP;
				#ifdef SHOWDIRECTION
				 NRF_LOG_INFO("YL");
				#endif
      }

      if (all_source.sixd_src.zh) {
				mma8x5x_pl = ADS018_POS_FRONT;
				#ifdef SHOWDIRECTION
				 NRF_LOG_INFO("ZH");
				
				#endif
      }

      if (all_source.sixd_src.zl) {
				mma8x5x_pl = ADS018_POS_REAR;
				#ifdef SHOWDIRECTION
				 NRF_LOG_INFO("ZL");
				#endif
      }

//   }
			//Adjust to define when mma8x5x_pl will be front or rear to pass to function ADS018_Z()
			}


//void ADS018_AZ(void)
//{
//    switch (ADS018_AZ_stt){
//		case  0: //check auto-zero position
////			ADS018_AZ_stt = 4;
////			if (mma8x5x_pl == ADS018_POS_BOT)
//			//{
//		//(int16_t)(((mma8x5x_regs.data[4] << 8) & 0xff00) | mma8x5x_regs.data[5]);//encontrar uma maneira de colocar a aceleracao em z aqui

//		ADS018_raw.Zaccel=1200;//simulation, that way I know it'll be always in range.
//							if ((ADS018_raw.Zaccel >= ADS018_AZ_Z_LI) && (ADS018_raw.Zaccel <= ADS018_AZ_Z_LS)){
//					ADS018_AZ_counter = 0;
//					ADS018_AZ_stt = 1;
//				}
//			//}
//			break;
//		case  1: //wait debounce
////			if (mma8x5x_pl != ADS018_POS_BOT){
////				ADS018_AZ_stt = 0;
////			}
////			else
//			{
//				ADS018_AZ_counter++;
//				if (ADS018_AZ_counter >= ADS018_AZ_tdebounce){
//					ADS018_AZ_mean = 0.0;
//					ADS018_AZ_sdev = 0.0;
//					ADS018_AZ_counter = 0;
//					ADS018_AZ_stt = 2;
//				}
//			}
//			break;
//		case  2: //evaluate mean and sdev(standard deviation

//			ADS018_raw.Load   = ADC_sample;
//			ads1120_ADC = ADC_sample;
//			ads1120_ADCB  = ADS018_Load_Balanced(ads1120_ADC); // !< ads1120 ADC binary balanced//ate aqui entendo 1200-50
//     
//			ADS018_AZ_load  = (1.0*ads1120_ADCB);
//			ADS018_AZ_mean=ADS018_AZ_mean + ADS018_AZ_load;
//		  ADS018_AZ_sdev=ADS018_AZ_sdev + (ADS018_AZ_load*ADS018_AZ_load);
//		  ADS018_AZ_counter++;
//			if (ADS018_AZ_counter >= ADS018_AZ_n){//ADS018_AZ_n=15 at this point
//				ADS018_AZ_mean = ADS018_AZ_mean / ADS018_AZ_counter;//here's the infamous strategy means are made with the division of total occurred event here he set to 15 values
//				ADS018_AZ_sdev = sqrtf(fabsf((ADS018_AZ_sdev / ADS018_AZ_counter) - (ADS018_AZ_mean * ADS018_AZ_mean)));
//				ADS018_AZ_tol  = ADS018_AZ_FACTOR * ADS018_AZ_sdev;
//				ADS018_AZ_stt = 3; //assign TARE
//			}
//			break;
//		case  3: //assign TARE
//			ADS018_AZ_absmean = fabsf(ADS018_AZ_mean);
//		 		  //			if (ADS018_AZ_absmean > ADS018_AZ_tol)
//			//{
//				if( mma8x5x_pl == ADS018_POS_FRONT )//ZH
//				{
//					ADS018_tare = ADS018_AZ_mean - 4.0;
//					ADS018_AZ_Tare = (int16_t)ceil(ADS018_AZ_mean-3.5); //set tare
//				}
//				else if( mma8x5x_pl == ADS018_POS_REAR )//ZL
//				{
//					ADS018_tare = ADS018_AZ_mean + 4.0;
//					ADS018_AZ_Tare = (int16_t)ceil(ADS018_AZ_mean+4.5); //set tare
//				}
//				else
//				{
//					ADS018_tare = ADS018_AZ_mean;
//					ADS018_AZ_Tare = (int16_t)ceil(ADS018_AZ_mean+0.5); //set tare
//					
//				}

//				ADS018_AZ_counter = 0;

//		//	}
////			else{
////				ADS018_PrintInt(">>> AZ: Tare = %6d not updated\n\r", (int)ADS018_AZ_Tare);
////			}
//			ADS018_AZ_stt = 4; // go to system off
//			break;
//		case 4: // start debounce
//			ADS018_AZ_counter = 0;
//			ADS018_AZ_stt = 5;
//			break;
//		case 5: // wait debounce time
//			ADS018_AZ_counter++;
//			if (ADS018_AZ_counter >= ADS018_AZ_tdebounce){
//				pADS018_transfer->Cmd = 3; // call BLE off
//				ADS018_AZ_flag = 0;
//				ADS018_AZ_stt = 6; // wait for BLE off
//			}
//			break;
//		case 6: // System_Off
//							ADS018_AZ_stt = 0;//mudaddo348
//			if (ADS018_AZ_flag != 0){
//			    //ADS018_System_Off_Prepare();//I wont use this
//			}
//			break;
//		default: ADS018_AZ_stt = 0;
//			break;
//    }
//}


int16_t ADS018_Load_Balanced(int16_t load_adc)
{
	return(load_adc - ADS018_Cal_ADC_Zero); // !< ads1120 ADC binary balanced /mauro:x-y0/m=x-erro de precisao do zero
}


void get_load (int16_t adc_measure){
	
	  ADS018_Cal_A=ADS018_Cal_A*1;
		ADS018_Cal_B=ADS018_Cal_B*1;  
	
  
	//ADS018_AZ_Tare=33;//later will be removed
		ADS018_raw.Load   = adc_measure;//dado bruto de carga vide linha 2026 do ads018.c
		ads1120_ADC = adc_measure;//dado bruto de carga vide linha 2026 do ads018.c
		ads1120_ADCB  = ADS018_Load_Balanced(ads1120_ADC); // !< ads1120 ADC binary balanced
//  	ads1120_ADCT  = ads1120_ADCB;     // !< ads1120 ADC binary balanced and tared
		ads1120_ADCT  = ads1120_ADCB - ADS018_AZ_Tare;     // !< ads1120 ADC binary balanced and tared
//		ADS018_Meas_Exec(1,(int16_t)ads1120_ADC);          // !< evaluate mean ads1120 ADC

}



//void ADS018_AZ(void)
//{
//		switch (ADS018_AZ_stt){
//		  case  0: //check auto-zero position//not used in mywatt v2
//				   ADS018_AZ_stt=1;
//			break;
//		case  1: //wait debounce
//					ADS018_AZ_stt = 2;
//			break;
//		case  2: //evaluate mean and sdev
//      ADS018_raw.Load   = ADC_sample;//pegando valor de ADC e colocando em uma variavel minha
//			ads1120_ADC = ADC_sample;
//			ads1120_ADCB  = ADS018_Load_Balanced(ads1120_ADC); // !< ads1120 ADC binary balanced

//			ADS018_AZ_load  = (1.0*ads1120_ADCB);
//			ADS018_AZ_mean=ADS018_AZ_mean + ADS018_AZ_load;
//			ADS018_AZ_sdev=ADS018_AZ_sdev + (ADS018_AZ_load*ADS018_AZ_load);
//			ADS018_AZ_counter++;
//			if (ADS018_AZ_counter >= ADS018_AZ_n){
//				ADS018_AZ_mean = ADS018_AZ_mean / ADS018_AZ_counter;
//				ADS018_AZ_sdev = sqrtf(fabsf((ADS018_AZ_sdev / ADS018_AZ_counter) - (ADS018_AZ_mean * ADS018_AZ_mean)));
//				ADS018_AZ_tol  = ADS018_AZ_FACTOR * ADS018_AZ_sdev;
//				ADS018_AZ_stt = 3; //assign TARE vou usar depois pra controlar o switch case
//			}
//      break ;
//			ADS018_AZ_absmean = fabsf(ADS018_AZ_mean);
////			if (ADS018_AZ_absmean > ADS018_AZ_tol)
//			//{
//				if( mma8x5x_pl == ADS018_POS_FRONT )
//				{
//					ADS018_tare = ADS018_AZ_mean - 4.0;
//					ADS018_AZ_Tare = (int16_t)ceil(ADS018_AZ_mean-3.5); //set tare
//				}
//				else if( mma8x5x_pl == ADS018_POS_REAR )
//				{
//					ADS018_tare = ADS018_AZ_mean + 4.0;
//					ADS018_AZ_Tare = (int16_t)ceil(ADS018_AZ_mean+4.5); //set tare
//				}
//				else
//				{
//					ADS018_tare = ADS018_AZ_mean;
//					ADS018_AZ_Tare = (int16_t)ceil(ADS018_AZ_mean+0.5); //set tare
//				}

//				ADS018_AZ_counter = 0;
//   			ADS018_AZ_stt = 4; 
//        break ;
//		case 4: // start debounce
//			ADS018_AZ_stt = 5;
//			break;
//	default: ADS018_AZ_stt = 0;
//			break;
//    }
//}


//end for now
void lis2dw12_status(void)
{
lis2dw12_all_sources_t all_source;

 lis2dw12_all_sources_get(&dev_ctx, &all_source);

    /* Check if Activity/Inactivity events */
    if (all_source.wake_up_src.sleep_state_ia) {
      //manda no pino sleep =====sd_power_system_off();
    }

    if (all_source.wake_up_src.wu_ia) {
      nrf_gpio_pin_write(26,0);//MAYBE
    }	
	
	

}

void get_session(int16_t accel){
	
	//conditions that the the set as normal operation
	//accel. presence
	if (accel<30){
		
		session=0;//the most restrictive power mode
	}
	else{
		session=1;
	}
	
	
	
}





