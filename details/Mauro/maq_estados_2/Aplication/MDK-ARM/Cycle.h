#include <stdint.h>
#include "filter.h"
#include "nrf_log_ctrl.h"
#include "nrf_delay.h"
//#include "nrf_log.h"
//DEFINES=======================================================================================================
#define ADS018_S_N 2
#define ADS018_S_N_MAX 10
#define ADS018_F_LEN_MIN 3     /**< min samples on advertising filter */
#define ADS018_F_N 8
#define ADS018_USE_FN 1
#define ADS1120_MAX_N_CAL 2
//STRUCTS,UNIONS and other struc like types===================================================================================
typedef struct {
	int32_t f;
	int32_t fp;
	int32_t fn;
	int32_t n;
} ADS018_mem_data_Type;
typedef struct {
	uint32_t n;
	int32_t rotation;
	int32_t load;
	int32_t torque;
	int32_t energy;
	int32_t power;
} ADS018_res_data_Type;
typedef struct {
	int32_t adc_value[ADS1120_MAX_N_CAL];
	int32_t uV_value[ADS1120_MAX_N_CAL];
	int32_t eng_value[ADS1120_MAX_N_CAL];
	uint32_t n_point;
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
	int32_t Kp; //kilopounds?
	int32_t Kn; //kilonewtons?
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
//VARIABLES=====================================================================================================================
// CYCLE
volatile uint16_t glob_var;
volatile int16_t ADS018_raw_Yaccel_F = 0;
volatile int16_t ADS018_y=0;
volatile int16_t ADS018_cyle_bac_limit_y = 500;//originally was 500
volatile int16_t ADS018_cyle_fro_limit_y =-500;//originally was -500
volatile uint16_t ADS018_last_flag_y=0;
volatile uint16_t ADS018_flag_y=0;
volatile uint16_t ADS018_stt_flag_y = 0;

volatile uint16_t ADS018_SCycle_Stt = 0;       // !< state of sector state machine
volatile uint32_t ADS018_SCycle_Tout_Count = 0;          // !< sample count for cyle timeout   
volatile uint32_t ADS018_SCycle_Tout_Limit = 75;         // !< limit count for cyle timeout
volatile uint32_t ADS018_SCycle_Tout_Num = 3;            // !< number times update zero adv
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
volatile int16_t ads1120_ADCT = 0;               // !< ads1120 ADC binary balanced and tared
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

ADS018_NV_Type *pADS018_NV_buf;
//FUNCTION PROTOTOTYPES=============================================================================================================================================================================================================================================================================================
void ADS018_Time_Update(void);
void ADS018_Set_Result_C(ADS018_mem_data_Type *pin, ADS018_res_data_Type *pout);
void ADS018_Set_Result(ADS018_res_data_Type *pin, ADS018_res_data_Type *pout, int len);
void ADS018_Cycle(int16_t vel,uint16_t *dir,int16_t *counter);
void ADS018_res_data_clear(ADS018_res_data_Type *p);
void init_vars();
void init_varc();
void init_f(void);
void remove_f(void);
void acc_f(void);
void insert_f(void);
void insert_zero(void);
void update_s(void);
void update_c(void);
void init_var(void);
void ADS018_Time_Update();
void ADS018_Update_SCycle(void);
void ADS018_Set_Mean_Data(void);
//FUNCTIONS=============================================================================================================================================================================================================================
                          //pin=&ADS018_mem_data_c
void ADS018_Set_Result_C(ADS018_mem_data_Type *pin, ADS018_res_data_Type *pout)//function declaration
{
#ifdef ADS018_USE_FN
	// calcula carga corrigida pela eficiencia usando carga positiva e negativa medida
	int32_t loadc = ((pADS018_NV_buf->Kp*(pin->fp)) + (pADS018_NV_buf->Kn*(pin->fn)))/1000;
#else
	// calcula carga corrigida pela eficiencia usando carga total medida
	int32_t loadc = (pADS018_NV_buf->Kp*(pin->f)/1000);
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
		//NRF_LOG_INFO("linha 145 do Cycle.h");
	}
	else{
//	    pout->load     = ADS018_load_factor_n[ADS018_Cycle_S_n] * loadc;
//        if (ADS018_uart_on){================================================================================================================
//            pout->torque   = ADS018_torque_factor_n[ADS018_Cycle_S_n] * loadc;
//        }

	}
	int32_t load = (pin->f);
	pout->load = ADS018_load_factor_n[ADS018_Cycle_S_n] * load;

    //inicia supondo n=0
	pout->rotation = 0;
	pout->energy   = 0;
	pout->power    = 0;
//NRF_LOG_INFO(" essa ele sempre vai passar linha 162 do Cycle.h");
//NRF_LOG_INFO("esse eh o n pro if da rotacao %d",pin->n);
//		pin->n++;//=========================================================APAGAR! AQUI ESTOU FORÇANDO O RESULTADO!!!!!!!FOI SÓ PRA DESCOBRIR QUEM É QUEM!==========================================================================================================================================================================
//    if(pin->n>59){
//			pin->n=0;
//		}
////		ADS018_mem_data_s[ADS018_Cycle_Last_S].n++;
////	if (ADS018_mem_data_s[ADS018_Cycle_Last_S].n>59) {
////		ADS018_mem_data_s[ADS018_Cycle_Last_S].n=0;
//	}=============================================================================================================================================================================================================================================================================================================================
//trata para n<>0 trata para n!=0!!!!!!!!!!
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
		          pout->energy   = ADS018_energy_factor_n[ADS018_Cycle_S_n] * loadc;
	            pout->power    = ADS018_power_factor_n[ADS018_Cycle_S_n] * loadc / (pin->n);
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


void ADS018_Cycle(int16_t vel,uint16_t *dir,int16_t *counter)
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

    if(ADS018_last_flag_y != ADS018_flag_y){ //y orientation changed //se houver qualquer mudança na orientação, podemos começar a medir ciclo.
    	ADS018_last_flag_y = ADS018_flag_y;
    	if(ADS018_SCycle_Tout_Count != 0){
    		ADS018_SCycle_Tout_Count = 0;
    		ADS018_SCycle_Tout_Num = 3;
    	}
        switch (ADS018_Cycle_Stt){
        case  0: //init
        	     if(ADS018_flag_y == 1){ // wait transition (-) -> (+)
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
//	NRF_LOG_INFO("ADS018_Cycle_Last_S: %d",ADS018_Cycle_Last_S);
//	NRF_LOG_FLUSH();
//	NRF_LOG_INFO("ADS018_mem_data_s[ADS018_Cycle_Last_S].n : %d",ADS018_mem_data_s[ADS018_Cycle_Last_S].n);
//	NRF_LOG_FLUSH();
//	NRF_LOG_INFO("ADS018_mem_data_c.n : %d",ADS018_mem_data_c.n);
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
				 //NRF_LOG_INFO("passei pelo Cycle Flag");
	    	 ADS018_Set_Result_C((ADS018_mem_data_Type *)&ADS018_mem_data_c, (ADS018_res_data_Type *)&ADS018_res_data_c);
			   //NRF_LOG_INFO(" rot inst : %d",ADS018_res_data_c.rotation);
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
//  init_vars();
//  init_varc();
//  init_f();
//  remove_f();
//  acc_f();
//	insert_zero();
//	update_s();//
//	update_c();
	//init_var();
	//-------------------------------------------------------------------------------------------------------------------------------------------------------
	//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
	//----------------------------------------------FIM DAS FUNÇÕES-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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
	        	 ADS018_mem_data_s[ADS018_Cycle_S].fp += ads1120_ADCT;
	         }
	         else{
	        	 ADS018_mem_data_s[ADS018_Cycle_S].fn += ads1120_ADCT;
	         }
#endif
		     break;
    case  2: // run unload
	         ADS018_mem_data_s[ADS018_Cycle_S].n++;
		 	 break;
	default: break;
	}
}

//=========================================================================================================================================================
//=========================================================================================================================================================
//====================================FUNCTIONS FOR TEST===================================================================================================
//=========================================================================================================================================================
//=========================================================================================================================================================
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


