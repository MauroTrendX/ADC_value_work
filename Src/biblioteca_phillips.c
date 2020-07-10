
#include "main.h"
#include	"fxi_map.h"
#include	"fxi.h"
#include	"fx_datatypes.h"
#include "lis2dw12_reg.h"
#include "Controle_LEDs.h"
#include "biblioteca_phillips.h"

#define V_BODYPOSITION_LEFT ( 1 )
#define V_BODYPOSITION_RIGHT ( 2 )
#define V_MEMORY_SIZE ( 13000 ) // must be optimised to value of the used library
#define FXI_MAX_METRIC_SIZE ( 80 )
#define FXI_NUMBER_OF_ACC_SAMPLES ( 8 )
#define FXI_NUMBER_OF_PPG_SAMPLES ( 2 )

#define sequence 01

		static FXI_ERROR status = 0;

		static FX_UINT32 libMemory[V_MEMORY_SIZE/4];
//		static FX_UINT16 acc[3][FXI_NUMBER_OF_ACC_SAMPLES];
//		static FX_UINT16 ppg[FXI_NUMBER_OF_PPG_SAMPLES];

		static PFXI_INST pFxInst = 0;

		extern stmdev_ctx_t dev_ctx;
		extern bool trigLoop;
		volatile uint8_t queroEsteLed=1;
		volatile selecionarCores queroEstaCor=0;

		FX_UINT08 ledPower = 3;
		FX_SINT08 adcGain = 3; // gain 0=gain 1, 1=gain 2, 2=gain4, 3=gain8
		FX_SINT08 bodyPosition = V_BODYPOSITION_LEFT;

void init_biblioteca_phillips(void){

		bool HR_enabled = true;
		/* VERSION INFO ************************************************* */
		FX_UINT08 versionInfo[20]; // declaration used in general
		FX_UINT16 versionSize = sizeof(versionInfo);
		status = FXI_GetVersionInfo( versionInfo, &versionSize );  
		/* INITIALISATION ********************************************** */
		FXI_INST_PARAMS fxParams;
		status = FXI_GetDefaultParams( &fxParams );
			
		fxParams.pMem = libMemory;
		fxParams.memorySize = sizeof(libMemory);
		fxParams.pSourceID = NULL;
		fxParams.sourceIDSize = 0;
		status = FXI_Initialise( &fxParams, &pFxInst );		

		/* ENABLE METRICS ******************************************** */
		FXI_METRIC_ID metricIdList[] = { FXI_METRIC_ID_HEARTRATE, FXI_METRIC_ID_SKINPROXIMITY};

		status = FXI_EnableMetrics( pFxInst, sizeof( metricIdList ), metricIdList );



}

void run_biblioteca_phillips(dadosBbPPG amostrasPPG, dadosBbACC amostrasACC, uint8_t * batimentos, bool * contact_detected){
 

		FX_UINT08 data[FXI_MAX_METRIC_SIZE];
		FX_UINT16 metricSize;
		FXI_METRIC_ID metricId;

		FX_UINT16 index = 0;
		
		/* GET AND FEED REQUIRED INPUT METRICS ( ACC, PPG ) ###################### */
		FXI_METRIC_ID requiredMetrics[10];
		FX_UINT08 numberOfRequiredMetrics = sizeof(requiredMetrics);
				
		status = FXI_ListRequiredMetrics( pFxInst, requiredMetrics, &numberOfRequiredMetrics );
		nrf_gpio_pin_set(20);
			
		/* transfer acc metric to library */
		metricId = FXI_METRIC_ID_ACCELERATION; 
		metricSize = ( FXI_NUMBER_OF_ACC_SAMPLES * 6 ) + 4;
		index = 55;
		data[0] = FXI_METRIC_ID_ACCELERATION;				
		data[1] = (uint8_t)( metricSize & 0xFF );
		data[2] = (uint8_t)(( metricSize >> 8 ) & 0xFF );
		data[3] = sequence; // index, for feed arbitrary number
		data[4] = 4; // Q(uality), set fixed to 4
		data[5] = bodyPosition; // fill in sensor wrist position
		data[6] = 0xFF; // SF, fixed to 0xFF for 8 ACC samples
				
#define USE_ACC
#ifdef USE_ACC		
		data[7]  = amostrasACC.amostra1.dadosACC.eixoXlsb; 	
		data[8]  = amostrasACC.amostra1.dadosACC.eixoXmsb;
		data[9]  = amostrasACC.amostra1.dadosACC.eixoYlsb;
		data[10] = amostrasACC.amostra1.dadosACC.eixoYmsb;   //amostra 1
		data[11] = amostrasACC.amostra1.dadosACC.eixoZlsb;
		data[12] = amostrasACC.amostra1.dadosACC.eixoZmsb;
		
		data[13] = amostrasACC.amostra2.dadosACC.eixoXlsb; 	
		data[14] = amostrasACC.amostra2.dadosACC.eixoXmsb;
		data[15] = amostrasACC.amostra2.dadosACC.eixoYlsb;
		data[16] = amostrasACC.amostra2.dadosACC.eixoYmsb;   //amostra 2
		data[17] = amostrasACC.amostra2.dadosACC.eixoZlsb;
		data[18] = amostrasACC.amostra2.dadosACC.eixoZmsb;
		
		data[19] = amostrasACC.amostra3.dadosACC.eixoXlsb; 	
		data[20] = amostrasACC.amostra3.dadosACC.eixoXmsb;
		data[21] = amostrasACC.amostra3.dadosACC.eixoYlsb;
		data[22] = amostrasACC.amostra3.dadosACC.eixoYmsb;   //amostra 3
		data[23] = amostrasACC.amostra3.dadosACC.eixoZlsb;
		data[24] = amostrasACC.amostra3.dadosACC.eixoZmsb;

		data[25] = amostrasACC.amostra4.dadosACC.eixoXlsb; 	
		data[26] = amostrasACC.amostra4.dadosACC.eixoXmsb;
		data[27] = amostrasACC.amostra4.dadosACC.eixoYlsb;
		data[28] = amostrasACC.amostra4.dadosACC.eixoYmsb;   //amostra 4
		data[29] = amostrasACC.amostra4.dadosACC.eixoZlsb;
		data[30] = amostrasACC.amostra4.dadosACC.eixoZmsb;

		data[31] = amostrasACC.amostra5.dadosACC.eixoXlsb; 	
		data[32] = amostrasACC.amostra5.dadosACC.eixoXmsb;
		data[33] = amostrasACC.amostra5.dadosACC.eixoYlsb;
		data[34] = amostrasACC.amostra5.dadosACC.eixoYmsb;   //amostra 5
		data[35] = amostrasACC.amostra5.dadosACC.eixoZlsb;
		data[36] = amostrasACC.amostra5.dadosACC.eixoZmsb;
				
		data[37] = amostrasACC.amostra6.dadosACC.eixoXlsb; 	
		data[38] = amostrasACC.amostra6.dadosACC.eixoXmsb;
		data[39] = amostrasACC.amostra6.dadosACC.eixoYlsb;
		data[40] = amostrasACC.amostra6.dadosACC.eixoYmsb;   //amostra 6
		data[41] = amostrasACC.amostra6.dadosACC.eixoZlsb;
		data[42] = amostrasACC.amostra6.dadosACC.eixoZmsb;				
				
		data[43] = amostrasACC.amostra7.dadosACC.eixoXlsb; 	
		data[44] = amostrasACC.amostra7.dadosACC.eixoXmsb;
		data[45] = amostrasACC.amostra7.dadosACC.eixoYlsb;
		data[46] = amostrasACC.amostra7.dadosACC.eixoYmsb;   //amostra 7
		data[47] = amostrasACC.amostra7.dadosACC.eixoZlsb;
		data[48] = amostrasACC.amostra7.dadosACC.eixoZmsb;				
				
		data[49] = amostrasACC.amostra8.dadosACC.eixoXlsb; 	
		data[50] = amostrasACC.amostra8.dadosACC.eixoXmsb;
		data[51] = amostrasACC.amostra8.dadosACC.eixoYlsb;
		data[52] = amostrasACC.amostra8.dadosACC.eixoYmsb;   //amostra 8
		data[53] = amostrasACC.amostra8.dadosACC.eixoZlsb;
		data[54] = amostrasACC.amostra8.dadosACC.eixoZmsb;				
		
#else		
		data[7]	 = 0x31; 
		data[8]	 = 0xFF;
		data[9]	 = 0xDF;					//1º samples
		data[10] = 0xFF;
		data[11] = 0xB6;
		data[12] = 0x00;

		//			status = FXI_SetMetric( pFxInst, metricId, data, index );

		data[13] = 0x34; 
		data[14] = 0xFF;
		data[15] = 0xDD;				//2º samples
		data[16] = 0xFF;
		data[17] = 0xB6;
		data[18] = 0x00; 

//					status = FXI_SetMetric( pFxInst, metricId, data, index );

		data[19] = 0x2F; 
		data[20] = 0xFF;
		data[21] = 0xDB;				
		data[22] = 0xFF;				//3º samples
		data[23] = 0xB4;
		data[24] = 0x00; 
					
	//				status = FXI_SetMetric( pFxInst, metricId, data, index );
					
		data[25] = 0x03; 
		data[26] = 0xFF;
		data[27] = 0xDD;				//4º samples
		data[28] = 0xFF;
		data[29] = 0xB3;
		data[30] = 0x00;
					
//					status = FXI_SetMetric( pFxInst, metricId, data, index );
					
		data[31] = 0x3C; 
		data[32] = 0xFF;
		data[33] = 0xDC;				//5º samples
		data[34] = 0xFF;
		data[35] = 0xAC;
		data[36] = 0x00;
					
	//				status = FXI_SetMetric( pFxInst, metricId, data, index );
					
		data[37] = 0x3F; 
		data[38] = 0xFF;
		data[39] = 0xDB;				//6º samples
		data[40] = 0xFF;
		data[41] = 0xA4;
		data[42] = 0x00;
					
	//				status = FXI_SetMetric( pFxInst, metricId, data, index );
					
		data[43] = 0x44; 
		data[44] = 0xFF;
		data[45] = 0xDA;				//7º samples
		data[46] = 0xFF;
		data[47] = 0xA2;
		data[48] = 0x00;
					
//					status = FXI_SetMetric( pFxInst, metricId, data, index );
					
		data[49] = 0x48; 
		data[50] = 0xFF;
		data[51] = 0xD8;				//8º samples
		data[52] = 0xFF;
		data[53] = 0xA2;
		data[54] = 0x00;
		
#endif		
					
		status = FXI_SetMetric( pFxInst, metricId, data, index );
				
		metricId = FXI_METRIC_ID_PPG; 
		metricSize = ( FXI_NUMBER_OF_PPG_SAMPLES * 4 ) + 6;
		index=17;
		data[0] = FXI_METRIC_ID_PPG;		
		data[1] = (uint8_t)( metricSize & 0xFF );
		data[2] = (uint8_t)(( metricSize >> 8 ) & 0xFF );
		data[3] = sequence; // index, for feed arbitrary number
		data[4] = 4; // Q(uality), set fixed to 4
		data[5] = bodyPosition; // fill in sensor wrist position
		data[6] = 0x7F; // SF, fixed to 0xFF for 2 PGG samples
		data[7] = ledPower; // 
		data[8] = adcGain; // 
								
		data[9]	 = amostrasPPG.amostra1.dadosPPG.PPGlsb; 				//lsb_ppg
		data[10] = amostrasPPG.amostra1.dadosPPG.PPGmsb;				//msb_ppg
		data[11] = amostrasPPG.amostra1.dadosPPG.ambientelsb;		//ambient_lsb		1º samples
		data[12] = amostrasPPG.amostra1.dadosPPG.ambientemsb;		//ambient_msb
					
		data[13] = amostrasPPG.amostra2.dadosPPG.PPGlsb; 				//lsb_ppg
		data[14] = amostrasPPG.amostra2.dadosPPG.PPGmsb;				//msb_ppg
		data[15] = amostrasPPG.amostra2.dadosPPG.ambientelsb;		//ambient_lsb		2º samples
		data[16] = amostrasPPG.amostra2.dadosPPG.ambientemsb;		//ambient_msb
				
		status = FXI_SetMetric( pFxInst, metricId, data, index );	
	
		/* DO PROCESSING */
		status = FXI_Process( pFxInst );
		/* get updated metrics */
		FXI_METRIC_ID metrics[16]; // size must be at least max number of metrics enabled
		FX_UINT08 numberOfMetrics = sizeof(metrics);
		/* Get updated output meatric values */
		status = FXI_ListUpdatedMetrics( pFxInst, metrics, &numberOfMetrics );	

		for ( int k = 0; k < numberOfMetrics; k++ )
		{
				FX_UINT08 metricData[FXI_MAX_METRIC_SIZE];
				FX_UINT16 metricDataSize = sizeof(metricData);
				status = FXI_GetMetric( pFxInst, metrics[k], metricData, &metricDataSize );

				if(metricData[0]==0x22){
						if(metricData[4]>1)
									if(metricData[5]==0){
											*contact_detected=true;
									}else {
											*contact_detected=false;
									}

				}			
				if(metricData[0]==0x20){
						if(metricData[4]>1)
								*batimentos=metricData[5];
						trigLoop=false;
				}else
							trigLoop=true;
				
				NRF_LOG_INFO("ID: %x  Index: %d Quality: %x Value: %d status: %d",metricData[0], metricData[3], metricData[4], metricData[5],status);	

		}
		
		nrf_gpio_pin_clear(20);
 
		return;
}

void terminate_biblioteca_phillips(void){
		
		status = FXI_Terminate( &pFxInst );	

}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

