/*
 ******************************************************************************
 * @file    MAX30110_API.c
 * @author  Vítor Eduardo Sabadine da Cruz
 * @version 2
 * @date    15-Setembro-2019
 * @brief   Biblioteca C.I. front-end analógico (AFE) MAX30110.
 *
 ******************************************************************************
 */

#include "MAX30110_port.h"
#include "MAX30110_API.h"
#include "string.h"
#include "main.h"
#include "nrf_delay.h"


#define ADC_MAX                         (524287)
#define ADC_USED_FACTOR                 (3)                    /* used range = ADC_MAX >> ADC_USED_FACTOR */

#define PPG_HIGH   (90) /* 90% of used scale: */
#define PPG_LOW    (45) /* 45% of used scale: */

#define LED_MIN    (10) /* 10% */
#define LED_MAX    (60) /* 60% */

bool	trigLoop=true;

typedef struct _max30110_data_t
{
    /* max30110 fifo sample */
    uint32_t ppg;
    
    uint32_t ambient;

    uint8_t  adcGain;
    uint16_t ledPower;

} max30110_data_t;

/**
 *  \brief Structure in which the current settings of AFE are kept
 */
typedef struct _PPG_commonSettings_t
{
    uint8_t ledRange;
    
    /* pulse width */
    uint8_t pulseWidth;
} PPG_commonSettings_t;

static max30110_data_t          sDataCh;


/**
 *  \brief Keep the sensor settings
 */
static PPG_commonSettings_t     sCommonSettings;

/** 
 *  \brief  Set MAX30110 led power.
 *  \param  percentage [IN]  : Normalized power.
 *  \return status
 */
static uint8_t SetLedPower(uint16_t value);

/** 
 *  \brief  Set MAX30110 led adc gain.
 *  \param  gain [IN]  : [0..3] , amplification = 2^gain ( or 1, 2, 4, 8 )
 *  \return status
 */
static uint8_t SetGain( uint8_t gain );

/*
 * Mapeamento dos endereços dos registradores internos do MAX30110.
 */
typedef enum {
	MAX30110_INTERRUPT_STATUS = 0,
	MAX30110_INTERRUPT_STATUS_1 = MAX30110_INTERRUPT_STATUS,
	MAX30110_INTERRUPT_STATUS_2,
	MAX30110_INTERRUPT_ENABLE,
	MAX30110_INTERRUPT_ENABLE_1 = MAX30110_INTERRUPT_ENABLE,
	MAX30110_INTERRUPT_ENABLE_2,
	MAX30110_FIFO_WRITE_POINTER,
	MAX30110_OVERFLOW_COUNTER,
	MAX30110_FIFO_READ_POINTER,
	MAX30110_FIFO_DATA_REGISTER,
	MAX30110_FIFO_CONFIGURATION,
	MAX30110_FIFO_DATA_CONTROL_REGISTERS,
	MAX30110_FIFO_DATA_CONTROL_REGISTER_1 = MAX30110_FIFO_DATA_CONTROL_REGISTERS,
	MAX30110_FIFO_DATA_CONTROL_REGISTER_2,
	MAX30110_SYSTEM_CONTROL = 0x0D,
	MAX30110_PPG_CONFIGURATION,
	MAX30110_PPG_CONFIGURATION_1 = MAX30110_PPG_CONFIGURATION,
	MAX30110_PPG_CONFIGURATION_2,
	MAX30110_PROX_INTERRUPT_THRESHOULD,
	MAX30110_LEDS_PA,
	MAX30110_LED1_PA = MAX30110_LEDS_PA,
	MAX30110_LED2_PA,
	MAX30110_LED_RANGE = 0x14,
	MAX30110_PILOT_PA,
	MAX30110_PART_ID = 0xFF
}MAX30110_mapaRegistrosEnum_t;

#define MAX30110_ASSERT(VALOR);												if(VALOR==0) return MAX30110_FALHOU_ERRO_PARAMETRO;

#define MAX30110_ASSERT_CORRENTE_LED(VALOR)							( VALOR >= 0.0 && VALOR <= 200.0 )
#define MAX30110_ASSERT_ESCALA_ADC(VALOR)								( VALOR >= MAX30110_ESCALA_ADC_6UA && VALOR <= MAX30110_ESCALA_ADC_48UA )
#define MAX30110_ASSERT_TEMPO_INTEGRACAO(VALOR)					( VALOR >= MAX30110_TEMPO_INTEGRACAO_52US && VALOR <= MAX30110_TEMPO_INTEGRACAO_417US )
#define MAX30110_ASSERT_TEMPO_ESTABILIZACAO(VALOR)			( VALOR >= MAX30110_TEMPO_ESTABILIZACAO_2_5MS && VALOR <= MAX30110_TEMPO_ESTABILIZACAO_20MS )
#define MAX30110_ASSERT_FREQ_AMOSTRA(VALOR)							( VALOR >= MAX30110_FREQ_AMOSTRA_20SPS && VALOR <= MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_100SPS )
#define MAX30110_ASSERT_QTD_MEDIA_AMOSTRAS(VALOR)				( VALOR >= MAX30110_QUANTIDADE_MEDIA_1_AMOSTRA && VALOR <= MAX30110_QUANTIDADE_MEDIA_32_AMOSTRAS )
#define MAX30110_ASSERT_SOBRESCREVE_FILA(VALOR)					( VALOR >= MAX30110_SOBRESCREVER_FILA_DESLIGADO && VALOR <= MAX30110_SOBRESCREVER_FILA_LIGADO )
#define MAX30110_ASSERT_FILA_QUASE_CHEIA(VALOR)					( VALOR >= MAX30110_TIPO_FILA_QUASE_CHEIA_REPETE && VALOR <= MAX30110_TIPO_FILA_QUASE_CHEIA_UMA_VEZ )
#define MAX30110_ASSERT_FILA_MODO_LIMPA_ESTADO(VALOR)		( VALOR >= MAX30110_MODO_LIMPA_ESTADO_LER_ESTADO && VALOR <= MAX30110_MODO_LIMPA_ESTADO_E_LER_FILA )
#define MAX30110_ASSERT_FILA_QUANTIDADE_AMOSTRAS(VALOR)	( VALOR >= 17 && VALOR <= 32 )

/*
 * Definições dos registros de habilitar interrupções.
 */
typedef union{
	struct {
		uint8_t unused1:3;														// Lê 0.
		uint8_t led1NotCompliance:1;									// Indica se o driver do LED1 não está em conformidade com a tensão de trabalho (VLED1 < 160 mV).
		uint8_t proximity:1;													// Detectou que o dispositivo está mais próximo que o gatilho programado.
		uint8_t ambientLightCancellationOverflow:1;		// Indica que o dispositivo não consegue cancelar a luz ambiente e as amostras estão comprometidas.
		uint8_t photoPlenthysmoGraphReady:1;					// Indica nova amostra de PPG disponível.
		uint8_t almostFull:1;													// Indica fila quase cheia, faltando um item para lotar.
		uint8_t unused2:7;														// Lê 0.
		uint8_t analogSupplyOutOfRange:1;							// Regulador de tensão analógico fora da faixa de trabalho.
	}data;
	uint8_t vector[2];
}MAX30110_interruptEnableRegistersUnion_t;

#define MAX30110_INTERRUPT_ENABLE_REGISTER(VARIAVEL)		((MAX30110_interruptEnableRegistersUnion_t*) VARIAVEL )

#define	MAX30110_INT_ENABLE_LED_COMPL_OFF				0	// Indica se o driver do LED1 não está em conformidade com a tensão de trabalho (VLED1 < 160 mV).
#define	MAX30110_INT_ENABLE_LED_COMPL_ON				1

#define	MAX30110_INT_ENABLE_PROX_INT_OFF				0	// Detectou que o dispositivo está mais próximo que o gatilho programado.
#define	MAX30110_INT_ENABLE_PROX_INT_ON					1

#define	MAX30110_INT_ENABLE_ALC_OVF_OFF					0	// Indica que o dispositivo não consegue cancelar a luz ambiente e as amostras estão comprometidas.
#define	MAX30110_INT_ENABLE_ALC_OVF_ON					1

#define	MAX30110_INT_ENABLE_PPG_RDY_OFF					0	// Indica nova amostra de PPG disponível.
#define	MAX30110_INT_ENABLE_PPG_RDY_ON					1

#define	MAX30110_INT_ENABLE_A_FULL_OFF					0	// Indica fila quase cheia, faltando um item para lotar.
#define	MAX30110_INT_ENABLE_A_FULL_ON						1

#define	MAX30110_INT_ENABLE_VDD_OOR_OFF					0	// Regulador de tensão analógico fora da faixa de trabalho.
#define	MAX30110_INT_ENABLE_VDD_OOR_ON					1

/*
 * Definições do registro para escrita do ponteiro da fila.
 */
typedef union {
	struct {
		uint8_t pointer:5;
		uint8_t :3;
	}data;
	uint8_t vector[1];
}MAX30110_fifoWritePointerStruct_t;

#define MAX30110_FIFO_WRITE_POINTER_REGISTER(VARIAVEL)			((MAX30110_fifoWritePointerUnion_t*) VARIAVEL )

/*
 * Definições do registro de contador de amostras sobrepostas da fila.
 */
typedef union {
	struct {
		uint8_t counter:5;
		uint8_t :3;
	}data;
	uint8_t vector[1];
}MAX30110_fifoOverflowCounterUnion_t;

#define MAX30110_FIFO_OVERFLOW_COUNTER_REGISTER(VARIAVEL)		((MAX30110_fifoOverflowCounterUnion_t*) VARIAVEL )

/*
 * Definições do registro para leitura do ponteiro da fila.
 */
typedef union {
	struct {
		uint8_t pointer:5;
		uint8_t :3;
	}data;
	uint8_t vector[1];
}MAX30110_fifoReadPointerUnion_t;

#define MAX30110_FIFO_READ_POINTER_REGISTER(VARIAVEL)				((MAX30110_fifoReadPointerUnion_t*) VARIAVEL )

/*
 * Definições do registro para leitura dos dados da fila.
 */
typedef struct {
	uint8_t data;
}MAX30110_fifoDataStruct_t;

#define MAX30110_FIFO_DATA_REGISTER(VARIAVEL)								((MAX30110_fifoDataStruct_t*) VARIAVEL )

/*
 * Definições do registro para configuração da fila.
 */
typedef union {
	struct {
		uint8_t almostFull:4;													// Quantidade de bytes que faltam antes de indicar fila quase cheia.
		uint8_t rollOver:1;														// Sobrescreve automaticamente os dados da fila que não foram lidos.
		uint8_t almostFullType:1;											// Configura o comportamento da flag de fila quase cheia.
		uint8_t statusClear:1;												// Configura método para apagar o estado de fila cheia.
		uint8_t :1;
	} data;
	uint8_t vector[1];
}MAX30110_fifoConfigurationUnion_t;

#define MAX30110_FIFO_CONFIGURATION_REGISTER(VARIAVEL)			((MAX30110_fifoConfigurationUnion_t*) VARIAVEL )

#define	MAX30110_FIFO_ROLL_OVER_OFF							0	// Sobrescreve automaticamente os dados da fila que não foram lidos.
#define	MAX30110_FIFO_ROLL_OVER_ON							1

#define	MAX30110_FIFO_A_FULL_TYPE_RPT						0	// Configura o comportamento da flag de fila quase cheia.
#define	MAX30110_FIFO_A_FULL_TYPE_ONCE					1

#define	MAX30110_FIFO_STATUS_CLEAR_RD_NOCLR			0	// Configura método para apagar o estado de fila cheia.
#define	MAX30110_FIFO_STATUS_CLEAR_RD_CLR				1

/*
 * Definições dos registros para controle dos dados da fila.
 */
typedef union {
	struct {
		uint8_t fifoData1Type:4;
		uint8_t fifoData2Type:4;
		uint8_t fifoData3Type:4;
		uint8_t fifoData4Type:4;
	}data;
	uint8_t vector[2];
}MAX30110_fifoDataControlUnion_t;

#define MAX30110_FIFO_DATA_CONTROL_REGISTER(VARIAVEL) 			((MAX30110_fifoDataControlUnion_t*) VARIAVEL )

/*
 * Definições do registro para controle do sistema interno.
 */
typedef union {
	struct {
		uint8_t reset:1;															// Realiza um soft reset no circuito integrado.
		uint8_t shutDown:1;														// Coloca o chip em modo de ultra-baixo consumo, desativando-o.
		uint8_t fifoEnabled:1;												// Habilita a função de fila de dados.
		uint8_t lowPowerMode:1;												// Habilita função de baixo consumo para taxa de amostras menores que 100 Hz.
		uint8_t frequencyClockPinControl:1;						// Habilita pino externo como entrada de oscilador externo.
		uint8_t :3;
	} data;
	uint8_t vector[1];
}MAX30110_systemControlUnion_t;

#define MAX30110_SYSTEM_CONTROL_REGISTER(VARIAVEL) 					((MAX30110_systemControlUnion_t*) VARIAVEL )

#define	MAX30110_SYS_CONTROL_RESET_ON						1	// Realiza um soft reset no circuito integrado.

#define	MAX30110_SYS_CONTROL_SHUTDOWN_OFF				0	// Coloca o chip em modo de ultra-baixo consumo, desativando-o.
#define	MAX30110_SYS_CONTROL_SHUTDOWN_ON				1

#define	MAX30110_SYS_CONTROL_FIFO_ENABLED_OFF		0	// Habilita a função de fila de dados.
#define	MAX30110_SYS_CONTROL_FIFO_ENABLED_ON		1

#define	MAX30110_SYS_CONTROL_LOW_POWER_OFF			0	// Habilita função de baixo consumo para taxa de amostras menores que 100 Hz.
#define	MAX30110_SYS_CONTROL_LOW_POWER_ON				1

#define	MAX30110_SYS_CONTROL_PIN_CLOCK_IN_OFF		0	// Habilita pino externo como entrada de oscilador externo.
#define	MAX30110_SYS_CONTROL_PIN_CLOCK_IN_ON		1

/*
 * Definições dos registros para configuração do PPG.
 */
typedef union {
	struct {
		uint8_t integrationTime:2;
		uint8_t sampleRate:4;
		uint8_t adcRange:2;
		uint8_t sampleAverage:3;
		uint8_t ledCurrentSettling:2;
		uint8_t :3;
	}data;
	uint8_t vector[2];
}MAX30110_ppgConfigurationUnion_t;

#define MAX30110_PPG_CONFIGURATION_REGISTER(VARIAVEL) 			((MAX30110_ppgConfigurationUnion_t*) VARIAVEL )

#define MAX30110_CFG_INTEGRATION_TIME_52US			0
#define	MAX30110_CFG_INTEGRATION_TIME_104US			1
#define	MAX30110_CFG_INTEGRATION_TIME_206US			2
#define	MAX30110_CFG_INTEGRATION_TIME_417US			3

#define MAX30110_CFG_SAMPLE_RATE_20_SPS					0
#define	MAX30110_CFG_SAMPLE_RATE_25_SPS					1
#define	MAX30110_CFG_SAMPLE_RATE_50_SPS					2
#define	MAX30110_CFG_SAMPLE_RATE_84_SPS					3
#define MAX30110_CFG_SAMPLE_RATE_100_SPS				4
#define	MAX30110_CFG_SAMPLE_RATE_200_SPS				5
#define	MAX30110_CFG_SAMPLE_RATE_400_SPS				6
#define	MAX30110_CFG_SAMPLE_RATE_800_SPS				7
#define MAX30110_CFG_SAMPLE_RATE_1000_SPS				8
#define	MAX30110_CFG_SAMPLE_RATE_1600_SPS				9
#define	MAX30110_CFG_SAMPLE_RATE_3200_SPS				10
#define	MAX30110_CFG_SAMPLE_RATE_DPULSE_20_SPS	11
#define MAX30110_CFG_SAMPLE_RATE_DPULSE_25_SPS	12
#define	MAX30110_CFG_SAMPLE_RATE_DPULSE_50_SPS	13
#define	MAX30110_CFG_SAMPLE_RATE_DPULSE_84_SPS	14
#define	MAX30110_CFG_SAMPLE_RATE_DPULSE_100_SPS	15

#define MAX30110_CFG_ADC_RANGE_6US							0
#define MAX30110_CFG_ADC_RANGE_12US							1
#define MAX30110_CFG_ADC_RANGE_24US							2
#define MAX30110_CFG_ADC_RANGE_48US							3

#define MAX30110_CFG_SAMPLE_AVERAGE_1_SPS				0
#define MAX30110_CFG_SAMPLE_AVERAGE_2_SPS				1
#define MAX30110_CFG_SAMPLE_AVERAGE_4_SPS				2
#define MAX30110_CFG_SAMPLE_AVERAGE_8_SPS				3
#define MAX30110_CFG_SAMPLE_AVERAGE_16_SPS			4
#define MAX30110_CFG_SAMPLE_AVERAGE_32_SPS			5

#define MAX30110_CFG_LED_SETTLING_2_5_MS				0
#define MAX30110_CFG_LED_SETTLING_5_MS					1
#define MAX30110_CFG_LED_SETTLING_10_MS					2
#define MAX30110_CFG_LED_SETTLING_20_MS					3

/*
 * Definições do registro do gatilho de interrupção de proximidade.
 */
typedef struct {
	uint8_t proximityInterruptThreshold;
}MAX30110_proximityIntThresholdStruct_t;

#define MAX30110_PROXIMITY_INT_THRESHOLD_REGISTER(VARIAVEL) ((MAX30110_proximityIntThresholdStruct_t*) VARIAVEL )

/*
 * Definições dos registros da amplitude de pico dos LEDs.
 */
typedef union {
	struct {
		uint8_t led1PeakAmplitude;
		uint8_t led2PeakAmplitude;
	}data;
	uint8_t vector[2];
}MAX30110_ledPeakAmplitudeUnion_t;

#define MAX30110_LED_PEAK_AMPLITUDE_REGISTER(VARIAVEL) 			((MAX30110_ledPeakAmplitudeUnion_t*) VARIAVEL )

#define MAX30110_CONVERT_CURRENT_TO_LED_PEAK_50MA(VARIAVEL)			(uint8_t)(VARIAVEL/0.196)
#define MAX30110_CONVERT_CURRENT_TO_LED_PEAK_100MA(VARIAVEL)		(uint8_t)(VARIAVEL/0.392)
#define MAX30110_CONVERT_CURRENT_TO_LED_PEAK_150MA(VARIAVEL)		(uint8_t)(VARIAVEL/0.588)
#define MAX30110_CONVERT_CURRENT_TO_LED_PEAK_200MA(VARIAVEL)		(uint8_t)(VARIAVEL/0.784)

/*
 * Definições do registro da seção da corrente dos LEDs.
 */
typedef union {
	struct {
		uint8_t led1Range:2;													// Seleciona ganho de corrente do LED 1.
		uint8_t led2Range:2;													// Seleciona ganho de corrente do LED 2.
		uint8_t :4;
	} data;
	uint8_t vector[1];
}MAX30110_ledRangeUnion_t;

#define MAX30110_LED_RANGE_REGISTER(VARIAVEL) 							((MAX30110_ledRangeUnion_t*) VARIAVEL )

#define MAX30110_LED_RANGE_50_MA									0 // Seleciona ganho de corrente dos LEDs.
#define MAX30110_LED_RANGE_100_MA									1
#define MAX30110_LED_RANGE_150_MA									2
#define MAX30110_LED_RANGE_200_MA									3

/*
 * Definições dos registros da seção da corrente do LED no modo de detecção de proximidade.
 */
typedef struct {
	uint8_t pilotPeakAmplitude;
}MAX30110_pilotPeakAmplitudeStruct_t;

#define MAX30110_PILOT_PEAK_AMPLITUDE_REGISTER(VARIAVEL) 		((MAX30110_pilotPeakAmplitudeStruct_t*) VARIAVEL )

typedef struct {
	MAX30110_interruptEnableRegistersUnion_t interruptEnableRegisters;
	MAX30110_fifoConfigurationUnion_t fifoConfiguration;
	MAX30110_fifoDataControlUnion_t fifoDataControl;
	MAX30110_systemControlUnion_t systemControl;
	MAX30110_ppgConfigurationUnion_t ppgConfiguration;
	MAX30110_ledPeakAmplitudeUnion_t ledPeakAmplitude;
	MAX30110_ledRangeUnion_t ledRange;
}MAX30110_configurationRegisters_t;

#define	MAX30110_PART_ID_VALOR					0x20

/*
 * Definição dos possíveis comandos do acelerômetro
 */
typedef enum {
	MAX30110_COMANDO_ESCREVE_REGISTRO = 0x00,
	MAX30110_COMANDO_LE_REGISTRO = 0x80,
}MAX30110_listaComandos;

/*
 * Estrutura básica do vetor de dados a ser transmitido.
 */
typedef struct {
	uint8_t enderecoRegistro;
	uint8_t byteComando;
	uint8_t dadoAdicional;
}MAX30110_leEscreveRegistroStruct_t;

/******************************************************************************/
/*
 * Rotinas para transição de dados via SPI.
 */
/******************************************************************************/

/*
 * @brief Organiza o vetor de transmissão de dados para englobar os bytes de controle e endereço do registro no mesmo vetor.
 * @param enderecoRegistro - Endereço do registro a ser lido pela SPI.
 * @param dadoAdicional - Byte com o dado a serem escrito a partir do endereço de registro fornecido.
 * @retval uint8_t - Quantidade de dados a ser transmitidos.
 */
uint8_t MAX30110_preparaVetorEscreveRegistro ( MAX30110_mapaRegistrosEnum_t enderecoRegistro, uint8_t dadoAdicional ) {
	MAX30110_leEscreveRegistroStruct_t* ponteiroVetorDados = (MAX30110_leEscreveRegistroStruct_t*) MAX30110_VETOR_DADOS;

	ponteiroVetorDados->byteComando = MAX30110_COMANDO_ESCREVE_REGISTRO;
	ponteiroVetorDados->enderecoRegistro = enderecoRegistro;
	ponteiroVetorDados->dadoAdicional = dadoAdicional;

	return 3;
}

/*
 * @brief Prepara o vetor para transmitir o endereço do registro a ser lido.
 * @param enderecoRegistro - Endereço do registro a ser lido pela SPI.
 * @retval uint8_t - Quantidade de dados a ser transmitidos.
 */
uint8_t MAX30110_preparaVetorLeRegistro ( MAX30110_mapaRegistrosEnum_t enderecoRegistro ) {
	MAX30110_leEscreveRegistroStruct_t* ponteiroVetorDados = (MAX30110_leEscreveRegistroStruct_t*) MAX30110_VETOR_DADOS;

	ponteiroVetorDados->byteComando = MAX30110_COMANDO_LE_REGISTRO;

	ponteiroVetorDados->enderecoRegistro = enderecoRegistro;

	return 2;
}

/*
 * @brief Espera a transição dos dados da SPI encerrar.
 * @param nenhum.
 * @retval MAX30110_returnEnum_t - MAX30110_FALHOU: Erro durante o procedimento.
 * 																MAX30110_SUCESSO: Operação realizada com sucesso.
 */
/*MAX30110_retorno_t MAX30110_AguardaTerminoEnvio( void ) {

	MAX30110_spiIniciaTempoParaEsgotar();
	while( !MAX30110_spiTerminouTransicao() ){
		if( MAX30110_spiEstaComErros() ) {
			MAX30110_spiDeselecionaChip();

			return MAX30110_FALHOU_ERRO_SPI;
		}
	}

	return MAX30110_SUCESSO;
}*/

/*
 * @brief Executa a transmissão dos dados para escrita dos registradores.
 * @param enderecoRegistro - Endereço do registro a ser lido pela SPI.
 * @param vetor - ponteiro do vetor onde os dados vão ser descarregados.
 * @param quantidade - Quantidade total de bytes a ser transmitidos pela SPI.
 * @retval MAX30110_returnEnum_t - MAX30110_FALHOU: Erro durante o procedimento.
 * 																MAX30110_SUCESSO: Operação realizada com sucesso.
 */
MAX30110_retorno_t MAX30110_escreveRegistroAguardaTermino( MAX30110_mapaRegistrosEnum_t enderecoRegistro, uint8_t* vetor, uint8_t quantidade ) {
	while(quantidade--) {
	//	if( !MAX30110_spiEstaPreparada() )
	//		return MAX30110_FALHOU_SPI_OCUPADA;

		MAX30110_spiSelecionaChip();

		MAX30110_escritaSPI(enderecoRegistro, vetor, quantidade);
		
		enderecoRegistro++;
		vetor++;

		MAX30110_spiDeselecionaChip();

		if( quantidade == 0 )
			return MAX30110_SUCESSO;
	}

	MAX30110_spiDeselecionaChip();

	return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
}

/*
 * @brief Executa a leitura dos dados da fila.
 * @param enderecoRegistro - Endereço do registro a ser lido pela SPI.
 * @param vetor - ponteiro do vetor onde os dados vão ser descarregados.
 * @param quantidade - Quantidade total de bytes a ser transmitidos pela SPI.
 * @retval MAX30110_returnEnum_t - MAX30110_FALHOU: Erro durante o procedimento.
 * 																MAX30110_SUCESSO: Operação realizada com sucesso.
 */
MAX30110_retorno_t MAX30110_leFila( MAX30110_dados24BitsFIFO_t* dados, uint16_t quantidade ) {
	uint8_t temp;
	
	do {
		/*if( !MAX30110_spiEstaPreparada() )
				return MAX30110_FALHOU_SPI_OCUPADA;*/
		
		MAX30110_spiSelecionaChip();
		
		
		MAX30110_leituraFilaSPI(MAX30110_FIFO_DATA_REGISTER, dados->vetor, quantidade*3);
					
		temp=dados->vetor[0];
		dados->vetor[0]=dados->vetor[2];
		dados->vetor[2]=temp;
			
		temp=dados->vetor[5];
		dados->vetor[5]=dados->vetor[4];
		dados->vetor[6]=dados->vetor[3];
		dados->vetor[4]=temp;
		dados->vetor[3]=0x00;
				
		MAX30110_spiDeselecionaChip();
		
//		ControlLoop((uint32_t)dados->dados.valorPPG, &sControlLoopParams.ledPower, &sControlLoopParams.adcGain);
		
//		if( sControlLoopParams.ledPower != sDataCh.ledPower )
//    {   /* take over the new calculated led power figure */
//        (void)SetLedPower( sControlLoopParams.ledPower );
//    }
            
    /* gain changed detection: outside controlLoopEnabled check, therefore can be set externally while Control loop is disabled */
//    if (sControlLoopParams.adcGain != sDataCh.adcGain)
//    {   /* take over the new calculated gain figure */
//        (void)SetGain( sControlLoopParams.adcGain );
//		}
				
		return MAX30110_SUCESSO;
	}while(0);
	
	MAX30110_spiDeselecionaChip();
	
	return MAX30110_FALHOU_ERRO_LEITURA_SPI;
}

/*
 * @brief Executa a leitura dos dados de registradores.
 * @param enderecoRegistro - Endereço do registro a ser lido pela SPI.
 * @param vetor - ponteiro do vetor onde os dados vão ser descarregados.
 * @param quantidade - Quantidade total de bytes a ser transmitidos pela SPI.
 * @retval MAX30110_returnEnum_t - MAX30110_FALHOU: Erro durante o procedimento.
 * 																MAX30110_SUCESSO: Operação realizada com sucesso.
 */
MAX30110_retorno_t MAX30110_leRegistroAguardaTermino( MAX30110_mapaRegistrosEnum_t enderecoRegistro, uint8_t* vetor, uint16_t quantidade ) {
	
	while(quantidade--) {
		//if( !MAX30110_spiEstaPreparada() )
			//	return MAX30110_FALHOU_SPI_OCUPADA;

		MAX30110_spiSelecionaChip();
		
		MAX30110_leituraSPI(enderecoRegistro, vetor, 1);
		
		enderecoRegistro++;
		vetor++;

		MAX30110_spiDeselecionaChip();

		if( quantidade == 0 )
			return MAX30110_SUCESSO;
	}

	MAX30110_spiDeselecionaChip();

	return MAX30110_FALHOU_ERRO_LEITURA_SPI;
}

MAX30110_retorno_t MAX30110_verificaConexaoSpi  ( void ) {
	uint8_t partID = 0;

	if( MAX30110_leRegistroAguardaTermino( MAX30110_PART_ID, &partID, 1 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_LEITURA_SPI;

	if( partID != MAX30110_PART_ID_VALOR ){
		NRF_LOG_INFO("Identidade MAX30110: %X",partID);
		return MAX30110_FALHOU_ERRO_LEITURA_SPI;}
	
	NRF_LOG_INFO("Identidade MAX30110: %X",partID);
	return MAX30110_SUCESSO;
}

MAX30110_retorno_t MAX30110_configura ( MAX30110_estruturaConfiguracao_t* configuracao ) {
	
  sCommonSettings.pulseWidth  = 1;
  sCommonSettings.ledRange    = 0;
	
	MAX30110_configurationRegisters_t configurationRegisters;
	uint8_t tentativas;

	MAX30110_ASSERT( MAX30110_ASSERT_CORRENTE_LED( configuracao->correnteLed1 ) );
	MAX30110_ASSERT( MAX30110_ASSERT_CORRENTE_LED( configuracao->correnteLed2 ) );
	MAX30110_ASSERT( MAX30110_ASSERT_ESCALA_ADC( configuracao->escalaAdc ) );
	MAX30110_ASSERT( MAX30110_ASSERT_TEMPO_INTEGRACAO( configuracao->tempoIntegracao ) );
	MAX30110_ASSERT( MAX30110_ASSERT_TEMPO_ESTABILIZACAO( configuracao->tempoEstabilizacaoLed ) );
	MAX30110_ASSERT( MAX30110_ASSERT_FREQ_AMOSTRA( configuracao->frequenciaAmostras ));
	MAX30110_ASSERT( MAX30110_ASSERT_QTD_MEDIA_AMOSTRAS( configuracao->quantidadeMediaAmostra ) );

	if( MAX30110_verificaConexaoSpi() != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_LEITURA_SPI;
	
	memset(configurationRegisters.systemControl.vector, 0, sizeof(MAX30110_systemControlUnion_t));

	configurationRegisters.systemControl.data.reset = MAX30110_SYS_CONTROL_RESET_ON;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_SYSTEM_CONTROL, configurationRegisters.systemControl.vector, sizeof(MAX30110_systemControlUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	
	DELAYMS(10);
		
	tentativas = 10;

	do {
		MAX30110_interruptStatusRegistersUnion_t estado;

		if( MAX30110_lerEstado( &estado ) != MAX30110_SUCESSO )
			return MAX30110_FALHOU_ERRO_LEITURA_SPI;
		
		if( estado.bits.powerReady == 0 )
			break;
		
		if( --tentativas == 0 )
			return MAX30110_FALHOU_INICIALIZACAO_CHIP;

	}while( tentativas > 0 );

/*	memset(configurationRegisters.systemControl.vector, 0, sizeof(MAX30110_systemControlUnion_t));

	configurationRegisters.systemControl.data.reset = MAX30110_SYS_CONTROL_RESET_ON;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_SYSTEM_CONTROL, configurationRegisters.systemControl.vector, sizeof(MAX30110_systemControlUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	
	DELAYMS(10);
		
*/	
	memset(configurationRegisters.ppgConfiguration.vector, 0, sizeof(MAX30110_ppgConfigurationUnion_t));

	if( configuracao->escalaAdc == MAX30110_ESCALA_ADC_6UA ) configurationRegisters.ppgConfiguration.data.adcRange = MAX30110_CFG_ADC_RANGE_6US;
	else if( configuracao->escalaAdc == MAX30110_ESCALA_ADC_12UA ) configurationRegisters.ppgConfiguration.data.adcRange = MAX30110_CFG_ADC_RANGE_12US;
	else if( configuracao->escalaAdc == MAX30110_ESCALA_ADC_24UA ) configurationRegisters.ppgConfiguration.data.adcRange = MAX30110_CFG_ADC_RANGE_24US;
	else if( configuracao->escalaAdc == MAX30110_ESCALA_ADC_48UA ) configurationRegisters.ppgConfiguration.data.adcRange = MAX30110_CFG_ADC_RANGE_48US;

	if( configuracao->tempoIntegracao == MAX30110_TEMPO_INTEGRACAO_52US ) configurationRegisters.ppgConfiguration.data.integrationTime = MAX30110_CFG_INTEGRATION_TIME_52US;
	else if( configuracao->tempoIntegracao == MAX30110_TEMPO_INTEGRACAO_104US ) configurationRegisters.ppgConfiguration.data.integrationTime = MAX30110_CFG_INTEGRATION_TIME_104US;
	else if( configuracao->tempoIntegracao == MAX30110_TEMPO_INTEGRACAO_206US ) configurationRegisters.ppgConfiguration.data.integrationTime = MAX30110_CFG_INTEGRATION_TIME_206US;
	else if( configuracao->tempoIntegracao == MAX30110_TEMPO_INTEGRACAO_417US ) configurationRegisters.ppgConfiguration.data.integrationTime = MAX30110_CFG_INTEGRATION_TIME_417US;

	if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_20SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_20_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_25SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_25_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_50SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_50_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_84SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_84_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_100SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_100_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_200SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_200_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_400SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_400_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_800SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_800_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_1000SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_1600_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_1600SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_3200_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_3200SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_20_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_20SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_DPULSE_20_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_25SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_DPULSE_25_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_50SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_DPULSE_50_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_PULSO_DUPLA_84SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_DPULSE_84_SPS;
	else if( configuracao->frequenciaAmostras == MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_100SPS ) configurationRegisters.ppgConfiguration.data.sampleRate = MAX30110_CFG_SAMPLE_RATE_DPULSE_100_SPS;

	if( configuracao->tempoEstabilizacaoLed == MAX30110_TEMPO_ESTABILIZACAO_2_5MS ) configurationRegisters.ppgConfiguration.data.ledCurrentSettling = MAX30110_CFG_LED_SETTLING_2_5_MS;
	else if( configuracao->tempoEstabilizacaoLed == MAX30110_TEMPO_ESTABILIZACAO_5MS ) configurationRegisters.ppgConfiguration.data.ledCurrentSettling = MAX30110_CFG_LED_SETTLING_5_MS;
	else if( configuracao->tempoEstabilizacaoLed == MAX30110_TEMPO_ESTABILIZACAO_10MS ) configurationRegisters.ppgConfiguration.data.ledCurrentSettling = MAX30110_CFG_LED_SETTLING_10_MS;
	else if( configuracao->tempoEstabilizacaoLed == MAX30110_TEMPO_ESTABILIZACAO_20MS ) configurationRegisters.ppgConfiguration.data.ledCurrentSettling = MAX30110_CFG_LED_SETTLING_20_MS;

	if( configuracao->quantidadeMediaAmostra == MAX30110_QUANTIDADE_MEDIA_1_AMOSTRA ) configurationRegisters.ppgConfiguration.data.sampleAverage = MAX30110_CFG_SAMPLE_AVERAGE_1_SPS;
	else if( configuracao->quantidadeMediaAmostra == MAX30110_QUANTIDADE_MEDIA_2_AMOSTRAS ) configurationRegisters.ppgConfiguration.data.sampleAverage = MAX30110_CFG_SAMPLE_AVERAGE_2_SPS;
	else if( configuracao->quantidadeMediaAmostra == MAX30110_QUANTIDADE_MEDIA_4_AMOSTRAS ) configurationRegisters.ppgConfiguration.data.sampleAverage = MAX30110_CFG_SAMPLE_AVERAGE_4_SPS;
	else if( configuracao->quantidadeMediaAmostra == MAX30110_QUANTIDADE_MEDIA_8_AMOSTRAS ) configurationRegisters.ppgConfiguration.data.sampleAverage = MAX30110_CFG_SAMPLE_AVERAGE_8_SPS;
	else if( configuracao->quantidadeMediaAmostra == MAX30110_QUANTIDADE_MEDIA_16_AMOSTRAS ) configurationRegisters.ppgConfiguration.data.sampleAverage = MAX30110_CFG_SAMPLE_AVERAGE_16_SPS;
	else if( configuracao->quantidadeMediaAmostra == MAX30110_QUANTIDADE_MEDIA_32_AMOSTRAS ) configurationRegisters.ppgConfiguration.data.sampleAverage = MAX30110_CFG_SAMPLE_AVERAGE_32_SPS;

	configurationRegisters.ppgConfiguration.data.sampleAverage = 0;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_PPG_CONFIGURATION, configurationRegisters.ppgConfiguration.vector, sizeof(MAX30110_ppgConfigurationUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(configurationRegisters.ledRange.vector, 0, sizeof(MAX30110_ledRangeUnion_t));

	if( configuracao->correnteLed1 > 150.0 ) configurationRegisters.ledRange.data.led1Range = MAX30110_LED_RANGE_200_MA;
	else if( configuracao->correnteLed1 > 100.0 ) configurationRegisters.ledRange.data.led1Range = MAX30110_LED_RANGE_150_MA;
	else if( configuracao->correnteLed1 > 50.0 ) configurationRegisters.ledRange.data.led1Range = MAX30110_LED_RANGE_100_MA;
	else configurationRegisters.ledRange.data.led1Range = MAX30110_LED_RANGE_50_MA;

	if( configuracao->correnteLed2 > 150.0 ) configurationRegisters.ledRange.data.led2Range = MAX30110_LED_RANGE_200_MA;
	else if( configuracao->correnteLed2 > 100.0 ) configurationRegisters.ledRange.data.led2Range = MAX30110_LED_RANGE_150_MA;
	else if( configuracao->correnteLed2 > 50.0 ) configurationRegisters.ledRange.data.led2Range = MAX30110_LED_RANGE_100_MA;
	else configurationRegisters.ledRange.data.led2Range = MAX30110_LED_RANGE_50_MA;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_LED_RANGE, configurationRegisters.ledRange.vector, sizeof(MAX30110_ledRangeUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(configurationRegisters.ledPeakAmplitude.vector, 0, sizeof(MAX30110_ledPeakAmplitudeUnion_t));

	
	
	if( configuracao->correnteLed1 > 150.0 ) configurationRegisters.ledPeakAmplitude.data.led1PeakAmplitude = MAX30110_CONVERT_CURRENT_TO_LED_PEAK_200MA(configuracao->correnteLed1);
	else if( configuracao->correnteLed1 > 100.0 ) configurationRegisters.ledPeakAmplitude.data.led1PeakAmplitude = MAX30110_CONVERT_CURRENT_TO_LED_PEAK_150MA(configuracao->correnteLed1);
	else if( configuracao->correnteLed1 > 50.0 ) configurationRegisters.ledPeakAmplitude.data.led1PeakAmplitude = MAX30110_CONVERT_CURRENT_TO_LED_PEAK_100MA(configuracao->correnteLed1);
	else configurationRegisters.ledPeakAmplitude.data.led1PeakAmplitude = MAX30110_CONVERT_CURRENT_TO_LED_PEAK_50MA(configuracao->correnteLed1);

	if( configuracao->correnteLed2 > 150.0 ) configurationRegisters.ledPeakAmplitude.data.led2PeakAmplitude = MAX30110_CONVERT_CURRENT_TO_LED_PEAK_200MA(configuracao->correnteLed2);
	else if( configuracao->correnteLed2 > 100.0 ) configurationRegisters.ledPeakAmplitude.data.led2PeakAmplitude = MAX30110_CONVERT_CURRENT_TO_LED_PEAK_150MA(configuracao->correnteLed2);
	else if( configuracao->correnteLed2 > 50.0 ) configurationRegisters.ledPeakAmplitude.data.led2PeakAmplitude = MAX30110_CONVERT_CURRENT_TO_LED_PEAK_100MA(configuracao->correnteLed2);
	else configurationRegisters.ledPeakAmplitude.data.led2PeakAmplitude = MAX30110_CONVERT_CURRENT_TO_LED_PEAK_50MA(configuracao->correnteLed2);

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_LEDS_PA, configurationRegisters.ledPeakAmplitude.vector, sizeof(MAX30110_ledPeakAmplitudeUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
		
	
	
	
	memset(configurationRegisters.fifoDataControl.vector, 0, sizeof(MAX30110_fifoDataControlUnion_t));

	if( configuracao->tipoDadosFila.FD1 == MAX30110_NONE ) configurationRegisters.fifoDataControl.data.fifoData1Type = 0x00;
	else if( configuracao->tipoDadosFila.FD1 == MAX30110_LED1 ) configurationRegisters.fifoDataControl.data.fifoData1Type = 0x01;
	else if( configuracao->tipoDadosFila.FD1 == MAX30110_LED2 ) configurationRegisters.fifoDataControl.data.fifoData1Type = 0x02;
	else if( configuracao->tipoDadosFila.FD1 == MAX30110_PILOT_LED1 ) configurationRegisters.fifoDataControl.data.fifoData1Type = 0x05;
	else if( configuracao->tipoDadosFila.FD1 == MAX30110_DIRECT_AMBIENT ) configurationRegisters.fifoDataControl.data.fifoData1Type = 0x0C;
	else if( configuracao->tipoDadosFila.FD1 == MAX30110_LED1_AND_LED2 ) configurationRegisters.fifoDataControl.data.fifoData1Type = 0x0D;
	else configurationRegisters.fifoDataControl.data.fifoData1Type = 0x00;
	
	if( configuracao->tipoDadosFila.FD2 == MAX30110_NONE ) configurationRegisters.fifoDataControl.data.fifoData2Type = 0x00;
	else if( configuracao->tipoDadosFila.FD2 == MAX30110_LED1 ) configurationRegisters.fifoDataControl.data.fifoData2Type = 0x01;
	else if( configuracao->tipoDadosFila.FD2 == MAX30110_LED2 ) configurationRegisters.fifoDataControl.data.fifoData2Type = 0x02;
	else if( configuracao->tipoDadosFila.FD2 == MAX30110_PILOT_LED1 ) configurationRegisters.fifoDataControl.data.fifoData2Type = 0x05;
	else if( configuracao->tipoDadosFila.FD2 == MAX30110_DIRECT_AMBIENT ) configurationRegisters.fifoDataControl.data.fifoData2Type = 0x0C;
	else if( configuracao->tipoDadosFila.FD2 == MAX30110_LED1_AND_LED2 ) configurationRegisters.fifoDataControl.data.fifoData2Type = 0x0D;
	else configurationRegisters.fifoDataControl.data.fifoData2Type = 0x00;
	
	if( configuracao->tipoDadosFila.FD3 == MAX30110_NONE ) configurationRegisters.fifoDataControl.data.fifoData3Type = 0x00;
	else if( configuracao->tipoDadosFila.FD3 == MAX30110_LED1 ) configurationRegisters.fifoDataControl.data.fifoData3Type = 0x01;
	else if( configuracao->tipoDadosFila.FD3 == MAX30110_LED2 ) configurationRegisters.fifoDataControl.data.fifoData3Type = 0x02;
	else if( configuracao->tipoDadosFila.FD3 == MAX30110_PILOT_LED1 ) configurationRegisters.fifoDataControl.data.fifoData3Type = 0x05;
	else if( configuracao->tipoDadosFila.FD3 == MAX30110_DIRECT_AMBIENT ) configurationRegisters.fifoDataControl.data.fifoData3Type = 0x0C;
	else if( configuracao->tipoDadosFila.FD3 == MAX30110_LED1_AND_LED2 ) configurationRegisters.fifoDataControl.data.fifoData3Type = 0x0D;
	else configurationRegisters.fifoDataControl.data.fifoData3Type = 0x00;
	
	if( configuracao->tipoDadosFila.FD4 == MAX30110_NONE ) configurationRegisters.fifoDataControl.data.fifoData4Type = 0x00;
	else if( configuracao->tipoDadosFila.FD4 == MAX30110_LED1 ) configurationRegisters.fifoDataControl.data.fifoData4Type = 0x01;
	else if( configuracao->tipoDadosFila.FD4 == MAX30110_LED2 ) configurationRegisters.fifoDataControl.data.fifoData4Type = 0x02;
	else if( configuracao->tipoDadosFila.FD4 == MAX30110_PILOT_LED1 ) configurationRegisters.fifoDataControl.data.fifoData4Type = 0x05;
	else if( configuracao->tipoDadosFila.FD4 == MAX30110_DIRECT_AMBIENT ) configurationRegisters.fifoDataControl.data.fifoData4Type = 0x0C;
	else if( configuracao->tipoDadosFila.FD4 == MAX30110_LED1_AND_LED2 ) configurationRegisters.fifoDataControl.data.fifoData4Type = 0x0D;
	else configurationRegisters.fifoDataControl.data.fifoData4Type = 0x00;	
	
	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_FIFO_DATA_CONTROL_REGISTERS, configurationRegisters.fifoDataControl.vector, sizeof(MAX30110_fifoDataControlUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
	
	
//	uint8_t vetor[2]={0x7F, 0x00};
//	if( MAX30110_escreveRegistroAguardaTermino( 0x08, vetor, 1 ) != MAX30110_SUCESSO )
//		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
	
	
	
//		vetor[0]=0x03;
//		vetor[1]=0x03;
//		if( MAX30110_escreveRegistroAguardaTermino( MAX30110_LEDS_PA, vetor, 2)  != MAX30110_SUCESSO )
//			return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
	
//		vetor[0]=0x0F;
		
//		if( MAX30110_escreveRegistroAguardaTermino( MAX30110_LED_RANGE, vetor, 1)  != MAX30110_SUCESSO )
//			return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
	


	memset(configurationRegisters.systemControl.vector, 0, sizeof(MAX30110_systemControlUnion_t));

	configurationRegisters.systemControl.data.frequencyClockPinControl = configuracao->configuracoesSistema.pinoOsciladorExterno;
	configurationRegisters.systemControl.data.lowPowerMode = configuracao->configuracoesSistema.modoBaixoConsumo;
	configurationRegisters.systemControl.data.shutDown = configuracao->configuracoesSistema.desligar;
	configurationRegisters.systemControl.data.fifoEnabled = configuracao->configuracoesSistema.habilitarFila;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_SYSTEM_CONTROL, configurationRegisters.systemControl.vector, sizeof(MAX30110_systemControlUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
	
	
	if( configuracao->configuracoesSistema.habilitarFila ) {
		MAX30110_ASSERT( MAX30110_ASSERT_SOBRESCREVE_FILA( configuracao->fila.sobrescreverFila ) );
		MAX30110_ASSERT( MAX30110_ASSERT_FILA_QUASE_CHEIA( configuracao->fila.tipoFilaQuaseCheia ) );
		MAX30110_ASSERT( MAX30110_ASSERT_FILA_MODO_LIMPA_ESTADO( configuracao->fila.modoLimpaEstado ) );
		MAX30110_ASSERT( MAX30110_ASSERT_FILA_QUANTIDADE_AMOSTRAS( configuracao->fila.quantidadeAmostras ) );

		if( configuracao->fila.modoLimpaEstado == MAX30110_MODO_LIMPA_ESTADO_LER_ESTADO ) configurationRegisters.fifoConfiguration.data.statusClear = MAX30110_FIFO_STATUS_CLEAR_RD_NOCLR;
		else if( configuracao->fila.modoLimpaEstado == MAX30110_MODO_LIMPA_ESTADO_E_LER_FILA ) configurationRegisters.fifoConfiguration.data.statusClear = MAX30110_FIFO_STATUS_CLEAR_RD_CLR;

		if( configuracao->fila.sobrescreverFila == MAX30110_SOBRESCREVER_FILA_DESLIGADO ) configurationRegisters.fifoConfiguration.data.rollOver = MAX30110_FIFO_ROLL_OVER_OFF;
		else if( configuracao->fila.sobrescreverFila == MAX30110_SOBRESCREVER_FILA_LIGADO ) configurationRegisters.fifoConfiguration.data.rollOver = MAX30110_FIFO_ROLL_OVER_ON;

		if( configuracao->fila.tipoFilaQuaseCheia == MAX30110_TIPO_FILA_QUASE_CHEIA_REPETE ) configurationRegisters.fifoConfiguration.data.almostFullType = MAX30110_FIFO_A_FULL_TYPE_RPT;
		else if( configuracao->fila.tipoFilaQuaseCheia == MAX30110_TIPO_FILA_QUASE_CHEIA_UMA_VEZ ) configurationRegisters.fifoConfiguration.data.almostFullType = MAX30110_FIFO_A_FULL_TYPE_ONCE;

		configurationRegisters.fifoConfiguration.data.almostFull = 32 - configuracao->fila.quantidadeAmostras;
		
		if( MAX30110_escreveRegistroAguardaTermino( MAX30110_FIFO_CONFIGURATION, configurationRegisters.fifoConfiguration.vector, sizeof(MAX30110_fifoConfigurationUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
		
	}
	
	memset(configurationRegisters.interruptEnableRegisters.vector, 0, sizeof(MAX30110_interruptEnableRegistersUnion_t));
	
	if( configuracao->ativaInterrupcoes.ledForaConformidade == 1 ) configurationRegisters.interruptEnableRegisters.data.led1NotCompliance = 1;
	else if( configuracao->ativaInterrupcoes.ledForaConformidade == 0 ) configurationRegisters.interruptEnableRegisters.data.led1NotCompliance = 0;

	if( configuracao->ativaInterrupcoes.interrrupcaoProxiidade == 1 ) configurationRegisters.interruptEnableRegisters.data.proximity = 1;
	else if( configuracao->ativaInterrupcoes.interrrupcaoProxiidade == 0 ) configurationRegisters.interruptEnableRegisters.data.proximity = 0;

	if( configuracao->ativaInterrupcoes.cancelamentoLuzAmbienteTransbordou == 1 ) configurationRegisters.interruptEnableRegisters.data.ambientLightCancellationOverflow = 1;
	else if( configuracao->ativaInterrupcoes.cancelamentoLuzAmbienteTransbordou == 0 ) configurationRegisters.interruptEnableRegisters.data.ambientLightCancellationOverflow = 0;
	
		if( configuracao->ativaInterrupcoes.amostraPpgPronta == 1 ) configurationRegisters.interruptEnableRegisters.data.photoPlenthysmoGraphReady = 1;
	else if( configuracao->ativaInterrupcoes.amostraPpgPronta == 0 ) configurationRegisters.interruptEnableRegisters.data.photoPlenthysmoGraphReady = 0;

	if( configuracao->ativaInterrupcoes.filaQuaseCheia == 1 ) configurationRegisters.interruptEnableRegisters.data.almostFull = 1;
	else if( configuracao->ativaInterrupcoes.filaQuaseCheia == 0 ) configurationRegisters.interruptEnableRegisters.data.almostFull = 0;

	if( configuracao->ativaInterrupcoes.vddAnalogicoOk == 1 ) configurationRegisters.interruptEnableRegisters.data.analogSupplyOutOfRange = 1;
	else if( configuracao->ativaInterrupcoes.vddAnalogicoOk == 0 ) configurationRegisters.interruptEnableRegisters.data.analogSupplyOutOfRange = 0;
		
	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_INTERRUPT_ENABLE, configurationRegisters.interruptEnableRegisters.vector, sizeof(MAX30110_interruptEnableRegistersUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
	
	
	return MAX30110_SUCESSO;

}

MAX30110_retorno_t MAX30110_testeProximidade ( void ) {
	uint8_t vetor[2];

	memset(vetor, 0, sizeof(vetor));

	MAX30110_SYSTEM_CONTROL_REGISTER(vetor)->data.reset = 1;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_SYSTEM_CONTROL, vetor, sizeof(MAX30110_systemControlUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	DELAYMS(10);

	memset(vetor, 0, sizeof(vetor));

	MAX30110_SYSTEM_CONTROL_REGISTER(vetor)->data.frequencyClockPinControl = 1;
	MAX30110_SYSTEM_CONTROL_REGISTER(vetor)->data.lowPowerMode = 1;
	MAX30110_SYSTEM_CONTROL_REGISTER(vetor)->data.shutDown = 1;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_SYSTEM_CONTROL, vetor, sizeof(MAX30110_systemControlUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_PPG_CONFIGURATION_REGISTER(vetor)->data.adcRange = 0;
	MAX30110_PPG_CONFIGURATION_REGISTER(vetor)->data.sampleRate = 1;
	MAX30110_PPG_CONFIGURATION_REGISTER(vetor)->data.integrationTime = 1;
	MAX30110_PPG_CONFIGURATION_REGISTER(vetor)->data.ledCurrentSettling = 3;
	MAX30110_PPG_CONFIGURATION_REGISTER(vetor)->data.sampleAverage = 0;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_PPG_CONFIGURATION, vetor, 2 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_LED_PEAK_AMPLITUDE_REGISTER(vetor)->data.led1PeakAmplitude = 100;
	MAX30110_LED_PEAK_AMPLITUDE_REGISTER(vetor)->data.led2PeakAmplitude = 100;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_LEDS_PA, vetor, 2 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_PILOT_PEAK_AMPLITUDE_REGISTER(vetor)->pilotPeakAmplitude = 10;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_PILOT_PA, vetor, 1 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_FIFO_CONFIGURATION_REGISTER(vetor)->data.almostFullType = 0;
	MAX30110_FIFO_CONFIGURATION_REGISTER(vetor)->data.rollOver = 0;
	MAX30110_FIFO_CONFIGURATION_REGISTER(vetor)->data.almostFull = 8;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_FIFO_CONFIGURATION, vetor, 1 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_INTERRUPT_ENABLE_REGISTER(vetor)->data.almostFull = 1;
	MAX30110_INTERRUPT_ENABLE_REGISTER(vetor)->data.proximity = 1;
	MAX30110_INTERRUPT_ENABLE_REGISTER(vetor)->data.ambientLightCancellationOverflow = 1;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_INTERRUPT_ENABLE, vetor, 2 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_FIFO_DATA_CONTROL_REGISTER(vetor)->data.fifoData1Type = 5;
	MAX30110_FIFO_DATA_CONTROL_REGISTER(vetor)->data.fifoData2Type = 13;
	MAX30110_FIFO_DATA_CONTROL_REGISTER(vetor)->data.fifoData3Type = 12;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_FIFO_DATA_CONTROL_REGISTERS, vetor, 2 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_LED_RANGE_REGISTER(vetor)->data.led1Range = 0;
	MAX30110_LED_RANGE_REGISTER(vetor)->data.led2Range = 0;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_LED_RANGE, vetor, 1 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_PROXIMITY_INT_THRESHOLD_REGISTER(vetor)->proximityInterruptThreshold = 0x2F;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_PROX_INTERRUPT_THRESHOULD, vetor, 1 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	memset(vetor, 0, sizeof(vetor));

	MAX30110_SYSTEM_CONTROL_REGISTER(vetor)->data.frequencyClockPinControl = 1;
	MAX30110_SYSTEM_CONTROL_REGISTER(vetor)->data.lowPowerMode = 1;
	MAX30110_SYSTEM_CONTROL_REGISTER(vetor)->data.fifoEnabled = 1;
	MAX30110_SYSTEM_CONTROL_REGISTER(vetor)->data.shutDown = 0;

	if( MAX30110_escreveRegistroAguardaTermino( MAX30110_SYSTEM_CONTROL, vetor, 1 ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	return MAX30110_SUCESSO;
}

MAX30110_retorno_t MAX30110_lerEstado ( MAX30110_interruptStatusRegistersUnion_t* valor ) {
	if( MAX30110_leRegistroAguardaTermino( MAX30110_INTERRUPT_STATUS, valor->vector, sizeof(MAX30110_interruptStatusRegistersUnion_t) ) != MAX30110_SUCESSO )
		return MAX30110_FALHOU_ERRO_LEITURA_SPI;

	return MAX30110_SUCESSO;
}


MAX30110_retorno_t MAX30110_shutdown ( MAX30110_estruturaConfiguracao_t* configuracao ) {
		
		MAX30110_configurationRegisters_t configurationRegisters;

		memset(configurationRegisters.systemControl.vector, 0, sizeof(MAX30110_systemControlUnion_t));

		configurationRegisters.systemControl.data.reset = MAX30110_SYS_CONTROL_RESET_ON;

		if( MAX30110_escreveRegistroAguardaTermino( MAX30110_SYSTEM_CONTROL, configurationRegisters.systemControl.vector, sizeof(MAX30110_systemControlUnion_t) ) != MAX30110_SUCESSO )
				return MAX30110_FALHOU_ERRO_ESCRITA_SPI;

	
		DELAYMS(10);
	
		memset(configurationRegisters.systemControl.vector, 0, sizeof(MAX30110_systemControlUnion_t));

		configurationRegisters.systemControl.data.shutDown = configuracao->configuracoesSistema.desligar;

		if( MAX30110_escreveRegistroAguardaTermino( MAX30110_SYSTEM_CONTROL, configurationRegisters.systemControl.vector, sizeof(MAX30110_systemControlUnion_t) ) != MAX30110_SUCESSO )
				return MAX30110_FALHOU_ERRO_ESCRITA_SPI;
		
		return MAX30110_SUCESSO;
}

/************************************************************************************************************/

uint8_t SetGain( uint8_t value )
{
  uint8_t regval;

  /* PPG configuration 1
   * b7-b6: ADC range                        (0b01)   0=6uA, 1=12uA, 2=24uA, 3=48uA
   * b5-b2: PPG sample rate 25 Hz, 1 pulse   (0b0001), set PPG_SAMPLING_FREQ according to this settings
   * b1-b0: PPG led pulse width, 100 uS      (0b01)      
   */

  /* In max30110, 0 = highest gain, 3 = lowest gain, therefore : ((~gain) & 0x03) or (gain ^ 0x03); */
  /* convert 0..3 to 3..0 */
  regval = (((value ^ 0x03) << 6) & 0xC0) | 0x04 | (1 & 0x03);

	MAX30110_escreveRegistroAguardaTermino( MAX30110_PPG_CONFIGURATION_1, &regval, 1 );
	
//  MAX30110_writeReg( AFE_PPG_CFG1, regval );
  
//  trigLoop        = true;                          // sensitivity is changed, trigger loop again for fine tuning 
  sDataCh.adcGain = value;                         // store the used setting

  // Error code, dummy for now
  return 0;
}

/************************************************************************************************************/
/************************************************************************************************************/

uint8_t SetLedPower( uint16_t value )
{
    uint8_t ledpower;
		ledpower=MAX30110_CONVERT_CURRENT_TO_LED_PEAK_50MA(value*25/100);
		MAX30110_escreveRegistroAguardaTermino( MAX30110_LED1_PA, &ledpower, 1 );
	
		MAX30110_escreveRegistroAguardaTermino( MAX30110_LED2_PA, &ledpower, 1 );

//    sDataCh.ledPower = value; 
	
    return 0;
}

/************************************************************************************************************/
/************************************************************************************************************/

/******************************************************************************/
void ControlLoop( uint32_t ppg, uint16_t *pwr, uint8_t *gain,g4_PPGControlLoopConfig_t sControlLoopParams)
{
    /* preset new values with old value */
    volatile uint32_t pwrNew  = *pwr;
    volatile uint8_t  gainNew = *gain;

    /* calculate thresholds for full scale (gain=0)  */
    uint32_t thrHigh = ((((uint32_t)sControlLoopParams.thHigh) * (ADC_MAX >> 4)) / 100);
    uint32_t thrLow  = ((((uint32_t)sControlLoopParams.thLow ) * (ADC_MAX >> 4)) / 100);
//    uint32_t target  = (thrLow + thrHigh) / 2;   /* put target in the middle of low-high thresholds */
/* check if thresholds are hit */
		if(trigLoop){
				if (ppg > thrHigh) 
    		{
			  		pwrNew >>= 1;   /* /2 */
//						pwrNew=pwrNew/1.7;

				}

				else if (ppg < thrLow) 
    		{
						pwrNew <<= 1;   /* x2 */
//						pwrNew=pwrNew*1.7;
				}
		
				if(pwrNew>70)
						pwrNew=70;
		
				if(pwrNew<10)
						pwrNew=10;
		
				/* Return the new gain value */
				*gain = gainNew;

				/* Return the new power value */
    		*pwr = (uint16_t) pwrNew;
				SetLedPower(pwrNew);
				NRF_LOG_INFO("Power LED: %d",pwrNew);
		}
}

/************************************************************************************************************/


