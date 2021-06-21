/*
 ******************************************************************************
 * @file    MAX30110_API.h
 * @author  Vítor Eduardo Sabadine da Cruz
 * @version 2
 * @date    15-Setembro-2019
 * @brief   Biblioteca C.I. front-end analógico (AFE) MAX30110.
 *
 ******************************************************************************
 */

/** @addtogroup MAX30110_estruturaConfiguracao
  * @brief definições dos tipos usados na estrutura de configuração do MAX30110.
  * @{
  */

typedef enum {
	MAX30110_SUCESSO,
	MAX30110_FALHOU,
	MAX30110_FALHOU_INICIALIZACAO_CHIP,
	MAX30110_FALHOU_ERRO_PARAMETRO,
	MAX30110_FALHOU_SPI_OCUPADA,
	MAX30110_FALHOU_ERRO_SPI,
	MAX30110_FALHOU_ERRO_LEITURA_SPI,
	MAX30110_FALHOU_ERRO_ESCRITA_SPI,
	MAX30110_FALHOU_AUTO_TESTE
}MAX30110_retorno_t;

typedef enum {
	MAX30110_ESCALA_ADC_6UA = 1,
	MAX30110_ESCALA_ADC_12UA,
	MAX30110_ESCALA_ADC_24UA,
	MAX30110_ESCALA_ADC_48UA
}MAX30110_escalaAdcEnum_t;

typedef enum {
	MAX30110_TEMPO_INTEGRACAO_52US = 1,
	MAX30110_TEMPO_INTEGRACAO_104US,
	MAX30110_TEMPO_INTEGRACAO_206US,
	MAX30110_TEMPO_INTEGRACAO_417US
}MAX30110_tempoIntegracaoEnum_t;

typedef enum {
	MAX30110_TEMPO_ESTABILIZACAO_2_5MS = 1,
	MAX30110_TEMPO_ESTABILIZACAO_5MS,
	MAX30110_TEMPO_ESTABILIZACAO_10MS,
	MAX30110_TEMPO_ESTABILIZACAO_20MS
}MAX30110_tempoEstabilizacaoLedEnum_t;

typedef enum {
	MAX30110_FREQ_AMOSTRA_20SPS = 1,
	MAX30110_FREQ_AMOSTRA_25SPS,
	MAX30110_FREQ_AMOSTRA_50SPS,
	MAX30110_FREQ_AMOSTRA_84SPS,
	MAX30110_FREQ_AMOSTRA_100SPS,
	MAX30110_FREQ_AMOSTRA_200SPS,
	MAX30110_FREQ_AMOSTRA_400SPS,
	MAX30110_FREQ_AMOSTRA_800SPS,
	MAX30110_FREQ_AMOSTRA_1000SPS,
	MAX30110_FREQ_AMOSTRA_1600SPS,
	MAX30110_FREQ_AMOSTRA_3200SPS,
	MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_20SPS,
	MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_25SPS,
	MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_50SPS,
	MAX30110_FREQ_AMOSTRA_PULSO_DUPLA_84SPS,
	MAX30110_FREQ_AMOSTRA_PULSO_DUPLO_100SPS
}MAX30110_frequenciaAmostraEnum_t;

typedef enum {
	MAX30110_QUANTIDADE_MEDIA_1_AMOSTRA = 1,
	MAX30110_QUANTIDADE_MEDIA_2_AMOSTRAS,
	MAX30110_QUANTIDADE_MEDIA_4_AMOSTRAS,
	MAX30110_QUANTIDADE_MEDIA_8_AMOSTRAS,
	MAX30110_QUANTIDADE_MEDIA_16_AMOSTRAS,
	MAX30110_QUANTIDADE_MEDIA_32_AMOSTRAS
}MAX30110_quantidadeMediaAmostraEnum_t;

typedef struct {
	uint8_t ledForaConformidade:1;
	uint8_t interrrupcaoProxiidade:1;
	uint8_t cancelamentoLuzAmbienteTransbordou:1;
	uint8_t amostraPpgPronta:1;
	uint8_t filaQuaseCheia:1;
	uint8_t vddAnalogicoOk:1;
}MAX30110_ativaInterrupcoesStruct_t;

typedef enum {
	MAX30110_SOBRESCREVER_FILA_DESLIGADO = 1,
	MAX30110_SOBRESCREVER_FILA_LIGADO
}MAX30110_sobrescreverFilaEnum_t;

typedef enum {
	MAX30110_TIPO_FILA_QUASE_CHEIA_REPETE = 1,
	MAX30110_TIPO_FILA_QUASE_CHEIA_UMA_VEZ
}MAX30110_tipoFilaQuaseCheiaEnum_t;

typedef enum {
	MAX30110_MODO_LIMPA_ESTADO_LER_ESTADO = 1,
	MAX30110_MODO_LIMPA_ESTADO_E_LER_FILA
}MAX30110_modoLimpaEstadoEnum_t;

typedef struct {
	MAX30110_sobrescreverFilaEnum_t sobrescreverFila;
	MAX30110_tipoFilaQuaseCheiaEnum_t tipoFilaQuaseCheia;
	MAX30110_modoLimpaEstadoEnum_t modoLimpaEstado;
	uint8_t quantidadeAmostras;
}MAX30110_configuracaoFilaStruct_t;

typedef struct {
	uint8_t pinoOsciladorExterno:1;
	uint8_t modoBaixoConsumo:1;
	uint8_t desligar:1;
	uint8_t habilitarFila:1;
}MAX30110_configuracoesSistema_t;

typedef enum{
	MAX30110_NONE,
	MAX30110_LED1,
	MAX30110_LED2,
	MAX30110_PILOT_LED1 = 5,
	MAX30110_DIRECT_AMBIENT = 12,
	MAX30110_LED1_AND_LED2
}MAX30110_dataItemsTypeEnum_t;

typedef struct{
	MAX30110_dataItemsTypeEnum_t	FD1;
	MAX30110_dataItemsTypeEnum_t	FD2;
	MAX30110_dataItemsTypeEnum_t	FD3;
	MAX30110_dataItemsTypeEnum_t	FD4;
}MAX30110_tipoDadosFila_t;

typedef struct {
	MAX30110_escalaAdcEnum_t escalaAdc;
	MAX30110_tempoIntegracaoEnum_t tempoIntegracao;
	MAX30110_frequenciaAmostraEnum_t frequenciaAmostras;
	MAX30110_tempoEstabilizacaoLedEnum_t tempoEstabilizacaoLed;
	MAX30110_quantidadeMediaAmostraEnum_t quantidadeMediaAmostra;
	float correnteLed1;
	float correnteLed2;
	MAX30110_ativaInterrupcoesStruct_t ativaInterrupcoes;
	MAX30110_configuracaoFilaStruct_t fila;
	MAX30110_configuracoesSistema_t configuracoesSistema;
	MAX30110_tipoDadosFila_t tipoDadosFila;
	float correnteProximidade;
	uint8_t limiteInterrupcaoProximidade;
}MAX30110_estruturaConfiguracao_t;

/*
 * Definições dos registros de indicador de interrupções.
 */
typedef union{
	struct {
		uint8_t powerReady:1;													// Indica o estado da alimentação do circuito interno.
		uint8_t :2;																		// Lê 0.
		uint8_t led1NotCompliance:1;									// Indica se o driver do LED1 não está em conformidade com a tensão de trabalho (VLED1 < 160 mV).
		uint8_t proximity:1;													// Detectou que o dispositivo está mais próximo que o gatilho programado.
		uint8_t ambientLightCancellationOverflow:1;		// Indica que o dispositivo não consegue cancelar a luz ambiente e as amostras estão comprometidas.
		uint8_t photoPlenthysmoGraphReady:1;					// Indica nova amostra de PPG disponível.
		uint8_t almostFull:1;													// Indica fila quase cheia, faltando um item para lotar.
		uint8_t :7;																		// Lê 0.
		uint8_t analogSupplyOutOfRange:1;							// Regulador de tensão analógico fora da faixa de trabalho.
	}bits;
	uint8_t vector[2];
}MAX30110_interruptStatusRegistersUnion_t;

typedef union {
	struct {
		 uint32_t valor:24;
	}dados;
	uint8_t vetor[3];
}MAX30110_dados24Bits_t;

typedef union {
	struct {
		 uint32_t valorPPG:24;
		 uint32_t valorAMB:24;
	}dados;
	uint8_t vetor[7];
}MAX30110_dados24BitsFIFO_t;


/**
 *  \brief Control loop parameters
 */
typedef struct _g4_PPGControlLoopConfig_t
{
    bool        enabled;

    uint8_t     thLow;
    uint8_t     thHigh;
    uint16_t    ledMin;
    uint16_t    ledMax;

    uint8_t     adcGain;
    uint16_t    ledPower;

} g4_PPGControlLoopConfig_t;

/**
 *  \brief PPG sensor configuration structure
 */
typedef struct _g4_PpgSensorConfig_t
{
    uint8_t     pulseWidth;
    uint8_t     ledRange;
    
    g4_PPGControlLoopConfig_t controlLoopParams;

} g4_PpgSensorConfig_t;

MAX30110_retorno_t MAX30110_verificaConexaoSpi ( void );
MAX30110_retorno_t MAX30110_leFila( MAX30110_dados24BitsFIFO_t* dados, uint16_t quantidade );
MAX30110_retorno_t MAX30110_configura ( MAX30110_estruturaConfiguracao_t* configuracao );
MAX30110_retorno_t MAX30110_lerEstado ( MAX30110_interruptStatusRegistersUnion_t* valor );
MAX30110_retorno_t MAX30110_testeProximidade ( void );
MAX30110_retorno_t MAX30110_shutdown ( MAX30110_estruturaConfiguracao_t* configuracao );
void ControlLoop( uint32_t ppg, uint16_t *pwr, uint8_t *gain,g4_PPGControlLoopConfig_t sControlLoopParams);
MAX30110_retorno_t MAX30110_verifica_interrupcao (uint8_t * p_interrupcao);

/******************************************************************************/
