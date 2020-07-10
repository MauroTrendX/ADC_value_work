/*
 ******************************************************************************
 * @file    MAX30110_port.h
 * @author  Vítor Eduardo Sabadine da Cruz
 * @version 2
 * @date    15-Setembro-2019
 * @brief   Biblioteca C.I. front-end analógico (AFE) MAX30110.
 *
 ******************************************************************************
 */

/*
 * Define macros para substituição das chamadas de rotina da SPI específicas da
 * aplicação em questão, permitindo o uso deste arquivo fonte de biblioteca para
 * diversos micro controladores.
 */

#include "string.h"
#include <inttypes.h>
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"


uint8_t MAX30110_vetorDados[30];

/*
 * A biblioteca verifica o retorno destas rotinas para prosseguir com a comunicação.
 */
#define	DELAYMS(valor)																			nrf_delay_ms(valor)//osDelay
#define MOCK_MAX30110 0


#if ( MOCK_MAX30110 == 0 )

#define MAX30110_spiSelecionaChip()						 	    nrf_gpio_pin_clear (AFE_nSS);nrf_gpio_pin_set(ACC_nSS)//;; nrf_gpio_pin_clear(	AFE_EN32K )	
#define MAX30110_spiDeselecionaChip();						 	nrf_gpio_pin_set (AFE_nSS)//;nrf_gpio_pin_set( AFE_EN32K ) // osSemaphoreRelease(spi1inUseBinarySemHandle); }

#define MAX30110_spiEstaPreparada()									//( osSemaphoreAcquire( spi1inUseBinarySemHandle, portMAX_DELAY ) == osOK )

#define MAX30110_spiIniciaTempoParaEsgotar()					// Nada declarado, o compilador vai ignorar a instrução.
#define MAX30110_spiTerminouTransicao()							//( osSemaphoreAcquire( spi1BusyBinarySemHandle, portMAX_DELAY ) == osOK )
#define MAX30110_spiEstaComErros()						  		( 1 )


#else
#define MAX30110_spiSelecionaChip()									nrf_gpio_pin_clear (25)
#define MAX30110_spiDeselecionaChip()								nrf_gpio_pin_set (25)

#define MAX30110_spiEstaPreparada()									( 1 )

#define MAX30110_spiIniciaTempoParaEsgotar()				// Nada declarado, o compilador vai ignorar a instrução.
#define MAX30110_spiTerminouTransicao()							( 1 )
#define MAX30110_spiEstaComErros()						  		( 0 )

uint8_t MAX30110_enviaDadosSpiValorRetorno = 1;

__attribute__((optimize(0))) uint8_t MAX30110_enviaDadosSpi ( uint8_t quantidade ){
	for(uint32_t delay=quantidade; delay>0; delay--);

	return MAX30110_enviaDadosSpiValorRetorno;
}

uint8_t vetorInjecaoDados[30];
uint8_t MAX30110_recebeDadosSpiValorRetorno = 1;

uint8_t MAX30110_recebeDadosSpi ( uint8_t*vetor, uint8_t quantidade ){
	uint8_t* ponteiro = vetorInjecaoDados;

	for(uint32_t laco=quantidade; laco>0; laco--)
		*vetor++ = *ponteiro++;
	return MAX30110_recebeDadosSpiValorRetorno;
}

#endif

/*
 * Definição do buffer de dados usado pela biblioteca do acelerômetro.
 */
#define MAX30110_VETOR_DADOS			MAX30110_vetorDados

/******************************************************************************/


/******************************************************************************/
