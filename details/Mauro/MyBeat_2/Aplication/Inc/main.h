#include <stdint.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#define DEV_KIT

#ifdef DEV_KIT

#define MBAT 				02
#define SW1 				13
#define SPI_SCK_PIN 26
#define SPI_MISO_PIN 30
#define SPI_MOSI_PIN 29
#define K_BLUE			03
#define K_RED				28
#define K_GREEN			04
#define LD1_A				22
#define LD2_A				23
#define LD3_A				24
#define LD4_A				25
#define RX					08
#define TX					06
#define NRST				21
#define AFE_IRQ			15
#define ACC_IRQ			16
#define AFE_EN32K		11
#define CLK_32K			27
#define CTS					07
#define RTS					05
#define USB					14

#define SPI_SS_PIN 31


#else

#define MBAT 				02
#define SW1 				03
#define SPI_SCK_PIN					04
#define SPI_MISO_PIN				05
#define SPI_MOSI_PIN				06
#define ACC_nSS			07
//#define SPI_SS_PIN 8

#define AFE_nSS			8
#define K_BLUE			11
#define K_RED				12
#define K_GREEN			13
#define LD1_A				17
#define LD2_A				16
#define LD3_A				15
#define LD4_A				14
#define RX					18
#define TX					20
#define NRST				21
#define AFE_IRQ			25
#define ACC_IRQ			26
#define AFE_EN32K		27
#define CLK_32K			28
#define CTS					29
#define RTS					30
#define USB					31

#endif

typedef union{
struct{
		uint8_t PPGlsb;
		uint8_t PPGmsb;
		uint8_t ambientelsb;
		uint8_t ambientemsb;	
}dadosPPG;
int16_t vetor[2];
}amostraPPG;

	
//typedef	struct{
//		uint8_t PPGlsb;
//		uint8_t PPGmsb;
//		uint8_t ambientelsb;
//		uint8_t ambientemsb;	
//}amostraPPG;


typedef struct{
		amostraPPG amostra1;
		amostraPPG amostra2;
}dadosBbPPG;

typedef union{
struct{
		uint8_t eixoXlsb;
		uint8_t eixoXmsb;
		uint8_t eixoYlsb;
		uint8_t eixoYmsb;
		uint8_t eixoZlsb;
		uint8_t eixoZmsb;	
}dadosACC;
int16_t vetor[3];
}amostraACC;


//typedef	struct{
//		uint8_t eixoXlsb;
//		uint8_t eixoXmsb;
//		uint8_t eixoYlsb;
//		uint8_t eixoYmsb;
//		uint8_t eixoZlsb;
//		uint8_t eixoZmsb;
//}amostraACC;


typedef struct{
		amostraACC amostra1;
		amostraACC amostra2;
		amostraACC amostra3;
		amostraACC amostra4;
		amostraACC amostra5;
		amostraACC amostra6;
		amostraACC amostra7;
		amostraACC amostra8;	
}dadosBbACC;


int32_t MAX30110_leituraSPI(uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t MAX30110_leituraFilaSPI(uint8_t reg, uint8_t *bufp, uint16_t len);
int32_t MAX30110_escritaSPI(uint8_t reg, uint8_t *bufp, uint16_t len);



