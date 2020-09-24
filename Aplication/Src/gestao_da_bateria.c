/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
//																																						//
//					Componente responsavel pela gestão da bateria 										//
//																																						//				
/******************************************************************************/
// get_status_bateria() - Rotina para verificação do estado a bateria, ela		//
// verifica o nivel percentual a bateria (0%- descarregada á 100%- carregada) //
// retornando avisos quando a bateria chega a 10% ou menos, e solicita o des- //
// ligamento preventivo do dispositvo a 0%.																		//
//																																						//
// get_status_bateria() - Rotina para o manuseio do carregador USB, retornan- //
// do flag de aviso sobre o modo de operação, como os LEDs devem se comportar //
// durante a carga da bateria.																								//
/******************************************************************************/
// Autor: Genival Ferreira 		Data:01/08/2019  - Revisão: 00									//
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/*Inclusao do arquivo contendo as configurações e definição das variaveis*/
#include "main.h"

/*Inclusao do arquivo contendo o prototipo de rotinas ultilizadas no loop principal*/
#include "gestao_bateria.h"

/******************************************************************************/
/******************************************************************************/

/*definição das variaveis utilizadas dentro e fora do componente*/
bool usb_DETECT=false; //flag gerada pela interrupção do carregador
bool usb_charge_alert=false; //flag que informa que o dispositivo está carregando pela USB
bool aviso_bateria_fraca=false; //flag de aviso sobre o nivel da bateria inferior a 10%
bool requisicao_desligamento=false; //flag que solicita o desligamento preventivo do dispositivo á 0% da bateria
uint8_t porcent_bat=100; //variavel que contem o percentual instantaneo da tensão da bateria

/*definição das variaveis utilizadas apenas dentro do componente*/
uint16_t valorADC=0; //valor lido pelo canal ADC do uC (precisao de 12 bits)
uint16_t amostrasADC[100]; //vetor com as amostras para realização do oversampling
uint32_t somatoriaADC=0; //variavel com a soma do vetor amostrasADC[100]
uint16_t mediaADC=0; //valor medio das amostras coletadas pelo ADC


/*definição das externas que sao utilizadas no componente*/
extern ADC_HandleTypeDef hadc1; //instância do objeto do canal ADC
extern uint8_t queroEstaCor; //variavel que selecionara a cor dos LEDs
extern uint8_t	queroEsteLED; //variavel que selecionara o numero de LEDs aceso

/******************************************************************************/
/******************************************************************************/

/*Rotina para leitura do ADC que retorna o nivel percentual da bateria e avisos de bateria fraca e solicitação de desligamento*/
uint8_t get_status_bateria(void){
		
	somatoriaADC=0;
	
	for(uint8_t n1=0;n1<100;n1++){
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 100000);
		amostrasADC[n1] = HAL_ADC_GetValue(&hadc1);
		somatoriaADC = somatoriaADC + amostrasADC[n1];
		HAL_ADC_Stop(&hadc1);
	}
	
	valorADC=somatoriaADC/100;
	porcent_bat= valorADC*100/4096;
	
	if(porcent_bat<=10&&!usb_DETECT)
		aviso_bateria_fraca=true;
	
	if(porcent_bat<=0&&!usb_DETECT)
		requisicao_desligamento=true;
	
	else{
		aviso_bateria_fraca=false;
		requisicao_desligamento=false;
	}	
	
	return porcent_bat;
}

/*Rotina do manuseio do carregador, retorna flags de aviso e o como os LEDs devem se comportar durante a carga */

bool get_status_carregador	(void){
		
	if(usb_DETECT==true){
		usb_charge_alert = true;
			HAL_GPIO_WritePin(GPIOB, CHARGE_Pin, GPIO_PIN_SET);
			
			if(porcent_bat<=10){
			queroEstaCor=1;
			queroEsteLED=1;
			}
			if(porcent_bat>10&&porcent_bat<50){
			queroEstaCor=2;
			queroEsteLED=1;
			}
			if(porcent_bat>=50&&porcent_bat<75){
			queroEstaCor=2;
			queroEsteLED=2;
			}
			if(porcent_bat>75&&porcent_bat<100){
			queroEstaCor=2;
			queroEsteLED=3;
			}
			if(porcent_bat>=100){
			queroEstaCor=3;
			queroEsteLED=4;
			}
		
			if(HAL_GPIO_ReadPin(GPIOC, USB_DETECT_Pin)==0){
				usb_DETECT=false;
				usb_charge_alert = false;
			}
	}
						
	else{
		usb_charge_alert = false;
		HAL_GPIO_WritePin(GPIOB, CHARGE_Pin, GPIO_PIN_RESET);
	}
	return usb_charge_alert;
}
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/