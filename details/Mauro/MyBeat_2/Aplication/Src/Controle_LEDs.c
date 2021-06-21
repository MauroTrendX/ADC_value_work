/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

/*inclusao do arquivo contendo as configurações e enumerações das cores e LEDs*/
#include "Controle_LEDs.h"

#include "nrf_drv_pwm.h"

/*inclusao do arquivo contendo as configurações e definição das variaveis*/
#include "main.h"

/*inclusao do arquivo contendo as configurações da camada HAL do microcontrolador utilizado*/
//#include "stm32l4xx_hal.h"

/******************************************************************************/
/******************************************************************************/

/*definição da constante do periodo maximo do PWM*/
#define PERIODO_PWM																32768 //em ticks

/*definição da constate com o valor percentual do brilho maximo*/
#define BRILHO																		80

/*define valor do balanço maximo dos LEDs*/
#define CALCULO_BALANCO_LED(BALANCO)			( ( ( PERIODO_PWM * BALANCO ) / 255 ) * BRILHO / 100 )

/*definição da intensidade do LED vermelho*/
#define CALCULO_INTENSIDADE_LED_VERMELHO(BALANCO)      /*    ( PERIODO_PWM - */(CALCULO_BALANCO_LED(BALANCO)*100/100) //)

/*definição da intensidade do LED verde*/
#define CALCULO_INTENSIDADE_LED_VERDE(BALANCO)         /* ( PERIODO_PWM - */( CALCULO_BALANCO_LED(BALANCO)*15/100 ) //)

/*definição da intensidade do LED azul*/
#define CALCULO_INTENSIDADE_LED_AZUL(BALANCO)         /* ( PERIODO_PWM - */( CALCULO_BALANCO_LED(BALANCO)*100/100 ) //)

/******************************************************************************/
/******************************************************************************/

nrf_drv_pwm_t m_pwm0 = NRF_DRV_PWM_INSTANCE(0);
nrf_drv_pwm_t m_pwm1 = NRF_DRV_PWM_INSTANCE(1);

// This is for tracking PWM instances being used, so we can unintialize only
// the relevant ones when switching from one demo to another.
#define USED_PWM(idx) (1UL << idx)
static uint8_t m_used = 0;


static nrf_pwm_values_individual_t /*const*/ pwm1_seq_values;

nrf_pwm_sequence_t const seq1 =
{
    .values.p_individual = &pwm1_seq_values,
    .length              = NRF_PWM_VALUES_LENGTH(pwm1_seq_values),
    .repeats             = 0,
    .end_delay           = 0
};


/*definição da variavel utilizada para a mudaça de cores*/
volatile selecionarCores corAtual = verde;

/*definição da variavel utilizada para a mudança dos LED*/
volatile selecionarLED ledAtual = LED_INVALIDO;

/*Ajuste do balanço RGB para cada cor definida na enumeração das cores*/
const balancoCorRGB cores[fimCores] = 
{
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(255), CALCULO_INTENSIDADE_LED_AZUL(255) }, //branco
	{ CALCULO_INTENSIDADE_LED_VERMELHO(0), CALCULO_INTENSIDADE_LED_VERDE(0), CALCULO_INTENSIDADE_LED_AZUL(255) },			//azul
	{ CALCULO_INTENSIDADE_LED_VERMELHO(0), CALCULO_INTENSIDADE_LED_VERDE(255), CALCULO_INTENSIDADE_LED_AZUL(0) },			//verde
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(255), CALCULO_INTENSIDADE_LED_AZUL(0) },   //amarelo	
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(55), CALCULO_INTENSIDADE_LED_AZUL(0) },		//laranja
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(0), CALCULO_INTENSIDADE_LED_AZUL(0) },			//vermelho
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(0), CALCULO_INTENSIDADE_LED_AZUL(255) }		//rosa
};

/*Ajuste do balanço RGB para cada cor definida na enumeração das cores*/
/*const balancoCorRGB cores[fimCores] = 
{
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(255), CALCULO_INTENSIDADE_LED_AZUL(255) }, //branco
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(255), CALCULO_INTENSIDADE_LED_AZUL(0) },   //amarelo
	{ CALCULO_INTENSIDADE_LED_VERMELHO(0), CALCULO_INTENSIDADE_LED_VERDE(255), CALCULO_INTENSIDADE_LED_AZUL(0) },			//verde		
	{ CALCULO_INTENSIDADE_LED_VERMELHO(0), CALCULO_INTENSIDADE_LED_VERDE(0), CALCULO_INTENSIDADE_LED_AZUL(255) },			//azul
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(0), CALCULO_INTENSIDADE_LED_AZUL(0) },			//vermelho
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(100), CALCULO_INTENSIDADE_LED_AZUL(0) },	//laranja
	{ CALCULO_INTENSIDADE_LED_VERMELHO(255), CALCULO_INTENSIDADE_LED_VERDE(0), CALCULO_INTENSIDADE_LED_AZUL(255) }		//rosa
};*/

/*Variavel utilizada pelo programa para manuseio do timer 2, (obs: extern, pois a variavel foi declarada no arquivo main.c*/
//extern TIM_HandleTypeDef htim2;
extern nrf_drv_pwm_t m_pwm1;
/******************************************************************************/
/******************************************************************************/

/*prototipo de rotina responsavel pelo manuseio dos erros*/
void Error_Handler(void);

/******************************************************************************/
/******************************************************************************/

		uint16_t calculoDePulsoVerde;
		uint16_t calculoDePulsoAzul;
		uint16_t calculoDePulsoVermelho;

uint16_t get_pulso(selecionarCores cor){
		switch (cor){
			case verde:	
				return calculoDePulsoVerde;
			case azul:
				return calculoDePulsoAzul;
			case vermelho:
				return calculoDePulsoVermelho;
			default:
				return 0;
		}
}

/*definição da rotina responsavel pela mudança de cores*/
void set_cor(selecionarCores cor)
{ if(cor<=inicioCores || cor>=fimCores){
	return;
	}
		
	if( corAtual != cor )
	{	
				
		calculoDePulsoVerde = cores[cor].verde;
		
		calculoDePulsoAzul = cores[cor].azul;
		
		calculoDePulsoVermelho = cores[cor].vermelho;
	
		corAtual = cor;
	}
}

/*definição da rotina responsavel pela mudança dos LEDs*/
void set_led(selecionarLED led){
	
	if (ledAtual != led){
		if(led==LEDS_DESLIGADOS){
			  		nrf_gpio_pin_clear(LD1_A);
						nrf_gpio_pin_clear(LD2_A);
						nrf_gpio_pin_clear(LD3_A);
						nrf_gpio_pin_clear(LD4_A);
			
		}
		
		if(led==LED1){
		    nrf_gpio_pin_set(LD1_A);
			  nrf_gpio_pin_clear(LD2_A);
				nrf_gpio_pin_clear(LD3_A);
				nrf_gpio_pin_clear(LD4_A);
		}
		
		if(led==LED2){
				nrf_gpio_pin_set(LD1_A);
				nrf_gpio_pin_set(LD2_A);
				nrf_gpio_pin_clear(LD3_A);
				nrf_gpio_pin_clear(LD4_A);
		}	  
		
		if(led==LED3){
				nrf_gpio_pin_set(LD1_A);
				nrf_gpio_pin_set(LD2_A);
				nrf_gpio_pin_set(LD3_A);
				nrf_gpio_pin_clear(LD4_A);
		}
		
		if(led==LED4){
			  nrf_gpio_pin_set(LD1_A);
				nrf_gpio_pin_set(LD2_A);
				nrf_gpio_pin_set(LD3_A);
				nrf_gpio_pin_set(LD4_A);
		}
		
		if(led==LED_BLINK){
			  nrf_gpio_pin_clear(LD1_A);
				nrf_gpio_pin_clear(LD2_A);
				nrf_gpio_pin_clear(LD3_A);
				nrf_gpio_pin_set(LD4_A);
		}

		ledAtual=led;
	}
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

void pwm1_handler(nrf_drv_pwm_evt_type_t event_type)
{
    if (event_type == NRF_DRV_PWM_EVT_FINISHED)
    {
				pwm1_seq_values.channel_0 = get_pulso(vermelho); 
				pwm1_seq_values.channel_1 = get_pulso(verde); 
				pwm1_seq_values.channel_2 = get_pulso(azul); 
				pwm1_seq_values.channel_3 = 0; 

    }
}

void pwm_init(void){

    nrf_drv_pwm_config_t config =
    {
				// These are the common configuration options we use for all PWM
        // instances.
        .irq_priority = APP_IRQ_PRIORITY_LOWEST,
        .count_mode   = NRF_PWM_MODE_UP,
        .step_mode    = NRF_PWM_STEP_AUTO,
			
		};
			
    config.output_pins[0] = CLK_32K;
		config.output_pins[1] = NRF_DRV_PWM_PIN_NOT_USED;
		config.output_pins[2] = NRF_DRV_PWM_PIN_NOT_USED;
		config.output_pins[3] = NRF_DRV_PWM_PIN_NOT_USED;
        
    config.base_clock   = NRF_PWM_CLK_2MHz;
    config.top_value    = 61;
    config.load_mode    = NRF_PWM_LOAD_INDIVIDUAL;
     
	  APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm0, &config, NULL));
    m_used |= USED_PWM(0);

    // This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    static nrf_pwm_values_individual_t /*const*/ pwm0_seq_values[] = {30,0,0,0};

    nrf_pwm_sequence_t const seq0 =
    {
        .values.p_individual = pwm0_seq_values,
        .length              = NRF_PWM_VALUES_LENGTH(pwm0_seq_values),
        .repeats             = 0,
        .end_delay           = 0
    };
		
		config.output_pins[0] = K_RED;
		config.output_pins[1] = K_GREEN;
		config.output_pins[2] = K_BLUE;
		config.output_pins[3] = NRF_DRV_PWM_PIN_NOT_USED;
        
    config.base_clock   = NRF_PWM_CLK_4MHz;
    config.top_value    = 32768;
    config.load_mode    = NRF_PWM_LOAD_INDIVIDUAL;
     
	  APP_ERROR_CHECK(nrf_drv_pwm_init(&m_pwm1, &config, pwm1_handler));
    m_used |= USED_PWM(1);
		
		// This array cannot be allocated on stack (hence "static") and it must
    // be in RAM (hence no "const", though its content is not changed).
    pwm1_seq_values.channel_0 = 0; 
		pwm1_seq_values.channel_1 = 0; 
		pwm1_seq_values.channel_2 = 0; 
		pwm1_seq_values.channel_3 = 0; 
		
    (void)nrf_drv_pwm_simple_playback(&m_pwm0, &seq0, 1, NRF_DRV_PWM_FLAG_LOOP);
		(void)nrf_drv_pwm_simple_playback(&m_pwm1, &seq1, 1, NRF_DRV_PWM_FLAG_LOOP);
}

void pwm_stop(void){
	
		nrf_drv_pwm_uninit(&m_pwm0);
		nrf_drv_pwm_uninit(&m_pwm1);

}

uint8_t get_current_color(void){
		return (uint8_t)corAtual;
}
