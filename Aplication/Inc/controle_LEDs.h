/*inclusao da biblioteca */
#include <stdint.h>

/*enumeração das cores configuradas*/

typedef enum
{
inicioCores = -1,
branco,//branco
azul,//amarelo
verde,//verde
amarelo,//azul
laranja,//vermelho
vermelho,//laranja
rosa,//rosa
fimCores//preto
}selecionarCores;

/*enumeração dos LEDs utilizados no programa*/

typedef enum
{
LED_INVALIDO=0,
LED1,
LED2,
LED3,
LED4,
LED_BLINK,	
LEDS_DESLIGADOS,
fimLEDs
}selecionarLED;

/*definição das variaves utilizadas para o balanço de cores*/

typedef struct 
{ 
uint32_t vermelho;
uint32_t verde;
uint32_t azul;
}balancoCorRGB;

/*prototipo de rotina responsavel para seleção de cores*/

void set_cor(selecionarCores cor);

/*prototipo de rotina responsavel pela seleção dos LEDs*/

void set_led(selecionarLED led);

void pwm_init(void);
void pwm_stop(void);
uint16_t get_pulso(selecionarCores cor);

uint8_t get_current_color(void);
