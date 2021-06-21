/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
//#include "buffer_services.h"
/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
typedef enum
{
ZONE_1 = 0,
ZONE_2,
ZONE_3,
ZONE_4,
ZONE_5,
ZONE_6,	
RESTING_ZONE,
END_ZONE
}zonesEnum;

typedef enum{
hr_zone_select_SPIVI = 0,
hr_zone_select_RHR,
hr_zone_select_CIA_ATHLETICA,
hr_zone_select_MYBEAT_V2
}hr_zone_select_t;

/******************************************************************************/
/******************************************************************************/
void zones_info_init ( void );

/******************************************************************************/
uint8_t obter_idade(void);
void definir_idade(uint8_t value);

uint8_t obter_resting_heart_rate_value(void);
void definir_resting_heart_rate_value(uint8_t value);

hr_zone_select_t get_hr_zone_select(void);
void set_hr_zone_select(hr_zone_select_t value);

uint8_t get_SPIVI_percentage_threshold( zonesEnum zone );
void set_SPIVI_percentage_threshold( zonesEnum zone, uint8_t value );

uint8_t get_rhr_percentage_threshold( zonesEnum zone );
void set_rhr_percentage_threshold( zonesEnum zone, uint8_t value );

/******************************************************************************/
zonesEnum get_current_zone ( float heart_rate );

float get_hr_zone_limits(zonesEnum zone);

void calcular_zona_atual(uint8_t BPM);/*,selecionarCores *queroEstaCor, selecionarLED *queroEsteLED)*/

uint8_t get_color(void);
uint8_t get_hr_percent(void);
void load_user_info_metrics(void);

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/
