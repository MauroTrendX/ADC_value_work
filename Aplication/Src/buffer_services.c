#include "buffer_services.h"

#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"
#include "app_util.h"
#include <string.h>
#include "app_error.h"
#include "nrf_log.h"

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
/******************************************************************************/
/******************************************************************************/
/*
 * TODO funções "get" e "set" para ler e gravar os valores atuais da memória
 * não volátil.
 */
/******************************************************************************/
/******************************************************************************/

static hr_module_metric_info_t p_metric_info;// __attribute__ ((section(".noinit")));;
static hr_module_nv_buf_t metric_nv_info;// __attribute__ ((section(".noinit")));;


#ifdef MODE_CIA_ATHLETICA
#define MAX_HR                          208
#else
#define MAX_HR                          220
#endif

#define AGE_DEFAULT_VALUE				36
#define USER_ID_DEFAULT_VALUE			90
#define VALUE_DEFAULT					85
#define VALUE_DEFAULT_NAME					"mbeat"

NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = 0x6F000,
    .end_addr   = 0x6FFFF,
};

uint32_t addr_start =0x6F000;


/**@brief   Helper function to obtain the last address on the last page of the on-chip flash that
 *          can be used to write user data.
 */
static uint32_t nrf5_flash_end_addr_get()
{
    uint32_t const bootloader_addr = BOOTLOADER_ADDRESS;
    uint32_t const page_sz         = NRF_FICR->CODEPAGESIZE;
    uint32_t const code_sz         = NRF_FICR->CODESIZE;

    return (bootloader_addr != 0xFFFFFFFF ?
            bootloader_addr : (code_sz * page_sz));
}

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
 //   (void) sd_app_evt_wait();
    __WFE();
}

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;
				
				case NRF_FSTORAGE_EVT_READ_RESULT:
        {
            NRF_LOG_INFO("--> Event received: read %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;
       
        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;
    
				default:
            break;
    }
}

void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        power_manage();
    }
}

static void print_flash_info(nrf_fstorage_t * p_fstorage)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      p_fstorage->p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    p_fstorage->p_flash_info->program_unit);
    NRF_LOG_INFO("==============================");
}

void buff_init(void){
		ret_code_t rc;
	
    nrf_fstorage_api_t * p_fs_api;		

		p_fs_api = &nrf_fstorage_sd;
		
		
		rc = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);		
		print_flash_info(&fstorage);
		
    /* It is possible to set the start and end addresses of an fstorage instance at runtime.
     * They can be set multiple times, should it be needed. The helper function below can
     * be used to determine the last address on the last page of flash memory available to
     * store data. */
    (void) nrf5_flash_end_addr_get();		
		
}
/*
uint8_t test[]="junior12";
uint8_t retorno_teste[8];
void buff_save_test(void){
		ret_code_t rc;
		volatile uint32_t retorno;
	
		retorno = nrf5_flash_end_addr_get();
    wait_for_flash_ready(&fstorage);
//		rc = nrf_fstorage_erase(&fstorage, 0x6F000, 1, NULL);
//    APP_ERROR_CHECK(rc);  
//		wait_for_flash_ready(&fstorage);
		rc = nrf_fstorage_write(&fstorage, 0x6F000, test, sizeof(test), NULL);
    APP_ERROR_CHECK(rc);

    wait_for_flash_ready(&fstorage);
    NRF_LOG_INFO("Done.");		
	
}

void buff_load_test(void){

		uint32_t err_code;

		err_code=nrf_fstorage_read(&fstorage, 0x6F000,&retorno_teste, sizeof(test));
		wait_for_flash_ready(&fstorage);
}
*/

void buff_save(void){
		ret_code_t rc;
		
		rc = nrf_fstorage_erase(&fstorage, addr_start, 1, NULL);
    APP_ERROR_CHECK(rc);
		wait_for_flash_ready(&fstorage);
	
    rc = nrf_fstorage_write(&fstorage, addr_start, (uint8_t*)&metric_nv_info, sizeof(hr_module_nv_buf_t), NULL);
    APP_ERROR_CHECK(rc);
    wait_for_flash_ready(&fstorage);
	
}


void buff_load(void){

uint32_t err_code;

	err_code=nrf_fstorage_read(&fstorage, addr_start,&metric_nv_info, sizeof(hr_module_nv_buf_t));
	wait_for_flash_ready(&fstorage);
	
if(err_code == NRF_SUCCESS){
//			if(metric_nv_info.metric_settings_nv_buf.age_metric_set)
//				p_metric_info.age.age_value = metric_nv_info.age.age_value;
//			else
//				p_metric_info.age.age_value = 0;

//			if(metric_nv_info.metric_settings_nv_buf.body_position_metric_set)
//				p_metric_info.body_position.body_position_value = metric_nv_info.body_position.body_position_value;
//			else
//				p_metric_info.body_position.body_position_value = 0;

//			if(metric_nv_info.metric_settings_nv_buf.height_metric_set)
//				p_metric_info.height.height_value = metric_nv_info.height.height_value;
//			else
//				p_metric_info.height.height_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.rhr_preference_metric_set)
				p_metric_info.rhr_preference.rhr_preference_value = metric_nv_info.rhr_preference.rhr_preference_value;
			else
				p_metric_info.rhr_preference.rhr_preference_value = 0;

//			if(metric_nv_info.metric_settings_nv_buf.sleep_preference_metric_set)
//				p_metric_info.sleep_preference.sleep_preference_value = metric_nv_info.sleep_preference.sleep_preference_value;
//			else
//				p_metric_info.sleep_preference.sleep_preference_value = 0;

//			if(metric_nv_info.metric_settings_nv_buf.time_metric_set)
//				p_metric_info.time.time_value = metric_nv_info.time.time_value;
//			else
//				p_metric_info.time.time_value = 0;

//			if(metric_nv_info.metric_settings_nv_buf.weight_metric_set)
//				p_metric_info.weight.weight_value = metric_nv_info.weight.weight_value;
//			else
//				p_metric_info.weight.weight_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.user_id_metric_set)						//@Including USER ID
				p_metric_info.user_id.user_id_value = metric_nv_info.user_id.user_id_value;
			else
				p_metric_info.user_id.user_id_value = 0;

//			if(metric_nv_info.metric_settings_nv_buf.aerobic_threshold_metric_set)				//@Including AEROBIC THRESHOLD
//				p_metric_info.aerobic_threshold.aerobic_threshold_value = metric_nv_info.aerobic_threshold.aerobic_threshold_value;
//			else
//				p_metric_info.aerobic_threshold.aerobic_threshold_value = 0;

//			if(metric_nv_info.metric_settings_nv_buf.anaerobic_threshold_metric_set)			//@Including ANAEROBIC THRESHOLD
//				p_metric_info.anaerobic_threshold.anaerobic_threshold_value = metric_nv_info.anaerobic_threshold.anaerobic_threshold_value;
//			else
//				p_metric_info.anaerobic_threshold.anaerobic_threshold_value = 0;

//			if(metric_nv_info.metric_settings_nv_buf.name_metric_set)							//@Including NAME
//				p_metric_info.name.name_value = metric_nv_info.name.name_value;
//			else
//				p_metric_info.name.name_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.hr_zone_preference_calc_metric_set)					//@Including FITNESS INDEX
				p_metric_info.hr_zone_preference_calc.hr_zone_preference_calc_value = metric_nv_info.hr_zone_preference_calc.hr_zone_preference_calc_value;
			else
				p_metric_info.hr_zone_preference_calc.hr_zone_preference_calc_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.first_name_metric_set)					//
				p_metric_info.first_name.first_name_value = metric_nv_info.first_name.first_name_value;
			else
				p_metric_info.first_name.first_name_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.last_name_metric_set)					//
				p_metric_info.last_name.last_name_value = metric_nv_info.last_name.last_name_value;
			else
				p_metric_info.last_name.last_name_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.email_address_metric_set)					//
				p_metric_info.email_address.email_address_value = metric_nv_info.email_address.email_address_value;
			else
				p_metric_info.email_address.email_address_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.age_metric_set)					//
				p_metric_info.age.age_value = metric_nv_info.age.age_value;
			else
				p_metric_info.age.age_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.date_of_birth_metric_set)					//
				p_metric_info.date_of_birth.date_of_birth_value = metric_nv_info.date_of_birth.date_of_birth_value;
			else
				p_metric_info.date_of_birth.date_of_birth_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.gender_metric_set)					//
				p_metric_info.gender.gender_value = metric_nv_info.gender.gender_value;
			else
				p_metric_info.gender.gender_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.weight_metric_set)					//
				p_metric_info.weight.weight_value = metric_nv_info.weight.weight_value;
			else
				p_metric_info.weight.weight_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.height_metric_set)					//
				p_metric_info.height.height_value = metric_nv_info.height.height_value;
			else
				p_metric_info.height.height_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.VO2_max_metric_set)					//
				p_metric_info.VO2_max.VO2_max_value = metric_nv_info.VO2_max.VO2_max_value;
			else
				p_metric_info.VO2_max.VO2_max_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.heart_rate_max_metric_set)					//
				p_metric_info.heart_rate_max.heart_rate_max_value = metric_nv_info.heart_rate_max.heart_rate_max_value;
			else
				p_metric_info.heart_rate_max.heart_rate_max_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.resting_heart_rate_metric_set)					//
				p_metric_info.resting_heart_rate.resting_heart_rate_value = metric_nv_info.resting_heart_rate.resting_heart_rate_value;
			else
				p_metric_info.resting_heart_rate.resting_heart_rate_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.maximum_recommended_heart_rate_metric_set)					//
				p_metric_info.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value = metric_nv_info.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value;
			else
				p_metric_info.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.aerobic_threshold_metric_set)					//
				p_metric_info.aerobic_threshold.aerobic_threshold_value = metric_nv_info.aerobic_threshold.aerobic_threshold_value;
			else
				p_metric_info.aerobic_threshold.aerobic_threshold_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.anaerobic_threshold_metric_set)					//
				p_metric_info.anaerobic_threshold.anaerobic_threshold_value = metric_nv_info.anaerobic_threshold.anaerobic_threshold_value;
			else
				p_metric_info.anaerobic_threshold.anaerobic_threshold_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.sport_type_for_aerobic_and_anaerobic_thresholds_metric_set)					//
				p_metric_info.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value = metric_nv_info.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value;
			else
				p_metric_info.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.date_of_threshold_assessment_metric_set)					//
				p_metric_info.date_of_threshold_assessment.date_of_threshold_assessment_value = metric_nv_info.date_of_threshold_assessment.date_of_threshold_assessment_value;
			else
				p_metric_info.date_of_threshold_assessment.date_of_threshold_assessment_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.waist_circumference_metric_set)					//
				p_metric_info.waist_circumference.waist_circumference_value = metric_nv_info.waist_circumference.waist_circumference_value;
			else
				p_metric_info.waist_circumference.waist_circumference_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.fat_burn_heart_rate_lower_limit_metric_set)					//
				p_metric_info.fat_burn_heart_rate_lower_limit.fat_burn_heart_rate_lower_limit_value = metric_nv_info.fat_burn_heart_rate_lower_limit.fat_burn_heart_rate_lower_limit_value;
			else
				p_metric_info.hr_zone_preference_calc.hr_zone_preference_calc_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.fat_burn_heart_rate_upper_limit_metric_set)					//
				p_metric_info.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value = metric_nv_info.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value;
			else
				p_metric_info.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.aerobic_heart_rate_lower_limit_metric_set)					//
				p_metric_info.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value = metric_nv_info.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value;
			else
				p_metric_info.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.aerobic_heart_rate_upper_limit_metric_set)					//
				p_metric_info.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value = metric_nv_info.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value;
			else
				p_metric_info.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.anaerobic_heart_rate_lower_limit_metric_set)					//
				p_metric_info.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value = metric_nv_info.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value;
			else
				p_metric_info.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.five_zone_heart_rate_limits_metric_set)					//
				p_metric_info.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value = metric_nv_info.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value;
			else
				p_metric_info.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.three_zone_heart_rate_limits_metric_set)					//
				p_metric_info.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value = metric_nv_info.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value;
			else
				p_metric_info.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.two_zone_heart_rate_limit_metric_set)					//
				p_metric_info.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value = metric_nv_info.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value;
			else
				p_metric_info.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.language_metric_set)					//
				p_metric_info.language.language_value = metric_nv_info.language.language_value;
			else
				p_metric_info.language.language_value = 0;

			if(metric_nv_info.metric_settings_nv_buf.fitnes_index_metric_set)					//
				p_metric_info.fitnes_index.fitnes_index_value = metric_nv_info.fitnes_index.fitnes_index_value;
			else
				p_metric_info.fitnes_index.fitnes_index_value = 0;

		}

		else
			;
	
	return ;

}

/******************************@Including USER ID*******************************/
uint8_t get_user_id(void){
	if( metric_nv_info.user_id.user_id_value == 0 || metric_nv_info.user_id.user_id_value > 0x63 ){
		metric_nv_info.user_id.user_id_value = USER_ID_DEFAULT_VALUE;
	}
	return metric_nv_info.user_id.user_id_value;
}

void set_user_id(uint8_t value){
	metric_nv_info.user_id.user_id_value = value;
}

/******************************@Including FITNESS INDEX*******************************/
uint8_t get_hr_zone_preference_calc(void){
	if( metric_nv_info.hr_zone_preference_calc.hr_zone_preference_calc_value >3 ){
		metric_nv_info.hr_zone_preference_calc.hr_zone_preference_calc_value = 3;
	}
	return metric_nv_info.hr_zone_preference_calc.hr_zone_preference_calc_value;
}

void set_hr_zone_preference_calc(uint8_t value){
	metric_nv_info.hr_zone_preference_calc.hr_zone_preference_calc_value = value;
}

/******************************************************************************/
/******************************************************************************/
hr_zone_calc_select_t get_hr_zone_calc_select(void){
	if( metric_nv_info.hr_zone_calc_select > hr_zone_calc_select_SPIVI ){
		metric_nv_info.hr_zone_calc_select =  hr_zone_calc_select_SPIVI;
	}
	return metric_nv_info.hr_zone_calc_select;
}

/******************************************************************************/
/******************************************************************************/
void set_hr_zone_calc_select(hr_zone_calc_select_t value){
	metric_nv_info.hr_zone_calc_select = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_resting_heart_rate_value_threshold(void){
	if( metric_nv_info.rhr_preference.rhr_preference_value == 0xFF ){
		metric_nv_info.rhr_preference.rhr_preference_value = 0;
	}

	return metric_nv_info.rhr_preference.rhr_preference_value;
}

/******************************************************************************/
void set_resting_heart_rate_value_threshold(uint8_t value){
	metric_nv_info.rhr_preference.rhr_preference_value = value;
}

/******************************************************************************/
/******************************************************************************/

hr_module_metric_info_t *get_metrics(void){
	return &p_metric_info;
}
/******************************************************************************/
/******************************************************************************/

/******************************************************************************/
/******************************************************************************/
uint64_t get_first_name_metric(void){    //alterar nome criar struct
	if( metric_nv_info.first_name.first_name_value == 0 ||metric_nv_info.first_name.first_name_value == 0xFFFFFFFFFFFFFFFF){
		metric_nv_info.first_name.first_name_value = 0x6D79626561740000;
	}

	return metric_nv_info.first_name.first_name_value;
}

/******************************************************************************/
void set_first_name_metric(uint64_t value){
	metric_nv_info.first_name.first_name_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_last_name_metric(void){
	if( metric_nv_info.last_name.last_name_value == 0 || metric_nv_info.last_name.last_name_value == 0xFF ){
		metric_nv_info.last_name.last_name_value = VALUE_DEFAULT;
	}

	return metric_nv_info.last_name.last_name_value;
}

/******************************************************************************/
void set_last_name_metric(uint8_t value){
	metric_nv_info.last_name.last_name_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_email_address_metric(void){
	if( metric_nv_info.email_address.email_address_value == 0 || metric_nv_info.email_address.email_address_value == 0xFF ){
		metric_nv_info.email_address.email_address_value = VALUE_DEFAULT;
	}

	return metric_nv_info.email_address.email_address_value;
}

/******************************************************************************/
void set_email_address_metric(uint8_t value){
	metric_nv_info.email_address.email_address_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_age_metric(void){
	if( metric_nv_info.age.age_value == 0 || metric_nv_info.age.age_value == 0xFF ){
		metric_nv_info.age.age_value = AGE_DEFAULT_VALUE;
	}

	return metric_nv_info.age.age_value;
}

/******************************************************************************/
void set_age_metric(uint8_t value){
	metric_nv_info.age.age_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_date_of_birth_metric(void){
	if( metric_nv_info.date_of_birth.date_of_birth_value == 0 || metric_nv_info.date_of_birth.date_of_birth_value == 0xFF ){
		metric_nv_info.date_of_birth.date_of_birth_value = VALUE_DEFAULT;
	}

	return metric_nv_info.date_of_birth.date_of_birth_value;
}

/******************************************************************************/
void set_date_of_birth_metric(uint8_t value){
	metric_nv_info.date_of_birth.date_of_birth_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_gender_metric(void){
	if( metric_nv_info.gender.gender_value == 0 || metric_nv_info.gender.gender_value == 0xFF ){
		metric_nv_info.gender.gender_value = 0x00;
	}

	return metric_nv_info.gender.gender_value;
}

/******************************************************************************/
void set_gender_metric(uint8_t value){
	metric_nv_info.gender.gender_value = value;
}/******************************************************************************/
/******************************************************************************/
uint16_t get_weight_metric(void){
	if( metric_nv_info.weight.weight_value == 0 || metric_nv_info.weight.weight_value == 0xFFFF ){
		metric_nv_info.weight.weight_value = 0x3E80;
	}

	return metric_nv_info.weight.weight_value;
}

/******************************************************************************/
void set_weight_metric(uint16_t value){
	metric_nv_info.weight.weight_value = value;
}/******************************************************************************/
/******************************************************************************/
uint16_t get_height_metric(void){
	if( metric_nv_info.height.height_value == 0 || metric_nv_info.height.height_value == 0xFFFF ){
		metric_nv_info.height.height_value = 0xAF;
	}

	return metric_nv_info.height.height_value;
}

/******************************************************************************/
void set_height_metric(uint16_t value){
	metric_nv_info.height.height_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_VO2_max_metric(void){
	if( metric_nv_info.VO2_max.VO2_max_value == 0 || metric_nv_info.VO2_max.VO2_max_value == 0xFF ){
		metric_nv_info.VO2_max.VO2_max_value = VALUE_DEFAULT;
	}

	return metric_nv_info.VO2_max.VO2_max_value;
}

/******************************************************************************/
void set_VO2_max_metric(uint8_t value){
	metric_nv_info.VO2_max.VO2_max_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_heart_rate_max_metric(void){
	if( metric_nv_info.heart_rate_max.heart_rate_max_value == 0 || metric_nv_info.heart_rate_max.heart_rate_max_value == 0xFF ){
		metric_nv_info.heart_rate_max.heart_rate_max_value = VALUE_DEFAULT;
	}

	return metric_nv_info.heart_rate_max.heart_rate_max_value;
}

/******************************************************************************/
void set_heart_rate_max_metric(uint8_t value){
	metric_nv_info.heart_rate_max.heart_rate_max_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_resting_heart_rate_metric(void){
	if( metric_nv_info.resting_heart_rate.resting_heart_rate_value == 0 || metric_nv_info.resting_heart_rate.resting_heart_rate_value == 0xFF ){
		metric_nv_info.resting_heart_rate.resting_heart_rate_value = 60;
	}

	return metric_nv_info.resting_heart_rate.resting_heart_rate_value;
}

/******************************************************************************/
void set_resting_heart_rate_metric(uint8_t value){
	metric_nv_info.resting_heart_rate.resting_heart_rate_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_maximum_recommended_heart_rate_metric(void){
	if( metric_nv_info.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value == 0 || metric_nv_info.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value == 0xFF ){
		metric_nv_info.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value = VALUE_DEFAULT;
	}

	return metric_nv_info.age.age_value;
}

/******************************************************************************/
void set_maximum_recommended_heart_rate_metric(uint8_t value){
	metric_nv_info.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_aerobic_threshold_metric(void){
	if( metric_nv_info.aerobic_threshold.aerobic_threshold_value == 0 || metric_nv_info.aerobic_threshold.aerobic_threshold_value == 0xFF ){
		metric_nv_info.aerobic_threshold.aerobic_threshold_value = 0x60;
	}

	return metric_nv_info.aerobic_threshold.aerobic_threshold_value;
}

/******************************************************************************/
void set_aerobic_threshold_metric(uint8_t value){
	metric_nv_info.aerobic_threshold.aerobic_threshold_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_anaerobic_threshold_metric(void){
	if( metric_nv_info.anaerobic_threshold.anaerobic_threshold_value == 0 || metric_nv_info.anaerobic_threshold.anaerobic_threshold_value == 0xFF ){
		metric_nv_info.anaerobic_threshold.anaerobic_threshold_value = 0x80;
	}

	return metric_nv_info.anaerobic_threshold.anaerobic_threshold_value;
}

/******************************************************************************/
void set_anaerobic_threshold_metric(uint8_t value){
	metric_nv_info.anaerobic_threshold.anaerobic_threshold_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_sport_type_for_aerobic_and_anaerobic_thresholds_metric(void){
	if( metric_nv_info.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value == 0 || metric_nv_info.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value == 0xFF ){
		metric_nv_info.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value = VALUE_DEFAULT;
	}

	return metric_nv_info.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value;
}

/******************************************************************************/
void set_sport_type_for_aerobic_and_anaerobic_thresholds_metric(uint8_t value){
	metric_nv_info.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_date_of_threshold_assessment_metric(void){
	if( metric_nv_info.date_of_threshold_assessment.date_of_threshold_assessment_value == 0 || metric_nv_info.date_of_threshold_assessment.date_of_threshold_assessment_value == 0xFF ){
		metric_nv_info.date_of_threshold_assessment.date_of_threshold_assessment_value = VALUE_DEFAULT;
	}

	return metric_nv_info.date_of_threshold_assessment.date_of_threshold_assessment_value;
}

/******************************************************************************/
void set_date_of_threshold_assessment_metric(uint8_t value){
	metric_nv_info.date_of_threshold_assessment.date_of_threshold_assessment_value = value;
}/******************************************************************************/
/******************************************************************************/
uint8_t get_waist_circumference_metric(void){
	if( metric_nv_info.waist_circumference.waist_circumference_value == 0 || metric_nv_info.waist_circumference.waist_circumference_value == 0xFF ){
		metric_nv_info.waist_circumference.waist_circumference_value = VALUE_DEFAULT;
	}

	return metric_nv_info.waist_circumference.waist_circumference_value;
}

/******************************************************************************/
void set_waist_circumference_metric(uint8_t value){
	metric_nv_info.waist_circumference.waist_circumference_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_fat_burn_heart_rate_lower_limit_metric(void){
	if( metric_nv_info.fat_burn_heart_rate_lower_limit.fat_burn_heart_rate_lower_limit_value == 0 || metric_nv_info.fat_burn_heart_rate_lower_limit.fat_burn_heart_rate_lower_limit_value == 0xFF ){
		metric_nv_info.fat_burn_heart_rate_lower_limit.fat_burn_heart_rate_lower_limit_value = VALUE_DEFAULT;
	}

	return metric_nv_info.fat_burn_heart_rate_lower_limit.fat_burn_heart_rate_lower_limit_value;
}

/******************************************************************************/
void set_fat_burn_heart_rate_lower_limit_metric(uint8_t value){
	metric_nv_info.fat_burn_heart_rate_lower_limit.fat_burn_heart_rate_lower_limit_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_fat_burn_heart_rate_upper_limit_metric(void){
	if( metric_nv_info.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value == 0 || metric_nv_info.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value == 0xFF ){
		metric_nv_info.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value = VALUE_DEFAULT;
	}

	return metric_nv_info.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value;
}

/******************************************************************************/
void set_fat_burn_heart_rate_upper_limit_metric(uint8_t value){
	metric_nv_info.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_aerobic_heart_rate_lower_limit_metric(void){
	if( metric_nv_info.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value == 0 || metric_nv_info.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value == 0xFF ){
		metric_nv_info.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value = VALUE_DEFAULT;
	}

	return metric_nv_info.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value;
}

/******************************************************************************/
void set_aerobic_heart_rate_lower_limit_metric(uint8_t value){
	metric_nv_info.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_aerobic_heart_rate_upper_limit_metric(void){
	if( metric_nv_info.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value == 0 || metric_nv_info.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value == 0xFF ){
		metric_nv_info.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value = VALUE_DEFAULT;
	}

	return metric_nv_info.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value;
}

/******************************************************************************/
void set_aerobic_heart_rate_upper_limit_metric(uint8_t value){
	metric_nv_info.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_anaerobic_heart_rate_lower_limit_metric(void){
	if( metric_nv_info.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value == 0 || metric_nv_info.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value == 0xFF ){
		metric_nv_info.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value = VALUE_DEFAULT;
	}

	return metric_nv_info.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value;
}

/******************************************************************************/
void set_anaerobic_heart_rate_lower_limit_metric(uint8_t value){
	metric_nv_info.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_five_zone_heart_rate_limits_metric(void){
	if( metric_nv_info.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value == 0 || metric_nv_info.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value == 0xFF ){
		metric_nv_info.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value = VALUE_DEFAULT;
	}

	return metric_nv_info.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value;
}

/******************************************************************************/
void set_five_zone_heart_rate_limits_metric(uint8_t value){
	metric_nv_info.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_three_zone_heart_rate_limits_metric(void){
	if( metric_nv_info.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value == 0 || metric_nv_info.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value == 0xFF ){
		metric_nv_info.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value = VALUE_DEFAULT;
	}

	return metric_nv_info.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value;
}

/******************************************************************************/
void set_three_zone_heart_rate_limits_metric(uint8_t value){
	metric_nv_info.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_two_zone_heart_rate_limit_metric(void){
	if( metric_nv_info.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value == 0 || metric_nv_info.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value == 0xFF ){
		metric_nv_info.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value = VALUE_DEFAULT;
	}

	return metric_nv_info.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value;
}

/******************************************************************************/
void set_two_zone_heart_rate_limit_metric(uint8_t value){
	metric_nv_info.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_language_metric(void){
	if( metric_nv_info.language.language_value == 0 || metric_nv_info.language.language_value == 0xFF ){
		metric_nv_info.language.language_value = VALUE_DEFAULT;
	}

	return metric_nv_info.language.language_value;
}

/******************************************************************************/
void set_language_metric(uint8_t value){
	metric_nv_info.language.language_value = value;
}
/******************************************************************************/
/******************************************************************************/
uint8_t get_fitnes_index_metric(void){
	if( metric_nv_info.fitnes_index.fitnes_index_value == 0 || metric_nv_info.fitnes_index.fitnes_index_value == 0xFF ){
		metric_nv_info.fitnes_index.fitnes_index_value = 0;
	}

	return metric_nv_info.fitnes_index.fitnes_index_value;
}

/******************************************************************************/
void set_fitnes_index_metric(uint8_t value){
	metric_nv_info.fitnes_index.fitnes_index_value = value;
}

/******************************************************************************/
/******************************************************************************/
/*
 * TODO Criar outro arquivo para as funções relacionadas à zona cardíaca.
 * Começo do arquivo:
 */
/******************************************************************************/
/******************************************************************************/
#ifdef MODE_CIA_ATHLETICA
#define SPIVI_ZONE_1_THRESHOLD  		50
#define SPIVI_ZONE_2_THRESHOLD  		61
#define SPIVI_ZONE_3_THRESHOLD  		71
#define SPIVI_ZONE_4_THRESHOLD  		84
#define SPIVI_ZONE_5_THRESHOLD			92

#else
#define SPIVI_ZONE_1_THRESHOLD  		50
#define SPIVI_ZONE_2_THRESHOLD  		60
#define SPIVI_ZONE_3_THRESHOLD  		70
#define SPIVI_ZONE_4_THRESHOLD  		80
#define SPIVI_ZONE_5_THRESHOLD			90
#endif



#define RHR_RESTING_ZONE_THRESHOLD  	54
#define RHR_ZONE_1_THRESHOLD  			28
#define RHR_ZONE_2_THRESHOLD  			42
#define RHR_ZONE_3_THRESHOLD  			56
#define RHR_ZONE_4_THRESHOLD  			70
#define RHR_ZONE_5_THRESHOLD			83

/******************************************************************************/
/******************************************************************************/
uint8_t get_SPIVI_zone1_percentage_threshold(void){
	if( metric_nv_info.hr_SPIVI_zone_limits.zone1_threshold == 0 || metric_nv_info.hr_SPIVI_zone_limits.zone1_threshold == 0xFF ){
		return  SPIVI_ZONE_1_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_SPIVI_zone_limits.zone1_threshold;
	}
}

/******************************************************************************/
void set_SPIVI_zone1_percentage_threshold(uint8_t value){
	metric_nv_info.hr_SPIVI_zone_limits.zone1_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_SPIVI_zone2_percentage_threshold(void){
	if( metric_nv_info.hr_SPIVI_zone_limits.zone2_threshold == 0 || metric_nv_info.hr_SPIVI_zone_limits.zone2_threshold == 0xFF ){
		return  SPIVI_ZONE_2_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_SPIVI_zone_limits.zone2_threshold;
	}
}

/******************************************************************************/
void set_SPIVI_zone2_percentage_threshold(uint8_t value){
	metric_nv_info.hr_SPIVI_zone_limits.zone2_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_SPIVI_zone3_percentage_threshold(void){
	if( metric_nv_info.hr_SPIVI_zone_limits.zone3_threshold == 0 || metric_nv_info.hr_SPIVI_zone_limits.zone3_threshold == 0xFF ){
		return  SPIVI_ZONE_3_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_SPIVI_zone_limits.zone3_threshold;
	}
}

/******************************************************************************/
void set_SPIVI_zone3_percentage_threshold(uint8_t value){
	metric_nv_info.hr_SPIVI_zone_limits.zone3_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_SPIVI_zone4_percentage_threshold(void){
	if( metric_nv_info.hr_SPIVI_zone_limits.zone4_threshold == 0 || metric_nv_info.hr_SPIVI_zone_limits.zone4_threshold == 0xFF ){
		return  SPIVI_ZONE_4_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_SPIVI_zone_limits.zone4_threshold;
	}
}

/******************************************************************************/
void set_SPIVI_zone4_percentage_threshold(uint8_t value){
	metric_nv_info.hr_SPIVI_zone_limits.zone4_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_SPIVI_zone5_percentage_threshold(void){
	if( metric_nv_info.hr_SPIVI_zone_limits.zone5_threshold == 0 || metric_nv_info.hr_SPIVI_zone_limits.zone5_threshold == 0xFF ){
		return  SPIVI_ZONE_5_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_SPIVI_zone_limits.zone5_threshold;
	}
}

/******************************************************************************/
void set_SPIVI_zone5_percentage_threshold(uint8_t value){
	metric_nv_info.hr_SPIVI_zone_limits.zone5_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
#ifdef MODE_CIA_ATHLETICA

/*uint8_t get_hr_SPIVI_zone_1_info(void){
    return (((MAX_HR - (0.7*get_age_metric()))*get_SPIVI_zone1_percentage_threshold())/100);
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_SPIVI_zone_2_info(void){
    return (((MAX_HR - (0.7*get_age_metric()))*get_SPIVI_zone2_percentage_threshold())/100);
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_SPIVI_zone_3_info(void){
    return (((MAX_HR - (0.7*get_age_metric()))*get_SPIVI_zone3_percentage_threshold())/100);
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_SPIVI_zone_4_info(void){
    return (((MAX_HR - (0.7*get_age_metric()))*get_SPIVI_zone4_percentage_threshold())/100);
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_SPIVI_zone_5_info(void){
    return (((MAX_HR - (0.7*get_age_metric()))*get_SPIVI_zone5_percentage_threshold())/100);
}
*/
#else

/*uint8_t get_hr_SPIVI_zone_1_info(void){
    return (((MAX_HR - get_age_metric())*get_SPIVI_zone1_percentage_threshold())/100);
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_SPIVI_zone_2_info(void){
    return (((MAX_HR - get_age_metric())*get_SPIVI_zone2_percentage_threshold())/100);
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_SPIVI_zone_3_info(void){
    return (((MAX_HR - get_age_metric())*get_SPIVI_zone3_percentage_threshold())/100);
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_SPIVI_zone_4_info(void){
    return (((MAX_HR - get_age_metric())*get_SPIVI_zone4_percentage_threshold())/100);
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_SPIVI_zone_5_info(void){
    return (((MAX_HR - get_age_metric())*get_SPIVI_zone5_percentage_threshold())/100);
}
*/
#endif
/******************************************************************************/
/*
 * Paramêtros para calcular levando em consideração a zona cardíaca de repouso.
 */
/******************************************************************************/
uint8_t get_rhr_zone1_percentage_threshold(void){
	if( metric_nv_info.hr_rhr_zone_limits.zone1_threshold == 0 || metric_nv_info.hr_rhr_zone_limits.zone1_threshold == 0xFF ){
		return  RHR_ZONE_1_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_rhr_zone_limits.zone1_threshold;
	}
}

/******************************************************************************/
void set_rhr_zone1_percentage_threshold(uint8_t value){
	metric_nv_info.hr_rhr_zone_limits.zone1_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_rhr_zone2_percentage_threshold(void){
	if( metric_nv_info.hr_rhr_zone_limits.zone2_threshold == 0 || metric_nv_info.hr_rhr_zone_limits.zone2_threshold == 0xFF ){
		return  RHR_ZONE_2_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_rhr_zone_limits.zone2_threshold;
	}
}

/******************************************************************************/
void set_rhr_zone2_percentage_threshold(uint8_t value){
	metric_nv_info.hr_rhr_zone_limits.zone2_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_rhr_zone3_percentage_threshold(void){
	if( metric_nv_info.hr_rhr_zone_limits.zone3_threshold == 0 || metric_nv_info.hr_rhr_zone_limits.zone3_threshold == 0xFF ){
		return  RHR_ZONE_3_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_rhr_zone_limits.zone3_threshold;
	}
}

/******************************************************************************/
void set_rhr_zone3_percentage_threshold(uint8_t value){
	metric_nv_info.hr_rhr_zone_limits.zone3_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_rhr_zone4_percentage_threshold(void){
	if( metric_nv_info.hr_rhr_zone_limits.zone4_threshold == 0 || metric_nv_info.hr_rhr_zone_limits.zone4_threshold == 0xFF ){
		return  RHR_ZONE_4_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_rhr_zone_limits.zone4_threshold;
	}
}

/******************************************************************************/
void set_rhr_zone4_percentage_threshold(uint8_t value){
	metric_nv_info.hr_rhr_zone_limits.zone4_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
uint8_t get_rhr_zone5_percentage_threshold(void){
	if( metric_nv_info.hr_rhr_zone_limits.zone5_threshold == 0 || metric_nv_info.hr_rhr_zone_limits.zone5_threshold == 0xFF ){
		return RHR_ZONE_5_THRESHOLD;
	}
	else{
		return metric_nv_info.hr_rhr_zone_limits.zone5_threshold;
	}
}

/******************************************************************************/
void set_rhr_zone5_percentage_threshold(uint8_t value){
	metric_nv_info.hr_rhr_zone_limits.zone5_threshold = value;
}

/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_rhr_set_zone_1_info(void){
    return (((MAX_HR - get_age_metric() - get_resting_heart_rate_value_threshold())*get_rhr_zone1_percentage_threshold())/100+get_resting_heart_rate_value_threshold());
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_rhr_set_zone_2_info(void){
    return (((MAX_HR - get_age_metric() - get_resting_heart_rate_value_threshold())*get_rhr_zone2_percentage_threshold())/100+get_resting_heart_rate_value_threshold());
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_rhr_set_zone_3_info(void){
    return (((MAX_HR - get_age_metric() - get_resting_heart_rate_value_threshold())*get_rhr_zone3_percentage_threshold())/100+get_resting_heart_rate_value_threshold());
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_rhr_set_zone_4_info(void){
    return (((MAX_HR - get_age_metric() - get_resting_heart_rate_value_threshold())*get_rhr_zone4_percentage_threshold())/100+get_resting_heart_rate_value_threshold());
}
*/
/******************************************************************************/
/******************************************************************************/
/*uint8_t get_hr_rhr_set_zone_5_info(void){
    return (((MAX_HR - get_age_metric() - get_resting_heart_rate_value_threshold())*get_rhr_zone5_percentage_threshold())/100+get_resting_heart_rate_value_threshold());
}
*/
/******************************************************************************/
/******************************************************************************/
hr_module_nv_buf_t * nv_mem_buf_get(void){
	return &metric_nv_info;
}

/******************************************************************************/
/******************************************************************************/

