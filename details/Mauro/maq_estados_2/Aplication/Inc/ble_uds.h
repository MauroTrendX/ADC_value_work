#ifndef BLE_UDS_H__
#define BLE_UDS_H__


#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_UDS_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_rtcs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_UDS_DEF(_name)                                                                          \
static ble_uds_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_UDS_BLE_OBSERVER_PRIO,                                                     \
                     ble_uds_on_ble_evt, &_name)



#define BLE_UUID_USER_DATA_SERVICE																			0x181C

#define BLE_UUID_FIRST_NAME_CHAR 																				0x2A8A
#define BLE_UUID_LAST_NAME_CHAR 																				0x2A90
#define BLE_UUID_EMAIL_ADDRESS 																					0x2A87
#define BLE_UUID_AGE_CHAR 																							0x2A80
#define BLE_UUID_DATE_OF_BIRTH_CHAR 																		0x2A85
#define BLE_UUID_GENDER_CHAR 																						0x2A8C
#define BLE_UUID_WEIGHT_CHAR 																						0x2A98
#define BLE_UUID_HEIGHT_CHAR 																						0x2A8E
#define BLE_UUID_VO2_MAX_CHAR 																					0x2A96
#define BLE_UUID_HEART_RATE_MAX_CHAR 																		0x2A8D
#define BLE_UUID_RESTING_HEART_RATE_CHAR 																0x2A92
#define BLE_UUID_MAXIMUM_RECOMMEENDED_CHAR 															0x2A91
#define BLE_UUID_AEROBIC_THRESHOLD_CHAR 																0x2A7F
#define BLE_UUID_ANAEROBIC_THRESHOLD_CHAR 															0x2A83
#define BLE_UUID_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLD_CHAR 		0x2A93
#define BLE_UUID_DATE_OF_THRESHOLD_ASSESSMENT_CHAR 											0x2A86
#define BLE_UUID_WAIST_CIRCUMFERENCE_CHAR 															0x2A97
#define BLE_UUID_HIP_CIRCUMFERENCE_CHAR 																0x2A8F
#define BLE_UUID_FAT_BURN_HEART_RATE_LOWER_LIMIT_CHAR 									0x2A88
#define BLE_UUID_FAT_BURN_HEART_RATE_UPPER_LIMIT_CHAR 									0x2A89
#define BLE_UUID_AEROBIC_HEART_RATE_LOWER_LIMIT_CHAR 										0x2A7E
#define BLE_UUID_AEROBIC_HEART_RATE_UPPER_LIMIT_CHAR 										0x2A84
#define BLE_UUID_ANAEROBIC_HEART_RATE_LOWER_LIMIT_CHAR 									0x2A81
#define BLE_UUID_ANAEROBIC_HEART_RATE_UPPER_LIMIT_CHAR 									0x2A82
#define BLE_UUID_FIVE_ZONE_HEART_RATE_LIMITS_CHAR 											0x2A8B
#define BLE_UUID_THREE_ZONE_HEART_RATE_LIMITS_CHAR 											0x2A94
#define BLE_UUID_TWO_ZONE_HEART_RATE_LIMITS_CHAR 												0x2A95
#define BLE_UUID_LANGUAGE_CHAR 																					0x2AA2




//forward declaration of the ble_rus_t type
typedef struct ble_uds_s ble_uds_t;

//handler type declaration
typedef void (*ble_uds_setup_handler_t) (ble_uds_t *p_uds, uint32_t new_setup);
typedef void (*ble_uds_setup_handler_64_t) (ble_uds_t *p_uds, uint64_t new_setup);

typedef struct{
//	ble_srv_utf8_str_t	first_name_write_handler;
//	ble_uds_setup_handler_t	first_name_write_handler;
	ble_uds_setup_handler_64_t	first_name_write_handler;
	ble_uds_setup_handler_t last_name_write_handler;
	ble_uds_setup_handler_t email_address_write_handler;
	ble_uds_setup_handler_t age_write_handler;
	ble_uds_setup_handler_t date_of_birth_write_handler;
	ble_uds_setup_handler_t gender_write_handler;
	ble_uds_setup_handler_t weight_write_handler;
	ble_uds_setup_handler_t height_write_handler;
	ble_uds_setup_handler_t VO2_max_write_handler;
	ble_uds_setup_handler_t heart_rate_max_write_handler;
	ble_uds_setup_handler_t resting_heart_write_handler;
	ble_uds_setup_handler_t maximum_recommended_heart_rate_write_handler;
	ble_uds_setup_handler_t aerobic_threshold_write_handler;
	ble_uds_setup_handler_t anaerobic_threshold_write_handler;
	ble_uds_setup_handler_t sport_type_for_aerobic_and_anaerobic_thresholds_write_handler;
	ble_uds_setup_handler_t date_of_threshold_assessment_write_handler;
	ble_uds_setup_handler_t waist_circumference_write_handler;
	ble_uds_setup_handler_t fat_burn_heart_rate_lower_limit_write_handler;
	ble_uds_setup_handler_t fat_burn_heart_rate_upper_limit_write_handler;
	ble_uds_setup_handler_t aerobic_heart_rate_lower_limit_write_handler;
	ble_uds_setup_handler_t aerobic_heart_rate_upper_limit_write_handler;
	ble_uds_setup_handler_t anaerobic_heart_rate_lower_limit_write_handler;
	ble_uds_setup_handler_t five_zone_heart_rate_limits_write_handler;
	ble_uds_setup_handler_t three_zone_heart_rate_limits_write_handler;
	ble_uds_setup_handler_t two_zone_heart_rate_limits_write_handler;
	ble_uds_setup_handler_t language_write_handler;
	ble_uds_setup_handler_t fitnes_index_write_handler;
} ble_uds_init_t;

struct ble_uds_s{
	uint16_t service_handle;
	ble_gatts_char_handles_t first_name_char_handler;
	ble_gatts_char_handles_t last_name_char_handler;
	ble_gatts_char_handles_t email_address_char_handler;
	ble_gatts_char_handles_t age_char_handler;
	ble_gatts_char_handles_t date_of_birth_char_handler;
	ble_gatts_char_handles_t gender_char_handler;
	ble_gatts_char_handles_t weight_char_handler;
	ble_gatts_char_handles_t height_char_handler;
	ble_gatts_char_handles_t VO2_max_char_handler;
	ble_gatts_char_handles_t heart_rate_max_char_handler;
	ble_gatts_char_handles_t resting_heart_char_handler;
	ble_gatts_char_handles_t maximum_recommended_heart_rate_char_handler;
	ble_gatts_char_handles_t aerobic_threshold_char_handler;
	ble_gatts_char_handles_t anaerobic_threshold_char_handler;
	ble_gatts_char_handles_t sport_type_for_aerobic_and_anaerobic_thresholds_char_handler;
	ble_gatts_char_handles_t date_of_threshold_assessment_char_handler;
	ble_gatts_char_handles_t waist_circumference_char_handler;
	ble_gatts_char_handles_t hip_circumference_char_handler;
	ble_gatts_char_handles_t fat_burn_heart_rate_lower_limit_char_handler;
	ble_gatts_char_handles_t fat_burn_heart_rate_upper_limit_char_handler;
	ble_gatts_char_handles_t aerobic_heart_rate_lower_limit_char_handler;
	ble_gatts_char_handles_t aerobic_heart_rate_upper_limit_char_handler;
	ble_gatts_char_handles_t anaerobic_heart_rate_lower_limit_char_handler;
	ble_gatts_char_handles_t anaerobic_heart_rate_upper_limit_char_handler;
	ble_gatts_char_handles_t five_zone_heart_rate_limits_char_handler;
	ble_gatts_char_handles_t three_zone_heart_rate_limits_char_handler;
	ble_gatts_char_handles_t two_zone_heart_rate_limits_char_handler;
	ble_gatts_char_handles_t language_char_handler;
	ble_gatts_char_handles_t fitnes_index_char_handler;
	uint8_t uuid_type;
	uint16_t conn_handle;
	ble_uds_setup_handler_64_t	first_name_write_handler;
	ble_uds_setup_handler_t last_name_write_handler;
	ble_uds_setup_handler_t email_address_write_handler;
	ble_uds_setup_handler_t age_write_handler;
	ble_uds_setup_handler_t date_of_birth_write_handler;
	ble_uds_setup_handler_t gender_write_handler;
	ble_uds_setup_handler_t weight_write_handler;
	ble_uds_setup_handler_t height_write_handler;
	ble_uds_setup_handler_t VO2_max_write_handler;
	ble_uds_setup_handler_t heart_rate_max_write_handler;
	ble_uds_setup_handler_t resting_heart_write_handler;
	ble_uds_setup_handler_t maximum_recommended_heart_rate_write_handler;
	ble_uds_setup_handler_t aerobic_threshold_write_handler;
	ble_uds_setup_handler_t anaerobic_threshold_write_handler;
	ble_uds_setup_handler_t sport_type_for_aerobic_and_anaerobic_thresholds_write_handler;
	ble_uds_setup_handler_t date_of_threshold_assessment_write_handler;
	ble_uds_setup_handler_t waist_circumference_write_handler;
	ble_uds_setup_handler_t fat_burn_heart_rate_lower_limit_write_handler;
	ble_uds_setup_handler_t fat_burn_heart_rate_upper_limit_write_handler;
	ble_uds_setup_handler_t aerobic_heart_rate_lower_limit_write_handler;
	ble_uds_setup_handler_t aerobic_heart_rate_upper_limit_write_handler;
	ble_uds_setup_handler_t anaerobic_heart_rate_lower_limit_write_handler;
	ble_uds_setup_handler_t five_zone_heart_rate_limits_write_handler;
	ble_uds_setup_handler_t three_zone_heart_rate_limits_write_handler;
	ble_uds_setup_handler_t two_zone_heart_rate_limits_write_handler;
	ble_uds_setup_handler_t language_write_handler;
	ble_uds_setup_handler_t fitnes_index_write_handler;
};


uint32_t ble_uds_init(ble_uds_t *p_uds, const ble_uds_init_t * p_uds_init);
void ble_uds_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);


#endif
