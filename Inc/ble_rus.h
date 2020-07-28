#ifndef BLE_RUS_H__
#define BLE_RUS_H__


#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_RUS_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_rtcs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_RUS_DEF(_name)                                                                          \
static ble_rus_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_RUS_BLE_OBSERVER_PRIO,                                                     \
                     ble_rus_on_ble_evt, &_name)


#define RUS_UUID_BASE 				{0x17, 0xE4, 0x15, 0x39, 0x10, 0x7D, 0x04, 0xA4, \
							     	 0xA4, 0x46, 0x53, 0x39, 0x00, 0x00, 0xA1, 0x7C}
//#define RUS_UUID_SERVICE					0x0294
#define RUS_UUID_SERVICE					0x0294
//#define RUS_UUID_YEAR_OF_BIRTH_CHAR			0x60A3
#define RUS_UUID_YEAR_OF_BIRTH_CHAR			0x60A3
//#define RUS_UUID_MONTH_OF_BIRTH_CHAR		0xE994
#define RUS_UUID_MONTH_OF_BIRTH_CHAR		0xE994
//#define RUS_UUID_DAY_OF_BIRTH_CHAR			0x0805
#define RUS_UUID_DAY_OF_BIRTH_CHAR			0x0805
//#define RUS_UUID_GENDER_CHAR				0x3609
#define RUS_UUID_GENDER_CHAR				0x3609
//#define RUS_UUID_HANDEDNESS_CHAR			0xE494
#define RUS_UUID_HANDEDNESS_CHAR			0xE494
//#define RUS_UUID_HEIGHT_CHAR				0x7A2C
#define RUS_UUID_HEIGHT_CHAR				0x7A2C
//#define RUS_UUID_WEIGHT_CHAR				0x22AC
#define RUS_UUID_WEIGHT_CHAR				0x22AC
//#define RUS_UUID_AGE_CHAR					0x46F9
#define RUS_UUID_AGE_CHAR					0x46F9
//#define RUS_UUID_BODY_POSITION_CHAR			0xBB48
#define RUS_UUID_BODY_POSITION_CHAR			0xBB48
//#define RUS_UUID_RESTING_HEART_RATE_CHAR	0x6FC0
#define RUS_UUID_RESTING_HEART_RATE_CHAR	0x6FC0
//#define RUS_UUID_USER_ID_CHAR				0x90F2		//@Including USER ID
#define RUS_UUID_USER_ID_CHAR				0x90F2		//@Including USER ID
//#define RUS_UUID_AEROBIC_THRESHOLD_CHAR		0xB870		//@Including AEROBIC THRESHOLD
#define RUS_UUID_AEROBIC_THRESHOLD_CHAR		0xB870		//@Including AEROBIC THRESHOLD
//#define RUS_UUID_ANAEROBIC_THRESHOLD_CHAR	0x99BA		//@Including ANAEROBIC THRESHOLD
#define RUS_UUID_ANAEROBIC_THRESHOLD_CHAR	0x99BA		//@Including ANAEROBIC THRESHOLD
//#define RUS_UUID_NAME_CHAR					0x3407		//@Including NAME
#define RUS_UUID_NAME_CHAR					0x3407		//@Including NAME
//#define RUS_UUID_FITNESS_INDEX				0xE184		//@Including FITNESS INDEX
#define RUS_UUID_FITNESS_INDEX				0xE184		//@Including FITNESS INDEX




//forward declaration of the ble_rus_t type
typedef struct ble_rus_s ble_rus_t;

//handler type declaration
typedef void (*ble_rus_setup_handler_t) (ble_rus_t *p_rus, uint32_t new_setup);

typedef struct{
	ble_rus_setup_handler_t	year_of_birth_write_handler;
	ble_rus_setup_handler_t month_of_birth_write_handler;
	ble_rus_setup_handler_t day_of_birth_write_handler;
//	ble_rus_setup_handler_t gender_write_handler;
	ble_rus_setup_handler_t handedness_write_handler;
//	ble_rus_setup_handler_t height_write_handler;
//	ble_rus_setup_handler_t weight_write_handler;
//	ble_rus_setup_handler_t age_write_handler;
	ble_rus_setup_handler_t body_position_write_handler;
	ble_rus_setup_handler_t resting_heart_rate_write_handler;
	ble_rus_setup_handler_t user_id_write_handler;					//@Including USER ID
//	ble_rus_setup_handler_t aerobic_threshold_write_handler;		//@Including AEROBIC THRESHOLD
//	ble_rus_setup_handler_t anaerobic_threshold_write_handler;		//@Including ANAEROBIC THRESHOLD
	ble_rus_setup_handler_t name_write_handler;						//@Including NAME
	ble_rus_setup_handler_t hr_zone_preference_calc_write_handler;			//@Including FITNESS INDEX
} ble_rus_init_t;

struct ble_rus_s{
	uint16_t service_handle;
	ble_gatts_char_handles_t	year_of_birth_char_handles;
	ble_gatts_char_handles_t	month_of_birth_char_handles;
	ble_gatts_char_handles_t	day_of_birth_char_handles;
//	ble_gatts_char_handles_t	gender_char_handles;
	ble_gatts_char_handles_t	handedness_char_handles;
//	ble_gatts_char_handles_t	height_char_handles;
//	ble_gatts_char_handles_t	weight_char_handles;
//	ble_gatts_char_handles_t	age_char_handles;
	ble_gatts_char_handles_t	body_position_char_handles;
	ble_gatts_char_handles_t	resting_heart_rate_char_handles;
	ble_gatts_char_handles_t	user_id_char_handles;				//@Including USER ID
//	ble_gatts_char_handles_t	aerobic_threshold_char_handles;		//@Including AEROBIC THRESHOLD
//	ble_gatts_char_handles_t	anaerobic_threshold_char_handles;	//@Including ANAEROBIC THRESHOLD
//	ble_gatts_char_handles_t	name_char_handles;					//@Including NAME
	ble_gatts_char_handles_t	hr_zone_preference_calc_char_handles;			//@Including FITNESS INDEX
	uint8_t uuid_type;
	uint16_t conn_handle;
	ble_rus_setup_handler_t		year_of_birth_write_handler;
	ble_rus_setup_handler_t		month_of_birth_write_handler;
	ble_rus_setup_handler_t		day_of_birth_write_handler;
//	ble_rus_setup_handler_t		gender_write_handler;
	ble_rus_setup_handler_t		handedness_write_handler;
//	ble_rus_setup_handler_t		height_write_handler;
//	ble_rus_setup_handler_t		weight_write_handler;
//	ble_rus_setup_handler_t		age_write_handler;
	ble_rus_setup_handler_t 	body_position_write_handler;
	ble_rus_setup_handler_t		resting_heart_rate_write_handler;
	ble_rus_setup_handler_t		user_id_write_handler;				//@Including USER ID
//	ble_rus_setup_handler_t 	aerobic_threshold_write_handler;	//@Including AEROBIC THRESHOLD
//	ble_rus_setup_handler_t 	anaerobic_threshold_write_handler;	//@Including ANAEROBIC THRESHOLD
//	ble_rus_setup_handler_t 	name_write_handler;					//@Including NAME
	ble_rus_setup_handler_t 	hr_zone_preference_calc_write_handler;		//@Including FITNESS INDEX
};


uint32_t ble_rus_init(ble_rus_t *p_rus, const ble_rus_init_t * p_rus_init);
void ble_rus_on_ble_evt(ble_evt_t const *	p_ble_evt, void* p_context);


#endif

