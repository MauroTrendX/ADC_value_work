#ifndef BLE_RTCS
#define BLE_RTCS

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define BLE_RTCS_BLE_OBSERVER_PRIO 2

/**@brief   Macro for defining a ble_rtcs instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_RTCS_DEF(_name)                                                                          \
static ble_rtcs_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_RTCS_BLE_OBSERVER_PRIO,                                                     \
                     ble_rtcs_on_ble_evt, &_name)



#define RTCS_UUID_BASE				{0xA8, 0xE4, 0x21, 0x08, 0x72, 0x9B, 0x4D, 0x91, \
							     	 0xFF, 0x48, 0x5B, 0xCB, 0x00, 0x00, 0x66, 0xCF}

#define RTCS_UUID_SERVICE						0x6B71
#define RTCS_UUID_SPIVI_ZONE_1_THRESHOLD_CHAR	0xA83D
#define RTCS_UUID_SPIVI_ZONE_2_THRESHOLD_CHAR	0xE2E8
#define RTCS_UUID_SPIVI_ZONE_3_THRESHOLD_CHAR	0x918A
#define RTCS_UUID_SPIVI_ZONE_4_THRESHOLD_CHAR	0xC122
#define RTCS_UUID_SPIVI_ZONE_5_THRESHOLD_CHAR	0x2D06

#define RTCS_UUID_RHR_ZONE_1_THRESHOLD_CHAR		0xDC48
#define RTCS_UUID_RHR_ZONE_2_THRESHOLD_CHAR		0x088F
#define RTCS_UUID_RHR_ZONE_3_THRESHOLD_CHAR		0xA915
#define RTCS_UUID_RHR_ZONE_4_THRESHOLD_CHAR		0x98DB
#define RTCS_UUID_RHR_ZONE_5_THRESHOLD_CHAR		0xC6BA

//forward declaration of the ble_rtcs_t type
typedef struct ble_rtcs_s	ble_rtcs_t;

//handler type declaration
typedef void (*ble_rtcs_setup_handler_t) (ble_rtcs_t *p_rtcs, uint32_t new_setup);

typedef struct{
	ble_rtcs_setup_handler_t SPIVI_zone1_threshold_write_handler;
	ble_rtcs_setup_handler_t SPIVI_zone2_threshold_write_handler;
	ble_rtcs_setup_handler_t SPIVI_zone3_threshold_write_handler;
	ble_rtcs_setup_handler_t SPIVI_zone4_threshold_write_handler;
	ble_rtcs_setup_handler_t SPIVI_zone5_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone1_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone2_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone3_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone4_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone5_threshold_write_handler;
} ble_rtcs_init_t;

struct ble_rtcs_s{
	ble_gatts_char_handles_t SPIVI_zone1_threshold_char_handles;
	ble_gatts_char_handles_t SPIVI_zone2_threshold_char_handles;
	ble_gatts_char_handles_t SPIVI_zone3_threshold_char_handles;
	ble_gatts_char_handles_t SPIVI_zone4_threshold_char_handles;
	ble_gatts_char_handles_t SPIVI_zone5_threshold_char_handles;
	ble_rtcs_setup_handler_t SPIVI_zone1_threshold_write_handler;
	ble_rtcs_setup_handler_t SPIVI_zone2_threshold_write_handler;
	ble_rtcs_setup_handler_t SPIVI_zone3_threshold_write_handler;
	ble_rtcs_setup_handler_t SPIVI_zone4_threshold_write_handler;
	ble_rtcs_setup_handler_t SPIVI_zone5_threshold_write_handler;
	ble_gatts_char_handles_t rhr_zone1_threshold_char_handles;
	ble_gatts_char_handles_t rhr_zone2_threshold_char_handles;
	ble_gatts_char_handles_t rhr_zone3_threshold_char_handles;
	ble_gatts_char_handles_t rhr_zone4_threshold_char_handles;
	ble_gatts_char_handles_t rhr_zone5_threshold_char_handles;
	ble_rtcs_setup_handler_t rhr_zone1_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone2_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone3_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone4_threshold_write_handler;
	ble_rtcs_setup_handler_t rhr_zone5_threshold_write_handler;
	uint16_t conn_handle;
	uint16_t service_handle;
	uint8_t uuid_type;
};

uint32_t ble_rtcs_init(ble_rtcs_t *p_rtcs, const ble_rtcs_init_t * p_rtcs_init);
void ble_rtcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);

#endif	//BLE_RTCS
