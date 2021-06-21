//SERVICE RESPONSIBLE FOR CALIBRATION

#ifndef BLE_RCS_H__
#define BLE_RCS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#define RCS_UUID_BASE 				{0xFB, 0x99, 0x0D, 0x1B, 0x22, 0xFF, 0x53, 0x9F, \
							     	 0xEE, 0x4D, 0xD5, 0x9B, 0x00, 0x00, 0x8E, 0x51}
#define RCS_UUID_SERVICE			0x1B6B
#define RCS_UUID_BIKE_ID_CHAR		0x74C8
#define RCS_UUID_ZERO_KG_POINT_CHAR	0x2301
#define RCS_UUID_TEN_KG_POINT_CHAR	0x2C15
#define RCS_UUID_P_LOAD_CHAR		0x2643
#define RCS_UUID_N_LOAD_CHAR		0xC161
#define RCS_UUID_ADC_RAW_CHAR		0xC3F9
#define RCS_UUID_LOAD_CHAR			0x1481
#define RCS_UUID_LOAD_T_CHAR		0x9DFA
#define RCS_UUID_ADC_UNBAL_CHAR		0x84FE
#define RCS_UUID_ADC_BAL_CHAR		0xBE5C
#define RCS_UUID_SERIAL_NUM_CHAR	0xF3C9
#define RCS_UUID_MEAN_INTERVAL_CHAR	0x847B
#define	RCS_UUID_MODE_CHAR			0x360D
										 
#define BLE_CUS_DEF(_name)                                                                          \
static calib_serv_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     2,                                                     \
                     calib_serv_on_ble_evt, &_name)

										 
										 
										 
//forward declaration of the ble_rcs_t type
typedef struct calib_serv_s calib_serv_t;
										 
//mudado348==================										 
//#define BLE_CUS_DEF(_name)                                                                          \
//static calib_serv_t _name;           
//==========================
//handler type declaration
typedef void (*calib_serv_handler_t) 	(calib_serv_t *p_rcs, uint32_t new_state);

typedef struct
{
	calib_serv_handler_t 	bike_id_write_handler;
	calib_serv_handler_t	bike_id_read_handler;	//experimental
	calib_serv_handler_t	zero_write_handler;
	calib_serv_handler_t	ten_write_handler;
	calib_serv_handler_t	p_load_write_handler;
	calib_serv_handler_t	n_load_write_handler;
	calib_serv_handler_t	serial_number_write_handler;
	calib_serv_handler_t	mean_interval_write_handler;
	calib_serv_handler_t	mode_write_handler;
} calib_serv_init_t;

struct calib_serv_s
{
	uint16_t 						service_handle;
	ble_gatts_char_handles_t		bike_id_char_handles;
	ble_gatts_char_handles_t		zero_char_handles;
	ble_gatts_char_handles_t		ten_char_handles;
	ble_gatts_char_handles_t		p_load_char_handles;
	ble_gatts_char_handles_t		n_load_char_handles;
	ble_gatts_char_handles_t		adc_raw_char_handles;
	ble_gatts_char_handles_t		load_char_handles;
	ble_gatts_char_handles_t		load_t_char_handles;
	ble_gatts_char_handles_t		adc_unbal_char_handles;
	ble_gatts_char_handles_t		adc_bal_char_handles;
	ble_gatts_char_handles_t		serial_number_char_handles;
	ble_gatts_char_handles_t		mean_interval_char_handles;
	ble_gatts_char_handles_t		mode_char_handles;
	uint8_t 						uuid_type;
	uint16_t 						conn_handle;
	calib_serv_handler_t	bike_id_write_handler;
	calib_serv_handler_t	bike_id_read_handler;	//experimental
	calib_serv_handler_t	zero_write_handler;
	calib_serv_handler_t	ten_write_handler;
	calib_serv_handler_t	p_load_write_handler;
	calib_serv_handler_t	n_load_write_handler;
	calib_serv_handler_t	serial_number_write_handler;
	calib_serv_handler_t	mean_interval_write_handler;
	calib_serv_handler_t	mode_write_handler;
};

uint32_t ble_calib_serv_init(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init);

void calib_serv_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);
                                        
uint32_t adc_raw_meas_update(calib_serv_t * p_rcs, uint8_t ADC_Meas_Addr);
uint32_t adc_raw_meas_send(calib_serv_t * p_rcs, uint16_t ADC_meas);

#endif
