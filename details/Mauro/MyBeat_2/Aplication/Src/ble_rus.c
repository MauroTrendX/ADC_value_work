/******************************************************************************/
/*
 *	Correção nos calculos e serviços do bluetooth relacionados ao cálculo do
 *	SPIVI e cálculo envolvendo o Resting Heart Rate.
 *
 *	Editor: Vítor Cruz.
 *	Data início:  13/07/18
 *	Data término: 17/07/18
 *	Número da revisão: BETA 4.
 */
/******************************************************************************/
/******************************************************************************/
/*
 *	Revisão para utilização do formato V1 do radio MyBeat.
 *
 *	Editor: Genival Ferreira.
 */
/******************************************************************************/

#include <string.h>
#include "ble_rus.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_l2cap.h"

#define OPCODE_LENGTH	1	
#define	HANDLE_LENGTH	2
#define	MAX_ADCM_LEN	(BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)

#define RUS_FLAG_MASK	(0x01 << 0);

#define APP_FEATURE_NOT_SUPPORTED	(BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2)


static void on_connect(ble_rus_t *p_rus, ble_evt_t const * p_ble_evt){
	p_rus->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_rus_t *p_rus, ble_evt_t const * p_ble_evt){
	p_rus->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_rus_t *p_rus, ble_evt_t const * p_ble_evt){
	//handle all write events for each specific service handle target.
	ble_gatts_evt_write_t const* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	//check here after compilation if the pointer will have the right structure contents.
	if((p_evt_write->handle == p_rus->user_id_char_handles.value_handle) && (p_rus->user_id_write_handler != NULL)){
		p_rus->user_id_write_handler(p_rus, p_evt_write->data[0]);          //@Including USER ID
	}
	else
	if((p_evt_write->handle == p_rus->hr_zone_preference_calc_char_handles.value_handle) && (p_rus->hr_zone_preference_calc_write_handler != NULL)){
		p_rus->hr_zone_preference_calc_write_handler(p_rus, p_evt_write->data[0]);          //@Including FITNESS INDEX
	}
	else
	if((p_evt_write->handle == p_rus->serial_number_char_handles.value_handle) && (p_rus->serial_number_write_handler != NULL)){
		p_rus->serial_number_write_handler(p_rus, (uint32_t) p_evt_write->data);          //@Including SERIAL NUMBER
	}
#ifdef MYBEAT_V1
	else
	if((p_evt_write->handle == p_rus->age_char_handles.value_handle) && (p_rus->age_write_handler != NULL)){
		p_rus->age_write_handler(p_rus, p_evt_write->data[0]);          //@Including FITNESS INDEX
	}
	else
	if((p_evt_write->handle == p_rus->resting_heart_rate_char_handles.value_handle) && (p_rus->resting_heart_rate_write_handler != NULL)){
		p_rus->resting_heart_rate_write_handler(p_rus, p_evt_write->data[0]);          //@Including FITNESS INDEX
	}	
#endif	
	
}

void ble_rus_on_ble_evt(ble_evt_t const * p_ble_evt, void* p_context){

		ble_rus_t * p_rus = (ble_rus_t *)p_context;

		switch(p_ble_evt->header.evt_id){
				case BLE_GAP_EVT_CONNECTED:
						on_connect(p_rus, p_ble_evt);
						break;
				case BLE_GAP_EVT_DISCONNECTED:
						on_disconnect(p_rus, p_ble_evt);
						break;		
				case BLE_GATTS_EVT_WRITE:
						on_write(p_rus, p_ble_evt);
						break;
				default:		
						break;
	}
}



#ifdef MYBEAT_V1
static uint32_t age_char_add(ble_rus_t *p_rus, const ble_rus_init_t *p_rus_init){
	
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;
	
	memset(&char_md, 0, sizeof(char_md));
	
	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf			= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md			= NULL;
	char_md.p_sccd_md			= NULL;
	
	ble_uuid.type = p_rus->uuid_type;
	ble_uuid.uuid = RUS_UUID_AGE_CHAR;
	
	memset(&attr_md, 0, sizeof(attr_md));
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc	= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth	= 1;
	attr_md.wr_auth	= 0;
	attr_md.vlen	= 1;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	
//	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_AGE_CHAR);

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= sizeof(uint8_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= sizeof(uint32_t);
	attr_char_value.p_value		= NULL;
	
	return sd_ble_gatts_characteristic_add(p_rus->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rus->age_char_handles);
}



static uint32_t resting_heart_rate_char_add(ble_rus_t *p_rus, const ble_rus_init_t *p_rus_init){
	
	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;
	
	memset(&char_md, 0, sizeof(char_md));
	
	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf			= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md			= NULL;
	char_md.p_sccd_md			= NULL;
	
	ble_uuid.type = p_rus->uuid_type;
	ble_uuid.uuid = RUS_UUID_RESTING_HEART_RATE_CHAR;

//	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_RESTING_HEART_RATE_CHAR);
	
	memset(&attr_md, 0, sizeof(attr_md));
	
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc	= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth	= 1;
	attr_md.wr_auth	= 0;
	attr_md.vlen	= 1;
	
	memset(&attr_char_value, 0, sizeof(attr_char_value));
	
	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= sizeof(uint8_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= sizeof(uint32_t);
	attr_char_value.p_value		= NULL;
	
	return sd_ble_gatts_characteristic_add(p_rus->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rus->resting_heart_rate_char_handles);
}
#endif
//@Including USER ID
static uint32_t user_id_char_add(ble_rus_t *p_rus, const ble_rus_init_t *p_rus_init){

	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf			= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md			= NULL;
	char_md.p_sccd_md			= NULL;

	ble_uuid.type = p_rus->uuid_type;
	ble_uuid.uuid = RUS_UUID_USER_ID_CHAR;

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc	= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth	= 1;
	attr_md.wr_auth	= 0;
	attr_md.vlen	= 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= sizeof(uint8_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= sizeof(uint32_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_rus->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rus->user_id_char_handles);
}

static uint32_t hr_zone_preference_calc_char_add(ble_rus_t *p_rus, const ble_rus_init_t *p_rus_init){

	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf			= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md			= NULL;
	char_md.p_sccd_md			= NULL;

	ble_uuid.type = p_rus->uuid_type;
	ble_uuid.uuid = RUS_UUID_FITNESS_INDEX;

//	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_USER_INDEX_CHAR);

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc	= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth	= 1;
	attr_md.wr_auth	= 0;
	attr_md.vlen	= 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= sizeof(uint8_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= sizeof(uint32_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_rus->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rus->hr_zone_preference_calc_char_handles);
}

//@Including SERIAL NUMBER
static uint32_t serial_number_char_add(ble_rus_t *p_rus, const ble_rus_init_t *p_rus_init){

	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf			= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md			= NULL;
	char_md.p_sccd_md			= NULL;

	ble_uuid.type = p_rus->uuid_type;
	ble_uuid.uuid = RUS_UUID_SERIAL_NUMBER;

//	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_USER_INDEX_CHAR);

	memset(&attr_md, 0, sizeof(attr_md));

	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	attr_md.vloc	= BLE_GATTS_VLOC_STACK;
	attr_md.rd_auth	= 1;
	attr_md.wr_auth	= 0;
	attr_md.vlen	= 1;

	memset(&attr_char_value, 0, sizeof(attr_char_value));

	attr_char_value.p_uuid		= &ble_uuid;
	attr_char_value.p_attr_md	= &attr_md;
	attr_char_value.init_len	= sizeof(uint8_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= 12;
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_rus->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rus->serial_number_char_handles);
}

uint32_t ble_rus_init(ble_rus_t *p_rus, const ble_rus_init_t *p_rus_init){
	
	uint32_t 	err_code;
	ble_uuid_t 	ble_uuid;
	
	//Initalize service structure.
	p_rus->conn_handle															= BLE_CONN_HANDLE_INVALID;

#ifdef MYBEAT_V1
	p_rus->age_write_handler												= p_rus_init->age_write_handler;
	p_rus->resting_heart_rate_write_handler					= p_rus_init->resting_heart_rate_write_handler;
	p_rus->user_id_write_handler										= p_rus_init->user_id_write_handler;   				//@Including USER ID
	p_rus->hr_zone_preference_calc_write_handler		= p_rus_init->hr_zone_preference_calc_write_handler;			//@Including FITNESS INDEX
	p_rus->serial_number_write_handler 							= p_rus_init->serial_number_write_handler;
#else
	p_rus->user_id_write_handler										= p_rus_init->user_id_write_handler;   				//@Including USER ID
	p_rus->hr_zone_preference_calc_write_handler		= p_rus_init->hr_zone_preference_calc_write_handler;			//@Including FITNESS INDEX
	p_rus->serial_number_write_handler 							= p_rus_init->serial_number_write_handler;
#endif	
	//Add service
	ble_uuid128_t base_uuid = {RUS_UUID_BASE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_rus->uuid_type);
	volatile uint8_t temp=err_code;
	if(err_code != NRF_SUCCESS){
		return err_code;
	}
	
	ble_uuid.type = p_rus->uuid_type;
	ble_uuid.uuid = RUS_UUID_SERVICE;

//	BLE_UUID_BLE_ASSIGN(ble_uuid, RUS_UUID_SERVICE);
	
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_rus->service_handle);
	if(err_code != NRF_SUCCESS){
		return err_code;
	}

	//Add characteristics.
#ifdef MYBEAT_V1

	err_code = age_char_add(p_rus, p_rus_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
		
	err_code = resting_heart_rate_char_add(p_rus, p_rus_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
		
#endif
	err_code = user_id_char_add(p_rus, p_rus_init);
		if(err_code != NRF_SUCCESS)
			return err_code;			//@Including USER ID

	err_code = hr_zone_preference_calc_char_add(p_rus, p_rus_init);
		if(err_code != NRF_SUCCESS)
			return err_code;			//@Including FITNESS INDEX

	err_code = serial_number_char_add(p_rus, p_rus_init);
		if(err_code != NRF_SUCCESS)
			return err_code;			//@Including FITNESS INDEX

		return NRF_SUCCESS;
}