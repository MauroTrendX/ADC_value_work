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
#include <string.h>
#include "ble_uds.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_l2cap.h"

#define OPCODE_LENGTH	1
#define	HANDLE_LENGTH	2
#define	MAX_ADCM_LEN	(BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)

#define UDS_FLAG_MASK	(0x01 << 0);

#define APP_FEATURE_NOT_SUPPORTED	(BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2)

static void on_connect(ble_uds_t *p_uds,  ble_evt_t const * p_ble_evt){
	p_uds->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_uds_t *p_uds,  ble_evt_t const * p_ble_evt){
	p_uds->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_uds_t *p_uds,  ble_evt_t const * p_ble_evt){
	//handle all write events for each specific service handle target.
	ble_gatts_evt_write_t const* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	//check here after compilation if the pointer will have the right structure contents.
//	if((p_evt_write->handle == p_uds->first_name_char_handler.value_handle) && (p_uds->first_name_write_handler != NULL)){
//		p_uds->first_name_write_handler(p_uds, (p_evt_write->data[0]<<24)|(p_evt_write->data[1]<<16)|(p_evt_write->data[2]<<8)|p_evt_write->data[3]);
//	}

	if((p_evt_write->handle == p_uds->first_name_char_handler.value_handle) && (p_uds->first_name_write_handler != NULL)){

		memset((void*)&p_evt_write->data[p_evt_write->len],0,(sizeof(uint8_t)*(8-p_evt_write->len)));


		p_uds->first_name_write_handler(p_uds, 	((uint64_t)p_evt_write->data[0]<<56)|
												((uint64_t)p_evt_write->data[1]<<48)|
												((uint64_t)p_evt_write->data[2]<<40)|
												((uint64_t)p_evt_write->data[3]<<32)|
												((uint64_t)p_evt_write->data[4]<<24)|
												((uint64_t)p_evt_write->data[5]<<16)|
												((uint64_t)p_evt_write->data[6]<< 8)|
												((uint64_t)p_evt_write->data[7]<< 0));
	}

	else
	if((p_evt_write->handle == p_uds->last_name_char_handler.value_handle) && (p_uds->last_name_write_handler != NULL)){
		p_uds->last_name_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->email_address_char_handler.value_handle) && (p_uds->email_address_write_handler != NULL)){
		p_uds->email_address_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->age_char_handler.value_handle) && (p_uds->age_write_handler != NULL)){
		p_uds->age_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
//	if((p_evt_write->handle == p_uds->first_name_char_handler.value_handle) && (p_uds->first_name_write_handler != NULL)){
//		p_uds->first_name_write_handler(p_uds, p_evt_write->data[0]);
//	}
//	else
	if((p_evt_write->handle == p_uds->date_of_birth_char_handler.value_handle) && (p_uds->date_of_birth_write_handler != NULL)){
		p_uds->date_of_birth_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->gender_char_handler.value_handle) && (p_uds->gender_write_handler != NULL)){
		p_uds->gender_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->weight_char_handler.value_handle) && (p_uds->weight_write_handler != NULL)){
		p_uds->weight_write_handler(p_uds, (p_evt_write->data[0])|(p_evt_write->data[1]<<8));
	}
	else
	if((p_evt_write->handle == p_uds->height_char_handler.value_handle) && (p_uds->height_write_handler != NULL)){
		p_uds->height_write_handler(p_uds, (p_evt_write->data[0])|(p_evt_write->data[1]<<8));
	}
	else
	if((p_evt_write->handle == p_uds->VO2_max_char_handler.value_handle) && (p_uds->VO2_max_write_handler != NULL)){
		p_uds->VO2_max_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->heart_rate_max_char_handler.value_handle) && (p_uds->heart_rate_max_write_handler != NULL)){
		p_uds->heart_rate_max_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->heart_rate_max_char_handler.value_handle) && (p_uds->heart_rate_max_write_handler != NULL)){
		p_uds->heart_rate_max_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->resting_heart_char_handler.value_handle) && (p_uds->resting_heart_write_handler != NULL)){
		p_uds->resting_heart_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->maximum_recommended_heart_rate_char_handler.value_handle) && (p_uds->maximum_recommended_heart_rate_write_handler != NULL)){
		p_uds->maximum_recommended_heart_rate_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->aerobic_threshold_char_handler.value_handle) && (p_uds->aerobic_threshold_write_handler != NULL)){
		p_uds->aerobic_threshold_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->anaerobic_threshold_char_handler.value_handle) && (p_uds->anaerobic_threshold_write_handler != NULL)){
		p_uds->anaerobic_threshold_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->sport_type_for_aerobic_and_anaerobic_thresholds_char_handler.value_handle) && (p_uds->sport_type_for_aerobic_and_anaerobic_thresholds_write_handler != NULL)){
		p_uds->sport_type_for_aerobic_and_anaerobic_thresholds_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->date_of_threshold_assessment_char_handler.value_handle) && (p_uds->date_of_threshold_assessment_write_handler != NULL)){
		p_uds->date_of_threshold_assessment_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->waist_circumference_char_handler.value_handle) && (p_uds->waist_circumference_write_handler != NULL)){
		p_uds->waist_circumference_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->fat_burn_heart_rate_lower_limit_char_handler.value_handle) && (p_uds->fat_burn_heart_rate_lower_limit_write_handler != NULL)){
		p_uds->fat_burn_heart_rate_lower_limit_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->fat_burn_heart_rate_upper_limit_char_handler.value_handle) && (p_uds->fat_burn_heart_rate_upper_limit_write_handler != NULL)){
		p_uds->fat_burn_heart_rate_upper_limit_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->aerobic_heart_rate_lower_limit_char_handler.value_handle) && (p_uds->aerobic_heart_rate_lower_limit_write_handler != NULL)){
		p_uds->aerobic_heart_rate_lower_limit_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->aerobic_heart_rate_upper_limit_char_handler.value_handle) && (p_uds->aerobic_heart_rate_upper_limit_write_handler != NULL)){
		p_uds->aerobic_heart_rate_upper_limit_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->anaerobic_heart_rate_lower_limit_char_handler.value_handle) && (p_uds->anaerobic_heart_rate_lower_limit_write_handler != NULL)){
		p_uds->anaerobic_heart_rate_lower_limit_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->five_zone_heart_rate_limits_char_handler.value_handle) && (p_uds->five_zone_heart_rate_limits_write_handler != NULL)){
		p_uds->five_zone_heart_rate_limits_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->three_zone_heart_rate_limits_char_handler.value_handle) && (p_uds->three_zone_heart_rate_limits_write_handler != NULL)){
		p_uds->three_zone_heart_rate_limits_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->two_zone_heart_rate_limits_char_handler.value_handle) && (p_uds->two_zone_heart_rate_limits_write_handler != NULL)){
		p_uds->two_zone_heart_rate_limits_write_handler(p_uds, p_evt_write->data[0]);
	}
	else
	if((p_evt_write->handle == p_uds->language_char_handler.value_handle) && (p_uds->language_write_handler != NULL)){
		p_uds->language_write_handler(p_uds, p_evt_write->data[0]);
	}


	else
	if((p_evt_write->handle == p_uds->fitnes_index_char_handler.value_handle) && (p_uds->fitnes_index_write_handler != NULL)){
		p_uds->fitnes_index_write_handler(p_uds, p_evt_write->data[0]);
	}


}

void ble_uds_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context){
	
	ble_uds_t * p_uds = (ble_uds_t *)p_context;
	
	switch(p_ble_evt->header.evt_id){
		case BLE_GAP_EVT_CONNECTED:
			on_connect(p_uds, p_ble_evt);
			break;
		case BLE_GAP_EVT_DISCONNECTED:
			on_disconnect(p_uds, p_ble_evt);
			break;
		case BLE_GATTS_EVT_WRITE:
			on_write(p_uds, p_ble_evt);
			break;
		default:
			break;
	}
}


static uint32_t first_name_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_YEAR_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, /*0x1986);*/BLE_UUID_FIRST_NAME_CHAR);

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
	attr_char_value.init_len	= sizeof(uint16_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= sizeof(uint64_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->first_name_char_handler);
}


/*static uint32_t first_name_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_YEAR_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_FIRST_NAME_CHAR);

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
	attr_char_value.init_len	= sizeof(uint16_t);
	attr_char_value.init_offs	= 0;
	attr_char_value.max_len		= sizeof(uint32_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->first_name_char_handler);
}*/

static uint32_t last_name_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_LAST_NAME_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->last_name_char_handler);
}

static uint32_t email_address_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_EMAIL_ADDRESS);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->email_address_char_handler);
}

static uint32_t age_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_AGE_CHAR);

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
	attr_char_value.max_len		= sizeof(uint8_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->age_char_handler);
}

static uint32_t date_of_birth_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DATE_OF_BIRTH_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->date_of_birth_char_handler);
}

static uint32_t gender_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_GENDER_CHAR);

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
	attr_char_value.max_len		= sizeof(uint8_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->gender_char_handler);
}

static uint32_t weight_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WEIGHT_CHAR);

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
	attr_char_value.max_len		= sizeof(uint16_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->weight_char_handler);
}

static uint32_t height_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEIGHT_CHAR);

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
	attr_char_value.max_len		= sizeof(uint16_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->height_char_handler);
}

static uint32_t VO2_max_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_VO2_MAX_CHAR);

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
	attr_char_value.max_len		= 3*(sizeof(uint8_t));
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->VO2_max_char_handler);
}


static uint32_t heart_rate_max_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HEART_RATE_MAX_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->heart_rate_max_char_handler);
}

static uint32_t resting_heart_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_RESTING_HEART_RATE_CHAR);

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
	attr_char_value.max_len		= sizeof(uint8_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->resting_heart_char_handler);
}

static uint32_t maximum_recommended_heart_rate_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_MAXIMUM_RECOMMEENDED_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->maximum_recommended_heart_rate_char_handler);
}

static uint32_t aerobic_threshold_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_AEROBIC_THRESHOLD_CHAR);

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
	attr_char_value.max_len		= sizeof(uint8_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->aerobic_threshold_char_handler);
}

static uint32_t anaerobic_threshold_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ANAEROBIC_THRESHOLD_CHAR);

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
	attr_char_value.max_len		= sizeof(uint8_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->anaerobic_threshold_char_handler);
}

static uint32_t sport_type_for_aerobic_and_anaerobic_thresholds_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_SPORT_TYPE_FOR_AEROBIC_AND_ANAEROBIC_THRESHOLD_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->sport_type_for_aerobic_and_anaerobic_thresholds_char_handler);
}

static uint32_t date_of_threshold_assessment_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_DATE_OF_THRESHOLD_ASSESSMENT_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->date_of_threshold_assessment_char_handler);
}

static uint32_t waist_circumference_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_WAIST_CIRCUMFERENCE_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->waist_circumference_char_handler);
}

static uint32_t hip_circumference_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_HIP_CIRCUMFERENCE_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->hip_circumference_char_handler);
}

static uint32_t fat_burn_heart_rate_lower_limit_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_FAT_BURN_HEART_RATE_LOWER_LIMIT_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->fat_burn_heart_rate_lower_limit_char_handler);
}

static uint32_t fat_burn_heart_rate_upper_limit_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_FAT_BURN_HEART_RATE_UPPER_LIMIT_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->fat_burn_heart_rate_upper_limit_char_handler);
}

static uint32_t aerobic_heart_rate_lower_limit_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_AEROBIC_HEART_RATE_LOWER_LIMIT_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->aerobic_heart_rate_lower_limit_char_handler);
}

static uint32_t aerobic_heart_rate_upper_limit_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_AEROBIC_HEART_RATE_UPPER_LIMIT_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->aerobic_heart_rate_upper_limit_char_handler);
}

static uint32_t anaerobic_heart_rate_lower_limit_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ANAEROBIC_HEART_RATE_LOWER_LIMIT_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->anaerobic_heart_rate_lower_limit_char_handler);
}

static uint32_t anaerobic_heart_rate_upper_limit_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

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

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_ANAEROBIC_HEART_RATE_UPPER_LIMIT_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->anaerobic_heart_rate_upper_limit_char_handler);
}

static uint32_t five_zone_heart_rate_limits_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf					= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md					= NULL;
	char_md.p_sccd_md					= NULL;

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_FIVE_ZONE_HEART_RATE_LIMITS_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->five_zone_heart_rate_limits_char_handler);
}

static uint32_t three_zone_heart_rate_limits_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf					= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md					= NULL;
	char_md.p_sccd_md					= NULL;

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_THREE_ZONE_HEART_RATE_LIMITS_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->three_zone_heart_rate_limits_char_handler);
}

static uint32_t two_zone_heart_rate_limit_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf					= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md					= NULL;
	char_md.p_sccd_md					= NULL;

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_TWO_ZONE_HEART_RATE_LIMITS_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->two_zone_heart_rate_limits_char_handler);
}

static uint32_t language_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf					= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md					= NULL;
	char_md.p_sccd_md					= NULL;

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = RUS_UUID_MONTH_OF_BIRTH_CHAR;

	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_LANGUAGE_CHAR);

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

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->language_char_handler);
}

//@Including FITNESS INDEX
static uint32_t fitnes_index_char_add(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

	ble_gatts_char_md_t char_md;
	ble_gatts_attr_t	attr_char_value;
	ble_uuid_t			ble_uuid;
	ble_gatts_attr_md_t	attr_md;

	memset(&char_md, 0, sizeof(char_md));

	char_md.char_props.read		= 1;
	char_md.char_props.write 	= 1;
	char_md.p_char_user_desc	= NULL;
	char_md.p_char_pf					= NULL;
	char_md.p_user_desc_md		= NULL;
	char_md.p_cccd_md					= NULL;
	char_md.p_sccd_md					= NULL;

//	ble_uuid.type = p_uds->uuid_type;
//	ble_uuid.uuid = 0xE184;

	BLE_UUID_BLE_ASSIGN(ble_uuid, 0xE184);

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
	attr_char_value.max_len		= sizeof(uint8_t);
	attr_char_value.p_value		= NULL;

	return sd_ble_gatts_characteristic_add(p_uds->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_uds->fitnes_index_char_handler);
}

uint32_t ble_uds_init(ble_uds_t *p_uds, const ble_uds_init_t *p_uds_init){

	uint32_t 	err_code;
	ble_uuid_t 	ble_uuid;

	//Initalize service structure.
	p_uds->conn_handle																											= BLE_CONN_HANDLE_INVALID;
	p_uds->first_name_write_handler																					= p_uds_init->first_name_write_handler;
	p_uds->last_name_write_handler																					= p_uds_init->last_name_write_handler;
	p_uds->email_address_write_handler																			= p_uds_init->email_address_write_handler;
	p_uds->age_write_handler																								= p_uds_init->age_write_handler;
	p_uds->date_of_birth_write_handler																			= p_uds_init->date_of_birth_write_handler;
	p_uds->gender_write_handler																							= p_uds_init->gender_write_handler;
	p_uds->weight_write_handler																							= p_uds_init->weight_write_handler;
	p_uds->height_write_handler																							= p_uds_init->height_write_handler;
	p_uds->VO2_max_write_handler																						= p_uds_init->VO2_max_write_handler;
	p_uds->heart_rate_max_write_handler																			= p_uds_init->heart_rate_max_write_handler;
	p_uds->resting_heart_write_handler																			= p_uds_init->resting_heart_write_handler;   				//@Including USER ID
	p_uds->maximum_recommended_heart_rate_write_handler											= p_uds_init->maximum_recommended_heart_rate_write_handler;		//@Including AEROBIC THRESHOLD
	p_uds->aerobic_threshold_write_handler																	= p_uds_init->aerobic_threshold_write_handler;	//@Including ANAEROBIC THRESHOLD
	p_uds->anaerobic_threshold_write_handler																= p_uds_init->anaerobic_threshold_write_handler;  					//@Including NAME
	p_uds->sport_type_for_aerobic_and_anaerobic_thresholds_write_handler		= p_uds_init->sport_type_for_aerobic_and_anaerobic_thresholds_write_handler;			//@Including FITNESS INDEX
	p_uds->date_of_threshold_assessment_write_handler												= p_uds_init->date_of_threshold_assessment_write_handler;
	p_uds->waist_circumference_write_handler																= p_uds_init->waist_circumference_write_handler;
	p_uds->fat_burn_heart_rate_lower_limit_write_handler										= p_uds_init->fat_burn_heart_rate_lower_limit_write_handler;
	p_uds->fat_burn_heart_rate_upper_limit_write_handler										= p_uds_init->fat_burn_heart_rate_upper_limit_write_handler;
	p_uds->aerobic_heart_rate_lower_limit_write_handler											= p_uds_init->aerobic_heart_rate_lower_limit_write_handler;
	p_uds->aerobic_heart_rate_upper_limit_write_handler											= p_uds_init->aerobic_heart_rate_upper_limit_write_handler;
	p_uds->anaerobic_heart_rate_lower_limit_write_handler										= p_uds_init->anaerobic_heart_rate_lower_limit_write_handler;
	p_uds->five_zone_heart_rate_limits_write_handler												= p_uds_init->five_zone_heart_rate_limits_write_handler;
	p_uds->three_zone_heart_rate_limits_write_handler												= p_uds_init->three_zone_heart_rate_limits_write_handler;
	p_uds->two_zone_heart_rate_limits_write_handler													= p_uds_init->two_zone_heart_rate_limits_write_handler;
	p_uds->language_write_handler																						= p_uds_init->language_write_handler;
	p_uds->fitnes_index_write_handler																				= p_uds_init->fitnes_index_write_handler;



	//Add service
/*	ble_uuid128_t base_uuid = {RUS_UUID_BASE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_uds->uuid_type);
	if(err_code != NRF_SUCCESS){
		return err_code;
	}

	ble_uuid.type = p_uds->uuid_type;
	ble_uuid.uuid = BLE_UUID_USER_DATA_SERVICE;
*/
	BLE_UUID_BLE_ASSIGN(ble_uuid, BLE_UUID_USER_DATA_SERVICE);

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_uds->service_handle);
	if(err_code != NRF_SUCCESS){
		return err_code;
	}

	//Add characteristics.

	err_code = first_name_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

//	err_code = last_name_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = email_address_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

	err_code = age_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

//	err_code = date_of_birth_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

	err_code = gender_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

	err_code = weight_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

	err_code = height_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

	err_code = VO2_max_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

//	err_code = heart_rate_max_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

	err_code = resting_heart_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

//	err_code = maximum_recommended_heart_rate_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

	err_code = aerobic_threshold_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

	err_code = anaerobic_threshold_char_add(p_uds, p_uds_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

//	err_code = sport_type_for_aerobic_and_anaerobic_thresholds_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = date_of_threshold_assessment_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = waist_circumference_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = fat_burn_heart_rate_lower_limit_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = fat_burn_heart_rate_upper_limit_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = aerobic_heart_rate_lower_limit_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = aerobic_heart_rate_upper_limit_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = anaerobic_heart_rate_lower_limit_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = five_zone_heart_rate_limits_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = three_zone_heart_rate_limits_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = two_zone_heart_rate_limit_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

//	err_code = language_char_add(p_uds, p_uds_init);
//	if(err_code != NRF_SUCCESS)
//		return err_code;

	err_code = fitnes_index_char_add(p_uds, p_uds_init);
			if(err_code != NRF_SUCCESS)
				return err_code;			//@Including FITNESS INDEX


		return NRF_SUCCESS;
}




