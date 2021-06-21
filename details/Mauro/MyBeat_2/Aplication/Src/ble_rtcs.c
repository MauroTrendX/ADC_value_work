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
#include "ble_rtcs.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_l2cap.h"

#define OPCODE_LENGTH	1
#define HANDLE_LENGTH	2
#define MAX_ADCM_LEN	(BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)

#define RTCS_FLAG_MASK	(0x01 << 0);

#define APP_FEATURE_NOT_SUPPORTED	(BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2)

static void on_connect(ble_rtcs_t * p_rtcs, ble_evt_t const * p_ble_evt){
	p_rtcs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(ble_rtcs_t * p_rtcs, ble_evt_t const * p_ble_evt){
	p_rtcs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write(ble_rtcs_t * p_rtcs, ble_evt_t const * p_ble_evt){
	//handle all write events for each specific service handle target.
	ble_gatts_evt_write_t const * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	//check here after compilation if the pointer will have the right structure contents.
	if((p_evt_write->handle == p_rtcs->SPIVI_zone1_threshold_char_handles.value_handle) && (p_rtcs->SPIVI_zone1_threshold_write_handler != NULL)){
		p_rtcs->SPIVI_zone1_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}

	else
	if((p_evt_write->handle == p_rtcs->SPIVI_zone2_threshold_char_handles.value_handle) && (p_rtcs->SPIVI_zone2_threshold_write_handler != NULL)){
		p_rtcs->SPIVI_zone2_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}
	
	else
	if((p_evt_write->handle == p_rtcs->SPIVI_zone3_threshold_char_handles.value_handle) && (p_rtcs->SPIVI_zone3_threshold_write_handler != NULL)){
		p_rtcs->SPIVI_zone3_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}

	else
	if((p_evt_write->handle == p_rtcs->SPIVI_zone4_threshold_char_handles.value_handle) && (p_rtcs->SPIVI_zone4_threshold_write_handler != NULL)){
		p_rtcs->SPIVI_zone4_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}
	
	else
	if((p_evt_write->handle == p_rtcs->SPIVI_zone5_threshold_char_handles.value_handle) && (p_rtcs->SPIVI_zone5_threshold_write_handler != NULL)){
		p_rtcs->SPIVI_zone5_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}
	
	else
	if((p_evt_write->handle == p_rtcs->rhr_zone1_threshold_char_handles.value_handle) && (p_rtcs->rhr_zone1_threshold_write_handler != NULL)){
		p_rtcs->rhr_zone1_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}

	else
	if((p_evt_write->handle == p_rtcs->rhr_zone2_threshold_char_handles.value_handle) && (p_rtcs->rhr_zone2_threshold_write_handler != NULL)){
		p_rtcs->rhr_zone2_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}

	else
	if((p_evt_write->handle == p_rtcs->rhr_zone3_threshold_char_handles.value_handle) && (p_rtcs->rhr_zone3_threshold_write_handler != NULL)){
		p_rtcs->rhr_zone3_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}

	else
	if((p_evt_write->handle == p_rtcs->rhr_zone4_threshold_char_handles.value_handle) && (p_rtcs->rhr_zone4_threshold_write_handler != NULL)){
		p_rtcs->rhr_zone4_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}

	else
	if((p_evt_write->handle == p_rtcs->rhr_zone5_threshold_char_handles.value_handle) && (p_rtcs->rhr_zone5_threshold_write_handler != NULL)){
		p_rtcs->rhr_zone5_threshold_write_handler(p_rtcs, p_evt_write->data[0]);
	}
}

void ble_rtcs_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context){

		ble_rtcs_t * p_rtcs = (ble_rtcs_t *)p_context;
		
		switch(p_ble_evt->header.evt_id){
			case BLE_GAP_EVT_CONNECTED:
				on_connect(p_rtcs, p_ble_evt);
				break;
			case BLE_GAP_EVT_DISCONNECTED:
				on_disconnect(p_rtcs, p_ble_evt);
				break;
			case BLE_GATTS_EVT_WRITE:
				on_write(p_rtcs, p_ble_evt);
				break;
			default:
				break;
		}
}

static uint32_t SPIVI_zone1_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){
	
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
	
	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_SPIVI_ZONE_1_THRESHOLD_CHAR;
	
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
	
	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->SPIVI_zone1_threshold_char_handles);
	
}

static uint32_t SPIVI_zone2_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){
	
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
	
	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_SPIVI_ZONE_2_THRESHOLD_CHAR;
	
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
	
	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->SPIVI_zone2_threshold_char_handles);
	
}

static uint32_t SPIVI_zone3_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){
	
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
	
	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_SPIVI_ZONE_3_THRESHOLD_CHAR;
	
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
	
	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->SPIVI_zone3_threshold_char_handles);
	
}

static uint32_t SPIVI_zone4_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){
	
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
	
	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_SPIVI_ZONE_4_THRESHOLD_CHAR;
	
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
	
	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->SPIVI_zone4_threshold_char_handles);
	
}

static uint32_t SPIVI_zone5_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){
	
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
	
	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_SPIVI_ZONE_5_THRESHOLD_CHAR;
	
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
	
	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->SPIVI_zone5_threshold_char_handles);
	
}

static uint32_t rhr_zone1_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){

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

	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_RHR_ZONE_1_THRESHOLD_CHAR;

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

	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->rhr_zone1_threshold_char_handles);

}

static uint32_t rhr_zone2_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){

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

	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_RHR_ZONE_2_THRESHOLD_CHAR;

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

	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->rhr_zone2_threshold_char_handles);

}

static uint32_t rhr_zone3_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){

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

	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_RHR_ZONE_3_THRESHOLD_CHAR;

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

	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->rhr_zone3_threshold_char_handles);

}

static uint32_t rhr_zone4_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){

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

	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_RHR_ZONE_4_THRESHOLD_CHAR;

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

	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->rhr_zone4_threshold_char_handles);

}

static uint32_t rhr_zone5_threshold_char_add(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){

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

	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_RHR_ZONE_5_THRESHOLD_CHAR;

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

	return sd_ble_gatts_characteristic_add(p_rtcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rtcs->rhr_zone5_threshold_char_handles);

}

uint32_t ble_rtcs_init(ble_rtcs_t * p_rtcs, const ble_rtcs_init_t * p_rtcs_init){
	uint32_t err_code;
	ble_uuid_t ble_uuid;
	
	//initialize service structure.
	p_rtcs->conn_handle = BLE_CONN_HANDLE_INVALID;
	p_rtcs->SPIVI_zone1_threshold_write_handler		= p_rtcs_init->SPIVI_zone1_threshold_write_handler;
	p_rtcs->SPIVI_zone2_threshold_write_handler		= p_rtcs_init->SPIVI_zone2_threshold_write_handler;
	p_rtcs->SPIVI_zone3_threshold_write_handler		= p_rtcs_init->SPIVI_zone3_threshold_write_handler;
	p_rtcs->SPIVI_zone4_threshold_write_handler		= p_rtcs_init->SPIVI_zone4_threshold_write_handler;
	p_rtcs->SPIVI_zone5_threshold_write_handler		= p_rtcs_init->SPIVI_zone5_threshold_write_handler;
	p_rtcs->rhr_zone1_threshold_write_handler		= p_rtcs_init->rhr_zone1_threshold_write_handler;
	p_rtcs->rhr_zone2_threshold_write_handler		= p_rtcs_init->rhr_zone2_threshold_write_handler;
	p_rtcs->rhr_zone3_threshold_write_handler		= p_rtcs_init->rhr_zone3_threshold_write_handler;
	p_rtcs->rhr_zone4_threshold_write_handler		= p_rtcs_init->rhr_zone4_threshold_write_handler;
	p_rtcs->rhr_zone5_threshold_write_handler		= p_rtcs_init->rhr_zone5_threshold_write_handler;
	
	//Add service
	ble_uuid128_t base_uuid = {RTCS_UUID_BASE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &p_rtcs->uuid_type);
	if(err_code != NRF_SUCCESS){
		return err_code;
	}
	
	ble_uuid.type = p_rtcs->uuid_type;
	ble_uuid.uuid = RTCS_UUID_SERVICE;
	
	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_rtcs->service_handle);
	if(err_code != NRF_SUCCESS){
		return err_code;
	}
	
	//Add characteristics.
	err_code = SPIVI_zone1_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
			return err_code;
		
	err_code = SPIVI_zone2_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	err_code = SPIVI_zone3_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	err_code = SPIVI_zone4_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	err_code = SPIVI_zone5_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
	

	err_code = rhr_zone1_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
			return err_code;

	err_code = rhr_zone2_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

	err_code = rhr_zone3_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

	err_code = rhr_zone4_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
		return err_code;

	err_code = rhr_zone5_threshold_char_add(p_rtcs, p_rtcs_init);
	if(err_code != NRF_SUCCESS)
		return err_code;
	
	return NRF_SUCCESS;
	
	
}

















