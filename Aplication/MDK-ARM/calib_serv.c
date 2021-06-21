#include <string.h>
#include "calib_serv.h"
#include "ble_gatts.h"
#include "nordic_common.h"
#include "ble_srv_common.h"
#include "app_util.h"
#include "ble_l2cap.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#define OPCODE_LENGTH	1
#define HANDLE_LENGTH	2
#define MAX_ADCM_LEN	  (BLE_L2CAP_MTU_DEF - OPCODE_LENGTH - HANDLE_LENGTH)

#define RCS_FLAG_MASK_ADC_RAW_VALUE_16BIT	(0x01 << 0)

#define APP_FEATURE_NOT_SUPPORTED      	BLE_GATT_STATUS_ATTERR_APP_BEGIN + 2        /**< Reply when unsupported features are requested. */

static void on_connect(calib_serv_t * p_rcs, ble_evt_t const *  p_ble_evt)
{
	p_rcs->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}

static void on_disconnect(calib_serv_t * p_rcs, ble_evt_t const * p_ble_evt)
{
	p_rcs->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write_2(calib_serv_t * p_rcs, ble_evt_t  const* p_ble_evt)
{
	ble_gatts_evt_write_t const* p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;//coloquei um const

	//check here after compilation if the pointer will have the right structure contents.
	if ((p_evt_write->handle == p_rcs->bike_id_char_handles.value_handle) &&
		(p_rcs->bike_id_write_handler != NULL))
	{
		if(p_evt_write->len == 1){
		  p_rcs->bike_id_write_handler(p_rcs, (uint16_t)p_evt_write->data[0]);
		}
		if(p_evt_write->len == 2){
			p_rcs->bike_id_write_handler(p_rcs, (p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
		}
		if(p_evt_write->len == 4){
			p_rcs->bike_id_write_handler(p_rcs, (p_evt_write->data[3]<<16)|(p_evt_write->data[2]<<12)|(p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
		}
		
	}

	if ((p_evt_write->handle == p_rcs->zero_char_handles.value_handle) &&
		(p_rcs->zero_write_handler != NULL))
	{	
		if(p_evt_write->len == 1)
			p_rcs->zero_write_handler(p_rcs, (uint16_t)p_evt_write->data[0]);
		if(p_evt_write->len == 2)
			p_rcs->zero_write_handler(p_rcs, (p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
		if(p_evt_write->len == 4)
			p_rcs->zero_write_handler(p_rcs, (p_evt_write->data[3]<<16)|(p_evt_write->data[2]<<12)|(p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
	}

	if ((p_evt_write->handle == p_rcs->ten_char_handles.value_handle) &&
		(p_rcs->ten_write_handler != NULL))
	{
		if(p_evt_write->len == 1)
			p_rcs->ten_write_handler(p_rcs, (uint16_t)p_evt_write->data[0]);
		if(p_evt_write->len == 2)
			p_rcs->ten_write_handler(p_rcs, (p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
		if(p_evt_write->len == 4)
			p_rcs->ten_write_handler(p_rcs, (p_evt_write->data[3]<<16)|(p_evt_write->data[2]<<12)|(p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
	}

	if ((p_evt_write->handle == p_rcs->p_load_char_handles.value_handle) &&
		(p_rcs->p_load_write_handler != NULL))
	{
		if(p_evt_write->len == 1)
			p_rcs->p_load_write_handler(p_rcs, (uint16_t)p_evt_write->data[0]);
		if(p_evt_write->len == 2){
			uint16_t data_write = (p_evt_write->data[1]<<8)|(p_evt_write->data[0]);
			p_rcs->p_load_write_handler(p_rcs, data_write);
		}
		if(p_evt_write->len == 4)
			p_rcs->p_load_write_handler(p_rcs, (p_evt_write->data[3]<<16)|(p_evt_write->data[2]<<12)|(p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
	}

	if ((p_evt_write->handle == p_rcs->n_load_char_handles.value_handle) &&
		(p_rcs->n_load_write_handler != NULL))
	{
		if(p_evt_write->len == 1)
			p_rcs->n_load_write_handler(p_rcs, (uint16_t)p_evt_write->data[0]);
		if(p_evt_write->len == 2){
			uint16_t data_write = (p_evt_write->data[1]<<8)|(p_evt_write->data[0]);
			p_rcs->n_load_write_handler(p_rcs, data_write);
		}
		if(p_evt_write->len == 4)
			p_rcs->n_load_write_handler(p_rcs, (p_evt_write->data[3]<<16)|(p_evt_write->data[2]<<12)|(p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
	}

	if ((p_evt_write->handle == p_rcs->serial_number_char_handles.value_handle) &&
			(p_rcs->serial_number_write_handler != NULL))
	{
		if(p_evt_write->len == 1)
			p_rcs->serial_number_write_handler(p_rcs, (uint16_t)p_evt_write->data[0]);
		if(p_evt_write->len == 2){
			uint16_t data_write = (p_evt_write->data[1]<<8)|(p_evt_write->data[0]);
			p_rcs->serial_number_write_handler(p_rcs, data_write);
		}
		if(p_evt_write->len == 4)
			p_rcs->serial_number_write_handler(p_rcs, (p_evt_write->data[3]<<16)|(p_evt_write->data[2]<<12)|(p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
	}

	if ((p_evt_write->handle == p_rcs->mean_interval_char_handles.value_handle) &&
				(p_rcs->mean_interval_write_handler != NULL))
	{
		if(p_evt_write->len == 1)
			p_rcs->mean_interval_write_handler(p_rcs, (uint16_t)p_evt_write->data[0]);
		if(p_evt_write->len == 2){
			uint16_t data_write = (p_evt_write->data[1]<<8)|(p_evt_write->data[0]);
			p_rcs->mean_interval_write_handler(p_rcs, data_write);
		}
		if(p_evt_write->len == 4)
			p_rcs->mean_interval_write_handler(p_rcs, (p_evt_write->data[3]<<16)|(p_evt_write->data[2]<<12)|(p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
	}

	if ((p_evt_write->handle == p_rcs->mode_char_handles.value_handle) &&
					(p_rcs->mode_write_handler != NULL))
	{
		if(p_evt_write->len == 1)
			p_rcs->mode_write_handler(p_rcs, (uint16_t)p_evt_write->data[0]);
		if(p_evt_write->len == 2){
			uint16_t data_write = (p_evt_write->data[1]<<8)|(p_evt_write->data[0]);
			p_rcs->mode_write_handler(p_rcs, data_write);
		}
		if(p_evt_write->len == 4)
			p_rcs->mode_write_handler(p_rcs, (p_evt_write->data[3]<<16)|(p_evt_write->data[2]<<12)|(p_evt_write->data[1]<<8)|(p_evt_write->data[0]));
	}


}

void calib_serv_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
	
	calib_serv_t * p_rcs = (calib_serv_t *)p_context;
	
	switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_rcs, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_rcs, p_ble_evt);
            break;
            
        case BLE_GATTS_EVT_WRITE:
            on_write_2(p_rcs, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}

static uint32_t bike_id_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_BIKE_ID_CHAR;
    
    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
		BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;
    
    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->bike_id_char_handles);
}

static uint32_t ten_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_TEN_KG_POINT_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint16_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->ten_char_handles);
}

static uint32_t zero_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_ZERO_KG_POINT_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint16_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->zero_char_handles);
}

static uint32_t p_load_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_P_LOAD_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint16_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->p_load_char_handles);
}

static uint32_t n_load_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_N_LOAD_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint16_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->n_load_char_handles);
}

static uint32_t load_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_LOAD_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->load_char_handles);
}

static uint32_t load_t_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_LOAD_T_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->load_t_char_handles);
}

static uint32_t adc_unbal_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_ADC_UNBAL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->adc_unbal_char_handles);
}

static uint32_t adc_bal_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 0;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_ADC_BAL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->adc_bal_char_handles);
}

static uint32_t serial_number_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_SERIAL_NUM_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->serial_number_char_handles);
}

static uint32_t mean_interval_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_MEAN_INTERVAL_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->mean_interval_char_handles);
}

static uint32_t mode_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
	ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_MODE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 1;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint32_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_rcs->mode_char_handles);
}


//further use for advertising values of 16 and 32 bits in characteristics.
static uint8_t adc_raw_encode(calib_serv_t * p_rcs, uint16_t adc_raw, uint8_t * p_encoded_buffer){
	uint8_t flags = 0;
	uint8_t len = 0;

	if(adc_raw > 0xFF){
		flags = adc_raw/0xFF;
		p_encoded_buffer[len++] = (adc_raw & 0xFF00 >> 8);
		p_encoded_buffer[len++] = (adc_raw & 0x00FF >> 0);
	}
	else{
		flags = 0;
		p_encoded_buffer[len++] = (uint8_t)adc_raw;
	}
	p_encoded_buffer[0] = flags;

	return len;
}


static uint32_t adc_raw_char_add(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_ADC_RAW_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 1;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint16_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_rcs->service_handle,
										   &char_md,
										   &attr_char_value,
										   &p_rcs->adc_raw_char_handles);
}

uint32_t ble_calib_serv_init(calib_serv_t * p_rcs, const calib_serv_init_t * p_rcs_init)
{
		uint32_t   err_code=0;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_rcs->conn_handle       	 		= BLE_CONN_HANDLE_INVALID;
    p_rcs->bike_id_write_handler 		= p_rcs_init->bike_id_write_handler;
    p_rcs->zero_write_handler 	 		= p_rcs_init->zero_write_handler;
    p_rcs->ten_write_handler 			= p_rcs_init->ten_write_handler;
    p_rcs->p_load_write_handler	 		= p_rcs_init->p_load_write_handler;
    p_rcs->n_load_write_handler  		= p_rcs_init->n_load_write_handler;
    p_rcs->serial_number_write_handler	= p_rcs_init->serial_number_write_handler;
    p_rcs->mean_interval_write_handler	= p_rcs_init->mean_interval_write_handler;
    p_rcs->mode_write_handler			= p_rcs_init->mode_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {RCS_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_rcs->uuid_type);
		if (err_code != NRF_SUCCESS)
    {
			        return err_code;
    }

		
    ble_uuid.type = p_rcs->uuid_type;
    ble_uuid.uuid = RCS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_rcs->service_handle);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    // Add characteristics.

    err_code = bike_id_char_add(p_rcs, p_rcs_init);
    if (err_code != NRF_SUCCESS)
    {
        return err_code;
    }

    err_code = zero_char_add(p_rcs, p_rcs_init);
    if (err_code != NRF_SUCCESS)
    {
    	return err_code;
    }

    err_code = ten_char_add(p_rcs, p_rcs_init);
    if(err_code != NRF_SUCCESS)
    {
    	return err_code;
    }

    err_code = p_load_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = n_load_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = load_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = load_t_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = adc_unbal_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = adc_bal_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = serial_number_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = mean_interval_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

	err_code = mode_char_add(p_rcs, p_rcs_init);
	if(err_code != NRF_SUCCESS)
	{
		return err_code;
	}

    return NRF_SUCCESS;
}

uint32_t adc_raw_meas_update(calib_serv_t * p_rcs, uint8_t ADC_meas){
	ble_gatts_hvx_params_t params;
	uint16_t len = sizeof(ADC_meas);

	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = p_rcs->adc_raw_char_handles.value_handle;
	params.p_data = &ADC_meas;
	params.p_len = &len;

	return sd_ble_gatts_hvx(p_rcs->conn_handle, &params);
}

//further use for sending data 16 bits long.
uint32_t adc_raw_meas_send(calib_serv_t * p_rcs, uint16_t ADC_meas){
	uint32_t err_code;

	if(p_rcs->conn_handle != BLE_CONN_HANDLE_INVALID){
		uint8_t					encoded_adc[MAX_ADCM_LEN];
		uint16_t				len;
		uint16_t				hvx_len;
		ble_gatts_hvx_params_t	hvx_params;

		len = adc_raw_encode(p_rcs, ADC_meas, encoded_adc);
		hvx_len = len;

		memset(&hvx_params, 0, sizeof(hvx_params));

		hvx_params.handle = p_rcs->adc_raw_char_handles.value_handle;
		hvx_params.type	  = BLE_GATT_HVX_NOTIFICATION;
		hvx_params.offset = 0;
		hvx_params.p_len  = &hvx_len;
		hvx_params.p_data = encoded_adc;

		err_code = sd_ble_gatts_hvx(p_rcs->conn_handle, &hvx_params);
		if((err_code == NRF_SUCCESS) && (hvx_len != len)){
			err_code = NRF_ERROR_DATA_SIZE;
		}
	}
	else{
		err_code = NRF_ERROR_INVALID_STATE;
	}
	return err_code;
}
