/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @example examples/ble_peripheral/ble_app_hrs/main.c
 *
 * @brief Heart Rate Service Sample Application main file.
 *
 * This file contains the source code for a sample application using the Heart Rate service
 * (and also Battery and Device Information services). This application uses the
 * @ref srvlib_conn_params module.
 */

#include "utills.h"
#include "calib_serv.h"

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nordic_common.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "ble.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "fds.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "ble_conn_state.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_power.h"
#include "nrf_drv_gpiote.h"
#include "main.h"
#include "biblioteca_phillips.h"
#include "acc_interface.h"
#include "nrf_drv_saadc.h"

#include "ant_key_manager.h"
#include "ant_hrm.h"
#include "ant_hrm_measurement.h"
#include "ant_state_indicator.h"

#include "ble_dfu.h"

#include "nrf_bootloader_info.h"

#include "nrf_drv_pwm.h"

#include "zonas_cardiacas.h"
#include "Controle_LEDs.h"

#include "ble_rus.h"
#include "ble_rtcs.h"
#include "ble_uds.h"
#include "buffer_services.h"
#include "ble_cscs.h"

#include "armazenamento_treino.h"

#define WDT_ATIVO

#ifdef WDT_ATIVO
#include "nrf_drv_wdt.h"
#include "nrf_drv_clock.h"
#endif

//#define MYBEAT_V1

#ifdef MYBEAT_V1

#define DEVICE_NAME                         "mybeat" 		                            /**< Name of device. Will be included in the advertising data. */

#else

#define DEVICE_NAME                         "Exe15" 		                            /**< Name of device. Will be included in the advertising data. */

#endif

#define MANUFACTURER_NAME                   "TrendX"    							              /**< Manufacturer. Will be passed to Device Information Service. */
#define APP_ADV_INTERVAL                    300                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 187.5 ms). */

#define FIRMWARE_VERSION 										"1.0.9"

#define APP_ADV_DURATION                    18000                                   /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define APP_COMPANY_IDENTIFIER          		0x02B6

#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */
#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define BATTERY_LEVEL_MEAS_INTERVAL         APP_TIMER_TICKS(2000)                   /**< Battery level measurement interval (ticks). */
           
#define HEART_RATE_MEAS_INTERVAL            APP_TIMER_TICKS(1000)                   /**< Heart rate measurement interval (ticks). */

#define RR_INTERVAL_INTERVAL                APP_TIMER_TICKS(300)                    /**< RR interval interval (ticks). */

#define SENSOR_CONTACT_DETECTED_INTERVAL    APP_TIMER_TICKS(5000)                   /**< Sensor Contact Detected toggle interval (ticks). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(30, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(70, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). *///original was 60 not 70
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      APP_TIMER_TICKS(5000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       APP_TIMER_TICKS(30000)                  /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define LESC_DEBUG_MODE                     0                                       /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      1                                       /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#include "nrf_drv_spi.h"
#include "nrfx_spi.h"
#include "lis2dw12_reg.h"

#include "nrf_delay.h"
#include "heritage.h"
#include "trigonometry.h"


#define SPI_INSTANCE  0 /**< SPI instance index. */
#define ACC_CS_PIN 07 //pino para seleção do acelerometro

/**
***Inclusão do protocolo ANT+ 
**/

#define APP_TICK_EVENT_INTERVAL  2000 /**< 2 second's tick event interval in timer tick units. */
#define HRM_CHANNEL_NUMBER       0x00 /**< Channel number assigned to HRM profile. */
#define ANTPLUS_NETWORK_NUMBER   0    /**< Network number. */
#define MIN_RUNNING_SPEED               3                                       /**< speed threshold to set the running bit. */

#define SPEED_AND_CADENCE_MEAS_INTERVAL 500                                        /**< Speed and cadence measurement interval (milliseconds). */
#define KPH_TO_MM_PER_SEC               278                                         /**< Constant to convert kilometers per hour into millimeters per second. */
#define WHEEL_CIRCUMFERENCE_MM          2100                                        /**< Simulated wheel circumference in millimeters. */
#define RPM_TO_DEGREES_PER_SEC          6                                           /**< Constant to convert revolutions per minute into degrees per second. */
#define DEGREES_PER_REVOLUTION          360                                         /**< Constant used in simulation for calculating crank speed. */
#define MIN_SPEED_KPH                   3                                          /**< Minimum speed in kilometers per hour for use in the simulated measurement function. */
#define MAX_SPEED_KPH                   10                                          /**< Maximum speed in kilometers per hour for use in the simulated measurement function. */
#define SPEED_KPH_INCREMENT             1                                           /**< Value by which speed is incremented/decremented for each call to the simulated measurement function. */
#define MIN_CRANK_RPM                   8                                          /**< Minimum cadence in RPM for use in the simulated measurement function. */
#define MAX_CRANK_RPM                   12                                         /**< Maximum cadence in RPM for use in the simulated measurement function. */
#define CRANK_RPM_INCREMENT             3                                           /**< Value by which cadence is incremented/decremented in the simulated measurement function. */

static ant_hrm_measurement_t  m_ant_hrm_measurement;    

static uint16_t          m_conn_handle = BLE_CONN_HANDLE_INVALID;                   /**< Handle of the current connection. */
static sensorsim_cfg_t   m_battery_sim_cfg;                                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                                       /**< Battery Level sensor simulator state. */

static sensorsim_cfg_t   m_speed_kph_sim_cfg;                                       /**< Speed simulator configuration. */
static sensorsim_state_t m_speed_kph_sim_state;                                     /**< Speed simulator state. */
static sensorsim_cfg_t   m_crank_rpm_sim_cfg;                                       /**< Crank simulator configuration. */
static sensorsim_state_t m_crank_rpm_sim_state;                                     /**< Crank simulator state. */

static uint32_t m_cumulative_wheel_revs;                                            /**< Cumulative wheel revolutions. */

static  bool     m_auto_calibration_in_progress;                                     /**< Set when an autocalibration is in progress. */

static uint16_t   glob_cumulative_crank_revs = 0;
static uint16_t   glob_event_time            = 0;
static uint16_t   glob_wheel_revolution_mm   = 0;
static uint16_t   glob_wheel_revolution_mm_F = 0;//filtered variable to be the output of filter fnction 
static uint16_t   glob_crank_rev_degrees     = 0;
//testes
volatile uint16_t IN_dummy=0;//just for the operation of the function filter
volatile uint16_t OUT_dummy=0;//just for the operation of the function filter
volatile int16_t value_for_simu=10;//just for project debugging purposes
volatile int16_t value_for_simu_F=0;//just for project debugging purposes
volatile int flag_for_simu=0;//just for project debugging purposes
volatile int limit_for_simu=10;//just for project debugging purposes
volatile int limit_neg_for_simu=-10;//just for project debugging purposes
volatile int32_t transfer_rpm=0;
//testes advertising=====================================================================================================================
volatile uint32_t ADS018_ShowCounter_AdvQuit   = 2;
volatile uint32_t ADS018_ShowCounter_SetMean   = 47;//originally 47.
volatile ADS018_cal_Type teste;
volatile int  i=0;//just for a tiny counter in function seno
//MIXING THE DATA LIS2DW12
volatile int16_t global_mixer ;
//FUNCTION PROTOTYPES===================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================
void get_accel(void);
void get_value (void); //protótipo pra função get_value
void cycle_treat(void);
//======================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================================
static ble_sensor_location_t supported_locations[] =                                /**< Supported location for the sensor location. */
{
	
    BLE_SENSOR_LOCATION_FRONT_WHEEL,
    BLE_SENSOR_LOCATION_LEFT_CRANK,
    BLE_SENSOR_LOCATION_RIGHT_CRANK,
    BLE_SENSOR_LOCATION_LEFT_PEDAL,
    BLE_SENSOR_LOCATION_RIGHT_PEDAL,
    BLE_SENSOR_LOCATION_FRONT_HUB,
    BLE_SENSOR_LOCATION_REAR_DROPOUT,
    BLE_SENSOR_LOCATION_CHAINSTAY,
    BLE_SENSOR_LOCATION_REAR_WHEEL,
    BLE_SENSOR_LOCATION_REAR_HUB
	
};

/** @snippet [ANT HRM TX Instance] */
void ant_hrm_evt_handler(ant_hrm_profile_t * p_profile, ant_hrm_evt_t event);

HRM_SENS_PROFILE_CONFIG_DEF(m_ant_hrm,
                            true,
                            ANT_HRM_PAGE_0,
                            ant_hrm_evt_handler);

static ant_hrm_profile_t m_ant_hrm;
/** @snippet [ANT HRM TX Instance] */


/**
 * @brief 2 seconds tick handler for updataing cumulative operating time.
 */
static void ant_plus_handler(void * p_context)
{
    // Only the first 3 bytes of this value are taken into account
    m_ant_hrm.HRM_PROFILE_operating_time++;
}														
														
/**@brief Function for setup all things not directly associated with ANT stack/protocol.
 *
 * @desc Initialization of: @n
 *         - app_timer, presetup for bsp and ant pulse simulation.
 *         - bsp for signaling leds and user buttons (if use button is enabled in example).
 *         - ant pulse simulate for task of filling hrm profile data.
 */
static void utils_setup(void)
{
    ret_code_t err_code = ant_state_indicator_init(m_ant_hrm.channel_number, HRM_SENS_CHANNEL_TYPE);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling ANT HRM events.
 */
/** @snippet [ANT HRM simulator call] */
void ant_hrm_evt_handler(ant_hrm_profile_t * p_profile, ant_hrm_evt_t event)
{
    nrf_pwr_mgmt_feed();

    switch (event)
    {
        case ANT_HRM_PAGE_0_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_1_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_2_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_3_UPDATED:
            /* fall through */
        case ANT_HRM_PAGE_4_UPDATED:
            ant_hrm_measurement_one_iteration(&m_ant_hrm_measurement);
            break;

        default:
            break;
    }
}

static void measurement_setup(void){

	const ant_hrm_measurement_cfg_t measurement_cfg_var =
	{
		.p_profile = &m_ant_hrm,
		{
				30, 220, 30
		}
	};

	ant_hrm_measurement_init(&m_ant_hrm_measurement, &measurement_cfg_var);
}


/** @snippet [ANT HRM simulator call] */

/**
 * @brief Function for HRM profile initialization.
 *
 * @details Initializes the HRM profile and open ANT channel.
 */
static void profile_setup(void)
{
/** @snippet [ANT HRM TX Profile Setup] */
    ret_code_t err_code;
    uint16_t device_ID = get_user_id();;

    ant_channel_config_t   m_ant_hrm_channel_hrm_sens_config =
	{
		.channel_number    = (HRM_CHANNEL_NUMBER),
		.channel_type      = HRM_SENS_CHANNEL_TYPE,
		.ext_assign        = HRM_EXT_ASSIGN,
		.rf_freq           = HRM_ANTPLUS_RF_FREQ,
		.transmission_type = (CHAN_ID_TRANS_TYPE),
		.device_type       = HRM_DEVICE_TYPE,
		.device_number     = (device_ID),
		.channel_period    = HRM_MSG_PERIOD_4Hz,
		.network_number    = (ANTPLUS_NETWORK_NUMBER),
	};
		
    err_code = ant_hrm_sens_init(&m_ant_hrm,
                                 HRM_SENS_CHANNEL_CONFIG(m_ant_hrm),
                                 HRM_SENS_PROFILE_CONFIG(m_ant_hrm));
    APP_ERROR_CHECK(err_code);

    m_ant_hrm.HRM_PROFILE_manuf_id   = HRM_MFG_ID;
    m_ant_hrm.HRM_PROFILE_serial_num = HRM_SERIAL_NUMBER;
    m_ant_hrm.HRM_PROFILE_hw_version = HRM_HW_VERSION;
    m_ant_hrm.HRM_PROFILE_sw_version = HRM_SW_VERSION;
    m_ant_hrm.HRM_PROFILE_model_num  = HRM_MODEL_NUMBER;

    err_code = ant_hrm_sens_open(&m_ant_hrm);
    APP_ERROR_CHECK(err_code);

    err_code = ant_state_indicator_channel_opened();
    APP_ERROR_CHECK(err_code);
/** @snippet [ANT HRM TX Profile Setup] */
}

typedef union{
  int16_t i16bit[3];//y,x,z
  uint8_t u8bit[6];//yh,yl,xh,xl,zh,zl
} axis3bit16_t;
//para retirar amostra de aceleração para teste
static axis3bit16_t get_raw_data;//will be used as global to get the value in the fuction get_value
uint8_t indiceAmostraACC=0;
stmdev_ctx_t dev_ctx;
void amazenar_ACC(uint8_t * numeroAmostra, dadosBbACC * bufferACC);//protótipo
dadosBbACC	amostrasACC_2;

const nrf_drv_spi_t mLisSpiInstance = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /**< SPI instance. */
static volatile bool spi_xfer_done;  /**< Flag used to indicate that SPI instance completed the transfer. */

static volatile bool mLisPacketTransferComplete = false;

volatile bool flagEstadoLed = true;

static uint8_t whoamI, rst;


//variaveis  utilizadas na gestão da bateria e carregamento USB
static uint8_t percent_batt=100;
bool avisoCarregadorUSB=false; 
volatile bool avisoBateriaFraca=false;
bool interrupcaoDeteccaoUSB=false;

volatile bool solicitacaoDesligamento=false; 

uint8_t BPM;

BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
APP_TIMER_DEF(m_battery_timer_id);                                  /**< Battery timer. */
APP_TIMER_DEF(m_rsc_meas_timer_id);                                             /**< RSC measurement timer. */
APP_TIMER_DEF(m_csc_meas_timer_id);                                                 /**< CSC measurement timer. */
APP_TIMER_DEF(m_biblioteca_phillips_id);                           	/**< Biblioteca Phillips timer. */
APP_TIMER_DEF(m_adv_update);                           							/**< update adv data */
APP_TIMER_DEF(m_tick_timer);                       								  /**< Timer used to update cumulative operating time. */


BLE_UDS_DEF(m_uds);                                                /**< User data service instance. */
BLE_RUS_DEF(m_rus);                                                 /**< RAE user service instance. */
BLE_RTCS_DEF(m_rtcs);                                              /**< RAE treshhold calibration service instance. */
BLE_CSCS_DEF(m_cscs);                                            /**< Cycling speed and cadence service instance. */
BLE_CUS_DEF(m_calib_serv);                                       /**< MyWatt custom service*/



//static bool     m_rr_interval_enabled = false;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
//    {BLE_UUID_HEART_RATE_SERVICE,           BLE_UUID_TYPE_BLE},
		{BLE_UUID_CYCLING_SPEED_AND_CADENCE,  BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE,            BLE_UUID_TYPE_BLE},
//    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

volatile bool flag_timer_bb_phillips=false;

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

void read_acc (stmdev_ctx_t *ctx, uint8_t *dadosAcc);

void spi_init(void);

bool lis2dw12_config (void);


bool MAX30110_ConfiguraSensorPPG ( void );


volatile bool update_nv_mem_buf_request;
volatile hr_module_nv_buf_t temporary_buff; //= {

volatile hr_module_nv_buf_update_flags_t hr_module_nv_buf_update_flags = {
	false,
	false,
	false,
	false,
	false,
	false,
	false,
	false,
	false,
	false,
	false,
	false,
	false,
};


void HR_advdata_manuf_data_update(void * p_context);//protótipo

#define MANUF_ID		0x07D0	/*< Manufacturer's ID, need to change from Polar Corporation ID to a vacant one */
#define N_USER_FIELD	17		/*< Length of user data in bytes */

typedef struct{
	uint8_t		Major;		/*< user_id > */		//@Including USER ID
	uint8_t		Minor;	/*< battery_level > */
	uint8_t		Datatype;			/*< age > */
	uint8_t		BikeID;				/*< hr > */
	uint16_t		RPM;			/*< percentual da fraquencia cardiaca maxima*/
	uint16_t		HR;			/*< Rotation field, unused */
	uint16_t		Power;			/*< Heart Rate field, unused */
	uint16_t		Kcal;			/*< Heart Rate field, unused */
	uint8_t		Minutes;			/*< Power field, unused */
	uint8_t		seconds;		/*< Seconds field, check if needs to be used */
	uint16_t	trip;			/*< Trip field, unused *///depois mudar para uint16_t
	uint8_t		gear;			/*< Gear field, unused */
	} hr_wrist_band_adv_user_data_t;


//verificar os tipos de dados pra ver se bate.
volatile hr_wrist_band_adv_user_data_t HR_advertising_data = {
		0x06,0x30,0x00,0x05,0x0000,0x0000,0x0000,0x0000,0x00,0x00,0x0000,0x00
};


ble_advdata_manuf_data_t hr_adv_manuf_data = {MANUF_ID, {N_USER_FIELD, (uint8_t *)&HR_advertising_data}};
hr_wrist_band_adv_user_data_t *pHR_adv_user_data = (hr_wrist_band_adv_user_data_t *)&HR_advertising_data;

static bool app_shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // YOUR_JOB: Get ready to reset into DFU mode
            //
            // If you aren't finished with any ongoing tasks, return "false" to
            // signal to the system that reset is impossible at this stage.
            //
            // Here is an example using a variable to delay resetting the device.
            //
            // if (!m_ready_for_reset)
            // {
            //      return false;
            // }
            // else
            //{
            //
            //    // Device ready to enter
            //    uint32_t err_code;
            //    err_code = sd_softdevice_disable();
            //    APP_ERROR_CHECK(err_code);
            //    err_code = app_timer_stop_all();
            //    APP_ERROR_CHECK(err_code);
            //}
            break;

        default:
            // YOUR_JOB: Implement any of the other events available from the power management module:
            //      -NRF_PWR_MGMT_EVT_PREPARE_SYSOFF
            //      -NRF_PWR_MGMT_EVT_PREPARE_WAKEUP
            //      -NRF_PWR_MGMT_EVT_PREPARE_RESET
            return true;
    }

    NRF_LOG_INFO("Power management allowed to reset to DFU mode.");
    return true;
}

//lint -esym(528, m_app_shutdown_handler)
/**@brief Register application shutdown handler with priority 0.
 */
NRF_PWR_MGMT_HANDLER_REGISTER(app_shutdown_handler, 0);


static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};



#ifdef WDT_ATIVO

nrf_drv_wdt_channel_id m_channel_id;

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
    bsp_board_leds_off();

    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

#endif


static void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}


#define SLEEP 0
#define RUNNING 1
#define CHARGERING 2

uint8_t OperationMode=1;


void lis2dw12_sleep(void){
		lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_OFF);
		nrf_delay_ms(5);
}

bool mybeat_sleep(void){
		lis2dw12_sleep();
		return true;
}

void get_status_carregador (void){
		
		if(percent_batt<=10){
				set_cor(amarelo);
				set_led(LED1);
		}
		if(percent_batt>10&&percent_batt<40){
				set_cor(verde);
				set_led(LED1);
		}
		if(percent_batt>=40&&percent_batt<70){
				set_cor(verde);
				set_led(LED2);
		}
		if(percent_batt>=70&&percent_batt<100){
				set_cor(verde);
				set_led(LED3);
		}
		if(percent_batt>=100){
				set_cor(azul);
				set_led(LED4);
		}

}

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{

}

void saadc_init(void)
{
    ret_code_t err_code;
   
    nrf_saadc_channel_config_t channel_config =
            NRF_DRV_SAADC_DEFAULT_CHANNEL_CONFIG_SE(NRF_SAADC_INPUT_AIN0);
    
		err_code = nrf_drv_saadc_init(NULL, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(0, &channel_config);
    APP_ERROR_CHECK(err_code);
		
}

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds)
{
    if (erase_bonds == true)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code;

        err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(false);
            break;

        default:
            break;
    }
}

uint8_t get_percent_batt(nrf_saadc_value_t sample){
		
		int8_t temp=(sample-455)*100/120;
		
		if(temp>90){
				avisoBateriaFraca=false;
				return percent_batt=100;}
		else if(temp<90&&temp>80){
				avisoBateriaFraca=false;
				return percent_batt=90;}
		else if(temp<80&&temp>70){
				avisoBateriaFraca=false;
				return percent_batt=80;}
		else if(temp<70&&temp>60){
				avisoBateriaFraca=false;
				return percent_batt=70;}
		else if(temp<60&&temp>50){
				avisoBateriaFraca=false;
				return percent_batt=60;}
		else if(temp<50&&temp>40){
				avisoBateriaFraca=false;
				return percent_batt=50;}
		else if(temp<40&&temp>30){
				avisoBateriaFraca=false;
				return percent_batt=40;}
		else if(temp<30&&temp>20){
				avisoBateriaFraca=false;
				return percent_batt=30;}
		else if(temp<20&&temp>10){
				avisoBateriaFraca=false;
				return percent_batt=20;}
		else if(temp>5&&temp<10){
				avisoBateriaFraca=true;
				return percent_batt=10;
		}
		else if(temp>1&&temp<=5){
				avisoBateriaFraca=true;
				return percent_batt=5;
		}
		else if(temp<=1){
				solicitacaoDesligamento=true;
				return percent_batt=0;
		}
		else
				return percent_batt;
}



/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
		nrf_saadc_value_t sample;
		

		nrf_drv_saadc_sample_convert(0, &sample);
	
    err_code = ble_bas_battery_level_update(&m_bas, get_percent_batt(sample), BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
//		NRF_LOG_INFO("Nivel da bateria: %d ",percent_batt);
}


/**@brief Function for handling the Battery measurement timer timeout.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void battery_level_meas_timeout_handler(void * p_context)
{
    UNUSED_PARAMETER(p_context);
    battery_level_update();
}
//function for collecting reading from lis2dw12
void get_value (void){//=================================================================================================================Apagar depois do teste=============================================================================================================================================================
	static axis3bit16_t raw_acceleration;
	lis2dw12_acceleration_raw_get(&dev_ctx, raw_acceleration.u8bit);
	get_raw_data.u8bit[0]=raw_acceleration.u8bit[2];//YLSB
	get_raw_data.u8bit[1]=raw_acceleration.u8bit[3];//YMSB
	global_mixer=raw_acceleration.i16bit[1]>>2;
}
//===========================================================================================================================================================================================================================================================================================================================
///TESTE
	void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    mLisPacketTransferComplete = true;
}

/**@brief Function for populating simulated cycling speed and cadence measurements.
 */
static void csc_sim_measurement(ble_cscs_meas_t * p_measurement)
{   
	  static uint16_t cumulative_crank_revs = 0;
    static uint16_t event_time            = 0;
    static  uint16_t wheel_revolution_mm  = 0;
    static uint16_t crank_rev_degrees     = 0;

  	uint16_t mm_per_sec;
    uint16_t degrees_per_sec;
    uint16_t event_time_inc;

    // Per specification event time is in 1/1024th's of a second.
    event_time_inc = (1024 * SPEED_AND_CADENCE_MEAS_INTERVAL) / 1000;

    // Calculate simulated wheel revolution values.
    p_measurement->is_wheel_rev_data_present = true;

    mm_per_sec = KPH_TO_MM_PER_SEC * sensorsim_measure(&m_speed_kph_sim_state,
                                                       &m_speed_kph_sim_cfg);

    wheel_revolution_mm     += mm_per_sec * SPEED_AND_CADENCE_MEAS_INTERVAL / 1000;
    m_cumulative_wheel_revs += wheel_revolution_mm / WHEEL_CIRCUMFERENCE_MM;
    wheel_revolution_mm     %= WHEEL_CIRCUMFERENCE_MM;
    p_measurement->cumulative_wheel_revs = m_cumulative_wheel_revs;
    p_measurement->last_wheel_event_time =
    event_time + (event_time_inc * (mm_per_sec - wheel_revolution_mm) / mm_per_sec);
    
    // Calculate simulated cadence values.
    p_measurement->is_crank_rev_data_present = true;
    
    degrees_per_sec = RPM_TO_DEGREES_PER_SEC * sensorsim_measure(&m_crank_rpm_sim_state,
                                                                    &m_crank_rpm_sim_cfg);
    
    crank_rev_degrees     += degrees_per_sec * SPEED_AND_CADENCE_MEAS_INTERVAL / 1000;
    cumulative_crank_revs += crank_rev_degrees / DEGREES_PER_REVOLUTION;
    crank_rev_degrees     %= DEGREES_PER_REVOLUTION;
    
    p_measurement->cumulative_crank_revs = cumulative_crank_revs;
    p_measurement->last_crank_event_time =
    event_time + (event_time_inc * (degrees_per_sec - crank_rev_degrees) / degrees_per_sec);
    
    event_time += event_time_inc;
    glob_wheel_revolution_mm= wheel_revolution_mm;
}

static bool sensor_contact_detected = false;

/**@brief Function for handling the Heart rate measurement timer timeout.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void heart_rate_meas_timeout_handler(void * p_context)
{		
    static uint32_t cnt = 0;
    ret_code_t      err_code;
		flagEstadoLed=!flagEstadoLed;
    UNUSED_PARAMETER(p_context);

    cnt++;

		    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, (uint16_t) BPM);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

		ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
				
}
//teste




/**@brief Function for handling the Cycling Speed and Cadence measurement timer timeouts.
 *
 * @details This function will be called each time the cycling speed and cadence
 *          measurement timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
static void csc_meas_timeout_handler(void * p_context)//changes in the characteristic.
{
    uint32_t        err_code;
    ble_cscs_meas_t cscs_measurement;
    UNUSED_PARAMETER(p_context);
    csc_sim_measurement(&cscs_measurement);
	  //mudanças valores testes
	 //ADS018_res_data_f_len
	  cscs_measurement.cumulative_wheel_revs=ADS018_res_data_c.rotation;
	  cscs_measurement.last_wheel_event_time=0x9;//tava o ADS018_Cycle_Flag
	  cscs_measurement.cumulative_crank_revs=0x9;
	  cscs_measurement.last_crank_event_time=0x9;
//	  uint32_t    cumulative_wheel_revs;                                  /**< Cumulative Wheel Revolutions. *///for testing reasons, should be the same value as the variable being tested
//    uint16_t    last_wheel_event_time;                                  /**< Last Wheel Event Time. */
//    uint16_t    cumulative_crank_revs;                                  /**< Cumulative Crank Revolutions. */
//    uint16_t    last_crank_event_time;                                  /**< Last Crank Event Time. */
	
	
	  err_code = ble_cscs_measurement_send(&m_cscs, &cscs_measurement);

    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
    if (m_auto_calibration_in_progress)
    {
        err_code = ble_sc_ctrlpt_rsp_send(&(m_cscs.ctrl_pt), BLE_SCPT_SUCCESS);
        if ((err_code != NRF_SUCCESS) &&
            (err_code != NRF_ERROR_INVALID_STATE) &&
            (err_code != NRF_ERROR_RESOURCES)
           )
        {
            APP_ERROR_HANDLER(err_code);
        }
        if (err_code != NRF_ERROR_RESOURCES)
        {
            m_auto_calibration_in_progress = false;
        }
    }
}


/**@brief Function for handling the biblioteca phillips.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] p_context  Pointer used for passing some arbitrary information (context) from the
 *                       app_start_timer() call to the timeout handler.
 */
//timer phillips

static void biblioteca_phillips_handler(void * p_context)
{			
  
		  UNUSED_PARAMETER(p_context);
			flag_timer_bb_phillips=true;
	
}
	

/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    ret_code_t err_code;

    // Initialize timer module.
    err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    err_code = app_timer_create(&m_battery_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                battery_level_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
    //create cscs timer
    err_code = app_timer_create(&m_csc_meas_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                 csc_meas_timeout_handler);
    APP_ERROR_CHECK(err_code);
	
	
	
		err_code = app_timer_create(&m_biblioteca_phillips_id,
                                APP_TIMER_MODE_REPEATED,
                                biblioteca_phillips_handler);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_adv_update,
                                APP_TIMER_MODE_REPEATED,
                                HR_advdata_manuf_data_update);
    APP_ERROR_CHECK(err_code);
		
		err_code = app_timer_create(&m_tick_timer,
                                APP_TIMER_MODE_REPEATED,
                                ant_plus_handler);
    APP_ERROR_CHECK(err_code);

		
}
/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_HEART_RATE_SENSOR);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED)
    {
        NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
                     p_evt->conn_handle,
                     p_evt->params.att_mtu_effective);
    }

    ble_hrs_on_gatt_evt(&m_hrs, p_evt);

}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
}

//@Including USER ID
/******************************************************************************/
static void user_id_write_handler(ble_rus_t *p_rus, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = new_state & 0xFF;

/* Validação da id que pode ser salva */
	if( temp > 99 )
		return;

/* Altera valor da id na variável temporária */
	temporary_buff.user_id.user_id_value = temp;

/* Solicita para o programa a atualização da id na memória NV */
	hr_module_nv_buf_update_flags.user_id = true;
	update_nv_mem_buf_request = true;
}

/******************************************************************************/

/******************************************************************************/

//@Including FITNESS INDEX
/******************************************************************************/
static void hr_zone_preference_calc_write_handler(ble_rus_t *p_rus, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = new_state & 0xFF;

/* Validação do coeficiente de condicionamento que pode ser salva */
	if( temp > 99 )
		return;

/* Altera valor do coeficiente de condicionamento na variável temporária */
	temporary_buff.hr_zone_preference_calc.hr_zone_preference_calc_value = temp;

/* Solicita para o programa a atualização do coeficiente de condicionamento na memória NV */
	hr_module_nv_buf_update_flags.hr_zone_preference_calc = true;
	update_nv_mem_buf_request = true;
}

/******************************************************************************/

//@Including FITNESS INDEX
/******************************************************************************/
static void serial_number_write_handler(ble_rus_t *p_rus, char * new_state){

	strcpy((char *)temporary_buff.serial_number.serial_number_value, new_state);

/* Solicita para o programa a atualização do coeficiente de condicionamento na memória NV */
	hr_module_nv_buf_update_flags.serial_number = true;
	update_nv_mem_buf_request = true;
}

static void first_name_write_handler(ble_uds_t *p_uds, uint64_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint64_t temp = new_state;// & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.first_name.first_name_value = temp;
/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.first_name = true;
	update_nv_mem_buf_request = true;
}



static void last_name_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = new_state;// & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.last_name.last_name_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.last_name = true;
	update_nv_mem_buf_request = true;
}



static void email_address_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = new_state;// & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.email_address.email_address_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.email_address = true;
	update_nv_mem_buf_request = true;
}


static void age_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = (uint8_t) new_state & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.age.age_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.age = true;
	update_nv_mem_buf_request = true;
}


static void date_of_birth_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = new_state & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.date_of_birth.date_of_birth_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.date_of_birth = true;
	update_nv_mem_buf_request = true;
}


static void gender_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = (uint8_t) new_state; // & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.gender.gender_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.gender = true;
	update_nv_mem_buf_request = true;
}


static void weight_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint16_t temp = new_state ;//& 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.weight.weight_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.weight = true;
	update_nv_mem_buf_request = true;
}

static void height_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint16_t temp = new_state;// & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.height.height_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.height = true;
	update_nv_mem_buf_request = true;
}

static void VO2_max_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = (uint8_t) new_state;// & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.VO2_max.VO2_max_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.VO2_max = true;
	update_nv_mem_buf_request = true;
}

static void heart_rate_max_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = new_state;// & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.heart_rate_max.heart_rate_max_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.heart_rate_max = true;
	update_nv_mem_buf_request = true;
}

static void resting_heart_rate_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = (uint8_t) new_state;// & 0xFF;

/* Altera valor da idade na variável temporária */
	temporary_buff.resting_heart_rate.resting_heart_rate_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.resting_heart_rate = true;
	update_nv_mem_buf_request = true;
}

static void maximum_recommended_heart_rate_write_handler(ble_uds_t *p_uds, uint32_t new_state){
/* Conversão do valor de entrada para uint8_t */
	uint8_t temp = new_state;// & 0xFF;

/* Validação da idade que pode ser salva */
	if( temp > 100 )
		return;

/* Altera valor da idade na variável temporária */
	temporary_buff.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value = temp;

/* Solicita para o programa a atualização da idade na memória NV */
	hr_module_nv_buf_update_flags.resting_heart_rate = true;
	update_nv_mem_buf_request = true;
}


/**@brief Function for handling Speed and Cadence Control point events
 *
 * @details Function for handling Speed and Cadence Control point events.
 *          This function parses the event and in case the "set cumulative value" event is received,
 *          sets the wheel cumulative value to the received value.
 */
ble_scpt_response_t sc_ctrlpt_event_handler(ble_sc_ctrlpt_t     * p_sc_ctrlpt,
                                            ble_sc_ctrlpt_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_SC_CTRLPT_EVT_SET_CUMUL_VALUE:
            m_cumulative_wheel_revs = p_evt->params.cumulative_value;
            break;

        case BLE_SC_CTRLPT_EVT_START_CALIBRATION:
            m_auto_calibration_in_progress = true;
            break;

        default:
            // No implementation needed.
            break;
    }
    return (BLE_SCPT_SUCCESS);
}



/******************************************************************************/
/******************************************************************************/
//HANDLERS FOR CALIBRATION


static void bike_id_write_handler(calib_serv_t * p_rcs, uint32_t new_state)
{
	
uint16_t i = (((0x00ff & new_state)<<8) + ((0xff00 & new_state)>>8));
teste.adc_value[0]=i;
NRF_LOG_INFO("adc_value[0] %u",teste.adc_value[0]);
				ADS018_Cal_Set((ADS018_cal_Type *)&(teste),
																	(float *)&ADS018_Cal_A,//m
																	(float *)&ADS018_Cal_B, //yo
																	(int16_t *)&ADS018_Cal_ADC_Zero, //yo/m
																	(int16_t *)&ADS018_Cal_ADC_Delta);//x-x0

}


static void zero_write_handler(calib_serv_t * p_rcs, uint32_t new_state){
//	read_gauge();
	//uint16_t temp = (((0x00ff & new_state)<<8) + ((0xff00 & new_state)>>8));
	teste.adc_value[1]=new_state;
	NRF_LOG_INFO("adc_value[1] %d",teste.adc_value[1]);
//	int16_t i = ADS018_Meas_Get_Mean();
//	// new_state not uset
//ADS018_Set_Cal_Adc_Zero(i);
					ADS018_Cal_Set((ADS018_cal_Type *)&(teste),
																	(float *)&ADS018_Cal_A,//m
																	(float *)&ADS018_Cal_B, //yo
																	(int16_t *)&ADS018_Cal_ADC_Zero, //yo/m
																	(int16_t *)&ADS018_Cal_ADC_Delta);//x-x0
}

static void ten_write_handler(calib_serv_t * p_rcs, uint32_t new_state){
	//uint16_t temp = (((0x00ff & new_state)<<8) + ((0xff00 & new_state)>>8));
	teste.eng_value[0]=new_state;
	NRF_LOG_INFO("eng_value[0] %d",teste.eng_value[0]);
//	int16_t i = ADS018_Meas_Get_Mean();
//	uint16_t j = (((0x00ff & new_state)<<8) + ((0xff00 & new_state)>>8)); //adjust byte endian
//	ADS018_Set_Cal_Adc_Ref(i);
//	ADS018_Set_Cal_Eng_Ref(Hex_To_Int(j)*10); // value in 0,01Kgf
					ADS018_Cal_Set((ADS018_cal_Type *)&(teste),
																	(float *)&ADS018_Cal_A,//m
																	(float *)&ADS018_Cal_B, //yo
																	(int16_t *)&ADS018_Cal_ADC_Zero, //yo/m
																	(int16_t *)&ADS018_Cal_ADC_Delta);//x-x0
}

static void p_load_write_handler(calib_serv_t * p_rcs, uint32_t new_state){
	//uint16_t temp = (((0x00ff & new_state)<<8) + ((0xff00 & new_state)>>8));
	teste.eng_value[1]=new_state;
	NRF_LOG_INFO("eng_value[1] %d",teste.eng_value[1]);
//	uint16_t i = (((0x00ff & new_state)<<8) + ((0xff00 & new_state)>>8));
//	ADS018_Set_Pload(Hex_To_Int(i));
}


static void n_load_write_handler(calib_serv_t * p_rcs, uint32_t new_state){
	if (new_state==2077){
		  NRF_LOG_FINAL_FLUSH();
//		  NRF_LOG_INFO("x-x0 %d",((int16_t)teste.adc_value[1]-(int16_t)teste.adc_value[0]));
      SUPER_LOG(ADS018_Cal_A,"ADS018_Cal_A ");
			SUPER_LOG(ADS018_Cal_B,"ADS018_Cal_B ");
			SUPER_LOG(ADS018_Cal_ADC_Zero,"ADS018_Cal_ADC_Zero ");
			SUPER_LOG(ADS018_Cal_ADC_Delta,"ADS018_Cal_ADC_Delta ");
	}
//	uint16_t i = (((0x00ff & new_state)<<8) + ((0xff00 & new_state)>>8));
//	ADS018_Set_Nload(Hex_To_Int(i));
}

static void serial_number_write_handler_2(calib_serv_t *p_rcs, uint32_t new_state){
	uint16_t temp= new_state;
//	ADC_sample=temp;
//	ADS018_Set_Serial_Number(i);
}

static void mode_write_handler(calib_serv_t *p_rcs, uint32_t new_state){
	if (new_state==2077){
  SUPER_LOG(ADS018_res_data.load,"pout>load");
	}
//	uint16_t i = (((0x00ff & new_state)<<8) + ((0xff00 & new_state)>>8));
//	if (i>6) i = 0;  // enable all on fault
//	ADS018_Set_Mode(i);
}

static void mean_interval_write_handler(calib_serv_t *p_rcs, uint32_t new_state){
//	__uint32_t i = (((0x000000ff & new_state)<<24) + ((0x0000ff00 & new_state)<<8) +
//				  ((0x00ff0000 & new_state)>>8)  + ((0xff000000 & new_state)>>24));
//	if (i>65536) i = 65536; //0x00010000
//	ADS018_Set_Mean_Time(Hex_To_Int2(i));
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    volatile ret_code_t         err_code;

    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    ble_sensor_location_t sensor_location;

    ble_cscs_init_t       cscs_init;
    calib_serv_init_t     calib_serv_init;
    
    ble_dfu_buttonless_init_t dfus_init = {0};
    
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
        
        // Initialize Cycling Speed and Cadence Service.
    memset(&cscs_init, 0, sizeof(cscs_init));

    cscs_init.evt_handler = NULL;
    cscs_init.feature     = BLE_CSCS_FEATURE_WHEEL_REV_BIT | BLE_CSCS_FEATURE_CRANK_REV_BIT |
                            BLE_CSCS_FEATURE_MULTIPLE_SENSORS_BIT;
   // Here the sec level for the Cycling Speed and Cadence Service can be changed/increased.
    cscs_init.csc_meas_cccd_wr_sec  = SEC_OPEN;
    cscs_init.csc_feature_rd_sec    = SEC_OPEN;
    cscs_init.csc_location_rd_sec   = SEC_OPEN;
    cscs_init.sc_ctrlpt_cccd_wr_sec = SEC_OPEN;
    cscs_init.sc_ctrlpt_wr_sec      = SEC_OPEN;

    cscs_init.ctrplt_supported_functions = BLE_SRV_SC_CTRLPT_CUM_VAL_OP_SUPPORTED
                                           | BLE_SRV_SC_CTRLPT_SENSOR_LOCATIONS_OP_SUPPORTED
                                           | BLE_SRV_SC_CTRLPT_START_CALIB_OP_SUPPORTED;
    cscs_init.ctrlpt_evt_handler            = sc_ctrlpt_event_handler;
    cscs_init.list_supported_locations      = supported_locations;
    cscs_init.size_list_supported_locations = sizeof(supported_locations) /
                                              sizeof(ble_sensor_location_t);

    sensor_location           = BLE_SENSOR_LOCATION_RIGHT_CRANK;                 // initializes the sensor location to add the sensor location characteristic.
    cscs_init.sensor_location = &sensor_location;

    err_code = ble_cscs_init(&m_cscs, &cscs_init);
    APP_ERROR_CHECK(err_code);
        
           // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
        
        ble_srv_ascii_to_utf8(&dis_init.fw_rev_str, (char*)FIRMWARE_VERSION);

        dis_init.serial_num_str.length = (12);
        dis_init.serial_num_str.p_str   = (uint8_t *) get_serial_number_metric();   
 
        dis_init.dis_char_rd_sec = SEC_OPEN;

        dfus_init.evt_handler = ble_dfu_evt_handler;
    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);      
        
        //initalize the calibration service
    calib_serv_init.bike_id_write_handler   =   bike_id_write_handler;
    calib_serv_init.zero_write_handler  =   zero_write_handler;
    calib_serv_init.ten_write_handler       =   ten_write_handler;
    calib_serv_init.p_load_write_handler    =   p_load_write_handler;
    calib_serv_init.n_load_write_handler    =   n_load_write_handler;
    calib_serv_init.serial_number_write_handler= serial_number_write_handler_2;
    calib_serv_init.mode_write_handler          = mode_write_handler;
    calib_serv_init.mean_interval_write_handler= mean_interval_write_handler;
        
    err_code = ble_calib_serv_init(&m_calib_serv, &calib_serv_init);
        APP_ERROR_CHECK(err_code);      



	
	
	
}




/**@brief Function for initializing the sensor simulators.
 */
static void sensor_simulator_init(void)
{
//    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
//    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
//    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
//    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_speed_kph_sim_cfg.min          = MIN_SPEED_KPH;
    m_speed_kph_sim_cfg.max          = MAX_SPEED_KPH;
    m_speed_kph_sim_cfg.incr         = SPEED_KPH_INCREMENT;
    m_speed_kph_sim_cfg.start_at_max = false;

    sensorsim_init(&m_speed_kph_sim_state, &m_speed_kph_sim_cfg);

    m_crank_rpm_sim_cfg.min          = MIN_CRANK_RPM;
    m_crank_rpm_sim_cfg.max          = MAX_CRANK_RPM;
    m_crank_rpm_sim_cfg.incr         = CRANK_RPM_INCREMENT;
    m_crank_rpm_sim_cfg.start_at_max = false;

    sensorsim_init(&m_crank_rpm_sim_state, &m_crank_rpm_sim_cfg);

    m_cumulative_wheel_revs        = 0;
    m_auto_calibration_in_progress = false;
}




/******************************************************************************/
/******************************************************************************/
void check_nv_update_request (void){
/* Verifica se existe alguma atualização requisitada pelo serviço bluetooth */
		if( update_nv_mem_buf_request )
		{
				update_nv_mem_buf_request = false;

				if( hr_module_nv_buf_update_flags.rhr_preference == true ){
						hr_module_nv_buf_update_flags.rhr_preference = false;

						set_resting_heart_rate_value_threshold( temporary_buff.rhr_preference.rhr_preference_value );
				}

				if( hr_module_nv_buf_update_flags.user_id == true ){
						hr_module_nv_buf_update_flags.user_id = false;

						set_user_id( temporary_buff.user_id.user_id_value );  //@Including USER ID
				}

				if( hr_module_nv_buf_update_flags.hr_zone_preference_calc == true ){
						hr_module_nv_buf_update_flags.hr_zone_preference_calc = false;

						set_hr_zone_preference_calc( temporary_buff.hr_zone_preference_calc.hr_zone_preference_calc_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.serial_number == true ){
						hr_module_nv_buf_update_flags.serial_number = false;

						set_serial_number_metric( (char *) temporary_buff.serial_number.serial_number_value );  //@Including
				}
				if( hr_module_nv_buf_update_flags.first_name == true ){
						hr_module_nv_buf_update_flags.first_name = false;
		
						set_first_name_metric( temporary_buff.first_name.first_name_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.last_name == true ){
						hr_module_nv_buf_update_flags.last_name = false;

						set_last_name_metric( temporary_buff.last_name.last_name_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.email_address == true ){
						hr_module_nv_buf_update_flags.email_address = false;

						set_email_address_metric( temporary_buff.email_address.email_address_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.age == true ){
						hr_module_nv_buf_update_flags.age = false;

						set_age_metric( temporary_buff.age.age_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.date_of_birth == true ){
						hr_module_nv_buf_update_flags.date_of_birth = false;

						set_date_of_birth_metric( temporary_buff.date_of_birth.date_of_birth_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.gender == true ){
						hr_module_nv_buf_update_flags.gender = false;

						set_gender_metric( temporary_buff.gender.gender_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.weight == true ){
						hr_module_nv_buf_update_flags.weight = false;

						set_weight_metric( temporary_buff.weight.weight_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.height == true ){
						hr_module_nv_buf_update_flags.height = false;

						set_height_metric( temporary_buff.height.height_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.VO2_max == true ){
						hr_module_nv_buf_update_flags.VO2_max = false;

						set_VO2_max_metric( temporary_buff.VO2_max.VO2_max_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.heart_rate_max == true ){
						hr_module_nv_buf_update_flags.heart_rate_max = false;

						set_heart_rate_max_metric( temporary_buff.heart_rate_max.heart_rate_max_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.resting_heart_rate == true ){
						hr_module_nv_buf_update_flags.resting_heart_rate = false;

						set_resting_heart_rate_metric( temporary_buff.resting_heart_rate.resting_heart_rate_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.maximum_recommended_heart_rate == true ){
						hr_module_nv_buf_update_flags.maximum_recommended_heart_rate = false;

						set_maximum_recommended_heart_rate_metric( temporary_buff.maximum_recommended_heart_rate.maximum_recommended_heart_rate_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.aerobic_threshold == true ){
						hr_module_nv_buf_update_flags.aerobic_threshold = false;

						set_aerobic_threshold_metric( temporary_buff.aerobic_threshold.aerobic_threshold_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.anaerobic_threshold == true ){
						hr_module_nv_buf_update_flags.anaerobic_threshold = false;

						set_anaerobic_threshold_metric( temporary_buff.anaerobic_threshold.anaerobic_threshold_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.sport_type_for_aerobic_and_anaerobic_thresholds == true ){
						hr_module_nv_buf_update_flags.sport_type_for_aerobic_and_anaerobic_thresholds = false;

						set_sport_type_for_aerobic_and_anaerobic_thresholds_metric( temporary_buff.sport_type_for_aerobic_and_anaerobic_thresholds.sport_type_for_aerobic_and_anaerobic_thresholds_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.date_of_threshold_assessment == true ){
						hr_module_nv_buf_update_flags.date_of_threshold_assessment = false;

						set_date_of_threshold_assessment_metric( temporary_buff.date_of_threshold_assessment.date_of_threshold_assessment_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.waist_circumference == true ){
						hr_module_nv_buf_update_flags.waist_circumference = false;

						set_waist_circumference_metric( temporary_buff.waist_circumference.waist_circumference_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.fat_burn_heart_rate_lower_limit == true ){
						hr_module_nv_buf_update_flags.fat_burn_heart_rate_lower_limit = false;

						set_fat_burn_heart_rate_lower_limit_metric( temporary_buff.fat_burn_heart_rate_lower_limit.fat_burn_heart_rate_lower_limit_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.fat_burn_heart_rate_upper_limit == true ){
						hr_module_nv_buf_update_flags.fat_burn_heart_rate_upper_limit = false;

						set_fat_burn_heart_rate_upper_limit_metric( temporary_buff.fat_burn_heart_rate_upper_limit.fat_burn_heart_rate_upper_limit_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.aerobic_heart_rate_lower_limit == true ){
						hr_module_nv_buf_update_flags.aerobic_heart_rate_lower_limit = false;

						set_aerobic_heart_rate_lower_limit_metric( temporary_buff.aerobic_heart_rate_lower_limit.aerobic_heart_rate_lower_limit_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.aerobic_heart_rate_upper_limit == true ){
						hr_module_nv_buf_update_flags.aerobic_heart_rate_upper_limit = false;

						set_aerobic_heart_rate_upper_limit_metric( temporary_buff.aerobic_heart_rate_upper_limit.aerobic_heart_rate_upper_limit_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.anaerobic_heart_rate_lower_limit == true ){
						hr_module_nv_buf_update_flags.anaerobic_heart_rate_lower_limit = false;

						set_anaerobic_heart_rate_lower_limit_metric( temporary_buff.anaerobic_heart_rate_lower_limit.anaerobic_heart_rate_lower_limit_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.five_zone_heart_rate_limits == true ){
						hr_module_nv_buf_update_flags.five_zone_heart_rate_limits = false;

						set_five_zone_heart_rate_limits_metric( temporary_buff.five_zone_heart_rate_limits.five_zone_heart_rate_limits_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.three_zone_heart_rate_limits == true ){
						hr_module_nv_buf_update_flags.three_zone_heart_rate_limits = false;

						set_three_zone_heart_rate_limits_metric( temporary_buff.three_zone_heart_rate_limits.three_zone_heart_rate_limits_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.two_zone_heart_rate_limit == true ){
						hr_module_nv_buf_update_flags.two_zone_heart_rate_limit = false;

						set_two_zone_heart_rate_limit_metric( temporary_buff.two_zone_heart_rate_limit.two_zone_heart_rate_limit_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.language == true ){
						hr_module_nv_buf_update_flags.language = false;

						set_language_metric( temporary_buff.language.language_value );  //@Including
				}

				if( hr_module_nv_buf_update_flags.fitnes_index == true ){
						hr_module_nv_buf_update_flags.fitnes_index = false;

						set_fitnes_index_metric( temporary_buff.fitnes_index.fitnes_index_value );  //@Including
				}



				if( hr_module_nv_buf_update_flags.hr_SPIVI_zone_limits == true ){
						hr_module_nv_buf_update_flags.hr_SPIVI_zone_limits = false;

				}

				if( hr_module_nv_buf_update_flags.hr_rhr_zone_limits == true ){
						hr_module_nv_buf_update_flags.hr_rhr_zone_limits = false;

				}

				buff_save();
	
				nrf_delay_ms(10);

		    buff_load();

				load_user_info_metrics();
		}
}



/**@brief Function for starting application timers.
 */
static void application_timers_start(void)
{
    ret_code_t err_code;
	   uint32_t csc_meas_timer_ticks;
	
    // Start application timers.
    err_code = app_timer_start(m_battery_timer_id, BATTERY_LEVEL_MEAS_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
		
//    err_code = app_timer_start(m_heart_rate_timer_id, HEART_RATE_MEAS_INTERVAL, NULL);
//    APP_ERROR_CHECK(err_code);
	  
//		err_code = app_timer_start(m_biblioteca_phillips_id, APP_TIMER_TICKS(64), NULL);
//    APP_ERROR_CHECK(err_code);
	
		err_code = app_timer_start(m_adv_update, APP_TIMER_TICKS(49), NULL);////APP_TIMER_TICKS(341)//err_code = app_timer_start(m_adv_update, APP_TIMER_TICKS(4400), NULL);//APP_TIMER_TICKS(341)
	  APP_ERROR_CHECK(err_code);
	  //mudado338
	
    // Schedule a timeout event every 2 seconds
    err_code = app_timer_start(m_tick_timer, APP_TICK_EVENT_INTERVAL, NULL);
    APP_ERROR_CHECK(err_code);
	
	//start timer for Cadence Speed
	csc_meas_timer_ticks = APP_TIMER_TICKS(SPEED_AND_CADENCE_MEAS_INTERVAL);//500

    err_code = app_timer_start(m_csc_meas_timer_id, csc_meas_timer_ticks, NULL);
    APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;
    
    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

static void sleep_mybeat(void){

		ret_code_t err_code;
		
		(void)mybeat_sleep();
		nrfx_saadc_uninit();
		nrf_drv_spi_uninit(&mLisSpiInstance);
		bsp_board_leds_off();	
		pwm_stop();
		NRF_LOG_FINAL_FLUSH();
		NRF_LOG_DEFAULT_BACKENDS_UNINIT();
	
		nrf_gpio_cfg_input(CLK_32K,NRF_GPIO_PIN_PULLDOWN);
	
		nrf_gpio_cfg_input(ACC_nSS,NRF_GPIO_PIN_PULLUP);
		nrf_gpio_cfg_input(AFE_nSS,NRF_GPIO_PIN_PULLUP);
	
		nrf_gpio_cfg_input(RX,NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(TX,NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(CTS,NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_input(RTS,NRF_GPIO_PIN_PULLDOWN);
	
		nrf_gpio_cfg_input(SPI_MISO_PIN, NRF_GPIO_PIN_PULLUP);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
			
}

/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
		solicitacaoDesligamento=true;
	
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

uint32_t hl_lh(uint32_t d)
{
       return ((((d & 0x000000ff)<<24)&0xff000000) | (((d & 0x0000ff00)<<8)&0x00ff0000) | (((d & 0x00ff0000)>>8)&0x0000ff00) | (((d & 0xff000000)>>24)&0x000000ff));
}

uint32_t Int_to_Hex(int16_t vi)
{
       return (((vi/10000)<<16) | (((vi%10000)/1000)<<12) | (((vi%1000)/100)<<8) | (((vi%100)/10)<<4) | (vi%10));
}

void reply_serial_number(char * value){
//		char *temp = malloc(2*(sizeof(uint8_t *)));
		ble_gatts_rw_authorize_reply_params_t auth_reply;
		auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
		auth_reply.params.read.update = 1;
		auth_reply.params.read.offset = 0;
		auth_reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;	
	
		auth_reply.params.read.len = 12;

		auth_reply.params.read.p_data = (uint8_t *) value;
		sd_ble_gatts_rw_authorize_reply(m_conn_handle, &auth_reply);

}


void reply(int16_t value){
		uint8_t *temp = malloc(2*(sizeof(uint8_t *)));
		ble_gatts_rw_authorize_reply_params_t auth_reply;
		auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
		auth_reply.params.read.update = 1;
		auth_reply.params.read.offset = 0;
		auth_reply.params.read.len = 1;
		auth_reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;	
	
		auth_reply.params.read.len = 4;

		uint32_t data2;
		if(value < 0)
				data2 = hl_lh(Int_to_Hex(-value));
		else
				data2 = hl_lh(Int_to_Hex(value));

		if(value < 0)
				temp[0] = 0xff;
		else
				temp[0] = (data2>>0);
		temp[1] = (data2>>8);
		temp[2] = (data2>>16);
		temp[3] = (data2>>24);
		auth_reply.params.read.p_data = temp;
		sd_ble_gatts_rw_authorize_reply(m_conn_handle, &auth_reply);
		free(temp);		
}

void reply_8bits(int8_t value){
		uint8_t *temp = malloc(2*(sizeof(uint8_t *)));
		ble_gatts_rw_authorize_reply_params_t auth_reply;
		auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
		auth_reply.params.read.update = 1;
		auth_reply.params.read.offset = 0;
		auth_reply.params.read.len = 1;
		auth_reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;	
	
		auth_reply.params.read.len = 1;

		uint8_t data2 = value;

		temp[0] = data2;

		auth_reply.params.read.p_data = temp;
		sd_ble_gatts_rw_authorize_reply(m_conn_handle, &auth_reply);
		free(temp);
}

void reply_16bits(int16_t value){
		uint8_t *temp = malloc(2*(sizeof(uint8_t *)));	
		ble_gatts_rw_authorize_reply_params_t auth_reply;
		auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
		auth_reply.params.read.update = 1;
		auth_reply.params.read.offset = 0;
		auth_reply.params.read.len = 1;
		auth_reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;	
		
		auth_reply.params.read.len = 2;

		uint16_t data2=value;
		if(value < 0)
				temp[0] = 0x00;
		else{
		temp[0] = (data2>>0);
		temp[1] = (data2>>8);}
		auth_reply.params.read.p_data = temp;
		sd_ble_gatts_rw_authorize_reply(m_conn_handle, &auth_reply);
		free(temp);
}


void reply_name(int64_t value){
		uint8_t *temp = malloc(2*(sizeof(uint8_t *)));
		ble_gatts_rw_authorize_reply_params_t auth_reply;
		auth_reply.type = BLE_GATTS_AUTHORIZE_TYPE_READ;
		auth_reply.params.read.update = 1;
		auth_reply.params.read.offset = 0;
		auth_reply.params.read.len = 1;
		auth_reply.params.read.gatt_status = BLE_GATT_STATUS_SUCCESS;	
	
		auth_reply.params.read.len = 8;

		uint64_t data2=value;

		temp[7] = (data2>>0);
		temp[6] = (data2>>8);
		temp[5] = (data2>>16);
		temp[4] = (data2>>24);
		temp[3] = (data2>>32);
		temp[2] = (data2>>40);
		temp[1] = (data2>>48);
		temp[0] = (data2>>56);
		auth_reply.params.read.p_data = temp;
		sd_ble_gatts_rw_authorize_reply(m_conn_handle, &auth_reply);
		free(temp);
}


static uint32_t rw_request(ble_evt_t * p_ble_evt){
	
		hr_module_metric_info_t *metrics = get_metrics();
		uint16_t handle = p_ble_evt->evt.gatts_evt.params.authorize_request.request.read.handle;
	
		if(handle == m_rus.user_id_char_handles.value_handle)
				reply((uint16_t)get_user_id());

		if(handle == m_rus.hr_zone_preference_calc_char_handles.value_handle)
				reply((uint16_t)get_hr_zone_preference_calc());					//@Including FITNESS INDEX

		if(handle == m_rus.serial_number_char_handles.value_handle)
				reply_serial_number(get_serial_number_metric());					//@Including

		if(handle == m_uds.first_name_char_handler.value_handle)
				reply_name((uint64_t)get_first_name_metric());					//@Including

		if(handle == m_uds.last_name_char_handler.value_handle)
				reply((uint16_t)get_last_name_metric());					//@Including

		if(handle == m_uds.email_address_char_handler.value_handle)
				reply((uint16_t)get_email_address_metric());					//@Including

		if(handle == m_uds.age_char_handler.value_handle)
				reply_8bits((uint8_t)get_age_metric());					//@Including

		if(handle == m_uds.date_of_birth_char_handler.value_handle)
				reply((uint16_t)get_date_of_birth_metric());					//@Including

		if(handle == m_uds.gender_char_handler.value_handle)
				reply_8bits((uint8_t)get_gender_metric());					//@Including

		if(handle == m_uds.weight_char_handler.value_handle)
				reply_16bits((uint16_t)get_weight_metric());					//@Including

		if(handle == m_uds.height_char_handler.value_handle)
				reply_16bits((uint16_t)get_height_metric());					//@Including

		if(handle == m_uds.VO2_max_char_handler.value_handle)
				reply_8bits((uint8_t)get_VO2_max_metric());					//@Including

		if(handle == m_uds.heart_rate_max_char_handler.value_handle)
				reply((uint16_t)get_heart_rate_max_metric());					//@Including

		if(handle == m_uds.resting_heart_char_handler.value_handle)
				reply_8bits((uint8_t)get_resting_heart_rate_metric());					//@Including

		if(handle == m_uds.maximum_recommended_heart_rate_char_handler.value_handle)
				reply((uint16_t)get_maximum_recommended_heart_rate_metric());					//@Including

		if(handle == m_uds.aerobic_threshold_char_handler.value_handle)
				reply_8bits((uint8_t)get_aerobic_threshold_metric());					//@Including

		if(handle == m_uds.anaerobic_threshold_char_handler.value_handle)
				reply_8bits((uint8_t)get_anaerobic_threshold_metric());					//@Including

		if(handle == m_uds.sport_type_for_aerobic_and_anaerobic_thresholds_char_handler.value_handle)
				reply((uint16_t)get_sport_type_for_aerobic_and_anaerobic_thresholds_metric());					//@Including

		if(handle == m_uds.date_of_threshold_assessment_char_handler.value_handle)
				reply((uint16_t)get_date_of_threshold_assessment_metric());					//@Including

		if(handle == m_uds.waist_circumference_char_handler.value_handle)
				reply((uint16_t)get_waist_circumference_metric());					//@Including

		if(handle == m_uds.fat_burn_heart_rate_lower_limit_char_handler.value_handle)
				reply((uint16_t)get_fat_burn_heart_rate_lower_limit_metric());					//@Including

		if(handle == m_uds.fat_burn_heart_rate_upper_limit_char_handler.value_handle)
				reply((uint16_t)get_fat_burn_heart_rate_upper_limit_metric());					//@Including

		if(handle == m_uds.aerobic_heart_rate_lower_limit_char_handler.value_handle)
				reply((uint16_t)get_aerobic_heart_rate_lower_limit_metric());					//@Including

		if(handle == m_uds.aerobic_heart_rate_upper_limit_char_handler.value_handle)
				reply((uint16_t)get_aerobic_heart_rate_upper_limit_metric());					//@Including

		if(handle == m_uds.anaerobic_heart_rate_lower_limit_char_handler.value_handle)
				reply((uint16_t)get_anaerobic_heart_rate_lower_limit_metric());					//@Including

		if(handle == m_uds.five_zone_heart_rate_limits_char_handler.value_handle)
				reply((uint16_t)get_five_zone_heart_rate_limits_metric());					//@Including

		if(handle == m_uds.three_zone_heart_rate_limits_char_handler.value_handle)
				reply((uint16_t)get_three_zone_heart_rate_limits_metric());					//@Including

		if(handle == m_uds.two_zone_heart_rate_limits_char_handler.value_handle)
				reply((uint16_t)get_two_zone_heart_rate_limit_metric());					//@Including

		if(handle == m_uds.language_char_handler.value_handle)
				reply((uint16_t)get_language_metric());					//@Including

		if(handle == m_uds.fitnes_index_char_handler.value_handle)
				reply_8bits((uint8_t)get_fitnes_index_metric());					//@Including

		if(handle == m_rtcs.SPIVI_zone1_threshold_char_handles.value_handle){
				reply((uint16_t)get_SPIVI_zone1_percentage_threshold());
		}

		if(handle == m_rtcs.SPIVI_zone2_threshold_char_handles.value_handle){
				reply((uint16_t)get_SPIVI_zone2_percentage_threshold());
		}

		if(handle == m_rtcs.SPIVI_zone3_threshold_char_handles.value_handle){
				reply((uint16_t)get_SPIVI_zone3_percentage_threshold());
		}

		if(handle == m_rtcs.SPIVI_zone4_threshold_char_handles.value_handle){
				reply((uint16_t)get_SPIVI_zone4_percentage_threshold());
		}

		if(handle == m_rtcs.SPIVI_zone5_threshold_char_handles.value_handle){
				reply((uint16_t)get_SPIVI_zone5_percentage_threshold());
		}

		if(handle == m_rtcs.rhr_zone1_threshold_char_handles.value_handle){
				reply((uint16_t)get_rhr_zone1_percentage_threshold());
		}

		if(handle == m_rtcs.rhr_zone2_threshold_char_handles.value_handle){
				reply((uint16_t)get_rhr_zone2_percentage_threshold());
		}

		if(handle == m_rtcs.rhr_zone3_threshold_char_handles.value_handle){
				reply((uint16_t)get_rhr_zone3_percentage_threshold());
		}

		if(handle == m_rtcs.rhr_zone4_threshold_char_handles.value_handle){
				reply((uint16_t)get_rhr_zone4_percentage_threshold());
		}

		if(handle == m_rtcs.rhr_zone5_threshold_char_handles.value_handle){
				reply((uint16_t)get_rhr_zone5_percentage_threshold());
		}

		return 1;
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected, reason %d.",
                          p_ble_evt->evt.gap_evt.params.disconnected.reason);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
    
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
            break;
        
        case BLE_GAP_EVT_AUTH_KEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
            break;

				case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
            NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
            break;

				case BLE_GAP_EVT_AUTH_STATUS:
             NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
                          p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
                          p_ble_evt->evt.gap_evt.params.auth_status.bonded,
                          p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
                          *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
            break;
				
				case BLE_GATTS_EVT_RW_AUTHORIZE_REQUEST:
						rw_request(p_ble_evt);
						break;
        
				default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

		err_code = nrf_sdh_ant_enable();
    APP_ERROR_CHECK(err_code);

    err_code = ant_plus_key_set(ANTPLUS_NETWORK_NUM);
    APP_ERROR_CHECK(err_code);
	
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
		NRF_SDH_ANT_OBSERVER(m_ant_observer, ANT_HRM_ANT_OBSERVER_PRIO, ant_hrm_sens_evt_handler, &m_ant_hrm);
		
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;
#ifdef WDT_ATIVO
		//		case BSP_EVENT_KEY_0:
            nrf_drv_wdt_channel_feed(m_channel_id);
    //        break;
#endif				
		
						
        default:
            break;
    }
}


/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)//aqui as métricas começam com um valor
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               	= BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      	= false;//true;
    init.advdata.flags                   	= BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt 	= sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  	= m_adv_uuids;
	
    init.config.ble_adv_fast_enabled  		= true;
    init.config.ble_adv_fast_interval 		= APP_ADV_INTERVAL;//originalmente setado como 300.
    init.config.ble_adv_fast_timeout  		= 0;
	
		HR_advertising_data.RPM       = 0x0000;//(uint8_t)get_user_id();
		HR_advertising_data.HR 		 		= 0x7777;//(uint8_t)get_gender_metric();
		HR_advertising_data.Power 		= 0x4444;//(uint8_t)get_color();
		HR_advertising_data.Kcal      = 0x5555;// BPM;
		HR_advertising_data.Minutes   = 0xAC;//(uint8_t)get_hr_percent();
		HR_advertising_data.seconds   = 0xAC;//((uint8_t)(get_first_name_metric()>>56));//primeira Letra do nome - m 
		HR_advertising_data.trip      = 0x3333;//((uint8_t)(get_first_name_metric()>>48));//segunda Letra do nome - y 
    init.evt_handler = on_adv_evt;
		
		init.srdata.p_manuf_specific_data 		= &hr_adv_manuf_data;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV,m_advertising.adv_handle,4);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
				
}


void get_accel(void)

{
   	get_value();
	  ADS018_Time_Update();
	  filter(global_mixer,IN_dummy,(int16_t *)&value_for_simu_F,(int16_t *)&OUT_dummy);
	
}
void cycle_treat(void)
{
	
	ADS018_Cycle(value_for_simu_F);
	ADS018_Update_SCycle();
	ADS018_Cycle_Flag=0;
	if (ADS018_ShowCounter == ADS018_ShowCounter_SetMean) ADS018_Set_Mean_Data(); 
	
}

void HR_advdata_manuf_data_update(void * p_context)//changes in the service.
{		
		UNUSED_PARAMETER(p_context);
		ble_advdata_t advdata;
	  ble_advdata_t srdata;
		ret_code_t           err_code;
		ble_advdata_manuf_data_t 		   adv_manuf_data;
    //ADS018_res_data_f_len=0xB;//testar posição do len no advertising.
	 	pHR_adv_user_data->RPM         = global_mixer; //ADS018_res_data_c.rotation;//(uint8_t)get_user_id();
		pHR_adv_user_data->HR 		 		 = 0xAA;//(uint8_t)get_gender_metric();
		pHR_adv_user_data->Power 		   = value_for_simu_F;//(uint8_t)get_color();//terá de ser valor aleatório sem biblioteca
		pHR_adv_user_data->Kcal        = 0xAA;//  BPM;aleatório
		pHR_adv_user_data->Minutes     = 0xAA;//uint8_t)get_hr_percent();====================================================================================================================================================================================================================
		pHR_adv_user_data->seconds     = 0xAF;//((uint8_t)(get_first_name_metric()>>56));//primeira Letra do nome - m 
		pHR_adv_user_data->trip        = ADS018_res_data_c.rotation;//((uint8_t)(get_first_name_metric()>>48));//segunda Letra do nome - y 
    
    	   
		memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = false;//true;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    srdata.uuids_complete.p_uuids  = m_adv_uuids;
		adv_manuf_data.data.p_data        = (uint8_t*)pHR_adv_user_data;
		adv_manuf_data.data.size          = N_USER_FIELD;
		adv_manuf_data.company_identifier = 0x02BE; //07D0
		advdata.p_manuf_specific_data     = &adv_manuf_data;
	
		err_code=ble_advertising_advdata_update(&m_advertising, &advdata, NULL);
		APP_ERROR_CHECK(err_code);
		
		nrf_drv_wdt_channel_feed(m_channel_id);
		
		ant_hrm_measurement_update(&m_ant_hrm_measurement, (uint32_t)BPM);
}




/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void button_and_leds_init(bool * p_erase_bonds)
{
		ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init( BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
	
		nrf_gpio_cfg_output(LD1_A);
		nrf_gpio_cfg_output(LD2_A);
		nrf_gpio_cfg_output(LD3_A);	
		nrf_gpio_cfg_output(LD4_A);
		nrf_gpio_cfg_input(AFE_EN32K,NRF_GPIO_PIN_PULLDOWN);
		nrf_gpio_cfg_output(AFE_nSS);
		nrf_gpio_pin_set(AFE_nSS);
		nrf_gpio_cfg_output(ACC_nSS);
		nrf_gpio_pin_set(ACC_nSS);
		nrf_gpio_cfg_input(ACC_IRQ,NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(AFE_IRQ,NRF_GPIO_PIN_NOPULL);
		nrf_gpio_cfg_input(USB,NRF_GPIO_PIN_NOPULL);
		
}

/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(app_timer_cnt_get);
		//ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**
 * @brief SPI user event handler.
 * @param event
 */
//void spi_event_handler(nrf_drv_spi_evt_t const * p_event,
//                       void *                    p_context)
//{
//    mLisPacketTransferComplete = true;
//}


dadosBbPPG bufferPPG;

void resample_PPG (uint8_t numeroAmostra, dadosBbPPG *dadosPPG ){
		if(numeroAmostra<2){
							
				dadosPPG->amostra2=dadosPPG->amostra1;

				dadosPPG->amostra1.vetor[0]=(bufferPPG.amostra2.vetor[0]+dadosPPG->amostra2.vetor[0])/2;							//PPG
				dadosPPG->amostra1.vetor[1]=(bufferPPG.amostra2.vetor[1]+dadosPPG->amostra2.vetor[1])/2;							//AMBIENT
		}
		
//		NRF_LOG_INFO("amostra1:%ld",dadosPPG->amostra1.vetor[0]);
		
//		NRF_LOG_INFO("amostra2:%ld",dadosPPG->amostra2.vetor[0]);

		bufferPPG=*dadosPPG;
		
}


dadosBbACC PPG_historico;

void resample_ACC (uint8_t numeroAmostra, dadosBbACC *dadosACC ){

		if(	numeroAmostra==6){
				dadosACC->amostra8=dadosACC->amostra6;
				dadosACC->amostra7=dadosACC->amostra5;
				dadosACC->amostra5=dadosACC->amostra4;
				dadosACC->amostra4=dadosACC->amostra3;
		
				dadosACC->amostra6.vetor[0]=(dadosACC->amostra5.vetor[0]+dadosACC->amostra7.vetor[0])/2;
				dadosACC->amostra6.vetor[1]=(dadosACC->amostra5.vetor[1]+dadosACC->amostra7.vetor[1])/2;
				dadosACC->amostra6.vetor[2]=(dadosACC->amostra5.vetor[2]+dadosACC->amostra7.vetor[2])/2;
		
				dadosACC->amostra3.vetor[0]=(dadosACC->amostra2.vetor[0]+dadosACC->amostra4.vetor[0])/2;
				dadosACC->amostra3.vetor[1]=(dadosACC->amostra2.vetor[1]+dadosACC->amostra4.vetor[1])/2;
				dadosACC->amostra3.vetor[2]=(dadosACC->amostra2.vetor[2]+dadosACC->amostra4.vetor[2])/2;
								
		}
		if(	numeroAmostra==7){
				dadosACC->amostra8=dadosACC->amostra7;
				dadosACC->amostra7=dadosACC->amostra6;
				dadosACC->amostra6=dadosACC->amostra5;
								
				dadosACC->amostra5.vetor[0]=(dadosACC->amostra4.vetor[0]+dadosACC->amostra6.vetor[0])/2;
				dadosACC->amostra5.vetor[1]=(dadosACC->amostra4.vetor[1]+dadosACC->amostra6.vetor[1])/2;
				dadosACC->amostra5.vetor[2]=(dadosACC->amostra4.vetor[2]+dadosACC->amostra6.vetor[2])/2;
							
		}		
	
}


void amazenar_ACC(uint8_t * numeroAmostra, dadosBbACC * bufferACC){
		uint8_t indiceAmostra = *numeroAmostra;
		static axis3bit16_t data_raw_acceleration;
	
	
		lis2dw12_acceleration_raw_get(&dev_ctx, data_raw_acceleration.u8bit);
		data_raw_acceleration.i16bit[1]=((data_raw_acceleration.i16bit[1]>>4)&0xFFFF);	
		data_raw_acceleration.i16bit[0]=((data_raw_acceleration.i16bit[0]>>4)&0xFFFF);
		data_raw_acceleration.i16bit[2]=((data_raw_acceleration.i16bit[2]>>4)&0xFFFF);
	
//		NRF_LOG_INFO("2B:%ld,%ld,%ld %X,%X,%X" ,data_raw_acceleration.i16bit[1],data_raw_acceleration.i16bit[0],data_raw_acceleration.i16bit[2],(uint16_t)data_raw_acceleration.i16bit[1],(uint16_t)data_raw_acceleration.i16bit[0],(uint16_t)data_raw_acceleration.i16bit[2]);
	
		if(indiceAmostra==0){						
				bufferACC->amostra1.dadosACC.eixoYlsb=data_raw_acceleration.u8bit[0]; 					//LSB EIXO Y
				bufferACC->amostra1.dadosACC.eixoYmsb=data_raw_acceleration.u8bit[1]; 					//MSB EIXO Y
				bufferACC->amostra1.dadosACC.eixoXlsb=data_raw_acceleration.u8bit[2]; 					//LSB EIXO X
				bufferACC->amostra1.dadosACC.eixoXmsb=data_raw_acceleration.u8bit[3]; 					//MSB EIXO X
				bufferACC->amostra1.dadosACC.eixoZlsb=data_raw_acceleration.u8bit[4]; 					//LSB EIXO Z
				bufferACC->amostra1.dadosACC.eixoZmsb=data_raw_acceleration.u8bit[5]; 					//MSB EIXO Z
				indiceAmostra++;
		}
		else if(indiceAmostra==1){
				bufferACC->amostra2.dadosACC.eixoYlsb=data_raw_acceleration.u8bit[0]; 					//LSB EIXO Y
				bufferACC->amostra2.dadosACC.eixoYmsb=data_raw_acceleration.u8bit[1]; 					//MSB EIXO Y
				bufferACC->amostra2.dadosACC.eixoXlsb=data_raw_acceleration.u8bit[2]; 					//LSB EIXO X
				bufferACC->amostra2.dadosACC.eixoXmsb=data_raw_acceleration.u8bit[3]; 					//MSB EIXO X
				bufferACC->amostra2.dadosACC.eixoZlsb=data_raw_acceleration.u8bit[4]; 					//LSB EIXO Z
				bufferACC->amostra2.dadosACC.eixoZmsb=data_raw_acceleration.u8bit[5]; 					//MSB EIXO Z
				indiceAmostra++;
		}	
		else if(indiceAmostra==2){
				bufferACC->amostra3.dadosACC.eixoYlsb=data_raw_acceleration.u8bit[0]; 					//LSB EIXO Y
				bufferACC->amostra3.dadosACC.eixoYmsb=data_raw_acceleration.u8bit[1]; 					//MSB EIXO Y
				bufferACC->amostra3.dadosACC.eixoXlsb=data_raw_acceleration.u8bit[2]; 					//LSB EIXO X
				bufferACC->amostra3.dadosACC.eixoXmsb=data_raw_acceleration.u8bit[3]; 					//MSB EIXO X
				bufferACC->amostra3.dadosACC.eixoZlsb=data_raw_acceleration.u8bit[4]; 					//LSB EIXO Z
				bufferACC->amostra3.dadosACC.eixoZmsb=data_raw_acceleration.u8bit[5]; 					//MSB EIXO Z
				indiceAmostra++;
		}	
		else if(indiceAmostra==3){
				bufferACC->amostra4.dadosACC.eixoYlsb=data_raw_acceleration.u8bit[0]; 					//LSB EIXO Y
				bufferACC->amostra4.dadosACC.eixoYmsb=data_raw_acceleration.u8bit[1]; 					//MSB EIXO Y
				bufferACC->amostra4.dadosACC.eixoXlsb=data_raw_acceleration.u8bit[2]; 					//LSB EIXO X
				bufferACC->amostra4.dadosACC.eixoXmsb=data_raw_acceleration.u8bit[3]; 					//MSB EIXO X
				bufferACC->amostra4.dadosACC.eixoZlsb=data_raw_acceleration.u8bit[4]; 					//LSB EIXO Z
				bufferACC->amostra4.dadosACC.eixoZmsb=data_raw_acceleration.u8bit[5]; 					//MSB EIXO Z
				indiceAmostra++;
		}	
		else if(indiceAmostra==4){
				bufferACC->amostra5.dadosACC.eixoYlsb=data_raw_acceleration.u8bit[0]; 					//LSB EIXO Y
				bufferACC->amostra5.dadosACC.eixoYmsb=data_raw_acceleration.u8bit[1]; 					//MSB EIXO Y
				bufferACC->amostra5.dadosACC.eixoXlsb=data_raw_acceleration.u8bit[2]; 					//LSB EIXO X
				bufferACC->amostra5.dadosACC.eixoXmsb=data_raw_acceleration.u8bit[3]; 					//MSB EIXO X
				bufferACC->amostra5.dadosACC.eixoZlsb=data_raw_acceleration.u8bit[4]; 					//LSB EIXO Z
				bufferACC->amostra5.dadosACC.eixoZmsb=data_raw_acceleration.u8bit[5]; 					//MSB EIXO Z
				indiceAmostra++;
		}						
		else if(indiceAmostra==5){
				bufferACC->amostra6.dadosACC.eixoYlsb=data_raw_acceleration.u8bit[0]; 					//LSB EIXO Y
				bufferACC->amostra6.dadosACC.eixoYmsb=data_raw_acceleration.u8bit[1]; 					//MSB EIXO Y
				bufferACC->amostra6.dadosACC.eixoXlsb=data_raw_acceleration.u8bit[2]; 					//LSB EIXO X
				bufferACC->amostra6.dadosACC.eixoXmsb=data_raw_acceleration.u8bit[3]; 					//MSB EIXO X
				bufferACC->amostra6.dadosACC.eixoZlsb=data_raw_acceleration.u8bit[4]; 					//LSB EIXO Z
				bufferACC->amostra6.dadosACC.eixoZmsb=data_raw_acceleration.u8bit[5]; 					//MSB EIXO Z
				indiceAmostra++;
		}	
		else if(indiceAmostra==6){
				bufferACC->amostra7.dadosACC.eixoYlsb=data_raw_acceleration.u8bit[0]; 					//LSB EIXO Y
				bufferACC->amostra7.dadosACC.eixoYmsb=data_raw_acceleration.u8bit[1]; 					//MSB EIXO Y
				bufferACC->amostra7.dadosACC.eixoXlsb=data_raw_acceleration.u8bit[2]; 					//LSB EIXO X
				bufferACC->amostra7.dadosACC.eixoXmsb=data_raw_acceleration.u8bit[3]; 					//MSB EIXO X
				bufferACC->amostra7.dadosACC.eixoZlsb=data_raw_acceleration.u8bit[4]; 					//LSB EIXO Z
				bufferACC->amostra7.dadosACC.eixoZmsb=data_raw_acceleration.u8bit[5]; 					//MSB EIXO Z
				indiceAmostra++;
		}	
		else if(indiceAmostra==7){
				bufferACC->amostra8.dadosACC.eixoYlsb=data_raw_acceleration.u8bit[0]; 					//LSB EIXO Y
				bufferACC->amostra8.dadosACC.eixoYmsb=data_raw_acceleration.u8bit[1]; 					//MSB EIXO Y
				bufferACC->amostra8.dadosACC.eixoXlsb=data_raw_acceleration.u8bit[2]; 					//LSB EIXO X
				bufferACC->amostra8.dadosACC.eixoXmsb=data_raw_acceleration.u8bit[3]; 					//MSB EIXO X
				bufferACC->amostra8.dadosACC.eixoZlsb=data_raw_acceleration.u8bit[4]; 					//LSB EIXO Z
				bufferACC->amostra8.dadosACC.eixoZmsb=data_raw_acceleration.u8bit[5]; 					//MSB EIXO Z
				indiceAmostra++;
		}
		else{
				indiceAmostra=0;
		}	
	
		*numeroAmostra=indiceAmostra;
}

void check_init(bool code_error1, bool code_error2){
		bool status = code_error1&&code_error2;
		if(!status){
				set_led(LED4);
				set_cor(vermelho);
				nrf_delay_ms(1000);
				solicitacaoDesligamento=true;
		}
		else{
				set_led(LED4);
				set_cor(azul);				
		}
		nrf_delay_ms(350);
}



void normal_operation(void)
{
//	  get_accel();//this or get_accel();
//	  simulador();//overwrite global_mixer for testing purpouses
         	
//	  contador7++;
//	  mean=global_mixer+mean;
//	if (contador7 >15){
//		     global_mixer=mean/contador7;
////		     NRF_LOG_INFO("media %d ", global_mixer);
//		     mean=0;
//		     contador7=0;
//		     get_direction();//to adjust the tare;
			   //later this direct input of values will be removed
//					teste.adc_value[0]=50; 
//					teste.adc_value[1]=1200;
//					teste.eng_value[0]=0;
//					teste.eng_value[1]=10;
			//later the above input of values will be removed
//				int lock=0;//this lock will be an AND between two variables in the 10 and 0 characteristic
//			if (lock==0){

//										}
//	        read_gauge_init();//inicializa leitura do adc
//					read_gauge();
//					nrfx_saadc_uninit();
//					
					filter(global_mixer,global_mixer,(int16_t *)&value_for_simu_F,(int16_t *)&OUT_dummy);
//					NRF_LOG_INFO("OUT_dummy %d",OUT_dummy);
					get_direction();
					//ADS018_AZ();//ajusta tara
					get_load(OUT_dummy);//ajusto todos os valores de load
					cycle_treat();
//																}		
#ifdef  CAL_SET_SHOW
			nrf_delay_us(500);
			NRF_LOG_FINAL_FLUSH();
      SUPER_LOG(ADS018_Cal_A,"ADS018_Cal_A ");
			SUPER_LOG(ADS018_Cal_B,"ADS018_Cal_B ");
			SUPER_LOG(ADS018_Cal_ADC_Zero,"ADS018_Cal_ADC_Zero ");
			SUPER_LOG(ADS018_Cal_ADC_Delta,"ADS018_Cal_ADC_Delta ");
			NRF_LOG_FINAL_FLUSH();
			nrf_delay_us(1000);										
#endif


}


/**@brief Function for application main entry.
 */
int main(void)
{	
	  ble_cscs_meas_t cscs_measurement;
	  //void teste;
		bool erase_bonds;
		ret_code_t err_code;
	  //Initialize.
    log_init();
    
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
    
    timers_init();
    
    button_and_leds_init(&erase_bonds);
    
		power_management_init();
    
    ble_stack_init();
    
    gap_params_init();
    
    gatt_init();
    
		buff_init();
    
		buff_load();
    
		load_user_info_metrics();
    
	  services_init();
    
		advertising_init();
    
    sensor_simulator_init();//somente para o teste RPM
    
    conn_params_init();
    
    peer_manager_init();
    
		spi_init();
    
		pwm_init();
    	
		saadc_init();

		lis2dw12_config();
		
		nrf_drv_spi_uninit(&mLisSpiInstance);
	 	 
		filter_init(global_mixer, glob_wheel_revolution_mm);//ADICIONADO===================================================================

		// Start execution.
		advertising_start(erase_bonds);

    application_timers_start();	

		utils_setup();
		measurement_setup();
    profile_setup();	
#ifdef WDT_ATIVO
//     Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
#endif				
				
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &spi_event_handler;
			
mwatt_flash_init();

mwatt_flash_erase();

mwatt_flash_write_zero();
mwatt_flash_write_ten();
mwatt_flash_write_ref();
mwatt_flash_write_ref2();

mwatt_read_anything(0x56024);
mwatt_read_anything(0x56044);
mwatt_read_anything(0x56064);
mwatt_read_anything(0x56084);
		
//Enter main loop========================================================================================
		for (;;){
#ifdef MAUROTESTE	
		check_nv_update_request();
		if(nrf_gpio_pin_read(26)){
		  get_accel();
		  cycle_treat();
		}
#endif

#ifdef MAUROTESTE_2



#endif			
		idle_state_handle();
		}
}

void spi_init(void)
{
		nrf_drv_spi_config_t spi_config = NRF_DRV_SPI_DEFAULT_CONFIG;
		
		spi_config.ss_pin   = NRF_DRV_SPI_PIN_NOT_USED;
    spi_config.miso_pin = SPI_MISO_PIN;
    spi_config.mosi_pin = SPI_MOSI_PIN;
    spi_config.sck_pin  = SPI_SCK_PIN;
    APP_ERROR_CHECK(nrf_drv_spi_init(&mLisSpiInstance, &spi_config, spi_event_handler, NULL));

    //NRF_LOG_INFO("SPI example started.");
 
}

 

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                               uint16_t len)
{         
    uint8_t tx_buf[] = {reg, *bufp};
    uint16_t length = sizeof(tx_buf);
		mLisPacketTransferComplete = false;
		nrf_gpio_pin_clear(ACC_nSS);
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&mLisSpiInstance, tx_buf, length, NULL, 0));
    // Check for successful transfer
    while (!mLisPacketTransferComplete) ;
    // Confirm we have a connection, should be I_AM_LIS2DH
		nrf_gpio_pin_set(ACC_nSS);
		return 0;
    
}
  
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
{ 
    reg|=0x80;
	  len++;
		uint8_t rx_buf[len];
 		nrf_gpio_pin_clear(ACC_nSS);  
  	mLisPacketTransferComplete = false;
    APP_ERROR_CHECK(nrf_drv_spi_transfer(&mLisSpiInstance,&reg, 1, rx_buf, len));
    // Check for successful transfer
    while (!mLisPacketTransferComplete) ;
		nrf_gpio_pin_set(ACC_nSS);	
		for(uint8_t i=0; i<len; i++){
		    bufp[i]=rx_buf[i+1];
    }

    return 0;

}

bool lis2dw12_config (void){
		nrf_gpio_pin_clear (ACC_nSS);

    lis2dw12_reg_t int_route;

    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.handle = &spi_event_handler;
		nrf_gpio_pin_set(AFE_nSS);
	
	 /*
    * Check device ID
    */
    lis2dw12_device_id_get(&dev_ctx, &whoamI);
    if (whoamI != LIS2DW12_ID)
   	{		//NRF_LOG_INFO("Identidade LIS2DW12 = %x", whoamI);
	      //NRF_LOG_INFO("Falha de identidade LIS2DW12");
        /* manage here device not found */
				nrf_delay_ms(10);
				return false;
	  }
    else{
       // NRF_LOG_INFO("Identidade LIS2DW12 = %x", whoamI);
    }

	 /*
    * Restore default configuration
    */ 
		lis2dw12_reset_get(&dev_ctx, &rst);
		lis2dw12_reset_set(&dev_ctx, PROPERTY_ENABLE);
    do 
		{
	      
    } while (rst);

	 /*
    *  Enable Block Data Update
    */

    lis2dw12_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
   /*
    * Set full scale
    */
    lis2dw12_full_scale_set(&dev_ctx, LIS2DW12_8g);

   /*
    * Configure filtering chain
    * Accelerometer - filter path / bandwidth
    */
    lis2dw12_filter_path_set(&dev_ctx, LIS2DW12_LPF_ON_OUT); 
    lis2dw12_filter_bandwidth_set(&dev_ctx, LIS2DW12_ODR_DIV_4);

   /*
    * Configure power mode
    */
    lis2dw12_power_mode_set(&dev_ctx, LIS2DW12_CONT_LOW_PWR_LOW_NOISE_2);//LIS2DW12_CONT_LOW_PWR_12bit);


   /*
    * Enable activity detection interrupt
    */
  
	  lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
    int_route.ctrl4_int1_pad_ctrl.int1_drdy = PROPERTY_ENABLE;
    lis2dw12_pin_int1_route_set(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
	  lis2dw12_pin_int1_route_get(&dev_ctx, &int_route.ctrl4_int1_pad_ctrl);
  
	 /*
    * Set Output Data Rate
    */
    lis2dw12_data_rate_set(&dev_ctx, LIS2DW12_XL_ODR_50Hz );
		nrf_gpio_pin_set (ACC_nSS);

  	//NRF_LOG_INFO("LIS2DW12 configurado");
		nrf_delay_ms(10);
		return true;

}




//teste corrente, verificar com adição do value_for_simu_F e com adição do zeramento do ADS018_Cycle_Flag após a aquisição. Na característica enviar
//ADS018_res_data.rotation (saída do Set_Mean).