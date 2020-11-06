/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_AppG4Ctrl
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - G4 Controller component implementation
 *
 *  \remarks (c) Copyright 2018 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

/* Application layer includes */
#include "g4_AppG4Ctrl.h"
#include "g4_AppG4Ctrl_Config.h"
//#include "g4_AppG4Ctrl_WIFFWriter.h"
//#include "g4_AppG4Ctrl_WIFFReader.h"
#include "g4_AppG4Ctrl_FX.h"

/* Hardware Interface Layer includes */
//#include "g4_HilAccelIntf.h"
#include "g4_HilPpgSensorIntf.h"
//#include "g4_HilButtonIntf.h"
//#include "g4_HilSystemIntf.h"
//#include "g4_HilTempIntf.h"
//#include "g4_HilUiIntf.h"

/* Utility Layer includes */
//#include "g4_UtilRtc.h"
//#include "g4_UtilStdio.h"
#include "g4_UtilAssert.h"
#include "g4_UtilRingBuffer.h"
#include "fx_named.h"

/* IDs for persistent storage */
//#include "g4_PersistentValueIds.h"

#include <string.h>
#include <stdio.h>

/* versioning information */
//#include "g4_UtilSoftwareVersion.h"

/******************************************************************************/
/* Local defines and types                                                    */
/******************************************************************************/

/**
 *  \brief  States for the application state machine
 */
typedef enum _g4_app_state_t
{
    STATE_INITIALIZED,
    STATE_STARTING,
    STATE_IDLE,
    STATE_LOGGING_STARTING,
    STATE_LOGGING_ACTIVE,
    STATE_LOGGING_STOPPING,
    STATE_LOW_BATTERY,
    STATE_ERROR,
    STATE_NO_SYNC,
    STATE_RESET_PENDING1,
    STATE_RESET_PENDING2,
    NUMBER_OF_STATES
} g4_app_state_t;


/**
 *  \brief  States for the USB state machine
 */
typedef enum
{
    USBSTATE_IDLE,
    USBSTATE_LISTEN,
    USBSTATE_CONFIGURE,
    USBSTATE_ENUMERATED,
    NUMBER_OF_USBSTATES
} usb_state_t;


/**
 *  \brief  Determine if a given state has USB interrupts active
 */
#define USB_ACTIVE_IN(_state)   (((_state) == USBSTATE_CONFIGURE) || ((_state) == USBSTATE_ENUMERATED))


/**
 *  \brief  Software component masks
 */
#define SWC_BUTTON          0x0001
#define SWC_UI              0x0002
#define SWC_FILEOUT         0x0004
#define SWC_FX              0x0008
#define SWC_BLE             0x0000

/**
 *  \brief  Time (in milliseconds) before we act on the USB removal
 */
#define USB_REMOVAL_DELAY   2500


/**
 *  \brief  Sync timeout: 7 days
 */
#define SYNC_TIMEOUT        (7*24*3600)


/**
 *  \brief  Threshold for low battery detection (in mV)
 */
#define LOW_BATTERY_THRESHOLD                   3400
#define ALMOST_LOW_BATTERY_THRESHOLD            3550


/**
 *  \brief  Button IDs
 */
#define START_BUTTON                    0
#define AUX_BUTTON                      1


/**
 *  \brief  Minumum amount of time the indication of certain states should remain.
 */
#define MINIMUM_ERROR_INDICATION_TIME_MS        3000
#define MINIMUM_NO_SYNC_INDICATION_TIME_MS      5000


/**
 *  \brief  The dynamic data for the button press tracking
 */
typedef struct _button_pattern_dyn_data_t
{
    uint32_t        startTime;
    uint8_t         count;
    uint8_t         physicalState;
    uint8_t         patternState;
} button_pattern_dyn_data_t;


/**
 *  \brief  Pressed status for buttons
 */
#define NOT_PRESSED     0
#define PRESSED         1

/******************************************************************************/
/* Local data                                                                 */
/******************************************************************************/


/**
 *  \brief  The table with software component activations depending on the state.
 */
const uint16_t    sStateActivations[NUMBER_OF_STATES] =
{
    /* STATE_INITIALIZED,       */ 0           | 0      | 0           | 0       | 0      ,
    /* STATE_STARTING,          */ 0           | 0      | 0           | 0       | 0      ,
    /* STATE_IDLE,              */ SWC_BUTTON  | SWC_UI | 0           | SWC_BLE | 0      ,
    /* STATE_LOGGING_STARTING,  */ SWC_BUTTON  | SWC_UI | 0           | SWC_BLE | 0      ,
    /* STATE_LOGGING_ACTIVE,    */ SWC_BUTTON  | SWC_UI | SWC_FILEOUT | SWC_BLE | SWC_FX ,
    /* STATE_LOGGING_STOPPING,  */ SWC_BUTTON  | SWC_UI | SWC_FILEOUT | SWC_BLE | SWC_FX ,
    /* STATE_LOW_BATTERY,       */ SWC_BUTTON  | SWC_UI | 0           | 0       | 0      ,
    /* STATE_ERROR,             */ SWC_BUTTON  | SWC_UI | 0           | 0       | 0      ,
    /* STATE_NO_SYNC,           */ SWC_BUTTON  | SWC_UI | 0           | 0       | 0      ,
    /* STATE_RESET_PENDING1     */ SWC_BUTTON  | SWC_UI | 0           | 0       | 0      ,
    /* STATE_RESET_PENDING2,    */ 0           | SWC_UI | 0           | 0       | 0     
};


/**
 *  \brief  The current state of the components
 */
static uint16_t sCompState;


/**
 *  \brief  The current USB/enumeration state
 */
static usb_state_t  sUsbState;


/**
 *  \brief  Application configuration
 */
static appConfig_t sAppConfig;


/**
 *  \brief  The state of the application state machine
 */
static g4_app_state_t sState = STATE_INITIALIZED;


/**
 *  \brief  Flag indicating whether the last USB connect resulted in a synchronization
 */
static bool         sSynced;


/**
 *  \brief  Timestamp of the last synchronization
 */
static uint32_t     sLastSyncTime;


/**
 *  \brief  Bookkeeping for decoupling the setting of the date and time
 */
//static g4_UtilRtc_Date_t    sPendingDate;
//static g4_UtilRtc_Time_t    sPendingTime;
static bool                 sDateTimePending;


/**
 *  \brief  Status indicating the battery is almost empty
 */
static bool     sBatteryAlmostEmpty;


/**
 *  \brief  Array of state names, used for terminal logging.
 */
static const char *sStateNames[] =
{
    "STATE_INITIALIZED",
    "STATE_STARTING",
    "STATE_IDLE",
    "STATE_LOGGING_STARTING",
    "STATE_LOGGING_ACTIVE",
    "STATE_LOGGING_STOPPING",
    "STATE_LOW_BATTERY",
    "STATE_ERROR",
    "STATE_NO_SYNC",
    "STATE_RESET_PENDING1",
    "STATE_RESET_PENDING2",
    "NUMBER_OF_STATES"
};


/**
 *  \brief  Array of USB state names, used for terminal logging.
 */
static const char *sUsbStateNames[] =
{
    "USBSTATE_IDLE",
    "USBSTATE_LISTEN",
    "USBSTATE_CONFIGURING",
    "USBSTATE_ENUMERATED",
    "USBSTATE_NUMBER"
};


/**
 *  \brief  Last voltage measured while not connected to charger
 */
static uint32_t sLastBatteryVoltage;
static uint32_t sLastReliableBatteryVoltage;


/**
 *  \brief  The amount of times that the battery must be measured before we can go to longer sleep.
 */
static uint32_t sBatteryMeasurementCount;


/**
 *  \brief  Timestamp of the last voltage measurement
 */
static uint32_t sLastReliableBatteryTime;
static uint32_t sLastBatteryTime;


/**
 *  \brief  ppg data fx-input ring buffer storage.
 */
uint8_t ppgDataBuffer[(sizeof(g4_ppgMeasurement_t)*25*2)+1];


/**
 *  \brief  acc data fx-input ring buffer storage.
 */
uint8_t accDataBuffer[(sizeof(g4_accMeasurement_t)*128*2)+1];


/**
 *  \brief  metrics fx-output ring buffer storage.
 */
uint8_t metricData[FX_MINIMUM_METRIC_BUFFER_SIZE+15];


/**
 *  \brief  ppg data fx-input ring buffer admin.
 */
static g4_UtilRingBuffer_t sPpgRB;


/**
 *  \brief  acc data fx-input ring buffer admin.
 */
static g4_UtilRingBuffer_t sAccRB;


/**
 *  \brief  metrics fx-output ring buffer admin.
 */
static g4_UtilRingBuffer_t sMetricOutRB;


/**
 *  \brief  Time of the last state change
 */
static uint32_t sStateChangeTime;


/**
 *  \brief  Set to true when any button is pressed
 */
static bool sPhysButtonPressed;


/**
 *  \brief  Blocking sleep for a period of time
 */
static uint32_t sSleepBlockStartTime;
static uint32_t sSleepBlockTime;
static bool     sSleepBlocked;


/**
 *  \brief  Data for LED blinks. Statusses and starttime
 */
static bool         sStartedLogging;
static uint32_t     sStartedLoggingStartTime;
static bool         sStoppedLogging;
static uint32_t     sStoppedLoggingStartTime;
static bool         sResetting;     /* no timeout here, is done in the state machine */
static bool         sStartedSleep;
static uint32_t     sStartedSleepStartTime;
static bool         sStoppedSleep;
static uint32_t     sStoppedSleepStartTime;


/**
 *  \brief  The currently active patterns on the channels
 */
//static indicationPatternId_t   sActivePattern[NUMBER_OF_RGB_LEDS][MAX_PATTERNS_IN_SEQUENCE];


/**
 *  \brief  The button press pattern - runtime data
 */
static button_pattern_dyn_data_t    sButtonPatternDynData[PressId_Num];
static bool                         sButtonCoolDown;

static bool                         sButtonStopLoggingActivated = false;

/**
 *  \brief  The currently used sleep preference - true = yes, false = no.
 */
static bool                         sSleepPreference;

/******************************************************************************/
/* Local function prototypes                                                  */
/******************************************************************************/

/**
 *  \brief  Change the USB state
 *  \return void
 */
static void changeUsbState(usb_state_t usbState);


/**
 *  \brief  Change the state of the G4.
 *  \param  [IN] newState : the new state
 *  \return void
 */
static void changeState(g4_app_state_t newState);


/**
 *  \brief  Change the activation state of the software components
 *  \param  [IN] newState : the new state
 *  \return void
 */
static void changeComponentState(uint16_t newState);


/**
 *  \brief  Evaluate whether the G4 controller can go to sleep, and if so, do so
 *  \return void
 */
static void evaluateSleep(void);


/**
 *  \brief  Determine the pattern to be displayed, based on various state variables
 *  \return void
 */
static void evaluateLedBlink(void);


/**
 *  \brief  Evaluate the button state and make the state changes that are involved
 *  \return void
 */
static void evaluateButtons(void);


/**
 *  \brief  Reset the button states
 *  \return void
 */
static void resetButtons(void);


/**
 *  \brief  Determine whether we need to go to low battery state or not.
 *  \return void
 */
static void evaluateLowBattery(void);


/**
 *  \brief  Evaluate the exposed application state
 *  \return void
 */
static void evaluateAppState(void);


/**
 *  \brief  Determine whether G4 is currently busy - used for evaluating whether
 *          the G4 controller can go to sleep.
 *  \return true when busy
 */
static bool isBusy(void);


/**
 *  \brief  Determine whether the USB has been connected
 *  \return true when USB connected
 */
static bool usbConnected(void);


static void g4_AppG4Ctrl_SyncTime(void);

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/

void g4_AppG4Ctrl_Init(void)
{    
    g4_AppG4Ctrl_SyncTime(); 
  
//    printf("\r\n\r\n *** G4 Application Starting ***\r\n");
    
    sCompState  = 0;
    sState      = STATE_INITIALIZED;
    
    /* All components have been initialized by main ctrl, so they are in a low
     * power state. No need to do that here. This does not apply to Start/Stop,
     * or course
     */
    
    /* Setup ringbuffers */
    g4_UtilRingBuffer_Init(&sPpgRB, ppgDataBuffer, sizeof(ppgDataBuffer) );
    g4_UtilRingBuffer_Init(&sAccRB, accDataBuffer, sizeof(accDataBuffer) );
    g4_UtilRingBuffer_Init(&sMetricOutRB, metricData, sizeof(metricData) );
    
    /* Init fx ctrl */
    g4_AppG4Ctrl_FX_Init(&sPpgRB,&sAccRB,&sMetricOutRB);
    
    /* initialize default in case config file does not contain them */
    g4_AppG4Ctrl_Config_Init(&sAppConfig);
    
}


/******************************************************************************/


void g4_AppG4Ctrl_Start(void)
{
    uint32_t *id = (uint32_t *) 0x1FFF7A10;
    
    changeState(STATE_STARTING);
    
//    printf("\r\n\r\n *** G4 Application Starting ***\r\n");
    
    /*if (g4_UtilRtc_PersistentRead(PERSISTENT_ID_SYNCHRONISED) == 1)
    {
        uint32_t low, high;
        
        // last sync time also survived the reset 
        low     = g4_UtilRtc_PersistentRead(PERSISTENT_ID_LAST_SYNC_TIME_LOW); 
        high    = g4_UtilRtc_PersistentRead(PERSISTENT_ID_LAST_SYNC_TIME_HIGH);
        
        sLastSyncTime = low + (high << 16);
        
        sSynced = true;
//        printf("Initial synchronization status: TRUE\r\n");
    }
    else
    {
        sSynced = false;
//        printf("Initial synchronization status: FALSE\r\n");
    }*/
    
    sSleepPreference    = false;
    
    /* Battery voltage unknown, we must measure it, but we can only do that when
     * there is no charger or USB attached
     */
    
    /* calculate current time */

    
    /* And go to initializing state */
    changeState(STATE_STARTING);
    
    /* And start detecting USB */
    changeUsbState(USBSTATE_CONFIGURE);
    
//    sSleepBlockStartTime    = g4_UtilRtc_GetTimeMs();
    sSleepBlockTime         = 2000;
    sSleepBlocked           = true;
    
    sStartedLogging     = false;
    sStoppedLogging     = false;
    sStartedSleep       = false;
    sStoppedSleep       = false;
    sResetting          = false;
    
    /* Configure system without read the config file */
    if (!g4_AppG4Ctrl_Config_Parse(&sAppConfig))
    {
//        g4_UtilStdio_AddLog("Configuration error");
        changeState(STATE_ERROR);
    }

}


/******************************************************************************/


void g4_AppG4Ctrl_Stop(void)
{
    /* Change state back to idle */
    changeState(STATE_IDLE);
    
}


/******************************************************************************/


void g4_AppG4Ctrl_Process(void)
{
    uint8_t metric[FX_MAX_METRIC_SIZE];
    bool display_NewLine = false;
		//sCompState=SWC_FX;
		changeState(STATE_LOGGING_ACTIVE);
    if (sCompState & SWC_FX)
    {		
        uint32_t mbytes=0;
        fx_MetricHeader_t mHdr;

        if (g4_AppG4Ctrl_FX_PendingError())
        {
//            g4_UtilStdio_AddLog("FX app error!");
            changeState(STATE_ERROR);
        }
        
        do
        {
            mbytes = g4_UtilRingBuffer_Peek(&sMetricOutRB, &mHdr, sizeof(mHdr));

            if ( mbytes )
            {
                (void) g4_UtilRingBuffer_Read(&sMetricOutRB, (void *) &metric[0] ,FX_GetMetricSize(&mHdr));
                    
                    if (sAppConfig.fx_useLibOutput)
                    {
                        display_NewLine = true;
                        switch (mHdr.id)
                        {
                        case FXI_METRIC_ID_TIME:
                            {
                                fx_MetricTime_t *timeMetric = (fx_MetricTime_t *) &metric[0];
  //                              g4_UtilRtc_Date_t mdate;
    //                            g4_UtilRtc_Time_t mtime;
                                uint32_t uTime;

                                uTime = (uint32_t) timeMetric->data.time[0];
                                uTime = uTime | (uint32_t) timeMetric->data.time[1]<<8;
                                uTime = uTime | (uint32_t) timeMetric->data.time[2]<<16;
                                uTime = uTime | (uint32_t) timeMetric->data.time[3]<<24;
                                
//                                g4_UtilRtc_UnxiTimeToDateTime(uTime, &mdate, &mtime);
//                                
//                                printf("FX.TIME:%02d:%02d:%02d(%d)...", mtime.hours, mtime.minutes, mtime.seconds, timeMetric->header.quality);
                            }
                            break;
                        case FXI_METRIC_ID_HEARTRATE:
                            {
                                fx_MetricAverageHeartRate_t *ahMetric = (fx_MetricAverageHeartRate_t *) &metric[0];
//                                printf("FX.AVH:%d(%d)...", ahMetric->data.bpm, ahMetric->header.quality);
                            }
                            break;
                        case FXI_METRIC_ID_RESTINGHEARTRATE:
                            {
                                fx_MetricRestingHeartRate_t *rhrMetric = (fx_MetricRestingHeartRate_t *) &metric[0];
//                                printf("FX.RHR:%d(%d)...", rhrMetric->data.restHeartrate, rhrMetric->header.quality);
                            }
                            break;
                        case FXI_METRIC_ID_SKINPROXIMITY:
                            {
                                fx_MetricSkinProximity_t *spMetric = (fx_MetricSkinProximity_t *) &metric[0];
//                                printf("FX.SP:%d(%d)...", spMetric->data.skinProximity, spMetric->header.quality );
                            }
                            break;
                        case FXI_METRIC_ID_ENERGYEXPENDITURE:
                            {
                                fx_MetricEnergyExpenditure_t *eeMetric = (fx_MetricEnergyExpenditure_t *) &metric[0];
//                                printf("FX.EE:%d(%d)...", (uint16_t)((uint16_t)eeMetric->data.energyExpenditureL) + (((uint16_t)eeMetric->data.energyExpenditureH) << 8), eeMetric->header.quality );
                            }
                            break;
                        case FXI_METRIC_ID_SPEED:
                            {
                                fx_MetricSpeed_t *speedMetric = (fx_MetricSpeed_t *) &metric[0];
//                                printf("FX.SPEED:%d(%d)...", speedMetric->data.speed, speedMetric->header.quality );
                            }
                            break;
                        case FXI_METRIC_ID_MOTIONCADENCE:
                            {
                                fx_MetricCadence_t *cdMetric = (fx_MetricCadence_t *) &metric[0];
//                                printf("FX.CD:%d(%d)...", cdMetric->data.cadence, cdMetric->header.quality );
                            }
                            break;
                        case FXI_METRIC_ID_ACTIVITYTYPE:
                            {
                                fx_MetricActivityType_t *ahMetric = (fx_MetricActivityType_t *) &metric[0];
//                                printf("FX.AT:%d(%d)...", ahMetric->data.type, ahMetric->header.quality);
                            }
                            break;
                        case FXI_METRIC_ID_VO2MAX:
                            {
                                fx_MetricVo2Max_t *v02mMetric = (fx_MetricVo2Max_t *) &metric[0];
 //                               printf("FX.VO2MAX:%d(%d)...", v02mMetric->data.vo2max, v02mMetric->header.quality);
                            }
                            break;
                        case FXI_METRIC_ID_CARDIOFITNESSINDEX:
                            {
                                fx_MetricCardioFitnessIndex_t *cfiMetric = (fx_MetricCardioFitnessIndex_t *) &metric[0];
 //                               printf("FX.CFI:%d(%d)...", cfiMetric->data.cardioFitnessIndex, cfiMetric->header.quality);
                            }
                            break;
                        case FXI_METRIC_ID_RESPIRATIONRATE:
                            {
                                fx_MetricRespirationRate_t *rrMetric = (fx_MetricRespirationRate_t *) &metric[0];
 //                               printf("FX.RR:%d(%d)...", rrMetric->data.respirationRate, rrMetric->header.quality);
                            }
                            break;
                        case FXI_METRIC_ID_ACTIVITYCOUNT:
                            {
                                fx_MetricActivityCount_t *acMetric = (fx_MetricActivityCount_t *) &metric[0];
 //                               printf("FX.BP:=%d FX.AC:=%d(%d)..", acMetric->data.bodyPosition, 
 //                                      (uint16_t)((uint16_t)acMetric->data.activityCountsL) + (((uint16_t)acMetric->data.activityCountsH)<<8), acMetric->header.quality );
                            }
                            break;    
                        case FXI_METRIC_ID_INTERMITTENTHEARTRATE:
                            {
                                fx_MetricIntermittentHeartRate_t *ihrMetric = (fx_MetricIntermittentHeartRate_t *) &metric[0];
 //                               printf("FX.IHR:=%d...", ihrMetric->data.heartRate);
                            }
                            break;    
                        case FXI_METRIC_ID_WESTPRIVATEDATA:
                            {
 //                               printf("FX.WP:len=%d...", metric[1]+256*metric[2]);
                            }
                            break;
                        case FXI_METRIC_ID_SLEEPSESSION:
                            {
 //                               printf("FX.ZZ...");
                            }
                            break;
                        case FXI_METRIC_ID_HEARTRHYTHMTYPE:
                            {
                                fx_MetricHeartRhythmType_t *ihrtMetric = (fx_MetricHeartRhythmType_t *) &metric[0];
 //                               printf("FX.HRT.D:=%d FX.HRT.T:=%d...", ihrtMetric->data.delay, ihrtMetric->data.heartRhythmType);
                            }
                            break;
                        case FXI_METRIC_ID_INTERMITTENTENERGYEXPENDITURE:
                            {
                                fx_MetricIntermittentenEnergyExpenditure_t *ieeMetric = (fx_MetricIntermittentenEnergyExpenditure_t *) &metric[0];
 //                               printf("FX.IEE:=%d...", (uint16_t)(((uint16_t)ieeMetric->data.ieeH) << 8) + (uint16_t)ieeMetric->data.ieeL);
                            }
                            break;
                        case FXI_METRIC_ID_PPG:
                            {
//                                printf("FX.PPG:%02X.\r\n",mHdr.id);
                                fx_MetricPpgRaw_t *ppgMetric = (fx_MetricPpgRaw_t *) &metric[0];
                                fx_MetricPpgRawDataPpgAmb_t *ppgAmbData = (fx_MetricPpgRawDataPpgAmb_t *) ppgMetric->data.ppgAmb;
//                                printf("\r\nFX.LED(GAIN):=%d(%d),PPG(AMB):=%d(%d),%d(%d).", ppgMetric->data.relLedPwr, ppgMetric->data.relAdcGain,
//                                    ((uint16_t)ppgAmbData[0].ppgH << 8) + (uint16_t)ppgAmbData[0].ppgL,
//                                    ((uint16_t)ppgAmbData[0].ambH << 8) + (uint16_t)ppgAmbData[0].ambL,
//                                    ((uint16_t)ppgAmbData[1].ppgH << 8) + (uint16_t)ppgAmbData[1].ppgL,
//                                    ((uint16_t)ppgAmbData[1].ambH << 8) + (uint16_t)ppgAmbData[1].ambL);
                            }
                            break;
                        case FXI_METRIC_ID_ACCELERATION:
                            {
//                                printf("FX.ACC:%02X.\r\n",mHdr.id);
                                fx_MetricAccRaw_t *accMetric = (fx_MetricAccRaw_t *) &metric[0];
                                FX_UINT08 *accData = (FX_UINT08 *) accMetric->data.acc;
//                                    printf("\r\nFX.BP:=%d..FX.ACC:=", accMetric->data.bodyPosition);
                                    //for (int n = 0; n < sizeof(accMetric->data); n += 6) 
//                                        printf("%i,%i,%i..", (int16_t)((accData[n+1] << 8) + accData[n]),
//                                        (int16_t)((accData[n+3] << 8) + accData[n+2]), (int16_t)((accData[n+5] << 8) + accData[n+4]) );
//                                printf("\r\nFX.ACC:=%i,%i,%i(%d).", (int16_t)((accData[1] << 8) + accData[0]),
//                                    (int16_t)((accData[3] << 8) + accData[2]), (int16_t)((accData[5] << 8) + accData[4]),
//                                    accMetric->data.bodyPosition);                              
                            }
                            break;
                        default:
                            //printf("FX.%02X...",mHdr.id);
                            break;
                        }
                        
                    }
            }
        } while ( mbytes );
        
        if (display_NewLine)
        {
 //           printf(".\r\n");
            display_NewLine = false;
        }
        
       volatile uint8_t comota= g4_AppG4Ctrl_FX_Process();
                    
    }
    
		if ( STATE_STARTING == sState )
    {
        /* change the state to IDLE */
        changeState(STATE_IDLE);
    }
    
    if (sDateTimePending)
    {
/*        uint32_t diff1 = g4_UtilRtc_GetCurrentUnixTime() - sLastReliableBatteryTime;
        uint32_t diff2 = g4_UtilRtc_GetCurrentUnixTime() - sLastBatteryTime;
        
        bool ok = g4_UtilRtc_SetDateTime(&sPendingDate, &sPendingTime);
        
        g4_UtilAssert(ok);
        
//        printf("[SYNC] Set time to %4d-%02d-%02d , %2d:%02d:%02d.%03d\r\n",
               sPendingDate.year, sPendingDate.month, sPendingDate.date,
               sPendingTime.hours, sPendingTime.minutes, sPendingTime.seconds, sPendingTime.milliseconds
                   );
        
        sDateTimePending = false;
        sLastReliableBatteryTime    = g4_UtilRtc_GetCurrentUnixTime() - diff1;
        sLastBatteryTime            = g4_UtilRtc_GetCurrentUnixTime() - diff2;
        sSynced = true;
        
        sLastSyncTime = g4_UtilRtc_GetUnixTime(&sPendingDate, &sPendingTime);
        
        g4_UtilRtc_PersistentWrite(PERSISTENT_ID_SYNCHRONISED, 1);
        g4_UtilRtc_PersistentWrite(PERSISTENT_ID_LAST_SYNC_TIME_LOW,  (uint16_t) (sLastSyncTime&0xffff)); 
        g4_UtilRtc_PersistentWrite(PERSISTENT_ID_LAST_SYNC_TIME_HIGH, (uint16_t) (sLastSyncTime>>16));*/
    }
    
		evaluateButtons();
    
   
}

/******************************************************************************/
/* Local functions                                                            */
/******************************************************************************/

//static void processUsbState(void)
//{
//}


/******************************************************************************/


static void changeUsbState(usb_state_t usbState)
{
    
}


/******************************************************************************/


static void changeState(g4_app_state_t newState)
{
    if (sState != newState)
    {
//        g4_UtilAssert(newState < NUMBER_OF_STATES);
        
//        printf("[State] %s -> %s\r\n", sStateNames[sState], sStateNames[newState]);
        
        if ( STATE_LOGGING_STARTING == sState )
        {
            sStartedLogging             = true;
//            sStartedLoggingStartTime    = g4_UtilRtc_GetTimeMs();
        }
        else if ( STATE_LOGGING_STOPPING == sState )
        {
            sStoppedLogging             = true;
//            sStoppedLoggingStartTime    = g4_UtilRtc_GetTimeMs();
        }
        
        if ( STATE_RESET_PENDING2 == newState )
        {
            sResetting                  = true;
        }

        sState = newState;
        changeComponentState(sStateActivations[sState]);
        
//        sStateChangeTime = g4_UtilRtc_GetTimeMs();
    }
}


/******************************************************************************/


static void changeComponentState(uint16_t newState)
{			g4_AppG4Ctrl_FX_Start(&sAppConfig);
//			changeState(STATE_LOGGING_ACTIVE);
/*    if (!(sCompState & SWC_FX) && (newState & SWC_FX))
    {
  */    
        /* We set sleep preference value to "no" as default. Button initial state is also set to false */
    //    sSleepPreference = false;
        
        /* Start library usage */
    //    g4_AppG4Ctrl_FX_Start(&sAppConfig);
    
    //}
//    else if ((sCompState & SWC_FX) && !(newState & SWC_FX))
//    {
        /* disable FX */
//        printf("[DISABLE] FX\r\n");
    /*    g4_AppG4Ctrl_FX_DeInit();
        g4_UtilRingBuffer_Reset(&sMetricOutRB);
        g4_UtilRingBuffer_Reset(&sPpgRB);
        g4_UtilRingBuffer_Reset(&sAccRB);
    }*/

    if (!(sCompState & SWC_BUTTON) && (newState & SWC_BUTTON))
    {
        /* enable button */
//        printf("[ENABLE] Button\r\n");
//        g4_HilButtonIntf_Start();
    }
    else if ((sCompState & SWC_BUTTON) && !(newState & SWC_BUTTON))
    {
        /* disable button */
//        printf("[DISABLE] Button\r\n");
//        g4_HilButtonIntf_Stop();
    }

    if (!(sCompState & SWC_UI) && (newState & SWC_UI))
    {
        /* enable UI */
//        printf("[ENABLE] UI\r\n");
//        g4_HilUiIntf_Start();
    }
    else if ((sCompState & SWC_UI) && !(newState & SWC_UI))
    {
        /* disable UI */
//        printf("[DISABLE] UI\r\n");
//        g4_HilUiIntf_Stop();
    }


    sCompState = newState;
}


/******************************************************************************/
static void g4_AppG4Ctrl_SyncTime(void)
{            
/*            sPendingDate.year           = 2019;   // date.year = 2000 + buf[2];
            sPendingDate.month          = 9;      // date.month; = buf[3];
            sPendingDate.date           = 3;      // date.date; = buf[4];
            sPendingDate.weekDay        = 2;      // date.weekDay; = buf[5];

            sPendingTime.hours          = 16;      // time.hours; = buf[6];
            sPendingTime.minutes        = 00;     // time.minutes; = buf[7];
            sPendingTime.seconds        = 00;     // time.seconds; = buf[8];
            sPendingTime.milliseconds   = 00;  // time.milliseconds = (uint16_t) buf[9] + (buf[10] << 8);
            sDateTimePending = true;            
*/};


/******************************************************************************/


static void evaluateSleep(void)
{
    if (!isBusy())
    {
        /* determine when we need attention again... */
        uint32_t minDly = 0xffffffff, dly;

//        dly = g4_HilUiIntf_NextBlinkEvent();

        if (dly < minDly)
        {
            minDly = dly;
        }

        /* limit the delay when we are starting */
        if ( STATE_LOGGING_STARTING == sState )
        {
            if (minDly > 100)
            {
                minDly = 100;
            }
        }

        if (minDly > 1)
        {
            /* We're going to sleep for at least 1 ms, if allowed. */
//            g4_HilSystemIntf_PrepareSleep();
            
            /* now check if we can still sleep */
            if (!isBusy())
            {
                if (minDly == 0xffffffff)
                {
                    if (sBatteryMeasurementCount > 0)
                    {
 //                       (void) g4_HilSystemIntf_Sleep(1500000);
                        sBatteryMeasurementCount--;
                    }
                    else
                    {
 //                       (void) g4_HilSystemIntf_Sleep(120000000);
                    }
                }
                else
                {
 //                   (void) g4_HilSystemIntf_Sleep((minDly+2) * 1000);
                }
            }
        }
    }
}


/******************************************************************************/


static void evaluateButtons(void)
{
   if ( STATE_IDLE == sState )
    {
        if (sButtonPatternDynData[PressId_Reset].patternState == PRESSED)
        {
            resetButtons();
            changeState(STATE_RESET_PENDING1);
        }
        else if (sButtonPatternDynData[PressId_ResetToDsw].patternState == PRESSED)
        {
            resetButtons();
            changeState(STATE_RESET_PENDING1);
        }
        else if (sButtonPatternDynData[PressId_Start].patternState == PRESSED)
        {
            resetButtons();
            if ( !USB_ACTIVE_IN(sUsbState) )
            {
                changeState(STATE_LOGGING_STARTING);
            }
        }
    }
    else if (STATE_LOGGING_STARTING == sState )
    {
        if (!sButtonCoolDown)
        {
            if (!sSynced)
            {
                changeState(STATE_NO_SYNC);
            }
            else
            {
                /* initialize default in case config file does not contain them */
                g4_AppG4Ctrl_Config_Init(&sAppConfig);

                /* read the config file */
                if (!g4_AppG4Ctrl_Config_Parse(&sAppConfig))
                {
                    changeState(STATE_ERROR);
                }
                else
                {
                    /* change the state to start logging */
                    changeState(STATE_LOGGING_ACTIVE);
                }
            }
        }
    }
    else if ( STATE_LOGGING_ACTIVE == sState )
    {
        if (sButtonPatternDynData[PressId_Stop].patternState == PRESSED)
        {
            resetButtons();
            
            sButtonStopLoggingActivated = true;
            //g4_AppG4Ctrl_FX_Stop(&sAppConfig);
            //changeState(STATE_LOGGING_STOPPING);
        }
        else if (!sSleepPreference && (sButtonPatternDynData[PressId_StartSleep].patternState == PRESSED))
        {
            resetButtons();
            
            if ( !g4_AppG4Ctrl_FX_SetSleepPreferece((bool) true) )
            {
                
                sSleepPreference        = true;
                sStartedSleep           = true;
//                sStartedSleepStartTime  = g4_UtilRtc_GetTimeMs();
            }

        }
        

    }
    else if ( STATE_LOGGING_STOPPING == sState )
    {
        /* check button release */
        //if (!sButtonCoolDown)
        //{
        //    resetButtons();
        //    changeState(STATE_IDLE);
        //}
    }
    else if ( STATE_RESET_PENDING1 == sState )
    {
        /* check button release */
        if (!sButtonCoolDown)
        {
            resetButtons();
            changeState(STATE_RESET_PENDING2);
        }
    }
}


/******************************************************************************/


void resetButtons(void)
{
   }


/******************************************************************************/


static void evaluateLedBlink(void)
{
   
}


/******************************************************************************/


static void evaluateLowBattery(void)
{
   
}

 
/******************************************************************************/


static void evaluateAppState(void)
{
#ifdef G4_BLE
    bleTransport_ApplicationState_t state = APPSTATE_UNDEFINED;

    /* TODO: the APPSTATE is still very GTEP oriented and a good translation is not possible */
    switch (sState)
    {
        case STATE_INITIALIZED:         state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_STARTING:            state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_IDLE:                state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_LOGGING_STARTING:    state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_LOGGING_ACTIVE:      state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_LOGGING_STOPPING:    state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_LOW_BATTERY:         state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_ERROR:               state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_NO_SYNC:             state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_RESET_PENDING1:      state = APPSTATE_UNDEFINED;                                                             break;
        case STATE_RESET_PENDING2:      state = APPSTATE_UNDEFINED;                                                             break;
        case NUMBER_OF_STATES:          state = APPSTATE_UNDEFINED;                                                             break;
    }
    
    g4_AppG4Ctrl_BLETransport_SetAppState(state);
#endif // G4_BLE
}


/******************************************************************************/


static bool isBusy(void)
{
    bool busy = false;

    if (sPhysButtonPressed)                 { busy = true; }

    /* simple implementation.... */
//    busy = (busy || g4_HilAccelIntf_Busy());
//    busy = (busy || g4_HilAppSelectIntf_Busy());
//    busy = (busy || g4_HilButtonIntf_Busy());
//    busy = (busy || g4_HilFatFsIntf_Busy());
//    busy = (busy || usbConnected());
//    busy = (busy || g4_HilSystemIntf_Busy());
//    busy = (busy || g4_HilUiIntf_Busy());
//    busy = (busy || (sUsbState == USBSTATE_CONFIGURE));
//    busy = (busy || g4_HilPpgSensorIntf_Busy());
    
    if (sCompState & SWC_FX) busy = (busy || g4_AppG4Ctrl_FX_Busy());
    if ( STATE_LOGGING_STARTING == sState )   { busy = true; }
    
    if ( STATE_LOGGING_STOPPING == sState )   { busy = true; }
    


    return busy;
}


/******************************************************************************/


bool usbConnected(void)
{
    bool rv = false;
    
    /* If charger tells us we're charging, then we are charging, regardless of enumeration state */
//    rv = rv || g4_HilSystemIntf_Charging();
    
    /* If we're enumerated, we must remain in USB state */
//    rv = rv || g4_HilMscHidIntf_Configured();

    return rv;
}


/**
 * \}
 * End of file.
 */

