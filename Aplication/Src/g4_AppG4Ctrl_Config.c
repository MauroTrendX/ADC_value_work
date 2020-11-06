/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_AppG4Ctrl
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Config component implementation
 *
 *  \remarks (c) Copyright 2018 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

/* Application layer includes */
#include "g4_AppG4Ctrl_Config.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>

//#include "g4_UtilStdio.h"
//#include "g4_UtilRtc.h"

//#include "g4_HilSystemIntf.h"
#include "g4_HilPpgSensorIntf.h"

#include "g4_AppG4Ctrl.h"
#include "fx_named.h"

/* versioning information */
//#include "g4_UtilSoftwareVersion.h"

/******************************************************************************/
/* Local defines and types                                                    */
/******************************************************************************/

#define CONFIG_MAJOR_VERSION        1
#define CONFIG_MINOR_VERSION        2

/*****************************************************************************/
/* Global data                                                               */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes                                                 */
/*****************************************************************************/


/*****************************************************************************/
/* Local data                                                                */
/*****************************************************************************/

static const indicationCategoryId_t  sCategory[Pattern_Num] =
{
    /* Pattern_Invalid,                     */  Category_Invalid,
    /* Pattern_USBState_Unknown,            */  Category_USBState,
    /* Pattern_USBState_Disconnected,       */  Category_USBState,
    /* Pattern_USBState_ConnectedCharging,  */  Category_USBState,
    /* Pattern_USBState_ConnectedFull,      */  Category_USBState,
    /* Pattern_SystemState_Unknown,         */  Category_SystemState,
    /* Pattern_SystemState_Logging,         */  Category_SystemState,
    /* Pattern_SystemState_Idle,            */  Category_SystemState,
    /* Pattern_ErrorState_Ok,               */  Category_ErrorState,
    /* Pattern_ErrorState_Error,            */  Category_ErrorState,
    /* Pattern_BatteryState_NotLow,         */  Category_BatteryState,
    /* Pattern_BatteryState_AlmostLow,      */  Category_BatteryState,
    /* Pattern_BatteryState_Low,            */  Category_BatteryState,
    /* Pattern_SyncState_Synced,            */  Category_SyncState,
    /* Pattern_SyncState_NotSynced,         */  Category_SyncState,
    /* Pattern_Event_ButtonPress,           */  Category_Event,
    /* Pattern_Event_StartLogging,          */  Category_Event,
    /* Pattern_Event_StopLogging,           */  Category_Event,
    /* Pattern_Event_StartSleep,            */  Category_Event,
    /* Pattern_Event_StopSleep,             */  Category_Event,
    /* Pattern_Event_Resetting,             */  Category_Event,
    /* Pattern_Event_DiskFull,              */  Category_Event
};


static const press_pattern_t sDefaultPressPatterns[PressId_Num] =
{
    /* ID                                       buttonMask            repeats      actOnRelease   timeMs */
    /* PressId_Invalid,                     */  { BUTTON_BOTH   ,         200,       false,         2000 },
    /* PressId_Start,                       */  { BUTTON_SW1    ,           1,       false,         2000 },
    /* PressId_Stop,                        */  { BUTTON_SW1    ,           1,       false,         2000 },
    /* PressId_StartSleep,                  */  { BUTTON_SW2    ,           1,       false,         2000 },
    /* PressId_StopSleep,                   */  { BUTTON_SW2    ,           1,       false,         2000 },
    /* PressId_Reset,                       */  { BUTTON_BOTH   ,           1,       false,         2000 },
    /* PressId_ResetToDsw,                  */  { BUTTON_SW2    ,           3,       false,         2000 },
    /* PressId_EnableDisableMetrics         */  { BUTTON_SW1    ,           3,       false,         2000 }
};

static uint32_t                 sLineNr;

static char *  fxMetricValueConfigName[]=
{
/*00*/ NULL, NULL, NULL, "Age", "Profile", "Height", "Weight", NULL,
/*08*/ NULL, "SleepPreference", NULL, NULL, NULL, NULL, NULL, NULL,
/*10*/ NULL, NULL, NULL, NULL, NULL, NULL, NULL, NULL,
/*18*/ NULL, NULL, NULL, NULL, NULL, NULL, "Time", NULL,
/*20*/ "HeartRate", "RestingHeartRate", "SkinProximity", "EnergyExpenditure", "Speed", "MotionCadence", "ActivityType", "Heartbeats",
/*28*/ "VO2Max", "CardioFitnessIndex", "RespirationRate", "Acceleration", "PPG", "IntermittentHeartRate", "ActivityCount", "WeSTPrivateData",
/*30*/ "SleepSession",NULL,NULL,NULL,"CmpAcceleration","CmpPPG", NULL, NULL,
/*38*/ NULL, NULL, NULL, "HeartRhythmType", "IntermittentEnergyExpenditure", NULL, NULL, NULL
};

/*****************************************************************************/
/* Exported functions                                                        */
/*****************************************************************************/

void g4_AppG4Ctrl_Config_Init(appConfig_t *cfg)
{
    int i;

    /* Set all zeros */
    memset(cfg, 0, sizeof(*cfg));

    cfg->majorVersion       = CONFIG_MAJOR_VERSION;
    cfg->minorVersion       = CONFIG_MINOR_VERSION;

    cfg->inputWiffFileName[0]=0;
    
    for ( i=0; i<(int) Pattern_Num; i++ )
    {
 //       cfg->pattern[i] = sDefaultPattern[i];
    }
    
    for ( i=0; i<(int) PressId_Num; i++ )
    {
        cfg->pressPattern[i] = sDefaultPressPatterns[i];
    }
    
    cfg->wiffEnabled    = false;

#ifdef INCLUDE_FXLIB_CONSUMPTION_MEASUREMENTS_SETTINGS
    cfg->fx_useLib=1;
#endif
    cfg->fx_useLibOutput=1;
    
    /* Init default ppg configuration data */
    g4_HilPpgSensorIntf_Config( &(cfg->ppgConfig), (bool) true );
    
    /* Init default accelerometer's configuration data */
//    g4_HilAccelIntf_Config( &(cfg->accelConfig), (bool) true );
    
    /* Init FX default configuration data */
    cfg->age             = 40;
    cfg->dayOfBirth      =  0;
    cfg->monthOfBirth    =  0;
    cfg->yearOfBirth     =  0;
    
    cfg->handedness      = FX_METRIC_PROFILE_HANDEDNESS_UNSPECIFIED;
    cfg->sex             = FX_METRIC_PROFILE_GENDER_UNSPECIFIED;
    cfg->height          =  0;
    cfg->weight          =  0;
    cfg->bodyPosition    = FX_METRIC_BODYPOSITION_UNSPECIFIEDWRIST;
}


/******************************************************************************/


bool g4_AppG4Ctrl_Config_Parse(appConfig_t *cfg)
{
    bool        rv = true;

    /* skip the config file parse section */

    /* keep watchdog happy */
//    g4_HilSystemIntf_Process();
        /* init them (again) because the config file reader may have modified them */
        g4_AppG4Ctrl_Config_Init(cfg);
        
        /* Without config, we must assume a small set of metrics, so the device actually does something */
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_AGE);
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_PROFILE);
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_HEIGHT);
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_WEIGHT);
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_HEARTRATE);
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_SKINPROXIMITY);
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_ACTIVITYCOUNT);
//        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_INTERMITTENTHEARTRATE);
//        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_ACCELERATION);
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_PPG);
        SET_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, FXI_METRIC_ID_TIME);

        cfg->fx_useLibOutput = 1;       //cfg->fx_useLibOutput==0   /*Ignoring FX library output !!!!!\r\n");
//        cfg->fx_useLib = 1;             //cfg->fx_useLib==0   /* Disabling FX library !!!!!\r\n")

    return rv;
}


/*****************************************************************************/
/* Local functions                                                           */
/*****************************************************************************/


/**
 * \}
 * End of file.
 */

