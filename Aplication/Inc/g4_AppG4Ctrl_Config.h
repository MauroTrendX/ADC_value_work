/******************************************************************************/

/**
 *  \file    g4_AppG4Ctrl_Config.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Config component definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_APP_CONFIG_H__
#define __G4_APP_CONFIG_H__

#include <stdint.h>
#include <stdbool.h>

//#include "g4_HilUiIntf.h"
//#include "g4_HilAccelIntf.h"
#include "g4_HilPpgSensorIntf.h"

#include "fxi.h"

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/


/**
 *  \brief  The length of the blink patterns (in milliseconds)
 */
#define BLINK_CYCLE         10000


/**
 *  \brief  Categories of LED indication
 */
typedef enum _indicationCategoryId_t
{
    Category_Invalid,
    Category_SystemState,
    Category_USBState,
    Category_ErrorState,
    Category_BatteryState,
    Category_SyncState,
    Category_Event,
    Category_Num
} indicationCategoryId_t;


/**
 *  \brief  Pattern identification
 */
typedef enum _indicationPatternId_t
{
    Pattern_Invalid,
    Pattern_USBState_Unknown,
    Pattern_USBState_Disconnected,
    Pattern_USBState_ConnectedCharging,
    Pattern_USBState_ConnectedFull,
    Pattern_SystemState_Unknown,
    Pattern_SystemState_Logging,
    Pattern_SystemState_Idle,
    Pattern_ErrorState_Ok,
    Pattern_ErrorState_Error,
    Pattern_BatteryState_NotLow,
    Pattern_BatteryState_AlmostLow,
    Pattern_BatteryState_Low,
    Pattern_SyncState_Synced,
    Pattern_SyncState_NotSynced,
    Pattern_Event_ButtonPress,
    Pattern_Event_StartLogging,
    Pattern_Event_StopLogging,
    Pattern_Event_StartSleep,
    Pattern_Event_StopSleep,
    Pattern_Event_Resetting,
    Pattern_Event_DiskFull,
    Pattern_Num
} indicationPatternId_t;


/**
 *  \brief  Pattern specification
 */
/*typedef struct _pattern_t
{
    uint16_t           color;
    uint16_t                onTime;
    uint16_t                offTime;
    uint16_t                repeats;
    uint8_t                 priority;
    uint8_t                 ledNr;
    uint8_t                 channel;
} pattern_t;*/


/**
 *  \brief  Button identification
 */
#define BUTTON_NONE         0
#define BUTTON_SW1          1
#define BUTTON_SW2          2
#define BUTTON_BOTH         (BUTTON_SW1 | BUTTON_SW2)

#define INCLUDE_FXLIB_CONSUMPTION_MEASUREMENTS_SETTINGS


/**
 *  \brief  Button press pattern identification
 */
typedef enum _buttonPressId_t
{
    PressId_Invalid,
    PressId_Start,
    PressId_Stop,
    PressId_StartSleep,
    PressId_StopSleep,
    PressId_Reset,
    PressId_ResetToDsw,
    PressId_EnableDisableMetrics,
    PressId_Num
} buttonPressId_t;


/**
 *  \brief  Button press pattern
 */
typedef struct _press_pattern_t
{
    uint8_t         buttonMask;
    uint8_t         repeats;
    bool            actOnRelease;
    uint16_t        timeMs;
} press_pattern_t;


/**
 *  \brief  Bit manipulation macros
 */
#define SET_ENABLE_METRIC_FLAG_BY_ID(flags, id)     { flags[id/8] |= ((uint8_t) (1<<(id&7))) ;          }
#define SET_DISABLE_METRIC_FLAG_BY_ID(flags, id)    { flags[id/8] &= (uint8_t) (255^(1<<(id&7))) ;      }
#define CHECK_ENABLE_METRIC_FLAG_BY_ID(flags, id)   ((flags[id/8] & ((uint8_t) (1<<(id&7)))) != 0       )


/**
 *  \brief  The application configuration structure
 */
typedef struct
{
    uint32_t    majorVersion;
    uint32_t    minorVersion;

    /* Subject data */
    char        subjectName[80];

#ifdef G4_BLE
    char        bleDeviceName[33];
#endif // G4_BLE
    
    bool        wiffEnabled;
    
    /* FXI data */
    uint16_t    age;
    uint32_t    yearOfBirth;
    uint8_t     monthOfBirth;
    uint8_t     dayOfBirth;
    uint8_t     sex;
    uint8_t     handedness;
    
    uint8_t     height;
    uint16_t    weight;
    
    uint8_t     bodyPosition;
    
    uint8_t     enabledMetric[8];

    bool        debugMetrics;
    
    uint32_t    time;

    char        inputWiffFileName[40];

#ifdef INCLUDE_FXLIB_CONSUMPTION_MEASUREMENTS_SETTINGS
    char        fx_useLib;
#endif
    char        fx_useLibOutput;

    /* PPG configuration data */
    g4_PpgSensorConfig_t ppgConfig;
    
    /* Accelerometer configuration data */
//    g4_AccelConfig_t accelConfig;

    /* LED blink sequence duration and delay between patterns/channels */
    uint16_t    blinkSequenceDuration;
    
    /* Items for button presses */
    press_pattern_t     pressPattern[PressId_Num];

    /* Items for LED indications */
    //pattern_t           pattern[Pattern_Num];
} appConfig_t;

/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/

/**
 *  \brief Initialize configuration structure
 */
void g4_AppG4Ctrl_Config_Init(appConfig_t *cfg);

/**
 *  \brief  Parse config file
 */
bool g4_AppG4Ctrl_Config_Parse(appConfig_t *cfg);

#endif /* defined __G4_APP_CONFIG_H__ */

