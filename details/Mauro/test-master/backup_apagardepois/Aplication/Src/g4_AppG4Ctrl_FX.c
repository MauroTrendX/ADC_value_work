//#ifdef  LIB_FXI_5
/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4AppG4Ctrl_FX.c
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   PEG Test and Evaluation Platform - Main component implementation
 *
 *  \remarks (c) Copyright 2016 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

/* Application Layer includes */
#include "g4_AppG4Ctrl_FX.h"
#include "g4_types.h"

/* Hardware Interface Layer includes */
//#include "g4_HilAccelIntf.h"
#include "g4_HilPpgSensorIntf.h"

#include <stdio.h>

/* Utility Layer includes */
//#include "g4_UtilRtc.h"
#include "g4_UtilReSample.h"
#ifdef  LIB_FXI_5
    #include "g4_UtilDownSample.h"
#endif
#include "g4_UtilAssert.h"
//#include "g4_UtilStdio.h"


/* Lib includes */
#include "fxi.h"
#include "fx_named.h"

#include <stdbool.h>

#ifdef DEBUG_FXI
#include <string.h>
#endif // DEBUG_FXI


/******************************************************************************/
/* Extern data                                                                 */
/******************************************************************************/

/******************************************************************************/
/* Local defines and types                                                    */
/******************************************************************************/
//#define USE_TIMESTAMP_SYNC

typedef enum MetricRequiredState_t
{
  METRIC_NOT_REQUIRED = 0,
  METRIC_REQUIRED = 1
} MetricRequiredState_t;


typedef enum
{
    METRICS_FROM_SENSORS,
    METRICS_FROM_FILE
} metric_input_mode_t;

/**
 *  \brief  FXI library required input frequency.
 */
#define FX_INPUT_FREQ         32

/******************************************************************************/
/* Local data                                                                 */
/******************************************************************************/
/**
 *  \brief  Acc old status
 */
static uint32_t sAccStatus_old = 0;

/**
 *  \brief  PPG old status
 */
static uint32_t sPpgStatus_old = 0;

/**
 *  \brief  Instance for FXI lib.
 */
static PFXI_INST                sInst;

/**
 *  \brief Memory used by FXI lib
 */
uint32_t fxmem[14285/4]; // not static because shared with DSW

/**
 *  \brief  Ringbuffer admin for ppg values needed by FXI
 */
static g4_UtilRingBuffer_t      *spPpgRingBuf;

/**
 *  \brief  Ringbuffer admin for acc values needed by FXI
 */
static g4_UtilRingBuffer_t      *spAccRingBuf;

/**
 *  \brief  Ringbuffer data for ppg values needed by FXI
 */
static g4_UtilRingBuffer_t      *spMetricRingBuf;

/**
 *  \brief  Resample data for ppg values needed by FXI
 */
static g4_UtilResampleAdm_t     sPpgResampler;

/**
 *  \brief  Resample data for ambient values needed by FXI
 */
static g4_UtilResampleAdm_t     sAmbientResampler;

#ifdef  LIB_FXI_5
/**
 *  \brief  Downsample data for Acc values needed by FXI
 */
static g4_UtilDownSampleAdm_t     sAccDownSamplerX;
static g4_UtilDownSampleAdm_t     sAccDownSamplerY;
static g4_UtilDownSampleAdm_t     sAccDownSamplerZ;
#endif

/**
 *  \brief  Indicate if FXI needs ppg data
 */
static MetricRequiredState_t    sPpgMetricRequired = METRIC_NOT_REQUIRED;

/**
 *  \brief  Indicate if FXI needs acc data
 */
static MetricRequiredState_t    sAccMetricRequired = METRIC_NOT_REQUIRED;

/**
 *  \brief  Indicate if FXI needs time data
 */
static MetricRequiredState_t    sTimeMetricRequired = METRIC_NOT_REQUIRED;

/**
 *  \brief  Last used FXI time. (This used to only update the time if it is a second further)
 */
static uint32_t                 sLastSetFxTime = 0;

/**
 *  \brief  Used to send ppg samples to FXI
 */
static fx_MetricPpgRaw_t        sPpgMetric={0};

/**
 *  \brief  Used to send acc samples to FXI
 */
static fx_MetricAccRaw_t        sAccMetric={0};

/**
 *  \brief  Index into ppg metric
 */
static int                      sPpgIdx=0;

/**
 *  \brief  Index into acc metric
 */
static int                      sAccIdx=0;

/**
 *  \brief  All the changed metrics after a Process() are stored here.
 */
static FXI_METRIC_ID            sChangedMetrics[FXI_MAX_UPDATED_METRICS];

/**
 *  \brief  Number of changed metrics in \ref sChangedMetrics.
 */
static FX_UINT08                sChangedMetricsCount = 0;

/**
 *  \brief  Index into changed metrics in \ref sChangedMetrics.
 */
static FX_UINT08                sChangedMetricsIdx = 0;

/**
 *  \brief  Indicate that Ppg needs to be sunchronized to Acc.
 */
static bool                     syncPpgToAcc = false;

/**
 *  \brief  Indicate that Acc needs to be sunchronized to Ppg.
 */
static bool                     syncAccToPpg = false;

/**
 *  \brief  Number of Ppg samples that needs to be faked. Used to synchronize with Acc.
 */
static uint32_t                 sNumPpgStuffing = 0;

/**
 *  \brief  Number of Acc samples that needs to be faked. Used to synchronize with Ppg.
 */
static uint32_t                 sNumAccStuffing = 0;

/**
 *  \brief  Indicates if raw metrics stream is used or sensor input is used.
 */
static metric_input_mode_t      sMetricsInputMode = METRICS_FROM_SENSORS;

/**
 *  \brief Indicate to use library. If 0 then all action with the library is
 *         skipped after startup.
 */
static bool                      sUseLib = true;

/**
 *  \brief  Indicate that there are two ppg and 8 acc samples so we can call process function
 */
static bool                      sPPGAvailable = false;
static bool                      sACCAvailable = false;

static bool                      sEnableDisableMetricsStatus = false;

/**
 *  \brief  User specific configuration items that need to be passed to the library
 */
static uint16_t sAge;
static uint8_t  sDayOfBirth;
static uint8_t  sMonthOfBirth;
static uint32_t sYearOfBirth;
static uint8_t  sHandedness;
static uint8_t  sSex;
static uint8_t  sHeight;
static uint16_t sWeight;
static uint8_t  sBodyPosition;

/**
 *  \brief  Metrics to be disabled
 */
static FXI_METRIC_ID sMetricsToBeDisabled[64]= {0};

/**
 *  \brief  Metrics to be disabled count
 */
static uint8_t sMetricsToBeDisabledCount = 0;

/**
 *  \brief  Error state boolean
 */
static bool     sError;

/******************************************************************************/
/* Local function prototypes                                                  */
/******************************************************************************/

static void checkRequiredInputMetrics( void );

static void CheckOutputMetrics(void);

/*static void EnablePpg(void);

static void DisablePpg(void);

static void EnableAcc(void);

static void DisableAcc(void);
*/
static bool IsInputMetricForFXI( FXI_METRIC_ID id );

static void SetTimeMetric(uint32_t curTime);

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/
void g4_AppG4Ctrl_FX_Init(g4_UtilRingBuffer_t * pRingBufPpg,
                          g4_UtilRingBuffer_t * pRingBufAcc,
                          g4_UtilRingBuffer_t * pRingBufMetric)
{
    /*  Initialize static variables */
    spPpgRingBuf = pRingBufPpg;
    spAccRingBuf = pRingBufAcc;
    spMetricRingBuf = pRingBufMetric;
    
    sPpgMetricRequired = METRIC_NOT_REQUIRED;
    sAccMetricRequired = METRIC_NOT_REQUIRED;
    sTimeMetricRequired = METRIC_NOT_REQUIRED;
    
    sNumAccStuffing = 0;
    sNumPpgStuffing = 0;
    
    syncAccToPpg = false;
    syncPpgToAcc = false;
    
    sPPGAvailable = false;
    sACCAvailable = false;
    
    sAccIdx=0;
    sPpgIdx=0;
    
    sChangedMetricsCount = 0;
    sChangedMetricsIdx = 0;
    
    sMetricsToBeDisabledCount=0;
    
    sEnableDisableMetricsStatus = false;
}


/******************************************************************************/


void g4_AppG4Ctrl_FX_Start(appConfig_t * cfg)
{
    volatile FXI_ERROR stat = 0;
    FXI_INST_PARAMS params ={0};
    
    /* Enable metrics */
    uint8_t index=0;
    int rv = 0;
    FXI_METRIC_ID metricsToBeEnabled[64]=
    {
#ifdef DEBUG_FXI
        /* do not enable metrics in configuration file if you want use this */
        //      FXI_METRIC_ID_WESTPRIVATEDATA,
        //      FXI_METRIC_ID_SLEEPSESSION,
        FXI_METRIC_ID_HEARTRATE,
        FXI_METRIC_ID_SKINPROXIMITY,
        //      FXI_METRIC_ID_ENERGYEXPENDITURE,
        //      FXI_METRIC_ID_SPEED,
        //      FXI_METRIC_ID_MOTIONCADENCE,
        //      FXI_METRIC_ID_ACTIVITYTYPE,
        //      FXI_METRIC_ID_HEARTBEATS,
        //      FXI_METRIC_ID_RESPIRATIONRATE,
        //      FXI_METRIC_ID_COMPRESSED_ACCELERATION,
        //      FXI_METRIC_ID_COMPRESSED_PPG,
        FXI_METRIC_ID_ACCELERATION,
        FXI_METRIC_ID_PPG,
        FXI_METRIC_ID_ACTIVITYCOUNT,
#endif // DEBUG_FXI
        0
    };
    
    /* assume no error has occured */
    sError = false;
    
    params.memorySize = sizeof(fxmem);
    params.pMem = fxmem;
    params.pSourceID = (FX_UINT08*) "G4FX";
    params.sourceIDSize  = 5;
    
    sAge                        = cfg->age;
    sDayOfBirth                 = cfg->dayOfBirth;
    sMonthOfBirth               = cfg->monthOfBirth;
    sYearOfBirth                = cfg->yearOfBirth;
    sHandedness                 = cfg->handedness;
    sSex                        = cfg->sex;
    sHeight                     = cfg->height;
    sWeight                     = cfg->weight;
    
    sBodyPosition               = cfg->bodyPosition;
    
    /* Configure ppg */
//    g4_HilPpgSensorIntf_Config( &(cfg->ppgConfig), (bool) false );
    
    /* Configure accelerometer */
//    g4_HilAccelIntf_Config( &(cfg->accelConfig), (bool) false );
    
    //stat=FXI_GetDefaultParams(&params);
    
    /* Do not remove following commented lines! */
//    ((int32_t*)fxmem)[0] = ~'tsew';
//    ((int32_t*)fxmem)[0] = ~'TSEW';
    
    /* Initialize ( start ) FXI library */
    stat = FXI_Initialise(&params, &sInst);
    while (stat != 0) {};
		
    /* Enable metrics defined in config file */
//    printf("Enabled metrics:");
    for (FXI_METRIC_ID id=0; id < (sizeof(cfg->enabledMetric)*8); id++)
    {
        if ( CHECK_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, id) )
        {
            metricsToBeEnabled[index] = id;
            rv = g4_AppG4Ctrl_FX_EnableMetrics(1, &metricsToBeEnabled[index]);
            
            if (rv==0)
            {
//                printf(" %02x", id);
                index++;
            }
            else
            {
//                printf(" !%02x", id);
//                g4_UtilStdio_AddLog("Error %x enabling metric %02X.", rv, metricsToBeEnabled[index]);
                sError = true;
            }   
        }   
    }
    if (!index)
    {
//        printf("none.\r\n");
    }
    else
    {
//        printf(".\r\n");
    }
    
    sEnableDisableMetricsStatus = true;
    
#ifdef DEBUG_FXI    
    {
        int tmp=0;
        for (int i=0; i<sizeof(cfg->enabledMetric); i++)
        {
            tmp += cfg->enabledMetric[i];
        }
        if (0 == tmp) 
        {        
            index=strlen((char*)metricsToBeEnabled);
            g4_AppG4Ctrl_FX_EnableMetrics(index, metricsToBeEnabled);
        }
    }
#endif // DEBUG_FXI
    
#ifdef DEBUG_FXI
    (void)FXI_GetDefaultParams(&params);
#endif // DEBUG_FXI    
    
    if ( cfg->inputWiffFileName[0] )
    {
        sMetricsInputMode = METRICS_FROM_FILE;
    }
    else
    {
        sMetricsInputMode = METRICS_FROM_SENSORS;
        
//        (void)FX_InitMetricPpgRaw(&sPpgMetric, sBodyPosition);
        
//        (void)FX_InitMetricAccRaw(&sAccMetric, sBodyPosition);
        
#ifdef  LIB_FXI_5
        g4_UtilDownSample_Init(&sAccDownSamplerX);
        g4_UtilDownSample_Init(&sAccDownSamplerY);
        g4_UtilDownSample_Init(&sAccDownSamplerZ);
#endif
    }
    
#ifdef INCLUDE_FXLIB_CONSUMPTION_MEASUREMENTS_SETTINGS
    sUseLib = cfg->fx_useLib;
#else
    (void)cfg;
#endif

    /*  Initialize static variables */
    sPpgMetricRequired  = METRIC_NOT_REQUIRED;
    sAccMetricRequired  = METRIC_NOT_REQUIRED;
    sTimeMetricRequired = METRIC_NOT_REQUIRED;
    
    sNumAccStuffing = 0;
    sNumPpgStuffing = 0;
    
    syncAccToPpg = false;
    syncPpgToAcc = false;
    
    sPPGAvailable = false;
    sACCAvailable = false;
    
    sAccIdx = 0;
    sPpgIdx = 0;
    
    sChangedMetricsCount = 0;
    sChangedMetricsIdx = 0;
    
    sAccStatus_old = 0;
    sPpgStatus_old = 0;
    
    sMetricsToBeDisabledCount = 0;

//    SetTimeMetric(g4_UtilRtc_GetCurrentUnixTime());
    
    checkRequiredInputMetrics();
}


/******************************************************************************/


void g4_AppG4Ctrl_FX_Stop(const appConfig_t * cfg)
{
    /* Disable metrics */
    uint8_t index = 0;
    
    /* Disable metrics defined in config file */
//    printf("Disabling metrics:");
    for (FXI_METRIC_ID id=0; id < (sizeof(cfg->enabledMetric)*8); id++)
    {
        if ( CHECK_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, id) )
        {
            sMetricsToBeDisabled[index] = id;
            
            index++;  
        }   
    }
    
    sMetricsToBeDisabledCount = index;
    
}


/******************************************************************************/


void g4_AppG4Ctrl_FX_DeInit(void)
{
    FXI_ERROR stat = 0;
 /*   
    if ( METRIC_REQUIRED == sPpgMetricRequired )
    {
        DisablePpg();
    }
    
    if ( METRIC_REQUIRED == sAccMetricRequired )
    {
        DisableAcc();
    }
*/
    /* Force initial state so isBusy will be false */
    sPpgMetricRequired  = METRIC_NOT_REQUIRED;
    sAccMetricRequired  = METRIC_NOT_REQUIRED;
    sTimeMetricRequired = METRIC_NOT_REQUIRED;
    
    sNumAccStuffing = 0;
    sNumPpgStuffing = 0;
    
    syncAccToPpg = false;
    syncPpgToAcc = false;
    
    sPPGAvailable = false;
    sACCAvailable = false;
    
    sAccIdx = 0;
    sPpgIdx = 0;
    
    sChangedMetricsCount = 0;
    sChangedMetricsIdx = 0;
    
    sAccStatus_old = 0;
    sPpgStatus_old = 0;
    
    sMetricsToBeDisabledCount = 0;
    
    /* Terminate library */
    stat = FXI_Terminate(&sInst);
    while (stat != 0) {};
}


/******************************************************************************/


int g4_AppG4Ctrl_FX_SetSleepPreferece(bool enabled)
{
    fx_MetricSleepPreference_t msp;
//    (void)FX_InitMetricHeader(&msp.header, FXI_METRIC_ID_SLEEPPREFERENCE);
    msp.data.preference = enabled ? FX_METRIC_PROFILE_SLEEPPREFERENCE_YES : FX_METRIC_PROFILE_SLEEPPREFERENCE_NO;
    return g4_AppG4Ctrl_FX_SetMetric(&msp);
}


/******************************************************************************/


bool g4_AppG4Ctrl_FX_PendingError(void)
{
    return sError;
}


/******************************************************************************/
/* Local functions                                                            */
/******************************************************************************/

/* In case of shared SPI and if both, PPG and ACC, are enabled then interupt source must be PPG xor ACC */
/*static void EnablePpg(void)
{
//    g4_AccelInterruptSource_t accIrqSource;
    uint32_t pin = 0xffffffff;
    uint32_t slot;
    
    g4_HilPpgSensorIntf_GetInterruptConfig(&pin, &slot);
    
//    accIrqSource.select=G4_HILACC_INTERRUPTSRC_OTHER;
//    accIrqSource.pin = (int16_t) pin;
//    accIrqSource.slot = (int8_t) slot+1;
//    g4_HilAccelIntf_SetInterruptSource(accIrqSource);
    // Assumption: Before this call PPG interupt source was disabled so we can call start (shared SPI problem)
    g4_HilPpgSensorIntf_Start(spPpgRingBuf);
}*/


/******************************************************************************/


/*static void DisablePpg(void)
{
//    g4_AccelInterruptSource_t accIrqSource;
    
//    accIrqSource.select=G4_HILACC_INTERRUPTSRC_NONE;
//    g4_HilAccelIntf_SetInterruptSource(accIrqSource);
    g4_HilPpgSensorIntf_Stop();
    // Patch (find other solution): hope we did not miss ACC interrupt
    // @TODO add miss check into g4_HilAccelIntf_SetInterruptSource
//    accIrqSource.select=G4_HILACC_INTERRUPTSRC_OWN;
//    g4_HilAccelIntf_SetInterruptSource(accIrqSource);
}*/


/******************************************************************************/

/* Before this function call ACC interrupt must be set (set in init )*/
/*static void EnableAcc(void)
{
//    g4_HilAccelIntf_Start(spAccRingBuf);
}*/


/******************************************************************************/

/* Before this function call ACC must be started on it's own or other interupt 
 * If PPG is enabled ACC interrupt must be set to other  
 */
/*static void DisableAcc(void)
{
//    g4_HilAccelIntf_Stop();
}*/


/******************************************************************************/


static void checkRequiredInputMetrics( void )
{
    /* ask what input metrics are needed */
    int i=0;
    volatile int stat;
    
    FXI_METRIC_ID requiredMetrics[FXI_MAX_REQUIRED_METRICS] = { 0 };
    FX_UINT08 requiredMetricsCount = sizeof(requiredMetrics);
    MetricRequiredState_t newPpgMetricRequired = METRIC_NOT_REQUIRED;
    MetricRequiredState_t newAccMetricRequired = METRIC_NOT_REQUIRED;
    MetricRequiredState_t newTimeMetricRequired = METRIC_NOT_REQUIRED;
    
    if (sMetricsInputMode == METRICS_FROM_FILE)
    {
        requiredMetricsCount=0;
    }
    else
    {
        /* Collect required metrics: */
        stat = FXI_ListRequiredMetrics(sInst, requiredMetrics, &requiredMetricsCount);
        while (stat) {};
    }
    
    while (i<requiredMetricsCount)
    {		
        /* loop through required metrics */
        switch (requiredMetrics[i])
        {
        case FXI_METRIC_ID_PPG:
            {
                newPpgMetricRequired = METRIC_REQUIRED;
            }
            break;
            
        case FXI_METRIC_ID_ACCELERATION:
            {
                newAccMetricRequired = METRIC_REQUIRED;
            }
            break;
            
        case FXI_METRIC_ID_TIME:
            {
                newTimeMetricRequired = METRIC_REQUIRED;
            }
            break;
            
        case FXI_METRIC_ID_SLEEPPREFERENCE:
            {
                fx_MetricSleepPreference_t msp;
                (void)FX_InitMetricHeader(&msp.header, FXI_METRIC_ID_SLEEPPREFERENCE);
                msp.data.preference = FX_METRIC_PROFILE_SLEEPPREFERENCE_NO;
                stat = g4_AppG4Ctrl_FX_SetMetric(&msp);

                if (stat == FXI_ERROR_NONE)
                {
//                    printf("FX_SetMetric FXI_METRIC_ID_SLEEPPREFERENCE\r\n");
                }
                else
                {
//                    g4_UtilStdio_AddLog("FX: Failed to set SleepPreference (%04x)", stat);
                    sError = true;
                }
            }
            break;
            
        case FXI_METRIC_ID_AGE:
            {
                /* Init age metric */
                fx_MetricAge_t ageMetric;
                (void)FX_InitMetricHeader(&(ageMetric.header), FXI_METRIC_ID_AGE );
                ageMetric.data.age = (uint8_t) sAge;
                stat = g4_AppG4Ctrl_FX_SetMetric( (void*) &ageMetric );

                if (stat == FXI_ERROR_NONE)
                {
//                    printf("FX_SetMetric FXI_METRIC_ID_AGE\r\n");
                }
                else
                {
//                    g4_UtilStdio_AddLog("FX: Failed to set age (%04x)", stat);
                    sError = true;
                }
            }
            break;
    
        case FXI_METRIC_ID_PROFILE:
            {
                /* Init profile metric*/
                fx_MetricProfile_t profileMetric;
                (void)FX_InitMetricHeader(&(profileMetric.header), FXI_METRIC_ID_PROFILE );
                profileMetric.data.dayOfBirth = sDayOfBirth;
                profileMetric.data.monthOfBirth = sMonthOfBirth;
                profileMetric.data.yearOfBirthH = ((sYearOfBirth & 0xFF00) >> 8);
                profileMetric.data.yearOfBirthL = (sYearOfBirth & 0x00FF);
                profileMetric.data.handedness = sHandedness;
                profileMetric.data.sex = sSex;
                stat = g4_AppG4Ctrl_FX_SetMetric( (void*) &profileMetric );

                if (stat == FXI_ERROR_NONE)
                {
//                    printf("FX_SetMetric FXI_METRIC_ID_PROFILE\r\n");
                }
                else
                {
//                    g4_UtilStdio_AddLog("FX: Failed to set profile (%04x)", stat);
                    sError = true;
                }
            }
            break;
    
        case FXI_METRIC_ID_HEIGHT:
            {
                /* Init height metric */
                fx_MetricHeight_t heightMetric;
                (void)FX_InitMetricHeader(&(heightMetric.header), FXI_METRIC_ID_HEIGHT );
                heightMetric.data.height = sHeight;
                stat = g4_AppG4Ctrl_FX_SetMetric( (void*) &heightMetric );

                if (stat == FXI_ERROR_NONE)
                {
//                    printf("FX_SetMetric FXI_METRIC_ID_HEIGHT\r\n");
                }
                else
                {
//                    g4_UtilStdio_AddLog("FX: Failed to set height (%04x)", stat);
                    sError = true;
                }
            }
            break;
    
        case FXI_METRIC_ID_WEIGHT:
            {
                /* Init weight metric */
                fx_MetricWeight_t weightMetric;
                (void)FX_InitMetricHeader(&(weightMetric.header), FXI_METRIC_ID_WEIGHT );
                weightMetric.data.weightH = ((sWeight & 0xFF00) >> 8);
                weightMetric.data.weightL = (sWeight & 0x00FF);
                stat = g4_AppG4Ctrl_FX_SetMetric( (void*) &weightMetric );

                if (stat == FXI_ERROR_NONE)
                {
//                    printf("FX_SetMetric FXI_METRIC_ID_WEIGHT\r\n");
                }
                else
                {
//                    g4_UtilStdio_AddLog("FX: Failed to set weight (%04x)", stat);
                    sError = true;
                }
            }
            break;

        default:
            {
#ifdef DEBUG_FXI
//                printf("Dunno how to get metric %02X.\r\n", requiredMetrics[i]);
#endif // DEBUG_FXI
            }
            break;
            
        }
        i++;
    }
    
    if ( newPpgMetricRequired != sPpgMetricRequired )
    {
        /* ppg requirement has changed */   
        if ( newPpgMetricRequired == METRIC_REQUIRED )
        {
            /* Initialize filters */
            g4_UtilReSample_Init(&sAmbientResampler, PPG_SAMPLING_FREQ, FX_INPUT_FREQ, 0);
            g4_UtilReSample_Init(&sPpgResampler, PPG_SAMPLING_FREQ, FX_INPUT_FREQ, 0);
            
//            EnablePpg();
            syncPpgToAcc = 1;
#ifdef DEBUG_FXI
//            printf("Ppg ON.\r\n");
#endif // DEBUG_FXI
        }
        else
        {
//            DisablePpg();
            g4_UtilRingBuffer_Reset(spPpgRingBuf);
#ifdef DEBUG_FXI
//            printf("Ppg OFF.\r\n");
#endif // DEBUG_FXI
        }
        sPpgMetricRequired = newPpgMetricRequired;
    }
    
    if ( newAccMetricRequired != sAccMetricRequired )
    {  
        /*  acc requirement has changed */
        if ( newAccMetricRequired == METRIC_REQUIRED )
        {
//            EnableAcc();
            syncAccToPpg = 1;
#ifdef DEBUG_FXI
//            printf("Acc ON.\r\n");
#endif // DEBUG_FXI
        }
        else
        {
//            DisableAcc();
//            g4_UtilRingBuffer_Reset(spAccRingBuf);
#ifdef DEBUG_FXI
//            printf("Acc OFF.\r\n");
#endif // DEBUG_FXI
        }
        sAccMetricRequired = newAccMetricRequired; 
    }
    
    /* time requirement might have changed. We just pass it on: */
    sTimeMetricRequired = newTimeMetricRequired;
}


/******************************************************************************/


static int MoveChangedOutputMetricsToOutputRB(void)
{
    /* Move output metrics to a ring buffer */
    while (sChangedMetricsIdx<sChangedMetricsCount)
    {
        FX_UINT08 aMetricBuffer[FX_MAX_METRIC_SIZE];
        FX_UINT16 aMetricBufferSize=sizeof(aMetricBuffer);
        FXI_ERROR stat;
        
        stat = FXI_GetMetric(sInst, sChangedMetrics[sChangedMetricsIdx], aMetricBuffer, &aMetricBufferSize);
        
        if ((!stat)&&(aMetricBufferSize))
        {
            uint32_t rv = g4_UtilRingBuffer_Write(spMetricRingBuf, aMetricBuffer, aMetricBufferSize);
            
            if (rv == 0)
            {
                if ( g4_UtilRingBuffer_GetSize(spMetricRingBuf) > aMetricBufferSize )
                {
                    /* Return and wait space for a metric */
                    return 0;
                }
                else
                {
                    /* Ignore this metric as it will never fit. */
//                    printf("Ignore metrics \r\n");
                }
            }
            
        }
        sChangedMetricsIdx++;
    }
    
    return 1;
}


/******************************************************************************/


static void CheckOutputMetrics(void)
{
    FXI_ERROR stat;
    FX_UINT08 changedMetricCount;
    
    /* Size of changed metrics ids array */
    changedMetricCount = sizeof(sChangedMetrics)/sizeof(sChangedMetrics[0]);
    
    /* Ask what metrics have been updated */
    stat = FXI_ListUpdatedMetrics(sInst, sChangedMetrics, &changedMetricCount);

    if ( !stat )
    {
        sChangedMetricsIdx = 0;
        sChangedMetricsCount = changedMetricCount;
    }
}


/******************************************************************************/


bool g4_AppG4Ctrl_FX_Busy(void)
{
    bool rv;

    rv = ( sChangedMetricsIdx < sChangedMetricsCount );

    /* No need to check input queues. Current implementation will stay in Process()
     * until queues are depleated.
     */

    return rv;
}


/******************************************************************************/


bool g4_AppG4Ctrl_FX_BusyStopping(void)
{
    bool rv;

    rv = ( sChangedMetricsCount ) || 
         ( METRIC_REQUIRED == sAccMetricRequired ) || 
         ( METRIC_REQUIRED == sPpgMetricRequired ) || 
         ( METRIC_REQUIRED == sTimeMetricRequired ) ||
         ( g4_UtilRingBuffer_GetUsedSize( spMetricRingBuf ) );

    return rv;
}


/******************************************************************************/


uint8_t g4_AppG4Ctrl_FX_Process(void)
{
    uint32_t ppgCount;
    uint32_t accCount;
    uint32_t ustat=0;
    uint32_t fxstat=0;
    bool continueDataProcessing=true;
    uint32_t accStatus_new;
    uint32_t ppgStatus_new;
    
    g4_ppgMeasurement_t ppgData = { 0 };
    g4_accMeasurement_t accData = { 0 };
    
    ppgCount = g4_UtilRingBuffer_GetUsedSize( spPpgRingBuf ) / sizeof(g4_ppgMeasurement_t);
    accCount = g4_UtilRingBuffer_GetUsedSize( spAccRingBuf ) / sizeof(g4_accMeasurement_t);
    
    
    if (!MoveChangedOutputMetricsToOutputRB())
    {
        return 0;
    }
    
    if ( !( sPpgMetricRequired || sAccMetricRequired || sTimeMetricRequired || sChangedMetricsCount || (sMetricsInputMode == METRICS_FROM_FILE) ) )
    {
        return 1;
    }
    
    if ((sPpgMetricRequired == METRIC_REQUIRED) && (ppgCount == 0))
    {
        return 2;
    }
    if ((sAccMetricRequired == METRIC_REQUIRED) && (accCount == 0))
    {
        return 3;
    }
    
    //printf(">%d,%d.",ppgCount,accCount);
    
#ifdef USE_TIMESTAMP_SYNC
    if ((sPpgMetricRequired == METRIC_REQUIRED) && (sAccMetricRequired == METRIC_REQUIRED))   // ???????
    {
        /* Sync */
        int32_t tsDiff;
        /* peek ppg ts */
        (void)g4_UtilRingBuffer_Peek(spPpgRingBuf, &ppgData, sizeof(g4_ppgMeasurement_t));
        /* peek acc ts */
        (void)g4_UtilRingBuffer_Peek(spAccRingBuf, &accData, sizeof(g4_accMeasurement_t));
        
        tsDiff = (int32_t)((unsigned)accData.timeStamp-(unsigned)ppgData.timeStamp);
#ifdef DEBUG_FXI
        //printf("**%5d-%5d=%6d.",accData.timeStamp,ppgData.timeStamp, tsDiff);
#endif // DEBUG_FXI
        
        if (syncAccToPpg && syncPpgToAcc)
        {
            if (tsDiff<0)
            {
                syncAccToPpg=0;
            }
            else
            {
                syncPpgToAcc=0;
            }
        }
        
        if (syncPpgToAcc)
        {
            if (tsDiff <= -(G4_MEASUREMENT_TIMESTAMP_ONESECOND/25))
            {
                sNumPpgStuffing = (uint32_t) -(tsDiff/(G4_MEASUREMENT_TIMESTAMP_ONESECOND/25));
            }
#ifdef DEBUG_FXI
            //printf("Syncing by fabricating %d Ppg samples.\r\n", sNumPpgStuffing);
#endif // DEBUG_FXI
//            g4_UtilStdio_AddLog("Sync %d Ppg smp.", sNumPpgStuffing);
            syncPpgToAcc=0;
        }
        
        if (syncAccToPpg)
        {
            if (tsDiff >= (G4_MEASUREMENT_TIMESTAMP_ONESECOND/128))
            {
                sNumAccStuffing = tsDiff/(G4_MEASUREMENT_TIMESTAMP_ONESECOND/128);
            }
#ifdef DEBUG_FXI
            //printf("Syncing by fabricating %d Acc samples.\r\n", sNumAccStuffing);
#endif // DEBUG_FXI
            
#ifdef DEBUG_TIMESTAMP_SYNC
//            g4_UtilStdio_AddLog("Sync %d Acc smp.", sNumAccStuffing);
#endif
            syncAccToPpg=0;
        }
        
    }
#endif
    
    int ppgRsStat = g4_UtilReSample_Status(&sPpgResampler);
    int ambRsStat = g4_UtilReSample_Status(&sAmbientResampler);
#ifdef  LIB_FXI_5
    int accRsStatX = g4_UtilDownSample_Status(&sAccDownSamplerX);
    int accRsStatY = g4_UtilDownSample_Status(&sAccDownSamplerY);
    int accRsStatZ = g4_UtilDownSample_Status(&sAccDownSamplerZ);
#endif
//    g4_UtilAssert(ppgRsStat == ambRsStat);
    
    if (sTimeMetricRequired)
    {
        uint32_t curTime;// = g4_UtilRtc_GetCurrentUnixTime();
        
        if (sLastSetFxTime!=curTime)
        {
            SetTimeMetric(curTime);
        }
        
        if ((sAccMetricRequired != METRIC_REQUIRED)&&(sPpgMetricRequired != METRIC_REQUIRED))
        {
            fxstat = FXI_Process(sInst);
            
            if (fxstat)
            {
//                printf("#%4x#",fxstat);
            }
        }
    }
    
    //copiar a partir daqui
    while (continueDataProcessing)
    {
        if (sAccMetricRequired == METRIC_REQUIRED)
        {
            while (sAccIdx < FX_NUMBER_OF_ACC_SAMPLES_PER_METRIC)
            {
#ifdef  LIB_FXI_5
              while ( accRsStatX & G4_UTIL_RESAMPLE_NEED_INPUT )
                {
                    if (sNumAccStuffing)
                    {
                        ustat = 1;
                        sNumAccStuffing--;
                    }
                    else
                    {
                        ustat = g4_UtilRingBuffer_Read(spAccRingBuf, &accData, sizeof(accData));
                    }
                    
                    if (ustat)
                    {
                        accRsStatX = g4_UtilDownSample_Write( &sAccDownSamplerX, accData.x );
                        accRsStatY = g4_UtilDownSample_Write( &sAccDownSamplerY, accData.y );
                        accRsStatZ = g4_UtilDownSample_Write( &sAccDownSamplerZ, accData.z );
                        g4_UtilAssert( accRsStatX == accRsStatY );
                        g4_UtilAssert( accRsStatY == accRsStatZ );
                    }
                    else
                    {
                        continueDataProcessing = false;
                        break;
                    }
                }
                
                while ( continueDataProcessing && ( accRsStatX & G4_UTIL_DOWNSAMPLE_OUTPUT_READY ) && ( sAccIdx < FX_NUMBER_OF_ACC_SAMPLES_PER_METRIC ))
                {
                    int16_t downSampleX;
                    int16_t downSampleY;
                    int16_t downSampleZ;
                    
                    accRsStatX = g4_UtilDownSample_Read( &sAccDownSamplerX, &downSampleX );
                    accRsStatY = g4_UtilDownSample_Read( &sAccDownSamplerY, &downSampleY );
                    accRsStatZ = g4_UtilDownSample_Read( &sAccDownSamplerZ, &downSampleZ );
                    g4_UtilAssert( accRsStatX == accRsStatY );
                    g4_UtilAssert( accRsStatY == accRsStatZ );
                    
                    FX_UpdateMetricAccWithSample( &sAccMetric, sAccIdx, 0, downSampleX );
                    FX_UpdateMetricAccWithSample( &sAccMetric, sAccIdx, 1, downSampleY );
                    FX_UpdateMetricAccWithSample( &sAccMetric, sAccIdx, 2, downSampleZ );
                    
                    sAccIdx++;
                }
#else
                if (sNumAccStuffing)
                {
                    ustat = 1;
                    sNumAccStuffing--;
                }
                else
                {
                    ustat = g4_UtilRingBuffer_Read(spAccRingBuf, &accData, sizeof(accData));
                }
                
                if (ustat==0)
                {
                    continueDataProcessing=false;
                    break;
                }
                
                FX_UpdateMetricAccWithSample(&sAccMetric, sAccIdx, 0, accData.x);
                FX_UpdateMetricAccWithSample(&sAccMetric, sAccIdx, 1, accData.y);
                FX_UpdateMetricAccWithSample(&sAccMetric, sAccIdx, 2, accData.z);

                sAccIdx++;
#endif                
            }
            
            if (sAccIdx == FX_NUMBER_OF_ACC_SAMPLES_PER_METRIC)
            {
                if (sUseLib)
                {
                    (void) FXI_SetMetric(sInst,sAccMetric.header.id,(FX_UINT08*)&sAccMetric,sizeof(sAccMetric));
                }
                
                sAccIdx++;
            }
        }
        
        if (sPpgMetricRequired == METRIC_REQUIRED)
        {//copiar essa parte para interpolaçao PPG
            while ((sPpgIdx < FX_NUMBER_OF_PPG_SAMPLES_PER_METRIC) && (continueDataProcessing))
            {
                
                while ( ppgRsStat & G4_UTIL_RESAMPLE_NEED_INPUT)
                {
                    if (sNumPpgStuffing)
                    {
                        ustat = 1;
                        sNumPpgStuffing--;
                    }
                    else
                    {
                        ustat = g4_UtilRingBuffer_Read(spPpgRingBuf, &ppgData, sizeof(ppgData));
                    }
                    
                    if (ustat)
                    {
                        ppgRsStat = g4_UtilReSample_Write(&sPpgResampler, ppgData.ppg, 25);
                        ambRsStat = g4_UtilReSample_Write(&sAmbientResampler, ppgData.amb, 25);
//                        g4_UtilAssert(ppgRsStat == ambRsStat);
                    }
                    else
                    {
                        continueDataProcessing=false;
                        break;
                    }
                }
                
                while (continueDataProcessing && ( ppgRsStat & G4_UTIL_RESAMPLE_OUTPUT_READY ) && ( sPpgIdx < FX_NUMBER_OF_PPG_SAMPLES_PER_METRIC ))
                {
                    int resample=0, ambient_resample=0;
                    
                    ppgRsStat = g4_UtilReSample_Read(&sPpgResampler,&resample);
                    ambRsStat = g4_UtilReSample_Read(&sAmbientResampler,&ambient_resample);
//                    g4_UtilAssert(ppgRsStat == ambRsStat);
                    
                    //lint -e572
                    FX_UpdateMetricPpgWithPpgSample(&sPpgMetric, sPpgIdx, resample);
#ifdef  LIB_FXI_5                  
                    FX_UpdateMetricPpgWithAmbSample(&sPpgMetric, sPpgIdx, ambient_resample);
#else
                    FX_UpdateMetricPpgWithAmbSample(&sPpgMetric, sPpgIdx, ambient_resample + 3000);                  
#endif
                    //lint +e572
                    sPpgIdx++;
                }
            }
            
            if (sPpgIdx == FX_NUMBER_OF_PPG_SAMPLES_PER_METRIC)
            {
                sPpgMetric.data.relAdcGain = (FX_UINT08) ppgData.gain;
                sPpgMetric.data.relLedPwr = (FX_UINT08) ppgData.pwr;
                if (sUseLib)
                {
                    (void) FXI_SetMetric(sInst, sPpgMetric.header.id, (FX_UINT08*)&sPpgMetric, sizeof(sPpgMetric));
                }
                
                sPpgIdx++;
            }
        }
        
        if (
              (( sAccIdx > FX_NUMBER_OF_ACC_SAMPLES_PER_METRIC) ||(sAccMetricRequired != METRIC_REQUIRED)) &&
              (( sPpgIdx > FX_NUMBER_OF_PPG_SAMPLES_PER_METRIC) ||(sPpgMetricRequired != METRIC_REQUIRED)) &&
               ( sMetricsInputMode == METRICS_FROM_SENSORS )
           )
        {
            int stat = 0;
            sAccIdx=0;
            sPpgIdx=0;
            
            if (sUseLib)
            {
                fxstat = FXI_Process(sInst);
                
                if (fxstat) 
                {
//                    g4_UtilStdio_AddLog("Library proccesing error %4x", fxstat);
                }
                
                CheckOutputMetrics();
            }
            
            stat = MoveChangedOutputMetricsToOutputRB();
            
            if (!stat)
            {
                /* If otput buffer is full then proccesing will wait for a space */
                /* There is MoveChangedOutputMetricsToOutputRB on the begining of the function */
                //g4_UtilStdio_AddLog("Output Ring Buffer is full!");
                continueDataProcessing=false;
                break;
            }
            
            /* This is the moment when we should disable metrics if needed */
            if ( sMetricsToBeDisabledCount > 0 )
            {
                bool rv = g4_AppG4Ctrl_FX_DisableMetrics(sMetricsToBeDisabledCount, &sMetricsToBeDisabled[0]);
                
                if (!rv)
                {
//                    printf("Disabling metrics...\r\n");
                }
                
                sMetricsToBeDisabledCount=0;
            }
            
            if (sUseLib)
            {
                checkRequiredInputMetrics();
            }
        }
        
        if (sMetricsInputMode == METRICS_FROM_FILE)
        {
            int readBytes;
            uint8_t metric[FX_MAX_METRIC_SIZE];
            
            fx_MetricHeader_t * pMetric = (fx_MetricHeader_t*)metric;
            
            readBytes = (int) g4_UtilRingBuffer_Peek(spAccRingBuf, pMetric, sizeof(fx_MetricHeader_t));
            
            if (readBytes)
            {
                uint32_t size = FX_GetMetricSize(metric);
                (void)g4_UtilRingBuffer_Read(spAccRingBuf, pMetric, size);
                
                /* filter out settable metrics: */
                if ( IsInputMetricForFXI( pMetric->id ) )
                {
                    (void)g4_AppG4Ctrl_FX_SetMetric((void*)metric);
                    
                    if ( pMetric->id == FXI_METRIC_ID_ACCELERATION)
                    {
                        sACCAvailable = true;
                    }
                    if ( pMetric->id == FXI_METRIC_ID_PPG)
                    {
                        sPPGAvailable = true;
                    }
                    
                    if (sPPGAvailable && sACCAvailable)
                    {
                        sPPGAvailable = false;
                        sACCAvailable = false;
                        fxstat = FXI_Process(sInst);

                        if (fxstat)
                        {
//                            printf("Library proccesing error %4x", fxstat);
                        }
                        
                        CheckOutputMetrics();
                        {
                            int stat = MoveChangedOutputMetricsToOutputRB();
                            
                            if (!stat)
                            {
                                continueDataProcessing=false;
                                break;
                            }
                        }
                    }
                }
            }
            else
            {
                continueDataProcessing=false;
                break;
            }
        }
        
        
        if ((sAccMetricRequired != METRIC_REQUIRED) && (sPpgMetricRequired != METRIC_REQUIRED) && (sMetricsInputMode == METRICS_FROM_SENSORS))
        {
            continueDataProcessing=false;
        }
        
//        accStatus_new = g4_HilAccelIntf_GetAccStatus();
        
        if ( accStatus_new != sAccStatus_old )
        {
//            g4_UtilStdio_AddLog("SamplesNum: %d, Status:%X", ( ( accStatus_new & ACC_STATUS_FLAG_NUM_OF_SAMPLES ) >> 8), ( accStatus_new & (~ACC_STATUS_FLAG_NUM_OF_SAMPLES) ) );
            
//            if ( ( accStatus_new & ACC_STATUS_FLAG_STUCKED ) && ( !(sAccStatus_old & (~ACC_STATUS_FLAG_STUCKED) )) )
            {
                for (uint8_t i = 0; i < 15; i++ )
                {
                    uint8_t regValue;
//                    if ( !g4_HilAccelIntf_GetRegBackupData(i, &regValue ))
                    {
//                        g4_UtilStdio_AddLog("RegAddr: %X Val: %X", i  + ACC_REG_INDEX_OFFSET, regValue );
                    }
                }
            }
            
            sAccStatus_old = accStatus_new;
        }
        
//        ppgStatus_new = g4_HilPpgSensorIntf_GetStatus();
        
        if ( ppgStatus_new != sPpgStatus_old )
        {
//            g4_UtilStdio_AddLog("PPG status: %x", ppgStatus_new);
            sPpgStatus_old = ppgStatus_new;
        }
    }
		return 10;
}


/******************************************************************************/


void g4_AppG4Ctrl_EnableDisableMetricsTest( const appConfig_t * cfg )
{
    uint8_t index=0;
    int rv = 0;
    FXI_METRIC_ID metricsToBeEnabled[32] =
    {
        0
    };
    
    if ( sEnableDisableMetricsStatus )
    {
        /* Disable metrics defined in config file except activity type */
        for(FXI_METRIC_ID id=0; id < (sizeof(cfg->enabledMetric)*8); id++)
        {
            if( CHECK_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, id) )
            {
                if ( ( id != FXI_METRIC_ID_ACTIVITYTYPE ) && ( id >= FXI_METRIC_ID_HEARTRATE ) )
                {
                    metricsToBeEnabled[index] = id;
                    rv = g4_AppG4Ctrl_FX_DisableMetrics( 1, &metricsToBeEnabled[index] );
                    
                    if (rv==0)
                    {
                        index++;
                    }
                    else
                    {
//                        printf("Error %x enabling metric %02X.\n\r", rv, metricsToBeEnabled[index]);
                    }
                }
            }
        }
        
        sEnableDisableMetricsStatus = false;
    }
    else
    {
        for(FXI_METRIC_ID id=0; id < (sizeof(cfg->enabledMetric)*8); id++)
        {
            if( CHECK_ENABLE_METRIC_FLAG_BY_ID(cfg->enabledMetric, id) )
            {
                if ( id != FXI_METRIC_ID_ACTIVITYTYPE )
                {
                    metricsToBeEnabled[index] = id;
                    rv = g4_AppG4Ctrl_FX_EnableMetrics( 1, &metricsToBeEnabled[index] );
                    
                    if (rv==0)
                    {
                        index++;
                    }
                    else
                    {
//                        printf("Error %x enabling metric %02X.\n\r", rv, metricsToBeEnabled[index]);
                    }
                }
            }
        }
        
        sEnableDisableMetricsStatus = true;
    }
}


/******************************************************************************/


int g4_AppG4Ctrl_FX_SetMetric(const void * pMetric)
{
    int rv=-1;
    if (sInst)
    {
//lint -e826
        rv = FXI_SetMetric(sInst, ((fx_MetricHeader_t*)pMetric)->id , pMetric, FX_GetMetricSize(pMetric));
//lint +e826
    }
    return rv;
}


/******************************************************************************/


int g4_AppG4Ctrl_FX_GetMetric(void * pMetric, int maxBufSize)
{
    if (sInst)
    {
        FX_UINT16 bufSize = (FX_UINT16) maxBufSize;
//lint -e826
        return FXI_GetMetric(sInst, ((fx_MetricHeader_t*)pMetric)->id , pMetric, &bufSize);
//lint +e826
    }
    return -1;
}


/******************************************************************************/


int g4_AppG4Ctrl_FX_EnableMetrics(int numberOfMetrics, const FXI_METRIC_ID * metrics)
{
    int rv = -1;

    if (sInst)
    {
        rv = FXI_EnableMetrics(sInst, (FX_UINT08) numberOfMetrics, metrics);
    }
    
    return rv;
}

/******************************************************************************/

int g4_AppG4Ctrl_FX_DisableMetrics(int numberOfMetrics, const FXI_METRIC_ID * metrics)
{
    int rv = -1;
    
    if (sInst)
    {
        rv = FXI_DisableMetrics(sInst, (FX_UINT08) numberOfMetrics, metrics);
    }
    
    return rv;
}


/******************************************************************************/


void SetTimeMetric(uint32_t curTime)
{
    fx_MetricTime_t mTime;
    
    mTime.data.time[3] = (uint8_t) ((curTime>>24) & 255);
    mTime.data.time[2] = (uint8_t) ((curTime>>16) & 255);
    mTime.data.time[1] = (uint8_t) ((curTime>>8) & 255);
    mTime.data.time[0] = (uint8_t) (curTime & 255);
    
    (void) FX_InitMetricHeader(&mTime.header, FXI_METRIC_ID_TIME);
    
    if (sUseLib)
    {
        (void) FXI_SetMetric(sInst, mTime.header.id, (FX_UINT08*)&mTime, sizeof(mTime));
    }
    
    sLastSetFxTime = curTime;
    
//    printf("Setting FX time to:%d.\r\n",curTime);
}


/******************************************************************************/


static bool IsInputMetricForFXI( FXI_METRIC_ID id )
{
    bool rv;
    
    switch(id)
    {
    case FXI_METRIC_ID_PPG:
    case FXI_METRIC_ID_ACCELERATION:
    case FXI_METRIC_ID_AGE:
    case FXI_METRIC_ID_PROFILE:
    case FXI_METRIC_ID_HEIGHT:
    case FXI_METRIC_ID_WEIGHT:
    case FXI_METRIC_ID_SLEEPPREFERENCE:
    case FXI_METRIC_ID_TIME:
        rv = true;
        break;
        
    default:
        rv = false;
        break;
    }
    
    return rv;
}

        
/******************************************************************************/
/* Local functions                                                            */
/******************************************************************************/

/**
 * \}
 * End of file.
 */

