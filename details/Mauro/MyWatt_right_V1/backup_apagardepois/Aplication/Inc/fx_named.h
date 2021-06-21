/**
 *  \file    fx_structs.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands.
 *  \brief   Fx library interface. Defines the interface to stream video
 *           images in order to retrieve pulserate information.
 *
 *  \remarks (c) Copyright 2017 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

/*============================================================================*
 * Multiple inclusion protection
 *============================================================================*/
#ifndef __FX_NAMED_H__
#define __FX_NAMED_H__
/*============================================================================*
 *  C++ protection
 *============================================================================*/
#ifdef __cplusplus
extern "C" {
#endif
/*============================================================================*
 *
 *============================================================================*/



#include "fxi.h"

#if defined(__GNUC__) || defined (_MSC_VER)
# pragma pack(push,1)
# define __PACKED  
#elif defined ICCARM
# pragma push
# pragma pack(1)
# define __PACKED  
#elif defined __PACKED  
#else
//# warning No mechanism to apply packing
# define __PACKED  __packed
#endif

////#define  LIB_FXI_5

#ifdef  LIB_FXI_5
#define FX_PPG_DEFAULT_BODYPOSITION           ( FX_METRIC_BODYPOSITION_LEFTWRIST )
#define FX_PPG_DEFAULT_SAMPLEFORMAT           ( 0x60 ) //for FXI 5.0

#define FX_ACC_DEFAULT_BODYPOSITION           ( FX_METRIC_BODYPOSITION_LEFTWRIST )
#define FX_ACC_DEFAULT_SAMPLEFORMAT           ( 0x6E ) //for FXI 5.0

#define FX_NUMBER_OF_PPG_SAMPLES_PER_METRIC      32 //for FXI 5.0
#define FX_NUMBER_OF_ACC_SAMPLES_PER_METRIC      32 //for FXI 5.0
#else
#define FX_PPG_DEFAULT_BODYPOSITION           ( FX_METRIC_BODYPOSITION_LEFTWRIST )
#define FX_PPG_DEFAULT_SAMPLEFORMAT           ( 0x7F ) //for prior FXI 5.0

#define FX_ACC_DEFAULT_BODYPOSITION           ( FX_METRIC_BODYPOSITION_LEFTWRIST )
#define FX_ACC_DEFAULT_SAMPLEFORMAT           ( 0xFF ) //for prior FXI 5.0

#define FX_NUMBER_OF_PPG_SAMPLES_PER_METRIC      2 //for prior FXI 5.0
#define FX_NUMBER_OF_ACC_SAMPLES_PER_METRIC      8 //for prior FXI 5.0
#endif

#define FX_MAX_N_HEARTBEATS                      5

#define FX_METRIC_PROFILE_GENDER_UNSPECIFIED    0
#define FX_METRIC_PROFILE_GENDER_MALE           1
#define FX_METRIC_PROFILE_GENDER_FEMALE         2

#define FX_METRIC_PROFILE_HANDEDNESS_UNSPECIFIED        0
#define FX_METRIC_PROFILE_HANDEDNESS_RIGHT              1
#define FX_METRIC_PROFILE_HANDEDNESS_LEFT               2
#define FX_METRIC_PROFILE_HANDEDNESS_MIXED              3
     
#define FX_METRIC_BODYPOSITION_UNSPECIFIED              0
#define FX_METRIC_BODYPOSITION_LEFTWRIST                1
#define FX_METRIC_BODYPOSITION_RIGHTWRIST               2
#define FX_METRIC_BODYPOSITION_UNSPECIFIEDWRIST         3

#define FX_METRIC_PROFILE_SLEEPPREFERENCE_UNSPECIFIED   0
#define FX_METRIC_PROFILE_SLEEPPREFERENCE_YES           1
#define FX_METRIC_PROFILE_SLEEPPREFERENCE_NO            2

#define FX_METRIC_SKINPROXIMITY_ONSKIN          0
#define FX_METRIC_SKINPROXIMITY_OFFSKIN         254
#define FX_METRIC_SKINPROXIMITY_UNSPECIFIED     255

#define FX_METRIC_ACTIVITYTYPE_UNSPECIFIED      0
#define FX_METRIC_ACTIVITYTYPE_OTHER            1
#define FX_METRIC_ACTIVITYTYPE_WALK             2
#define FX_METRIC_ACTIVITYTYPE_RUN              3
#define FX_METRIC_ACTIVITYTYPE_CYCLE            4
#define FX_METRIC_ACTIVITYTYPE_REST             5

#define FX_METRIC_SLEEPSESSION_STARTSRC_AUTO    0
#define FX_METRIC_SLEEPSESSION_STARTSRC_PREF    1
#define FX_METRIC_SLEEPSESSION_ENDSRC_AUTO      0
#define FX_METRIC_SLEEPSESSION_ENDSRC_PREF      1
#define FX_METRIC_SLEEPSESSION_ENDSRC_EOS       2
#define FX_METRIC_SLEEPSESSION_STAGE_UNSPECIFIED 0
#define FX_METRIC_SLEEPSESSION_STAGE_DEEPSLEEP  1
#define FX_METRIC_SLEEPSESSION_STAGE_LIGHTSLEEP 2
#define FX_METRIC_SLEEPSESSION_STAGE_REMSLEEP   3
#define FX_METRIC_SLEEPSESSION_STAGE_AWAKE      4

#define FX_MAX_METRIC_SIZE                      (336+sizeof(fx_MetricHeader_t))


/**
 *  \brief Structure of the common data of a metric sent through FX_INT
 *     
 */
typedef __PACKED struct fx_MetricHeader_t
{
    FX_UINT08   id;
    FX_UINT08   sizeL;
    FX_UINT08   sizeH;
    FX_UINT08   idx;
    FX_UINT08   quality;
    
} fx_MetricHeader_t;



/**
 *  \brief Structures for the \ref FX_METRIC_ID_AGE metric.
 *  
 */
typedef __PACKED struct fx_MetricAgeData_t
{
    FX_UINT08   age;
    
} fx_MetricAgeData_t;

typedef __PACKED struct fx_MetricAge_t
{
    fx_MetricHeader_t header;
    fx_MetricAgeData_t data;
    
} fx_MetricAge_t;




/**
 *  \brief Structures for the \ref FX_METRIC_ID_PROFILE metric.
 *
 */
typedef __PACKED struct fx_MetricProfileData_t
{
    FX_UINT08   yearOfBirthL;
    FX_UINT08   yearOfBirthH;
    FX_UINT08   monthOfBirth;
    FX_UINT08   dayOfBirth;
    FX_UINT08   sex;
    FX_UINT08   handedness;
    
} fx_MetricProfileData_t;

typedef __PACKED struct fx_MetricProfile_t
{
    fx_MetricHeader_t header;
    fx_MetricProfileData_t data;
    
} fx_MetricProfile_t;



/**
 *  \brief Structures for the \ref FX_METRIC_ID_HEIGHT metric
 *
 */
typedef __PACKED struct fx_MetricHeightData_t
{
    FX_UINT08   height;
    
} fx_MetricHeightData_t;

typedef __PACKED struct fx_MetricHeight_t
{
    fx_MetricHeader_t header;
    fx_MetricHeightData_t data;
    
} fx_MetricHeight_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_WEIGHT metric
 *
 */
typedef __PACKED struct fx_MetricWeightData_t
{
    FX_UINT08   weightL;
    FX_UINT08   weightH;
    
} fx_MetricWeightData_t;

typedef __PACKED struct fx_MetricWeight_t
{
    fx_MetricHeader_t header;
    fx_MetricWeightData_t data;
    
} fx_MetricWeight_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_RHRPREF metric (does not exist in new lib version)
 *
 */
typedef __PACKED struct fx_MetricRestheartRatePreferenceData_t
{
    FX_UINT08   restHeartrate;
    
} fx_MetricRestheartRatePreferenceData_t;

typedef __PACKED struct fx_MetricRestheartRatePreference_t
{
    fx_MetricHeader_t header;
    fx_MetricRestheartRatePreferenceData_t data;
    
} fx_MetricRestheartRatePreference_t;




/**
 *  \brief Structures for \ref FX_METRIC_ID_SLEEPPREFERENCE metric
 *
 */
typedef __PACKED struct fx_MetricSleepPreferenceData_t
{
    FX_UINT08   preference;
    
} fx_MetricSleepPreferenceData_t;

typedef __PACKED struct fx_MetricSleepPreference_t
{
    fx_MetricHeader_t header;
    fx_MetricSleepPreferenceData_t data;
    
} fx_MetricSleepPreference_t;




/**
 *  \brief Structures for \ref FX_METRIC_ID_TIME metric
 *
 */
typedef __PACKED struct fx_MetricTimeData_t
{
    FX_UINT08   time[4];
    
} fx_MetricTimeData_t;

typedef __PACKED struct fx_MetricTime_t
{
    fx_MetricHeader_t header;
    fx_MetricTimeData_t data;
    
} fx_MetricTime_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_AVERAGE_HEARTRATE metric
 *     
 */
typedef __PACKED struct fx_MetricAverageHeartRateData_t
{
    FX_UINT08   bpm;
    
} fx_MetricAverageHeartRateData_t;

typedef __PACKED struct fx_MetricAverageHeartRate_t
{
    fx_MetricHeader_t                 header;
    fx_MetricAverageHeartRateData_t   data;
    
} fx_MetricAverageHeartRate_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_RESTINGHEARTRATE metric
 *
 */
typedef __PACKED struct fx_MetricRestingHeartRateData_t
{
    FX_UINT08   restHeartrate;
    
} fx_MetricRestingHeartRateData_t;

typedef __PACKED struct fx_MetricRestingHeartRate_t
{
    fx_MetricHeader_t header;
    fx_MetricRestingHeartRateData_t data;
    
} fx_MetricRestingHeartRate_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_SKINPROXIMITY metric
 *
 */
typedef __PACKED struct fx_MetricSkinProximityData_t
{
    FX_UINT08   skinProximity;
    
} fx_MetricSkinProximityData_t;

typedef __PACKED struct fx_MetricSkinProximity_t
{
    fx_MetricHeader_t header;
    fx_MetricSkinProximityData_t data;
    
} fx_MetricSkinProximity_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_ENERGYEXPENDITURE metric
 *
 */
typedef __PACKED struct fx_MetricEnergyExpenditureData_t
{
    FX_UINT08   energyExpenditureL;
    FX_UINT08   energyExpenditureH;
    
} fx_MetricEnergyExpenditureData_t;

typedef __PACKED struct fx_MetricEnergyExpenditure_t
{
    fx_MetricHeader_t header;
    fx_MetricEnergyExpenditureData_t data;
    
} fx_MetricEnergyExpenditure_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_SPEED metric
 *
 */
typedef __PACKED struct fx_MetricSpeedData_t
{
    FX_UINT08   speed;
    
} fx_MetricSpeedData_t;

typedef __PACKED struct fx_MetricSpeed_t
{
    fx_MetricHeader_t header;
    fx_MetricSpeedData_t data;
    
} fx_MetricSpeed_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_CADENCE metric
 *     
 */
typedef __PACKED struct fx_MetricCadenceData_t
{
    FX_UINT08   cadence;
    
} fx_MetricCadenceData_t;

typedef __PACKED struct fx_MetricCadence_t
{
    fx_MetricHeader_t                 header;
    fx_MetricCadenceData_t            data;
    
} fx_MetricCadence_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_ACTIVITYTYPE metric
 *     
 */
typedef __PACKED struct fx_MetricActivityTypeData_t
{
    FX_UINT08   type;
    
} fx_MetricActivityTypeData_t;

typedef __PACKED struct fx_MetricActivityType_t
{
    fx_MetricHeader_t                 header;
    fx_MetricActivityTypeData_t       data;
    
} fx_MetricActivityType_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_HEARTBEATS metric
 *
 */
typedef __PACKED struct fx_MetricHeartBeatData_t
{
    FX_UINT08           quality;
    FX_UINT08           time[4];
    FX_UINT08           envL;
    FX_UINT08           envH;
    FX_UINT08           type;
    
} fx_MetricHeartBeatData_t;

typedef __PACKED struct fx_MetricHeartBeatsData_t
{
    FX_UINT08           nBeats;
    FX_UINT08           bodyPosition;
    fx_MetricHeartBeatData_t  beats[FX_MAX_N_HEARTBEATS];
    
} fx_MetricHeartBeatsData_t;

typedef __PACKED struct fx_MetricHeartBeats_t
{
    fx_MetricHeader_t header;
    fx_MetricHeartBeatsData_t data;
    
} fx_MetricHeartBeats_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_VO2MAX metric
 *     
 */
typedef __PACKED struct fx_MetricVo2MaxData_t
{
    FX_UINT08   vo2max;
    
} fx_MetricVo2MaxData_t;

typedef __PACKED struct fx_MetricVo2Max_t
{
    fx_MetricHeader_t header;
    fx_MetricVo2MaxData_t data;
    
} fx_MetricVo2Max_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_CARDIOFITNESSINDEX metric
 *     
 */
typedef __PACKED struct fx_MetricCardioFitnessIndexData_t
{
    FX_UINT08   cardioFitnessIndex;
    
} fx_MetricCardioFitnessIndexData_t;

typedef __PACKED struct fx_MetricCardioFitnessIndex_t
{
    fx_MetricHeader_t header;
    fx_MetricCardioFitnessIndexData_t data;
    
} fx_MetricCardioFitnessIndex_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_RESPIRATIONRATE metric
 *     
 */
typedef __PACKED struct fx_MetricRespirationRateData_t
{
    FX_UINT08   respirationRate;
    
} fx_MetricRespirationRateData_t;

typedef __PACKED struct fx_MetricRespirationRate_t
{
    fx_MetricHeader_t header;
    fx_MetricRespirationRateData_t data;
    
} fx_MetricRespirationRate_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_ACC_RAW metric
 *     
 */
typedef __PACKED struct fx_MetricAccRawData_t
{
    FX_UINT08   bodyPosition;
    FX_UINT08   sampleFormat;
    FX_UINT08   acc[2*3*FX_NUMBER_OF_ACC_SAMPLES_PER_METRIC];
    
} fx_MetricAccRawData_t;

typedef __PACKED struct fx_MetricAccRaw_t
{
    fx_MetricHeader_t       header;
    fx_MetricAccRawData_t   data;
    
} fx_MetricAccRaw_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_PPG_RAW metric
 *     
 */
typedef __PACKED struct fx_MetricPpgRawDataPpgAmb_t
{
    FX_UINT08 ppgL;
    FX_UINT08 ppgH;
    FX_UINT08 ambL;
    FX_UINT08 ambH;
    
} fx_MetricPpgRawDataPpgAmb_t;

typedef __PACKED struct fx_MetricPpgRawData_t
{
    FX_UINT08   bodyPosition;
    FX_UINT08   sampleFormat;
    FX_UINT08   relLedPwr;
    FX_UINT08   relAdcGain;
    fx_MetricPpgRawDataPpgAmb_t ppgAmb[FX_NUMBER_OF_PPG_SAMPLES_PER_METRIC];
    
} fx_MetricPpgRawData_t;

typedef __PACKED struct fx_MetricPpgRaw_t
{
    fx_MetricHeader_t       header;
    fx_MetricPpgRawData_t   data;
    
} fx_MetricPpgRaw_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_INTERMITTENT_HEARTRATE metric
 *     
 */
typedef __PACKED struct fx_MetricIntermittentHeartRateData_t
{
    FX_UINT08   heartRate;
    
} fx_MetricIntermittentHeartRateData_t;

typedef __PACKED struct fx_MetricIntermittentHeartRate_t
{
    fx_MetricHeader_t header;
    fx_MetricIntermittentHeartRateData_t data;
    
} fx_MetricIntermittentHeartRate_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_ACTIVITY_COUNT metric (does not exist in new lib version)
 *     
 */
typedef __PACKED struct fx_MetricActivityCountData_t
{
    FX_UINT08              bodyPosition;
    FX_UINT08              activityCountsL;
    FX_UINT08              activityCountsH;
  
} fx_MetricActivityCountData_t;

typedef __PACKED struct fx_MetricActivityCount_t
{
    fx_MetricHeader_t header;
    fx_MetricActivityCountData_t data;
  
} fx_MetricActivityCount_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_WEST_FEATURE metric
 *     
 */
typedef __PACKED struct fx_MetricWestPrivateData_t
{
    fx_MetricHeader_t header;
    FX_UINT08 data[];
    
} fx_MetricWestPrivateData_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_SLEEP metric
 *     
 */
typedef __PACKED struct fx_MetricSleep_t
{
    fx_MetricHeader_t header;
    
} fx_MetricSleep_t;



/**
 *  \brief Structures for \ref FX_METRIC_ID_MOTION_EVENT metric (does not exist in new lib version)
 *     
 */
typedef __PACKED struct fx_MetricMotionEventData_t
{
    FX_UINT08              bodyPosition;
    FX_UINT08              status[4];
  
} fx_MetricMotionEventData_t;

typedef __PACKED struct fx_MetricMotionEvent_t
{
    fx_MetricHeader_t header;
    fx_MetricMotionEventData_t data;
    
} fx_MetricMotionEvent_t;



/**
 *  \brief Structures for \ref FXI_METRIC_ID_COMPRESSED_ACCELERATION metric
 *     
 */
typedef __PACKED struct fx_MetricAccCompressed_t
{
    fx_MetricHeader_t       header;
    FX_UINT08               data[1]; /* should actually be dimensionless, but that is not possible in a typedef */
    
} fx_MetricAccCompressed_t;



/**
 *  \brief Structures for \ref FXI_METRIC_ID_COMPRESSED_PPG metric
 *     
 */
typedef __PACKED struct fx_MetricPpgCompressed_t
{
    fx_MetricHeader_t       header;
    FX_UINT08               data[1]; /* should actually be dimensionless, but that is not possible in a typedef */
    
} fx_MetricPpgCompressed_t;



/**
 *  \brief Structures for \ref FXI_METRIC_ID_HEARTRHYTHMTYPE metric
 *     
 */
typedef __PACKED struct fx_MetricHeartRhythmTypeData_t
{
    FX_UINT08              delay;
    FX_UINT08              heartRhythmType;
  
} fx_MetricHeartRhythmTypeData_t;

typedef __PACKED struct fx_MetricHeartRhythmType_t
{
    fx_MetricHeader_t                     header;
    fx_MetricHeartRhythmTypeData_t        data;
    
} fx_MetricHeartRhythmType_t;



/**
 *  \brief Structures for \ref FXI_METRIC_ID_INTERMITTENTENERGYEXPENDITURE metric
 *     
 */
typedef __PACKED struct fx_MetricIntermittentenEnergyExpenditureData_t
{
    FX_UINT08   ieeL;
    FX_UINT08   ieeH;
  
} fx_MetricIntermittentenEnergyExpenditureData_t;

typedef __PACKED struct fx_MetricIntermittentenEnergyExpenditure_t
{
    fx_MetricHeader_t                                        header;
    fx_MetricIntermittentenEnergyExpenditureData_t           data; /* should actually be dimensionless, but that is not possible in a typedef */
  
} fx_MetricIntermittentenEnergyExpenditure_t;


#ifdef ICCARM
# pragma pop
#endif
#if defined(__GNUC__) || defined (_MSC_VER)
# pragma pack(pop)
#endif



/**
 *  \brief General function to initialize the header of a metric struct
 *  It is not needed to call this function when using dedicated init
 *  functions like \ref FX_InitMetricAccRaw and \ref FX_InitMetricPpgRaw
 *
 *  \param pointer to metric
 *  \param metric id to use for setting up metric data.
 *
 *  \return size of metric
 *
 */
FX_UINT16 FX_InitMetricHeader( fx_MetricHeader_t * hdr, FX_UINT08 metric_id );


/**
 *  \brief Initialize a ppg metric struct (except the ppg samples)
 *
 *  \param pointer to ppg metric
 *
 *  \return size of metric
 *
 */
FX_UINT16 FX_InitMetricPpgRaw( fx_MetricPpgRaw_t * metric, FX_UINT08 bodyPosition );


/**
 *  \brief Fill a ppg metric struct with ppg samples
 *
 *  \param metric points to ppg metric buffer
 *  \param ppgs pointer to ppg samples
 *  \param ambs pointer to ambient samples
 *
 */
void FX_UpdateMetricPpgWithSamples(fx_MetricPpgRaw_t * metric, const FX_SINT16 * ppgs, const FX_SINT16 * ambs);


/**
 *  \brief Initialize an acc metric struct (except the acc fields)
 *
 *  \param pointer to acc metric
 *
 *  \return size of metric
 *
 */
FX_UINT16 FX_InitMetricAccRaw( fx_MetricAccRaw_t * metric, FX_UINT08 bodyPosition );


/**
 *  \brief Fill an acc metric struct with acc samples
 *
 *  \param metric points to acc metric buffer
 *  \param accs points to accelerometer samples.
 *
 */
void FX_UpdateMetricAccWithSamples(fx_MetricAccRaw_t * metric, const FX_SINT16 * accs);



/**
 *  \brief Get size of metric (serialized in buffer) 
 *
 *  \param pointer to metric
 *
 *  \return metric size  (minimum: 16 bits)
 *
 */
#define FX_GetMetricSize(metric) ((((fx_MetricHeader_t*)metric)->sizeH*256)+(((fx_MetricHeader_t*)metric)->sizeL)+3)



/**
 *  \brief Get ID of metric (serialized in buffer) 
 *
 *  \param pointer to metric
 *
 *  \return ID of metric (minimum: 8 bits)
 *
 */
#define FX_GetMetricID(metric) (((fx_MetricHeader_t*)metric)->id)


/**
 *  \brief Set a specific ppg sample in a ppg metric struct
 *
 *  \param metric points to ppg metric buffer
 *  \param idx indicates the samples
 *  \param value contains the ppg sample value
 *
 */
#define FX_UpdateMetricPpgWithPpgSample(metric, idx, value) {(metric)->data.ppgAmb[idx].ppgL=(FX_UINT08)((value)&0xff); (metric)->data.ppgAmb[idx].ppgH=(FX_UINT08)((((uint16_t) (value))>>8)&0xff);}


/**
 *  \brief Set a specific ambient sample in a ppg metric struct
 *
 *  \param metric points to ppg metric buffer
 *  \param idx indicates the samples
 *  \param value contains the ambient sample value
 *
 */
#define FX_UpdateMetricPpgWithAmbSample(metric, idx, value) {(metric)->data.ppgAmb[idx].ambL=(FX_UINT08)((value)&0xff); (metric)->data.ppgAmb[idx].ambH=(FX_UINT08)((((uint16_t) (value))>>8)&0xff); }


/**
 *  \brief Set a specific acc sample in an acc metric struct
 *
 *  \param metric points to acc metric buffer
 *  \param idx indicates the acc xyz location
 *  \param xyz indicates which of x,y or z is indicated
 *  \param value contains the x,y or z acc sample value
 *
 */
#define FX_UpdateMetricAccWithSample(metric, idx, xyz, value) {(metric)->data.acc[(((idx)*3)+(xyz))*2]=(FX_UINT08)((value)&0xff); (metric)->data.acc[(((idx)*3)+(xyz))*2+1]=(FX_UINT08)((((uint16_t) (value))>>8)&0xff);}


/*============================================================================*
 *  End of C++ protection
 *============================================================================*/

#ifdef __cplusplus
}
#endif

/*============================================================================*
 *  End of multiple inclusion protection
 *============================================================================*/

#endif
