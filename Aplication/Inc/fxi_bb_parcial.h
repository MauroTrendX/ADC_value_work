/**
* \addtogroup Philips VitalSigns Optical Software Library
* \{
*/
/**
*  \file    fxi.h
*  \author  Philips Intellectual Property & Standards, The Netherlands
*  \brief   VitalSigns Optical Heart Rate Library 
*
*  \remarks (c) Copyright 2019 Koninklijke Philips N.V. All rights reserved.
*  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
*  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
*  All other names are the trademark or registered trademarks of their respective holders.
*/

/*============================================================================*/
/*  Multiple inclusion protection                                             */
/*============================================================================*/

#ifndef __FXI_H__
#define __FXI_H__

/*============================================================================*/
/*  C++ protection                                                            */
/*============================================================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* Include files                                                              */
/*============================================================================*/

#include "fx_datatypes.h"

/*============================================================================*/
/* Constant definitions                                                       */
/*============================================================================*/

/* input metrics */
#define FXI_METRIC_ID_PPG                           ( 0x2C )
#define FXI_METRIC_ID_ACCELERATION                  ( 0x2B )
#define FXI_METRIC_ID_AGE                           ( 0x03 )
#define FXI_METRIC_ID_PROFILE                       ( 0x04 )
#define FXI_METRIC_ID_HEIGHT                        ( 0x05 )
#define FXI_METRIC_ID_WEIGHT                        ( 0x06 )
#define FXI_METRIC_ID_SLEEPPREFERENCE               ( 0x09 )
#define FXI_METRIC_ID_TIME                          ( 0x1E )

/* extracted metrics */
#define FXI_METRIC_ID_HEARTRATE                     ( 0x20 )
#define FXI_METRIC_ID_RESTINGHEARTRATE              ( 0x21 )
#define FXI_METRIC_ID_SKINPROXIMITY                 ( 0x22 )
#define FXI_METRIC_ID_ENERGYEXPENDITURE             ( 0x23 )
#define FXI_METRIC_ID_SPEED                         ( 0x24 )
#define FXI_METRIC_ID_MOTIONCADENCE                 ( 0x25 )
#define FXI_METRIC_ID_ACTIVITYTYPE                  ( 0x26 )
#define FXI_METRIC_ID_HEARTBEATS                    ( 0x27 )
#define FXI_METRIC_ID_VO2MAX                        ( 0x28 )
#define FXI_METRIC_ID_CARDIOFITNESSINDEX            ( 0x29 )
#define FXI_METRIC_ID_RESPIRATIONRATE               ( 0x2A )
#define FXI_METRIC_ID_INTERMITTENTHEARTRATE         ( 0x2D )
#define FXI_METRIC_ID_ACTIVITYCOUNT                 ( 0x2E )
#define FXI_METRIC_ID_WESTPRIVATEDATA               ( 0x2F )
#define FXI_METRIC_ID_SLEEPSESSION                  ( 0x30 )
#define FXI_METRIC_ID_COMPRESSED_ACCELERATION       ( 0x34 )
#define FXI_METRIC_ID_COMPRESSED_PPG                ( 0x35 )
#define FXI_METRIC_ID_HEARTRHYTHMTYPE               ( 0x3B )
#define FXI_METRIC_ID_INTERMITTENTENERGYEXPENDITURE ( 0x3C )

/** @defgroup FXI_ERROR FXI_ERROR 
 *  @{ */
#define FXI_ERROR_NONE                              ( 0x0000 ) /**< @brief no error */
#define FXI_ERROR_NOT_ENOUGH_MEMORY                 ( 0x9001 ) /**< @brief too little memory provided */
#define FXI_ERROR_INITIALISATION_FAILED             ( 0x9002 ) /**< @brief initialisation failed */
#define FXI_ERROR_SIZE_CONFLICT                     ( 0x9003 ) /**< @brief metric data conflicts its size definition */
#define FXI_ERROR_METRIC_NOT_SUPPORTED              ( 0x9004 ) /**< @brief undefined metric id used */
#define FXI_ERROR_MODULE_NOT_INITIALISED            ( 0x9005 ) /**< @brief invalid instance provided */
#define FXI_ERROR_MEMORY_CORRUPTED                  ( 0x9006 ) /**< @brief library corrupted */
#define FXI_ERROR_INVALID_PARAMS                    ( 0x9007 ) /**< @brief provided parameters invalid */
#define FXI_ERROR_MULTIPLE_SET                      ( 0x9008 ) /**< @brief trying to set metric multiple times */
#define FXI_ERROR_DATA_INCOMPLETE                   ( 0x9009 ) /**< @brief not all metric data set as required */
/** @} */

#define FXI_MAX_UPDATED_METRICS                     ( 32 )
#define FXI_MAX_REQUIRED_METRICS                    ( 16 )

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/

typedef FX_UINT16         FXI_ERROR;
typedef FX_UINT08         FXI_METRIC_ID;

/* Parameter definition ------------------------------------------------------*/

/** @brief   FXI Instance Parameter structure type. */
typedef struct fxi_inst_params_tag
{
    FX_VOID             * pMem;         /**< Pointer to memory block used for instance initialisation */
    FX_UINT08           * pSourceID;    /**< Pointer to memory block containing the SourceID */
    FX_UINT16             memorySize;   /**< Size of memory block appointed by pMem field */
    FX_UINT16             sourceIDSize; /**< Size of memory block appointed by pSourceID field */

} FXI_INST_PARAMS, * PFXI_INST_PARAMS;

/* Internal data definition --------------------------------------------------*/

/** @brief FXI Instance Parameter structure containing the actual FXI Instance. */
typedef struct fxi_inst_tag * PFXI_INST;

#ifdef FXI_DYNAMIC_BUILD
    #ifdef FXI_BUILD
    #define FXI_API       FX_INTF_EXPORT
    #else
    #define FXI_API       FX_INTF_IMPORT
    #endif /* FXI_BUILD */
#else
    #define FXI_API       extern
#endif

/*============================================================================*/
/* Extern data declarations                                                   */
/*============================================================================*/

/*============================================================================*/
/* Extern function prototypes                                                 */
/*============================================================================*/

/*----------------------------------------------------------------------------*/
/**
 *  @brief        FXI_GetVersionInfo
 *
 *                Fills teh memory block, appointed by pData, with version information
 *                that is unique for each version of the library.
 * 
 *                The array has to be allocated/maintained by the application.
 *
 *  @param        [in]  pData : Pointer to memory block where the version
 *                              information will be written to.
 *  @param        [in]  pSize : Length of provided memory block, must be at least 20
 *
 *  @param        [out] pData : Pointer to memory block that now contains version information
 *  @param        [out] pSize : Length of the provided version information
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note
 *
**/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_GetVersionInfo
(
  FX_UINT08  * const pData,
  FX_UINT16  * const pSize
);

/*----------------------------------------------------------------------------*/
/**
 *  @brief        FXI_GetDefaultParams
 *
 *                Fills a structure, appointed by pParams, with default data,
 *                e.g. minimal size of provided memory block,  to be used in 
 *                function FXI_Initialise
 * 
 *                The structure has to be allocated/maintained by the application.
 *
 *  @param        [in]  pParams : Pointer to 'empty' parameter struct to be filled with default data
 *
 *  @param        [out] pParams : Pointer to 'filled' parameter struct
 *                                memorySize contains the size needed for an instance.
 *                                sourceIDSize contains the maximum size that is used for source id.
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note
 *
 **/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_GetDefaultParams
(
  FXI_INST_PARAMS * const pParams
);

/* ---------------------------------------------------------------------------*/
/**
 * @brief FXI_Initialise
 *
 *                Creates an @c FXI instance. Allocates and initialises the
 *                resources needed using the data provided by the @c pParams
 *                structure. The actual used memory block from the parameters
 *                must be maintained and not modified during the lifetime of the instance. 
 *
 *                On successful return, @p ppInst identifies the created FXI
 *                instance. This pointer must be used in subsequent calls to
 *                functions relating to this instance.
 *
 *                To release and cleanup the created instance the
 *                {@link #FXI_Terminate()} function must be called.
 *
 *                The FXI instance is allocated/maintained by the application.
 *
 *  @param        [in] pParams  : Pointer to a structure that contains parameters
 *                                used for creation of an FXI instance.
 *  @param        [in]  ppInst  : Pointer to an FXI instance pointer.
 *
 *  @param        [out] ppInst  : On success, a valid pointer to an FXI instance pointer.
 *  @param        [out] pParams : Pointer to a structure that contains updated 
 *                                information in the created instance ( e.g. actual memory used
                                  from the memory pool )
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note
 **/
/*----------------------------------------------------------------------------*/

FXI_API  FXI_ERROR FXI_Initialise
(
  FXI_INST_PARAMS * const pParams,
  PFXI_INST       * const ppInst
);

/*----------------------------------------------------------------------------*/
/**
 *  @brief        FXI_Terminate
 *
 *                Releases and cleans up the resources used by the FXI instance.
 * 
 *                After this function the memory block reserved for this instance,
 *                done with FXI_Initialise function, can be released by the application
 *
 *  @param        [in]  ppInst : Pointer to an FXI instance pointer.
 *
 *  @param        [out] ppInst : On success, pointer to FXI instance pointer is set to NULL.
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note
 **/

/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_Terminate
(
  PFXI_INST       * const ppInst
);

/*----------------------------------------------------------------------------*/
/**
 *
 *  @brief        FXI_Process
 *
 *                Performs the actual processing on provided data/metrics for 
 *                producing the enabled metric(s)
 * 
 *                Data/metrics can be provided with function: FXI_SetMetric
 *                Which data/metrics should be set can be queried
 *                with function: FXI_ListRequestedMetrics
 * 
 *                Metrics can be enabled with function FXI_EnableMetric
 * 
 *                After FXI_Process metrics can be updated, which metrics are updated
 *                can be queried with function:  FX_ListUpdatedMetrics
 * 
 *                The actual (updated) metric data can be retrieved with 
 *                function: FXI_GetMetric 
 *
 *  @param        [in] pInst   : Pointer to a FXI instance.
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note
 *
 **/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_Process
(
  PFXI_INST const pInst
);

/* ---------------------------------------------------------------------------*/
/**
 *
 *  @brief        FXI_EnableMetrics
 *
 *                Enables metrics specified in provided list
 *
 *  @param        [in] pInst           : Pointer to an FXI instance.
 *  @param        [in] numberOfMetrics : number of metrics in the list.
 *  @param        [in] pMetricIdList   : List of metric id's.
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note         The function will only enable 'known' metrics.
 *
 **/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_EnableMetrics
(
  PFXI_INST             const pInst,
  FX_UINT08             const numberOfMetrics,
  FXI_METRIC_ID const * const pMetricIdList
);

/*----------------------------------------------------------------------------*/
/**
 *
 *  @brief        FXI_DisableMetrics
 *
 *                Disables metrics specified in provided list
 *
 *  @param        [in] pInst           : Pointer to an FXI instance.
 *  @param        [in] numberOfMetrics : number of metrics in the list.
 *  @param        [in] pMetricIdList   : List of metric id's.
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note         The function will disable any supported metrics.
 *
 **/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_DisableMetrics
(
  PFXI_INST             const pInst,
  FX_UINT08             const numberOfMetrics,
  FXI_METRIC_ID const * const pMetricIdList
);

/*----------------------------------------------------------------------------*/
/**
 *
 *  @brief        FXI_ListUpdatedMetrics
 *
 *                Returns list if mectric id's that are updated as consequence 
 *                of FXI_Process function call(s)
 * 
 *                This list can dynamically change after FXI_Process function.
 *
 *  @param        [in]  pInst            : Pointer to an FXI instance.
 *  @param        [in]  pMetricIdList    : 'empty' list of metricId.
 *  @param        [in]  pNumberOfMetrics : length of the 'empty' metricId list, maximal FXI_MAX_UPDATED_METRICS
 * 
 *  @param        [out] pMetricIdList    : filled list of metricId.
 *  @param        [out] pNumberOfMetrics : actual valid/written number of metricId's in the list
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note
 *
 **/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_ListUpdatedMetrics
(
  PFXI_INST       const pInst,
  FXI_METRIC_ID * const pMetricIdList,
  FX_UINT08     * const pNumberOfMetrics
);

/*----------------------------------------------------------------------------*/
/**
 *
 *  @brief        FXI_ListRequiredMetrics
 *
 *                Returns list of metric id's that are required to be set with 
 *                FXI_SetMetric before next call to FXI_Process function
 * 
 *                This list can dynamically change after functions: 
 *                FXI_EnableMetrics / FXI_DisableMetrics / FXI_Process functions
 *
 *  @param        [in]  pInst            : Pointer to an FXI instance.
 *  @param        [in]  pMetricIdList    : 'empty' list of metricId.
 *  @param        [in]  pNumberOfMetrics : length of the 'empty' metricId list, maximal FXI_MAX_REQUIRED_METRICS.
 * 
 *  @param        [out] pMetricIdList    : filled list of metricId.
 *  @param        [out] pNumberOfMetrics : actual valid/written number of metricId's in the list
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note
 *
 **/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_ListRequiredMetrics
(
  PFXI_INST       const pInst,
  FXI_METRIC_ID * const pMetricIdList,
  FX_UINT08     * const pNumberOfMetrics
);

/*----------------------------------------------------------------------------*/
/**
 *
 *  @brief        FXI_SetMetric
 *
 *                Feed metric data
 *
 *  @param        [in] pInst      : Pointer to an FXI instance.
 *  @param        [in] metricId   : metric id.
 *  @param        [in] pData      : Pointer to location filled with metric data.
 *  @param        [in] size       : The size ( in bytes ) of the pData location ( metric data ).
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note         The metric data consists of metricId,length,index,quality,data
 *
 **/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_SetMetric
(
  PFXI_INST            const pInst,
  FXI_METRIC_ID        const metricId,
  FX_UINT08    const * const pData,
  FX_UINT16            const size
);

/*----------------------------------------------------------------------------*/
/**
 *
 *  @brief        FXI_GetMetric
 *
 *                Get metric data
 *
 *  @param        [in] pInst           : Pointer to an FXI instance.
 *  @param        [in] metricId        : metric id.
 *  @param        [in] pData           : Pointer to location where metric data will be written to.
 *  @param        [in] pSize           : The size ( in bytes ) of the pData location.
 *
 *  @param        [out] pData          : Pointer to location filled with metric data.
 *  @param        [out] pSize          : The size ( in bytes ) of the pData location ( metric data ).
 *
 *  @return       Value of type \ref FXI_ERROR identifying the error ( \ref FXI_ERROR_NONE if succesfull )
 *
 *  @note         The metric data consists of metricId,length,index,quality,data
 *
 **/
/*----------------------------------------------------------------------------*/

FXI_API FXI_ERROR FXI_GetMetric
(
  PFXI_INST       const pInst,
  FXI_METRIC_ID   const metricId,
  FX_UINT08     * const pData,
  FX_UINT16     * const pSize
);

/*============================================================================*/
/*  End of C++ protection                                                     */
/*============================================================================*/

#ifdef __cplusplus
}
#endif

/*============================================================================*/
/*  End of multiple inclusion protection                                      */
/*============================================================================*/

#endif

/**
* \}
* End of file.
*/
