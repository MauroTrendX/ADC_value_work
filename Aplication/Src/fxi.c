/**
* \addtogroup Philips VitalSigns Optical Software Library
* \{
*/
/**
*  \file    fxi.c
*  \author  Philips Intellectual Property & Standards, The Netherlands
*  \brief   VitalSigns Optical Heart Rate Library 
*
*  \remarks (c) Copyright 2019 Koninklijke Philips N.V. All rights reserved.
*  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Software License Agreement dated 1 November 2018.
*  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
*  All other names are the trademark or registered trademarks of their respective holders.
*/

/*============================================================================*/
/* Include files                                                              */
/*============================================================================*/

#include <stdint.h>
#include <stdbool.h>

#include "fxi.h"
#include "fxi_map.h"

/*============================================================================*/
/* Constant definitions                                                       */
/*============================================================================*/

/*============================================================================*/
/* Macro definitions                                                          */
/*============================================================================*/

/*============================================================================*/
/* Type definitions                                                           */
/*============================================================================*/

/*============================================================================*/
/* Static data definitions                                                    */
/*============================================================================*/

/*============================================================================*/
/* Extern data definitions                                                    */
/*============================================================================*/

/*============================================================================*/
/* Local function prototypes                                                  */
/*============================================================================*/

/*============================================================================*/
/* Extern function implementations                                            */
/*============================================================================*/

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_GetVersionInfo                                          *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_GetVersionInfo
(
  FX_UINT08 * const pData,
  FX_UINT16 * const pSize
)
{
  return (( FXI_ERROR  (*)
          ( FX_UINT08 * const, FX_UINT16 * const)) 
            FXI_GetVersionInfo_Address )( pData, pSize );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_GetDefaultParams                                        *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_GetDefaultParams
(
  FXI_INST_PARAMS * const pParams
)
{
  return (( FXI_ERROR  (*)
          ( FXI_INST_PARAMS * const)) 
            FXI_GetDefaultParams_Address )( pParams );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_Initialise                                              *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_Initialise
( 
  FXI_INST_PARAMS * const pParams,
  PFXI_INST       * const ppInst
)
{
  return (( FXI_ERROR  (*)
          ( FXI_INST_PARAMS * const, PFXI_INST * const )) 
            FXI_Initialise_Address )( pParams, ppInst );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_Terminate                                               *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_Terminate
(   
  PFXI_INST * const ppInst
)
{
  return (( FXI_ERROR  (*)
          ( PFXI_INST * const )) 
            FXI_Terminate_Address )( ppInst );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_Process                                                 *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_Process
(
  PFXI_INST const pInst
)
{
  return (( FXI_ERROR  (*)
          ( PFXI_INST const )) 
            FXI_Process_Address )( pInst );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_EnableMetrics                                           *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_EnableMetrics
(
  PFXI_INST             const pInst,
  FX_UINT08             const numberOfMetrics,
  FXI_METRIC_ID const * const pMetricIdList
)
{
  return (( FXI_ERROR  (*)
          ( PFXI_INST const, FX_UINT08 const, FXI_METRIC_ID const * const )) 
            FXI_EnableMetrics_Address )( pInst, numberOfMetrics, pMetricIdList );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_DisableMetrics                                          *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_DisableMetrics
(
  PFXI_INST             const pInst,
  FX_UINT08             const numberOfMetrics,
  FXI_METRIC_ID const * const pMetricIdList
)
{
  return (( FXI_ERROR  (*)
          ( PFXI_INST const, FX_UINT08 const, FXI_METRIC_ID const * const )) 
            FXI_DisableMetrics_Address )( pInst, numberOfMetrics, pMetricIdList );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_ListUpdatedMetrics                                      *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_ListUpdatedMetrics
(
  PFXI_INST       const pInst,
  FXI_METRIC_ID * const pMetricIdList, 
  FX_UINT08     * const pNumberOfMetrics 
)
{
  return (( FXI_ERROR  (*)
          ( PFXI_INST const, FXI_METRIC_ID * const, FX_UINT08 * const)) 
            FXI_ListUpdatedMetrics_Address )( pInst, pMetricIdList, pNumberOfMetrics );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_ListRequiredMetrics                                     *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_ListRequiredMetrics
(
  PFXI_INST       const pInst,
  FXI_METRIC_ID * const pMetricIdList, 
  FX_UINT08     * const pNumberOfMetrics 
)
{
  return (( FXI_ERROR  (*)
          ( PFXI_INST const, FXI_METRIC_ID * const, FX_UINT08 * const)) 
            FXI_ListRequiredMetrics_Address )( pInst, pMetricIdList, pNumberOfMetrics );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_SetMetric                                               *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_SetMetric
( 
  PFXI_INST           const pInst,
  FXI_METRIC_ID       const metricId,
  FX_UINT08   const * const pData,
  FX_UINT16           const size
)
{
  return (( FXI_ERROR  (*)
          ( PFXI_INST const, FXI_METRIC_ID const, FX_UINT08 const * const, FX_UINT16 const )) 
            FXI_SetMetric_Address )( pInst, metricId, pData, size );
}

/*----------------------------------------------------------------------------*
 *                                                                            *
 *  Name        : FXI_GetMetric                                               *
 *                                                                            *
 *----------------------------------------------------------------------------*/

FXI_ERROR FXI_GetMetric
(
  PFXI_INST     const pInst,
  FXI_METRIC_ID const metricId,
  FX_UINT08   * const pData,
  FX_UINT16   * const pSize
)
{
  return (( FXI_ERROR  (*)
          ( PFXI_INST const, FXI_METRIC_ID const, FX_UINT08 * const, FX_UINT16 * const )) 
            FXI_GetMetric_Address )( pInst, metricId, pData, pSize );
}

/**
* \}
* End of file.
*/
