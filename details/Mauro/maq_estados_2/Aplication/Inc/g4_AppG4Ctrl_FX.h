/******************************************************************************/

/**
 *  \file    g4_AppG4Ctrl_FX.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 Test and Evaluation Platform - Main component definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_APP_G4_CTRL_FX_H__
#define __G4_APP_G4_CTRL_FX_H__


#include <stdint.h>
#include <stdbool.h>
#include "g4_UtilRingBuffer.h"
#include "fx_named.h"

#include "g4_AppG4Ctrl_Config.h"

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/
#define FX_MINIMUM_METRIC_BUFFER_SIZE       1164
//90
/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/


/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


/**
 *  \brief  Component proxy interface
 */
void g4_AppG4Ctrl_FX_Init(g4_UtilRingBuffer_t * pRingBufPpg,
                        g4_UtilRingBuffer_t * pRingBufAcc,
                        g4_UtilRingBuffer_t * pRingBufMetric);
void g4_AppG4Ctrl_FX_Start(appConfig_t * cfg);
void g4_AppG4Ctrl_FX_Stop(const appConfig_t * cfg);
void g4_AppG4Ctrl_FX_DeInit(void);
uint8_t g4_AppG4Ctrl_FX_Process(void);
bool g4_AppG4Ctrl_FX_Busy(void);
bool g4_AppG4Ctrl_FX_BusyStopping(void);

void g4_AppG4Ctrl_EnableDisableMetricsTest( const appConfig_t * cfg );

/** 
 *  \brief   Set sleep preference
 *  \param   enabled[IN] : when true, sleep preference set to 'yes', 'no' otherwise.
 *  \return  int status
 */
int g4_AppG4Ctrl_FX_SetSleepPreferece(bool enabled);


/** 
 *  \brief   Set a metric
 *  \param   pMetric[IN] : fully filled metric structure.(Eg: fx_int_MetricAge_t )
 *  \return  0 : OK
 *           else: ERROR
 */
int g4_AppG4Ctrl_FX_SetMetric(const void * pMetric);


/** 
 *  \brief   Get a metric
 *  \param   pMetric[IN/OUT] : fully filled metric structure.(Eg: fx_int_MetricAge_t )
 *                             On entry, the metric ID needs to be filled in.
 *  \param   maxBufSize :      max size of metric at pMetric.
 *  \return  0 : OK
 *           else: ERROR
 */
int g4_AppG4Ctrl_FX_GetMetric(void * pMetric, int maxBufSize);


/** 
 *  \brief   Enable one or more metrics
 *  \param   numberOfMetrics[IN] : nr of metric ids to be enabled.
 *  \param   metrics[IN] : list of metric ids to be enabled
 *  \return  0 : OK
 *           else: ERROR
 */
int g4_AppG4Ctrl_FX_EnableMetrics(int numberOfMetrics, const FXI_METRIC_ID * metrics);


/** 
 *  \brief   Disable one or more metrics
 *  \param   numberOfMetrics[IN] : nr of metric ids to be disables.
 *  \param   metrics[IN] : list of metric ids to be disabled
 *  \return  0 : OK
 *           else: ERROR
 */
int g4_AppG4Ctrl_FX_DisableMetrics(int numberOfMetrics, const FXI_METRIC_ID * metrics);


/** 
 *  \brief   Check for pending errors
 *  \return  true   : error has occured
 *           false  : no error
 */
bool g4_AppG4Ctrl_FX_PendingError(void);

#endif /* defined __G4_APP_G4_CTRL_FX_H__ */

