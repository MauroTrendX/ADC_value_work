/******************************************************************************/

/**
 *  \file    g4_UtilDownSample.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Stdio component definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_UTIL_DOWNSAMPLE_H__
#define __G4_UTIL_DOWNSAMPLE_H__

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/

#define G4_UTIL_DOWNSAMPLE_INIT            0x01
#define G4_UTIL_DOWNSAMPLE_NEED_INPUT      0x02
#define G4_UTIL_DOWNSAMPLE_OUTPUT_READY    0x04


typedef struct _g4_UtilDownSampleAdm_t
{
    int inFreq;
    int32_t transition[4];
    int16_t inSamples[4];
    int index;
    int state;
} g4_UtilDownSampleAdm_t;


/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


/**
 *  \brief  Initialize a downsample structure
 *  \param [IN/OUT] adm handle (pointer to ringbuf administration)
 */
void g4_UtilDownSample_Init(g4_UtilDownSampleAdm_t * adm );

/**
 *  \brief  Add a sample 
 *  \return status 
 */
int g4_UtilDownSample_Write(g4_UtilDownSampleAdm_t * adm, int16_t sample );

/**
 *  \brief  Add a message to the SD card log file
 *  \param  [IN/OUT] adm
 *  \param  [OUT] sample 
 *  \return status
 */
int g4_UtilDownSample_Read( g4_UtilDownSampleAdm_t * adm, int16_t* sample );


/**
 *  \brief  Get downsample status.
 */
int g4_UtilDownSample_Status(const g4_UtilDownSampleAdm_t * adm);

/**
 *  \brief reset adm struct as if just after the init()
 */
void g4_UtilDownSample_Reset(g4_UtilDownSampleAdm_t * adm);


#endif /* defined __G4_UTIL_DOWNSAMPLE_H__ */

