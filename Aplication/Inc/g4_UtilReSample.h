/******************************************************************************/

/**
 *  \file    g4_UtilReSample.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Resampler util definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_UTIL_RESAMPLE_H__
#define __G4_UTIL_RESAMPLE_H__

#include <stdbool.h>
#include <stdint.h>

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/

#define G4_UTIL_RESAMPLE_INIT            0x01
#define G4_UTIL_RESAMPLE_NEED_INPUT      0x02
#define G4_UTIL_RESAMPLE_OUTPUT_READY    0x04




typedef struct _g4_UtilResampleAdm_t
{
    int inputL;
    int inputR;
    int inPos;
    int outPos;
    int inFreq;
    int newInFreq;
    int outFreq;
    uint8_t scaleFactor;
    uint8_t state;
} g4_UtilResampleAdm_t;




/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


/**
 *  \brief  Init a resample struct.
 */
void g4_UtilReSample_Init(g4_UtilResampleAdm_t * adm, uint32_t inFreq, uint32_t outFreq, uint8_t scaleFactor);

/**
 *  \brief  Add a sample 
 *  \return true status 
 */
int g4_UtilReSample_Write(g4_UtilResampleAdm_t * adm, int sample, int freq);

/**
 *  \brief  Add a message to the SD card log file
 *  \param  [IN] fmt - printf-like format string, followed by a variable number of args
 *  \return void
 */
int g4_UtilReSample_Read(g4_UtilResampleAdm_t * adm, int * sample);


/**
 *  \brief  Get resample status.
 */
int g4_UtilReSample_Status(const g4_UtilResampleAdm_t * adm);

/**
 *  \brief reset adm struct as if just after the init()
 */
void g4_UtilReSample_Reset(g4_UtilResampleAdm_t * adm);


#endif /* defined __G4_UTIL_RESAMPLE_H__ */

