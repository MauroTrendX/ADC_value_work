/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_UtilDownSample.c
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Stdio component implementation
 *
 *  \remarks (c) Copyright 2016 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#include "g4_UtilDownSample.h"
#include "g4_UtilAssert.h"

#include <stdio.h>
#include <string.h>


/******************************************************************************/
/* Local defines and types                                                    */
/******************************************************************************/


/******************************************************************************/
/* Local data                                                                 */
/******************************************************************************/

 
/******************************************************************************/
/* Local function prototypes                                                  */
/******************************************************************************/


   
/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


void g4_UtilDownSample_Init(g4_UtilDownSampleAdm_t * adm )
{
    memset(adm, 0, sizeof(g4_UtilDownSampleAdm_t));
    adm->inFreq = 128;
    adm->state = G4_UTIL_DOWNSAMPLE_INIT;
}


/******************************************************************************/


int g4_UtilDownSample_Write(g4_UtilDownSampleAdm_t * adm, int16_t sample )
{
    if ( G4_UTIL_DOWNSAMPLE_NEED_INPUT == adm->state )
    {
        adm->inSamples[adm->index] = sample;
        adm->index = (adm->index + 1) % 4;
        
        if ( 0 == adm->index )
        {
            adm->state = G4_UTIL_DOWNSAMPLE_OUTPUT_READY;
        }
    }
    else if ( G4_UTIL_DOWNSAMPLE_INIT == adm->state )
    {
        adm->inSamples[adm->index] = sample;
        adm->index = (adm->index + 1) % 4;
        
        adm->state = G4_UTIL_DOWNSAMPLE_NEED_INPUT;
    }
    
    return adm->state;
}


/******************************************************************************/


int g4_UtilDownSample_Read( g4_UtilDownSampleAdm_t * adm, int16_t* sample )
{
    int32_t tmp[2];
    
    if ( G4_UTIL_DOWNSAMPLE_OUTPUT_READY == adm->state )
    {
        adm->transition[1] = ( adm->inSamples[1] - adm->transition[1] + ((int32_t)adm->transition[0] * 2) ) / 2;
        adm->transition[0] = adm->inSamples[1];
        tmp[0] = adm->transition[1] + adm->inSamples[0];
        
        adm->transition[1] = ( adm->inSamples[3] - adm->transition[1] + ((int32_t)adm->transition[0] * 2) ) / 2;
        adm->transition[0] = adm->inSamples[3];
        tmp[1] = adm->transition[1] + adm->inSamples[2];
        
        adm->transition[3] = ( tmp[1] - adm->transition[3] + ((int32_t)adm->transition[2] * 2) ) / 2;
        adm->transition[2] = tmp[1];
        tmp[0] = adm->transition[3] + tmp[0];
        
        *sample = (int16_t) ( ( tmp[0] / 4) < -32768 ? -32768 : ( tmp[0] / 4) > 32767 ? 32767 : ( tmp[0] / 4) );
        
        adm->state = G4_UTIL_DOWNSAMPLE_NEED_INPUT;
    }
    
    return adm->state;
}


/******************************************************************************/

    
int g4_UtilDownSample_Status(const g4_UtilDownSampleAdm_t * adm)
{
    uint8_t state;
    
    if ( G4_UTIL_DOWNSAMPLE_INIT != adm->state )
    {
      state = adm->state;
    }
    else
    {
      state = G4_UTIL_DOWNSAMPLE_NEED_INPUT;
    }
    
    return state;
}

/**
 * \}
 * End of file.
 */

