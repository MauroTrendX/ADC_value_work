/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_UtilReSample.c
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Resampler util implementation
 *
 *  \remarks (c) Copyright 2016 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#include "g4_UtilReSample.h"
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

void g4_UtilReSample_Init( g4_UtilResampleAdm_t * adm, uint32_t inFreq, uint32_t outFreq, uint8_t scaleFactor )
{
    /* initialise structure */
    memset(adm, 0, sizeof(g4_UtilResampleAdm_t));
    adm->inFreq         = (int) inFreq;
    adm->newInFreq      = (int) inFreq;
    adm->outFreq        = (int) outFreq;
    adm->scaleFactor    = (int) scaleFactor;
    
    adm->inputR = 0;
    adm->inPos = 0 - adm->outFreq;
    adm->outPos = adm->inFreq;
        
    adm->state = G4_UTIL_RESAMPLE_NEED_INPUT;
}


/******************************************************************************/


int g4_UtilReSample_Write( g4_UtilResampleAdm_t * adm, int sample, int freq )
{
    if ( adm->state == G4_UTIL_RESAMPLE_NEED_INPUT )
    {
        adm->inputL = adm->inputR;
        adm->inputR = sample;
        
        adm->inPos += adm->outFreq;
        
        adm->newInFreq = freq;
        
        if ( ( adm->outPos >= adm->inPos ) && ( adm->outPos <= ( adm->inPos + adm->outFreq)) )
        {
            adm->state = G4_UTIL_RESAMPLE_OUTPUT_READY;
        }
        else
        {
            adm->state = G4_UTIL_RESAMPLE_NEED_INPUT;
        }
    }
    
    return adm->state;
}


/******************************************************************************/


int g4_UtilReSample_Read( g4_UtilResampleAdm_t * adm, int * sample )
{
    if ( adm->state == G4_UTIL_RESAMPLE_OUTPUT_READY )
    {
        *sample = ( adm->outPos - adm->inPos);
        *sample = (*sample) * ( adm->inputR - adm->inputL );
        *sample = (*sample) / adm->outFreq;
        *sample = (*sample) + adm->inputL;
        *sample = (int) ((unsigned int) (*sample)>>adm->scaleFactor);
       
        if ( (adm->outPos) == ((adm->inPos) + (adm->outFreq)))
        {
            adm->inFreq = adm->newInFreq;
            adm->outPos = adm->inFreq; 
            adm->inPos = 0 - adm->outFreq;
            adm->state = G4_UTIL_RESAMPLE_NEED_INPUT;
        }
        else 
        {
            adm->outPos += adm->inFreq;
            
            if ( ( adm->outPos < adm->inPos ) || ( adm->outPos > ( adm->inPos + adm->outFreq )) )
            {          
                adm->state = G4_UTIL_RESAMPLE_NEED_INPUT;
            }
        }
    }
    
    return adm->state;
}


/******************************************************************************/


int g4_UtilReSample_Status( const g4_UtilResampleAdm_t * adm )
{
    return adm->state;
}


/******************************************************************************/


void g4_UtilReSample_Reset( g4_UtilResampleAdm_t * adm )
{
    adm->state = G4_UTIL_RESAMPLE_INIT;
}


/**
 * \}
 * End of file.
 */

