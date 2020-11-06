/******************************************************************************/

/**
 *  \file    g4_AppG4Ctrl_WIFFReader.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - PCS Interchangeable File Format Reader component definitions
 *
 *  \remarks (c) Copyright 2018 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_APP_G4_CTRL_WIFF_READER_H__
#define __G4_APP_G4_CTRL_WIFF_READER_H__

#include <stdint.h>
#include <stdbool.h>

#include "g4_UtilRingBuffer.h"

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/

/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/

void    g4_AppG4Ctrl_WIFFReader_Init(void);
void    g4_AppG4Ctrl_WIFFReader_Start(const char *fileName, g4_UtilRingBuffer_t * outRB);
void    g4_AppG4Ctrl_WIFFReader_Stop(void);
void    g4_AppG4Ctrl_WIFFReader_Process(void);

bool    g4_AppG4Ctrl_WIFFReader_Busy(void);
int32_t g4_AppG4Ctrl_WIFFReader_Error(void);


#endif /* defined __G4_APP_G4_CTRL_WIFF_READER_H__ */

