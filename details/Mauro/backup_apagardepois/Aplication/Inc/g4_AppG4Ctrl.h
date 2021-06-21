/******************************************************************************/

/**
 *  \file    g4_AppG4Ctrl.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - G4 component definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_APP_G4_CTRL_H__
#define __G4_APP_G4_CTRL_H__

#include "g4_types.h"

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/

/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

// Exports to share buffers with DSW:
extern uint8_t ppgDataBuffer[(sizeof(g4_ppgMeasurement_t)*25*2)+1];
extern uint8_t accDataBuffer[(sizeof(g4_accMeasurement_t)*128*2)+1];

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


/**
 *  \brief  Component proxy interface
 */
void g4_AppG4Ctrl_Init(void);
void g4_AppG4Ctrl_Start(void);
void g4_AppG4Ctrl_Stop(void);
void g4_AppG4Ctrl_Process(void);

#endif /* defined __G4_APP_G4_CTRL_H__ */

