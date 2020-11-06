/******************************************************************************/

/**
 *  \file    g4_AppMainCtrl.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Main component definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_APP_MAIN_CTRL_H__
#define __G4_APP_MAIN_CTRL_H__


/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/

/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


/**
 *  \brief  Component proxy interface
 */
void g4_AppMainCtrl_Init(void);
void g4_AppMainCtrl_Start(void);
void g4_AppMainCtrl_Stop(void);
void g4_AppMainCtrl_Process(void);


#endif /* defined __G4_APP_MAIN_CTRL_H__ */

