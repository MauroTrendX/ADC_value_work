/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_AppMainCtrl.c
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Main component implementation
 *
 *  \remarks (c) Copyright 2016 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

/* Application Layer includes */
#include "g4_AppMainCtrl.h"
#include "g4_AppG4Ctrl.h"

/* Hardware Interface Layer includes */
//#include "g4_HilAccelIntf.h"
#include "g4_HilPpgSensorIntf.h"
//#include "g4_HilButtonIntf.h"
//#include "g4_HilSystemIntf.h"
//#include "g4_HilUiIntf.h"

/* Utility Layer includes */
//#include "g4_UtilRtc.h"

#include <stdio.h>

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

void g4_AppMainCtrl_Init(void)
{
    /* Start with the bare necessities */
//    g4_HilSystemIntf_Init();
//    g4_HilSystemIntf_Start();
    
    /* Both HIL and UTIL make use of HAL components. The following rules apply
     * to this:
     *
     * - All HAL components are exclusively used by a single HIL or UTIL component
     * - The only exceptions are GPIO and SYSTEM and SPI. They are controlled (init/start/stop)
     *   by Hil System Interface, but any other component may use it.
     *
     * Because all Util, GPIO and SYSTEM are already initialized (through HilSystem), there is
     * no dependency in the order in which the other HIL components are initialized.
     */
    
    /* And then all the rest of HIL */
//    g4_HilAccelIntf_Init();
    g4_HilPpgSensorIntf_Init();
//    g4_HilButtonIntf_Init();
//    g4_HilUiIntf_Init();
    
    /* And now the APP layer init */
    g4_AppG4Ctrl_Init();
}

/******************************************************************************/

void g4_AppMainCtrl_Start(void)
{
//    printf(" Main application starting......... \r\n\r\n");
    
    /* Check reset cause */
//    g4_HilSystemIntf_ResetCause();

  /* And utils will need to be started at well.*/
//    g4_UtilRtc_Start();

    g4_AppG4Ctrl_Start();
}

/******************************************************************************/

void g4_AppMainCtrl_Stop(void)
{
     g4_AppG4Ctrl_Stop();
}

/******************************************************************************/

void g4_AppMainCtrl_Process(void)
{		
//    g4_HilSystemIntf_Process();
    g4_AppG4Ctrl_Process();
    //process other applications
}


/******************************************************************************/
/* Local functions                                                            */
/******************************************************************************/

/**
 * \}
 * End of file.
 */

