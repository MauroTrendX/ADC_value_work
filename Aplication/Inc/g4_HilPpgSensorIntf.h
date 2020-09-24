/******************************************************************************/

/**
 *  \file    g4_HilPpgSensorIntf.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Accel component definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_HIL_PPG_INTF_H__
#define __G4_HIL_PPG_INTF_H__

#include <stdint.h>
#include <stdbool.h>

//#include "g4_HalGpio.h"
#include "g4_UtilRingBuffer.h"

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/

/**
 *  \brief Control loop parameters
 */
typedef struct _g4_PPGControlLoopConfig_t
{
    bool        enabled;

    uint8_t     thLow;
    uint8_t     thHigh;
    uint16_t    ledMin;
    uint16_t    ledMax;

    uint8_t     adcGain;
    uint16_t    ledPower;

} g4_PPGControlLoopConfig_t;

/**
 *  \brief PPG sensor configuration structure
 */
typedef struct _g4_PpgSensorConfig_t
{
    uint8_t     pulseWidth;
    uint8_t     ledRange;
    
    g4_PPGControlLoopConfig_t controlLoopParams;

} g4_PpgSensorConfig_t;


#define PPG_CS_PORT               GPIO_PORT_B
#define PPG_CS_PIN                          6

#define PPG_SAMPLING_FREQ                  25

/**
 *  \brief  PPG status flags
 */
#define PPG_STATUS_FLAG_ERR_SUBDAC_SET_ZERO             0x00000001
#define PPG_STATUS_FLAG_OVERFLOW_COUNTER                0x00000002
#define PPG_STATUS_FLAG_SAMPES_CNT_NOT_25               0x00000004
#define PPG_STATUS_FLAG_FAKE_INTERRUPT                  0x00000008
#define PPG_STATUS_FLAG_VDD_ANA_ERROR                   0x00000010
#define PPG_STATUS_FLAG_PROX_INT_ERROR                  0x00000020
#define PPG_STATUS_FLAG_ALC_OVF_ERROR                   0x00000040
#define PPG_STATUS_FLAG_LED_COMPB_ERROR                 0x00000080     

/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/**
 *  \brief Device identifier 
 */
extern uint8_t gPpgDevice_id[2];

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/

/**
 *  \brief Initialize PPG Sensor
 */
void g4_HilPpgSensorIntf_Init(void);

/**
 *  \brief Configure PPG sensor parameters. Use NULL for default settings.
 */
void g4_HilPpgSensorIntf_Config( g4_PpgSensorConfig_t* pPpgSensorConfig, bool setDefaults );

/**
 *  \brief Start PPG Sensor
 */
void g4_HilPpgSensorIntf_Start( g4_UtilRingBuffer_t* pRingBufferPpg);//, g4_Gpio_edge_cb_t pfExtern_IrqCallback );

/**
 *  \brief Process PPG Sensor
 */
void g4_HilPpgSensorIntf_Process(void);

/**
 *  \brief Stop PPG Sensor
 */
void g4_HilPpgSensorIntf_Stop(void);

/**
 *  \brief Check PPG Sensor busy status
 */
#ifdef DEBUG_RAW_PPG     
bool g4_HilPpgSensorIntf_Busy(void);
#endif // DEBUG_RAW_PPG

/**
 *  \brief  Request Identity of the PPG Sensor
 *  \return : data size.
 */
uint8_t g4_HilPpgSensorIntf_Identify(uint8_t* pData, uint8_t size);
     
uint32_t g4_HilPpgSensorIntf_GetStatus(void);

/**
 *  \brief  Get IRQ pin config
 */
void g4_HilPpgSensorIntf_GetInterruptConfig(uint32_t* pin, uint32_t * slot);

/**
 *  \brief  LED overcurrent check
 */
#if ( defined(G4_EVA_WATCH_TARGET) || defined(OVERCURRENT_PROTECTION_DBG) )
bool g4_HilPpgSensorIntf_OverCurrentCheck(void);
#endif


#endif /* defined __G4_HIL_PPG_INTF_H__ */
