/******************************************************************************/

/**
 *  \file    g4_HalPpgSensorSpi.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - PpgSensorSpi component definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_HAL_PPG_SENSORSPI_H__
#define __G4_HAL_PPG_SENSORSPI_H__

#include <stdint.h>
#include <stdbool.h>

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
 *  \brief  spi init
 */
void g4_HalPpgSensorSpi_Init(void);

/**
 *  \brief  spi start
 */
void g4_HalPpgSensorSpi_Start(void);

/**
 *  \brief  spi stop
 */
void g4_HalPpgSensorSpi_Stop(void);

/**
 *  \brief  spi process
 */
 
void g4_HalPpgSensorSpi_Process(void);

/**
 *  \brief SPI Transmit to PPG Sensor Module
 *  \param [IN] TxBuffer : pointer to the SPI transmit buffer
 *  \param [IN] length   : number of bytes to be transmitted
 */
void g4_HalPpgSensorSpi_Transmit(const uint8_t *TxBuffer, uint8_t numOfBytes);

/**
 *  \brief SPI Receive from PPG Sensor Module
 *  \param [OUT] RxBuffer: pointer to the SPI receive buffer
 *  \param [IN] length   : number of bytes to be received
 */

void g4_HalPpgSensorSpi_Receive(uint8_t *RxBuffer, uint8_t numOfBytes);

/**
 *  \brief Slave select enable for PPG SPI 
 */
void g4_HalPpgSensorSpi_SelectSlaveEnable(void);

/**
 *  \brief Slave select disbale for PPG SPI 
 */
void g4_HalPpgSensorSpi_SelectSlaveDisable(void);

#endif /* defined __G4_HAL_PPG_SENSORSPI_H__ */
