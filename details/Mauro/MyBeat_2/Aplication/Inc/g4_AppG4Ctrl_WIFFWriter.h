/******************************************************************************/

/**
 *  \file    g4_AppG4Ctrl_WIFFWriter.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - PCS Interchangeable File Format Writer component definitions
 *
 *  \remarks (c) Copyright 2018 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_APP_G4_CTRL_WIFF_WRITER_H__
#define __G4_APP_G4_CTRL_WIFF_WRITER_H__

#include <stdint.h>
#include <stdbool.h>

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/

#define SENSOR_SERIAL_LENGTH            8
#define SENSOR_CONFIG_LENGTH            0x1C

#define HOST_SERIAL_LENGTH              3
#define HOST_CONFIG_LENGTH              0x15

typedef struct _wiff_configuration_t
{
    uint8_t     sensorFWversion;
    uint8_t     sensorFWmajor;
    uint8_t     sensorFWminor;
    uint8_t     sensorSerial[SENSOR_SERIAL_LENGTH];
    uint8_t     sensorConfig[SENSOR_CONFIG_LENGTH];

    uint8_t     hostFWversion;
    uint8_t     hostFWmajor;
    uint8_t     hostFWminor;
    uint8_t     hostSerial[HOST_SERIAL_LENGTH];
    uint8_t     hostConfig[HOST_CONFIG_LENGTH];
} wiff_configuration_t;

/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/

void    g4_AppG4Ctrl_WIFFWriter_Init(void);
void    g4_AppG4Ctrl_WIFFWriter_Start(const wiff_configuration_t *configuration, const char *fileNamePrefix);
void    g4_AppG4Ctrl_WIFFWriter_Stop(void);
void    g4_AppG4Ctrl_WIFFWriter_Process(void);

bool    g4_AppG4Ctrl_WIFFWriter_Busy(void);
bool    g4_AppG4Ctrl_WIFFWriter_Error(void);
void    g4_AppG4Ctrl_WIFFWriter_ReportMetric(const uint8_t *metricData);
bool    g4_AppG4Ctrl_WIFFWriter_IsSpaceAvailable( uint32_t dataSize );

#endif /* defined __G4_APP_G4_CTRL_WIFF_WRITER_H__ */

