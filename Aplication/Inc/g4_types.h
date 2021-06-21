/******************************************************************************/

/**
 *  \file    g4_PersistentValueIds.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Persistent Value IDs definitions
 *
 *  \remarks (c) Copyright 2013 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_TYPES_H__
#define __G4_TYPES_H__


#include <stdint.h>

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/
/**
 *  \brief  Value for 1 second in timeStamp field of measurment structs.
 */
#define G4_MEASUREMENT_TIMESTAMP_ONESECOND_SHIFT 14
#define G4_MEASUREMENT_TIMESTAMP_ONESECOND (1 << G4_MEASUREMENT_TIMESTAMP_ONESECOND_SHIFT)

/**
 *  \brief  Structure for date of one Ppg(+Ambient) measurement.
 */
typedef struct g4_ppgMeasurement_t
{
    /*@{*/
    uint16_t  timeStamp;      /**< Timestamp. Needs to be the same resolution as for \ref g4_accMeasurement_t. */
    uint16_t  ppg;            /**< Ppg sample value. */
    uint16_t  amb;            /**< Ambient sample value. */
    uint8_t   pwr;            /**< Led power setting used for this measurement. */
    uint8_t   gain : 4;       /**< Adc gain setting used for this measurement. */
    uint8_t   ppgLow : 4;     /**< Ppg sample value. */
    /*@}*/
} g4_ppgMeasurement_t;



/**
 *  \brief  Structure for one Accelerometer sample.
 */
typedef struct g4_accMeasurement_t
{
    /*@{*/
    uint16_t  timeStamp;      /**< Timestamp. Needs to be the same resolution as for \ref g4__ppgMeasurement_t. */
    int16_t   x;              /**< x - axis for acc measurement. */
    int16_t   y;              /**< y - axis for acc measurement. */
    int16_t   z;              /**< z - axis for acc measurement. */
    /*@}*/
} g4_accMeasurement_t;



/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/

#endif /* defined __G4_TYPES_H__ */

