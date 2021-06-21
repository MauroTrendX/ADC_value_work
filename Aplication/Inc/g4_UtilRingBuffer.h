/******************************************************************************/

/**
 *  \file    g4_UtilRingBuffer.h
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Rtc component definitions
 *
 *  \remarks (c) Copyright 2017 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#ifndef __G4_UTIL_RINGBUFFER_H__
#define __G4_UTIL_RINGBUFFER_H__

#include <stdint.h>

/******************************************************************************/
/* Exported defines and types                                                 */
/******************************************************************************/

/**
 *  \brief Administration for a ringbuffer.
 *   Note: Do not modify any of these values.
 *  \params:
 *     pBuf        : pointer to buffer
 *     size        : size of buffer
 *     head        : head index
 *     tail        : tail index 
 */

typedef struct g4_UtilRingBuffer_tag
{
    uint8_t * pBuf;
    int32_t size;
    int32_t head;
    int32_t tail;
} g4_UtilRingBuffer_t;


/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


/**
 *  \brief  Initialize a buffer
 *  \param [IN/OUT] handle (pointer to ringbuf administration)
 *  \param [IN] pointer to buffer
 *  \param [IN] size of to buffer
 */
void g4_UtilRingBuffer_Init(g4_UtilRingBuffer_t * hnd, void * buffer, uint32_t size );


/**
 *  \brief  Reset a buffer
 *  \param [IN/OUT] handle (pointer to ringbuf administration)
 */
void g4_UtilRingBuffer_Reset(g4_UtilRingBuffer_t * hnd);


/**
 *  \brief  Get total buffer size
 *  \param [IN/OUT] handle (pointer to ringbuf administration)
 *  \return size of buffer
 */
#define g4_UtilRingBuffer_GetTotalSize(hnd) ((uint32_t) hnd->size)


/**
 *  \brief  Get free buffer space. Note: This might change when writing or reading is done in another context.
 *  \param [IN/OUT] handle (pointer to ringbuf administration)
 *  \return free buffer size
 */
uint32_t g4_UtilRingBuffer_GetFreeSize(const g4_UtilRingBuffer_t * const hnd);


/**
 *  \brief  Get occupied buffer space
 *  \param [IN/OUT] handle (pointer to ringbuf administration)
 *  \return size stored data
 */
uint32_t g4_UtilRingBuffer_GetUsedSize(const g4_UtilRingBuffer_t * const hnd);


/**
 *  \brief  Get total buffer size
 *  \param [IN] handle (pointer to ringbuf administration)
 *  \return total buffer size
 */
#define g4_UtilRingBuffer_GetSize(hnd)  (hnd->size)


/**
 *  \brief Write
 *  \param [IN/OUT] handle (pointer to ringbuf administration)
 *  \param [IN] pointer to data to write
 *  \param [IN] size of data to write
 *  \return nr of bytes written
 */
uint32_t g4_UtilRingBuffer_Write(g4_UtilRingBuffer_t * const hnd, const void * const pData, uint32_t size);


/**
 *  \brief Read
 *  \param [IN/OUT] handle (pointer to ringbuf administration)
 *  \param [OUT] pointer to where the date is read
 *  \param [IN] size of data to read
 *  \return : nr of bytes read
 */
uint32_t g4_UtilRingBuffer_Read(g4_UtilRingBuffer_t * hnd, void * pData, uint32_t size);


/**
 *  \brief Peek
 *  \param [IN/OUT] handle (pointer to ringbuf administration)
 *  \param [OUT] pointer to where the date is read
 *  \param [IN] size of data to read
 *  \return : nr of bytes read
 */
uint32_t g4_UtilRingBuffer_Peek(const g4_UtilRingBuffer_t * const hnd, void * const pData, uint32_t size);

#endif /* defined __G4_UTIL_RINGBUFFER_H__ */

