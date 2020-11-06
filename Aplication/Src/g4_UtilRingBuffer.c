/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_UtilRingBuffer
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - Rtc component implementation
 *
 *  \remarks (c) Copyright 2017 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#include "g4_UtilRingBuffer.h"
#include "g4_UtilAssert.h"
#include "string.h"

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

void g4_UtilRingBuffer_Init(g4_UtilRingBuffer_t * hnd, void * buffer, uint32_t size )
{
    hnd->size = (int32_t)size;
    hnd->pBuf = (uint8_t*) buffer;
    hnd->head = 0;
    hnd->tail = 0;

}



void g4_UtilRingBuffer_Reset(g4_UtilRingBuffer_t * hnd)
{
    hnd->head = 0;
    hnd->tail = 0;
}



#if 0    // Implemented in headerfile.
uint32_t g4_UtilRingBuffer_GetTotalSize(g4_UtilRingBuffer_t * hnd)
{
    
    return (uint32_t) hnd->size;
}
#endif


uint32_t g4_UtilRingBuffer_GetFreeSize(const g4_UtilRingBuffer_t * const hnd)
{
    int32_t size = hnd->tail - hnd->head;
    
    if ( size <= 0 )
    {
      size += hnd->size;
    }

    return (uint32_t)(size-1);
}



uint32_t g4_UtilRingBuffer_GetUsedSize(const g4_UtilRingBuffer_t * const hnd)
{
    int32_t size = hnd->head - hnd->tail;
    
    if ( size < 0 )
    {
      size += hnd->size;
    }

    return (uint32_t)size;
}



uint32_t g4_UtilRingBuffer_Write(g4_UtilRingBuffer_t * const hnd, const void * const pData, uint32_t size)
{
  if (hnd)
  {
    int32_t vTail = hnd->tail;
    int32_t copyCount;
    int32_t newHead = hnd->head + (signed) size;

    if (vTail <= hnd->head)
    {
      vTail += hnd->size;
    }

    if ( newHead >= vTail )
    {
      // Error
      return 0;
    }
    
    if (newHead >= hnd->size)
    {
      copyCount = hnd->size - hnd->head;
    }
    else
    {
      copyCount = (signed)size;
    }

    if ( copyCount )
    {
        memcpy(&hnd->pBuf[hnd->head], pData, (unsigned)copyCount);
    }
    
    if ( copyCount < (signed)size)
    {
      memcpy(hnd->pBuf, ((uint8_t*)pData) + copyCount, size - (unsigned)copyCount);
      hnd->head = (signed)size - copyCount;
    }
    else
    {
      hnd->head += (signed)size;
    }
    return size;
  }
  return 0;
}



uint32_t g4_UtilRingBuffer_Read(g4_UtilRingBuffer_t * hnd, void * pData, uint32_t size)
{
    int32_t vHead = hnd->head;
    int32_t copyCount;
    int32_t newTail = hnd->tail;

    newTail += (signed) size;
    
    if (vHead < hnd->tail)
    {
      vHead += hnd->size;
    }

    if ( (newTail > vHead) || (size == 0)  )
    {
      // Error
      return 0;
    }

    if (newTail >= hnd->size)
    {
      copyCount = hnd->size - hnd->tail;
    }
    else
    {
      copyCount = (signed)size;
    }
    
    if (pData && copyCount)
    {
      memcpy(pData, &hnd->pBuf[hnd->tail], (unsigned)copyCount);
    }
    
    if ( copyCount < (signed)size)
    {
      if (pData)
      {
        memcpy(((uint8_t*)pData) + copyCount, hnd->pBuf, size - (unsigned)copyCount);
      }
      hnd->tail = (signed)size - copyCount;
    }
    else
    {
      hnd->tail += (signed)size;
    }

    return size;
}



uint32_t g4_UtilRingBuffer_Peek(const g4_UtilRingBuffer_t * const hnd, void * const pData, uint32_t size)
{
    volatile int32_t vHead = hnd->head;
    int32_t copyCount;
    int32_t newTail = hnd->tail + (signed) size;

    if (vHead < hnd->tail)
    {
      vHead += hnd->size;
    }

    if ( newTail > vHead )
    {
      // Error
      return 0;
    }

    if (newTail > hnd->size)
    {
      newTail -= hnd->size;
      copyCount = hnd->size - hnd->tail;
    }
    else
    {
      copyCount = (signed)size;
    }
    
    memcpy(pData, &hnd->pBuf[hnd->tail], (unsigned)copyCount);
    
    if ( copyCount < (signed)size)
    {
      memcpy(((uint8_t*)pData) + copyCount, hnd->pBuf, size - (unsigned)copyCount);
    }

    return size;
}

/******************************************************************************/
/* Local functions                                                            */
/******************************************************************************/

/**
 * \}
 * End of file.
 */

