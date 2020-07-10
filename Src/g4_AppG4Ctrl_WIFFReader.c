/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_AppG4Ctrl_WIFFReader.c
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - PCS Interchangeable File Format Reader component definitions
 *
 *  \remarks (c) Copyright 2018 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#include "g4_AppG4Ctrl_WIFFReader.h"
//#include "g4_UtilRtc.h"
#include "g4_HilFatFsIntf.h"
#include "g4_HilMscHidIntf.h"
#include "g4_HilSystemIntf.h"

#include "ff.h"
#include "fx_named.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "g4_UtilAssert.h"
#include "g4_UtilStdio.h"

/******************************************************************************/
/* Local defines and types                                                    */
/******************************************************************************/

/*
 * \brief   Maximum length of a filename
 */
#define MAX_FILENAME_LENGTH         48


typedef struct
{
  uint8_t OHR[3];
  uint8_t sizeL;
  uint8_t sizeH;
  uint8_t id;
} wiff_chunkHdr_t;

/******************************************************************************/
/* Local data                                                                 */
/******************************************************************************/

/*
 * \brief   The file we are writing to
 */
static FIL      sFile;


/*
 * \brief   The name of the current file
 */
static char     sFileName[MAX_FILENAME_LENGTH];


/*
 * \brief   Pointer to outgoing ringbuffer
 */
static g4_UtilRingBuffer_t * rbOut;

/*
 * \brief   Offset within an ohr record.
 */
static uint32_t ohrOfs = 0;

/*
 * \brief   Size of an ohr record.
 */
static uint32_t ohrLen = 0;

/*
 * \brief   Busy status:  true if file is being read. Switches to false when EOF.
 */
static bool busy = false;

/*
 * \brief   processing error
 */
static int32_t sError  = 0;

/******************************************************************************/
/* Local function prototypes                                                  */
/******************************************************************************/

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/

void    g4_AppG4Ctrl_WIFFReader_Init(void)
{
}


/******************************************************************************/


void    g4_AppG4Ctrl_WIFFReader_Start(const char *fileName, g4_UtilRingBuffer_t * outRB)
{
    FRESULT res;
    
    sError = 0;

    rbOut = outRB;
    
    strncpy(sFileName, fileName, sizeof(sFileName)-1);
    

    /* start the FatFS */
    if (g4_HilFatFsIntf_Start())
    {
        /* open the file */
        res = f_open(&sFile, sFileName, FA_READ | FA_OPEN_EXISTING);
        
        if (res != FR_OK)
        {
            printf("Could not open file %s.\r\n", sFileName);
            g4_HilFatFsIntf_Stop();
        }
    }
    else
    {
        res = FR_DISK_ERR;
        printf("Could start FAT FS.\r\n");
    }

    if (res == FR_OK)
    {
        printf("Started reading WIFF file `%s`\r\n", sFileName);
        
        busy = true;
    }

    sError = (int32_t)res;
}


/******************************************************************************/


void g4_AppG4Ctrl_WIFFReader_Stop(void)
{
    printf("Stopping WIFF file `%s`\r\n", sFileName);
    (void)f_close(&sFile);
    g4_HilFatFsIntf_Stop();
    busy = false;
}


/******************************************************************************/

void g4_AppG4Ctrl_WIFFReader_Process(void)
{
    FRESULT     res = FR_OK;
    UINT br=1;
    
    
    while (busy)
    {
    // Check if we need to read another chunk
      if (ohrOfs >= ohrLen)
      {
        bool ohrOk = false;
        wiff_chunkHdr_t hdr;

        res = f_read(&sFile, &hdr, sizeof(hdr), &br);
        
        if ((res != FR_OK) || (!br))
        {
          busy = false;
        }
        
        while ((res==FR_OK) && !(ohrOk) && (br))
        {
          ohrOk = (hdr.OHR[0]=='O')&&(hdr.OHR[1]=='H')&&(hdr.OHR[2]=='R');
          if (!ohrOk)
          {
            memcpy(&hdr, ((char*)&hdr)+1, sizeof(hdr)-1);
            res = f_read(&sFile, ((char*)&hdr)+(sizeof(hdr)-1),1, &br);
          }
          else
          {
            ohrLen = (hdr.sizeH<<8) + hdr.sizeL;
            if ( hdr.id == 0x0f )
            {
              ohrOfs = 1;
            }
            else
            {
              UINT pos = f_tell(&sFile);
              res = f_lseek( &sFile, pos + ohrLen - 1 );
              ohrOfs = ohrLen;
            }
          }
        }
      }
      else
      {
        static uint8_t metricBuf[0x40];
        fx_MetricHeader_t * pMetric = (fx_MetricHeader_t*)metricBuf;
        uint32_t size=0;
        uint32_t rsize = sizeof(fx_MetricHeader_t);
        
        if (pMetric->id == 0)
        {
            res = f_read(&sFile, metricBuf, rsize, &br);
        }
        size = FX_GetMetricSize(pMetric);

        if (size < (uint32_t) g4_UtilRingBuffer_GetSize(rbOut))
        { 

          if (size >= g4_UtilRingBuffer_GetFreeSize(rbOut))
          { // no space in return queue, bail out.
            return;
          }
          
          ohrOfs += size;
          
          while (size>0)
          {
            (void) g4_UtilRingBuffer_Write(rbOut, metricBuf, rsize);
            size -= rsize;
            rsize = (size > sizeof(metricBuf)) ? sizeof(metricBuf) : size;
            if (rsize>0)
            {
              res = f_read(&sFile, metricBuf, rsize, &br);
            }
          }
        }
        else
        {  // metric too big for ringbuf. Just skip
          UINT pos = f_tell(&sFile);
          ohrOfs += size + 3;
          res = f_lseek( &sFile, pos + size - 3 );
        }
        
        pMetric->id=0;
        
      }

    }
    if ((res == FR_OK) && (!br))
    {   
      sError = -1;
    }
    else
    {
      sError = (int32_t) res;
    }
}


/******************************************************************************/


bool g4_AppG4Ctrl_WIFFReader_Busy(void)
{
    return ( busy || g4_UtilRingBuffer_GetUsedSize(rbOut) );
}


/******************************************************************************/
int32_t g4_AppG4Ctrl_WIFFReader_Error(void)
{
  return sError;
}


/******************************************************************************/


/******************************************************************************/
/* Local functions                                                            */
/******************************************************************************/



/******************************************************************************/


/**
 * \}
 * End of file.
 */

