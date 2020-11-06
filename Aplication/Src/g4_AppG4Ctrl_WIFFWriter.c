/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_AppG4Ctrl_WIFFWriter.c
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - PCS Interchangeable File Format Writer component definitions
 *
 *  \remarks (c) Copyright 2018 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#include "g4_AppG4Ctrl_WIFFWriter.h"
//#include "g4_UtilRtc.h"
#include "g4_UtilRingBuffer.h"
#include "g4_HilFatFsIntf.h"
#include "g4_HilMscHidIntf.h"
#include "g4_HilSystemIntf.h"

#include "ff.h"

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "g4_UtilAssert.h"
#include "g4_UtilStdio.h"

/******************************************************************************/
/* Local defines and types                                                    */
/******************************************************************************/

/*
 * \brief   Keep data for one seconds including new line delimiter + one extra byte(requared by ring buffer)
 */
#define WIFF_DATA_RING_BUFFER_SIZE         512

/*
 * \brief   Static buffer data size, for best perfomance should be multiple of 512
 */
#define WIFF_DATA_STATIC_BUFFER_SIZE      4096

/*
 * \brief   Maximum length of a filename
 */
#define WIFF_MAX_FILE_NAME_LENGTH           48

/*
 * \brief   Size of wiff file data header
 */
#define CHUNK_HEADER_SIZE                    6

/*
 * \brief   WIFF header information
 */
typedef struct _wiff_required_chunks_t
{
    /* NB: this sub-struct is made so, that the 'Header'
     * chunk is already formatted */
    struct
    {
        uint8_t     totalLength[4];
        uint8_t     BCDStartTime[8];    /* obsolete! */
        uint8_t     EpochStartTime[4];
    } header;

    /* NB: this sub-struct is made so, that the 'Sensor Source Information'
     * chunk is already formatted */
    struct
    {
        uint8_t     serial[8];
        uint8_t     firmwareVersion[16];
        uint8_t     configuration[SENSOR_CONFIG_LENGTH];
    } sensor;

    /* NB: this sub-struct is made so, that the 'Source Host Version Information'
     * chunk is already formatted */
    struct
    {
        uint8_t     serial[3];
        uint8_t     firmwareVersion[16];
        uint8_t     configuration[HOST_CONFIG_LENGTH];
    } host;
} wiff_required_chunks_t;

/*
 * \brief   static buffer structure
 */
typedef struct
{
    uint8_t    data[WIFF_DATA_STATIC_BUFFER_SIZE];
    uint16_t   filled;
    uint16_t   written;
} wiff_static_buffer_t;

/******************************************************************************/
/* Local data                                                                 */
/******************************************************************************/

/*
 * \brief   The number of bytes written for the current file
 */
static uint32_t sBytesWritten;

/**
 *  \brief PPG raw data ring buffer handler
 */
static g4_UtilRingBuffer_t wiffRB;

/**
 *  \brief PPG raw data buffer holder
 */
static uint8_t wiffRingBufferData[WIFF_DATA_RING_BUFFER_SIZE];

/**
 *  \brief Chunk buffer used for SD card writter
 */
static wiff_static_buffer_t g_wiff_static_buffer;

/*
 * \brief   The file we are writing to
 */
static FIL      sFile;

/*
 * \brief   Set to true when an error occurs
 */
static bool     sError = false;

/*
 * \brief   Count system starting calls
 */
static uint8_t  sStartCounter = 0;

/*
 * \brief   Raw ppg file name
 */
static char sWiffFileName[WIFF_MAX_FILE_NAME_LENGTH];

/******************************************************************************/
/* Local function prototypes                                                  */
/******************************************************************************/

static void wiffAddChunk(uint8_t id, uint16_t length, const uint8_t *data);
static bool isBusy(void);


/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/

void    g4_AppG4Ctrl_WIFFWriter_Init(void)
{
}


/******************************************************************************/
void g4_AppG4Ctrl_WIFFWriter_Start(const wiff_configuration_t *configuration, const char *fileNamePrefix)
{
    uint32_t i;
    wiff_required_chunks_t  reqChunks = { 0 };
    uint32_t utm;
    g4_UtilRtc_Date_t date;
    g4_UtilRtc_Time_t time;

    g4_UtilRtc_GetDateTime(&date, &time);
    
    sprintf(sWiffFileName, "0:%s-%02X-%4d%02d%02d-%02d%02d%02d.wiff",
            fileNamePrefix,
            configuration->hostSerial[0] + (configuration->hostSerial[1]<<8),
            date.year,
            date.month,
            date.date,
            time.hours,
            time.minutes,
            time.seconds
                );
    /* Set error to false */
    sError = false;
    sStartCounter = 0;
    
    /* Initialize wiff writer strucure */
    sBytesWritten = 0;
    g_wiff_static_buffer.filled = 0;
    g_wiff_static_buffer.written = WIFF_DATA_STATIC_BUFFER_SIZE;
    
    g4_UtilRingBuffer_Init(&wiffRB, wiffRingBufferData, sizeof(wiffRingBufferData) );
    
    /* Chunk 0x0A, 'Header', length of payload is 16 bytes      */
    /* Total length is set upon closure - skip                  */
    /* BCDStartTime is obsolete - skip                          */
    /* EpochStartTime                                           */
    utm = g4_UtilRtc_GetCurrentUnixTime();
    for (i=0; i<sizeof(reqChunks.header.totalLength); i++)  { reqChunks.header.totalLength[i]   = 0; }
    for (i=0; i<sizeof(reqChunks.header.BCDStartTime); i++) { reqChunks.header.BCDStartTime[i]  = 0; }
    reqChunks.header.EpochStartTime[0] = (uint8_t) (utm& 0xff);
    reqChunks.header.EpochStartTime[1] = (uint8_t) ((utm>>8) & 0xff);
    reqChunks.header.EpochStartTime[2] = (uint8_t) ((utm>>16) & 0xff);
    reqChunks.header.EpochStartTime[3] = (uint8_t) ((utm>>24) & 0xff);
    wiffAddChunk(0x0A, 16, (uint8_t *) &reqChunks.header);
    
    /* Chunk 0x0B, 'Sensor Source Information', length of payload is 0x34 bytes */
    reqChunks.sensor.firmwareVersion[0] = configuration->sensorFWversion;
    reqChunks.sensor.firmwareVersion[1] = configuration->sensorFWmajor;
    reqChunks.sensor.firmwareVersion[2] = configuration->sensorFWminor;
    /* TODO: we are supposed to have build time information here, not current time! */
    sprintf((char *) &reqChunks.sensor.firmwareVersion[3], "%04d%02d%02d_%02d%02d", date.year, date.month, date.date, time.hours, time.minutes);
    memcpy(reqChunks.sensor.serial, configuration->sensorSerial, sizeof(configuration->sensorSerial));
    memcpy(reqChunks.sensor.configuration, configuration->sensorConfig, sizeof(configuration->sensorConfig));
    wiffAddChunk(0x0B, 0x34, (uint8_t *) &reqChunks.sensor);
    
    /* Chunk 0x0C, 'Source Host Version Information', length of payload is 0x28 bytes */
    reqChunks.host.firmwareVersion[0] = configuration->hostFWversion;
    reqChunks.host.firmwareVersion[1] = configuration->hostFWmajor;
    reqChunks.host.firmwareVersion[2] = configuration->hostFWminor;
    /* TODO: we are supposed to have build time information here, not current time! */
    sprintf((char *) &reqChunks.host.firmwareVersion[3], "%04d%02d%02d_%02d%02d", date.year, date.month, date.date, time.hours, time.minutes);
    memcpy(reqChunks.host.serial, configuration->hostSerial, sizeof(configuration->hostSerial));
    memcpy(reqChunks.host.configuration, configuration->hostConfig, sizeof(configuration->hostConfig));
    wiffAddChunk(0x0C, 0x28, (uint8_t *) &reqChunks.host);
    
    g4_UtilStdio_AddLog("WIFF file writing is started `%s`", sWiffFileName);
}


/******************************************************************************/

void g4_AppG4Ctrl_WIFFWriter_Stop(void)
{    
    FRESULT res = FR_OK;
    uint8_t lengthArr[4];
    UINT written=0;
    uint16_t length = 512;
    
    char buffer_char;
    
    /* This part will empty wiffRB */
    while ( !g4_AppG4Ctrl_WIFFWriter_Error() && g4_AppG4Ctrl_WIFFWriter_Busy() )
    {
        g4_AppG4Ctrl_WIFFWriter_Process();
        g4_HilSystemIntf_Process();
    }
    
    if (!g4_AppG4Ctrl_WIFFWriter_Error())
    {
        if (( 0 != g4_UtilRingBuffer_GetUsedSize(&wiffRB) ) || ( 0 != g_wiff_static_buffer.filled ))
        {
            while (( 0 != g4_UtilRingBuffer_GetUsedSize(&wiffRB) ) || ( 0 != g_wiff_static_buffer.filled ))
            {
                /* This part of code should never run because wiffRB is empty */
                while ( ( g_wiff_static_buffer.filled < WIFF_DATA_STATIC_BUFFER_SIZE ) &&  ( 0 != g4_UtilRingBuffer_GetUsedSize(&wiffRB) ))
                {
                    if ( 0 != g4_UtilRingBuffer_Read(&wiffRB, &buffer_char, 1))
                    {
                        g_wiff_static_buffer.data[g_wiff_static_buffer.filled] = buffer_char;
                        
                        g_wiff_static_buffer.filled++;
                    }
                }
                
                
                if ( ( 0 < g_wiff_static_buffer.filled ) && ( WIFF_DATA_STATIC_BUFFER_SIZE == g_wiff_static_buffer.written ))
                {
                    /* first chunk in this series */
                    if (g4_HilFatFsIntf_Start())
                    {
                        /* open the file */   
                        res = f_open(&sFile, sWiffFileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
                        sStartCounter++;
                    }
                    else
                    {
                        printf("Disk mount error.\r\n");
                        res = FR_DISK_ERR;
                        sError = true;
                        break;
                    }
                    
                    if (res == FR_OK)
                    {
                        /* seek to the nr of data records and fill in */
                        res = f_lseek(&sFile, sFile.fsize); /* append! */
                        
                        if (res != FR_OK)
                        {
                            printf("Could not seek to end of file for writing record.\r\n");
                            sError = true;
                            break;
                        }
                        
                        g_wiff_static_buffer.written = 0;
                    }
                    else
                    {
                        printf("Could not open log for writing record.\r\n");
                        sError = true;
                        break;
                    }
                }
                
                if (( FR_OK == res ) && ( g_wiff_static_buffer.written < g_wiff_static_buffer.filled ))
                {
                    if ((g_wiff_static_buffer.written + length) > g_wiff_static_buffer.filled)
                    {
                        length = g_wiff_static_buffer.filled - g_wiff_static_buffer.written;
                    }
                    
                    res = f_write(&sFile, &g_wiff_static_buffer.data[g_wiff_static_buffer.written], length, &written);
                    
                    if ((res != FR_OK) || (length!=written))
                    {
                        printf("Could not write(stop) chunk of data.\r\n");
                        sError = true;
                        break;
                    }
                    else
                    {
                        g_wiff_static_buffer.written += (uint16_t) written;
                        sBytesWritten += written;
                    }
                }
                
                if ( ( FR_OK == res ) && ( g_wiff_static_buffer.written == g_wiff_static_buffer.filled ) )
                {
                    
                    g_wiff_static_buffer.filled = 0;
                    g_wiff_static_buffer.written = WIFF_DATA_STATIC_BUFFER_SIZE;
                }
                
                g4_HilSystemIntf_Process();
            }
        }
        else
        {
            /* In case file is not opened */
            if (g4_HilFatFsIntf_Start())
            {
                /* open the file */   
                res = f_open(&sFile, sWiffFileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
                sStartCounter++;
            }
            else
            {
                printf("Disk mount error.\r\n");
                res = FR_DISK_ERR;
                sError = true;
            }
        }
        
        
        if (res == FR_OK)
        {
            res = f_lseek(&sFile, 6); /* 6 is the offset for the number of data records */
            if (res != FR_OK)
            {
                printf("Could not seek for writing length.\r\n");
                sError = true;
            }
        }
        
        if (res == FR_OK)
        {
            lengthArr[0] = (uint8_t) (sBytesWritten & 0xff);
            lengthArr[1] = (uint8_t) ((sBytesWritten>>8) & 0xff);
            lengthArr[2] = (uint8_t) ((sBytesWritten>>16) & 0xff);
            lengthArr[3] = (uint8_t) ((sBytesWritten>>24) & 0xff);
            
            res = f_write(&sFile, &lengthArr[0], 4, &written);
            
            if ((res != FR_OK) || (written != 4))
            {
                printf("Failed to write length.\r\n");
                sError = true;
            }
        }
        
    }
    
    res = f_close(&sFile);
    
    if ((res != FR_OK) || ( 1 != sStartCounter ))
    {
        printf("Error closing file or stopping.\r\n");
        sError = true;
    }
    
    while ( 0 < sStartCounter )
    {
        g4_HilFatFsIntf_Stop();
        sStartCounter--;
    }
    
    g4_UtilStdio_AddLog("WIFF file writing is stopped `%s`", sWiffFileName);
    
    /* ReInitialize structures in case of error */
    g_wiff_static_buffer.filled = 0;
    g_wiff_static_buffer.written = WIFF_DATA_STATIC_BUFFER_SIZE;
    g4_UtilRingBuffer_Reset( &wiffRB );
}


/******************************************************************************/


void g4_AppG4Ctrl_WIFFWriter_Process(void)
{
    if ( !g4_AppG4Ctrl_WIFFWriter_Error() && isBusy() )
    {
        FRESULT res = FR_OK;
        UINT written;
        uint16_t length = 512;    
        
        char buffer_char;
        
        while ( ( g_wiff_static_buffer.filled < WIFF_DATA_STATIC_BUFFER_SIZE ) &&  ( 0 != g4_UtilRingBuffer_GetUsedSize(&wiffRB) ))
        {
            if ( 0 != g4_UtilRingBuffer_Read(&wiffRB, &buffer_char, 1))
            {
                g_wiff_static_buffer.data[g_wiff_static_buffer.filled] = buffer_char;
                
                g_wiff_static_buffer.filled++;
            }
        }
        
        if ( WIFF_DATA_STATIC_BUFFER_SIZE == g_wiff_static_buffer.filled && ( WIFF_DATA_STATIC_BUFFER_SIZE == g_wiff_static_buffer.written ))
        {   
            /* first chunk in this series */
            if (g4_HilFatFsIntf_Start())
            {
                /* open the file */
                res = f_open(&sFile, sWiffFileName, FA_OPEN_ALWAYS | FA_READ | FA_WRITE);
                sStartCounter++;
            }
            else
            {
                res = FR_DISK_ERR;
                sError = true;
            }
            
            if ( FR_OK == res )
            {
                /* seek to the nr of data records and fill in */
                res = f_lseek(&sFile, sFile.fsize); /* append! */
                
                if ( FR_OK != res )
                {
                    printf("Could not seek to end of file for writing record.\r\n");
                    sError = true;
                }
                
                //printf("OPENED\r\n");
                
                g_wiff_static_buffer.written = 0;
            }
            else
            {
                printf("Could not open log for writing record.\r\n");
                sError = true;
            }
        }
        
        if (( FR_OK == res ) && ( g_wiff_static_buffer.written < g_wiff_static_buffer.filled ))
        {
            if ((g_wiff_static_buffer.written + length) > g_wiff_static_buffer.filled)
            {
                length = g_wiff_static_buffer.filled - g_wiff_static_buffer.written;
            }
            
            res = f_write(&sFile, &g_wiff_static_buffer.data[g_wiff_static_buffer.written], length, &written);
            
            if (( FR_OK != res ) || (written!=length))
            {
                printf("Could not write(process) chunk of data.\r\n");
                sError = true;
            }
            else
            {
                g_wiff_static_buffer.written += (uint16_t) written;
                sBytesWritten += written;
            }
        }
        
        if ( ( FR_OK == res ) && ( WIFF_DATA_STATIC_BUFFER_SIZE == g_wiff_static_buffer.filled ) && ( WIFF_DATA_STATIC_BUFFER_SIZE == g_wiff_static_buffer.written ) )
        {
            g_wiff_static_buffer.filled = 0;
            
            /* Done for this chunk. Close file and stop Fat FS */
            res = f_close(&sFile);
            g4_HilFatFsIntf_Stop();
            sStartCounter--;
            
            //printf("CLOSED %d \r\n", g_raw_ppg_static_buffer.written);
        }
    }
}


/******************************************************************************/


bool g4_AppG4Ctrl_WIFFWriter_Busy(void)
{
    return isBusy();
}


/******************************************************************************/


bool g4_AppG4Ctrl_WIFFWriter_Error(void)
{
    return sError;
}


/******************************************************************************/


void g4_AppG4Ctrl_WIFFWriter_ReportMetric(const uint8_t *metricData)
{
    /* 0x0f is the chunk ID for metric data */
    /* metricData[1] is the length of the metric -> payload <- */
    /* The +3 is because the metric header up to and including the
     * length (metric type + length) is three bytes long */
    wiffAddChunk(0x0f, (uint16_t) (metricData[1] + (((uint16_t)metricData[2])<<8) + 3), metricData);
}


/******************************************************************************/


bool g4_AppG4Ctrl_WIFFWriter_IsSpaceAvailable( uint32_t dataSize )
{
    bool rv = true;
    
    /* Will calculate only if there is no error */
    if ( !g4_AppG4Ctrl_WIFFWriter_Error() && ( g4_UtilRingBuffer_GetFreeSize(&wiffRB) < ( dataSize + CHUNK_HEADER_SIZE) ) )
    {
        rv = false;
    }
    
    return rv;
}


/******************************************************************************/
/* Local functions                                                            */
/******************************************************************************/

static void wiffAddChunk(uint8_t id, uint16_t length, const uint8_t *data)
{
    uint8_t chunk_header[CHUNK_HEADER_SIZE];
    
    /* see [PRE13-13-583 C07S035] version 2.9 for details on magic numbers */
    chunk_header[0] = 0x4f; // 'O'
    chunk_header[1] = 0x48; // 'H'
    chunk_header[2] = 0x52; // 'R'
    
    /* We need to add 1 to the chunk data, as the id is considered part of
     * the payload of the chunk */
    chunk_header[3] = (uint8_t) ((length+1) & 0xff);         // length LSB
    chunk_header[4] = (uint8_t) (((length+1)>>8) & 0xff);    // length MSB
    
    chunk_header[5] = id;
    
    /* Copy the header */    
    if ( !g4_UtilRingBuffer_Write( &wiffRB, &chunk_header[0], CHUNK_HEADER_SIZE ) )
    {
        printf("AddChunk error");
        sError = true;
    }
    
    /* And copy the data */    
    if ( !g4_UtilRingBuffer_Write( &wiffRB, data, length ) )
    {
        printf("AddChunk error");
        sError = true;
    }
    
    g4_AppG4Ctrl_WIFFWriter_Process();
}


/******************************************************************************/


static bool isBusy(void)
{
    bool rv = false;
    
    if ( (( g_wiff_static_buffer.written < g_wiff_static_buffer.filled ) && ( WIFF_DATA_STATIC_BUFFER_SIZE == g_wiff_static_buffer.filled)) || 
        (( WIFF_DATA_STATIC_BUFFER_SIZE == g_wiff_static_buffer.written) && ( 0 != g4_UtilRingBuffer_GetUsedSize(&wiffRB) ) ) )
    {
        rv = true;
    }
    
    return rv;
}

/**
 * \}
 * End of file.
 */

