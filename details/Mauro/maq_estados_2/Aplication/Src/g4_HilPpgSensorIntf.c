/******************************************************************************/

/**
 * \addtogroup G4_Controller
 * \{
 */

/**
 *  \file    g4_HilPPGSensorIntf.c
 *  \author  Philips Intellectual Property & Standards, The Netherlands
 *  \brief   G4 project - PPG Sensor component implementation
 *
 *  \remarks (c) Copyright 2016 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */

#include "g4_HilPpgSensorIntf.h"

//#include "g4_HalSpi.h"
//#include "g4_HalGpio.h"
//#include "g4_HalSys.h"

//#include "g4_UtilRtc.h"
#include "g4_types.h"

//#include "g4_UtilRawPpgWriter.h"

#include <stdio.h>
#include "max30110.h"
#include "MAX30110_port.h"
#include "main.h"



/******************************************************************************/
/* Local defines and types                                                    */
/******************************************************************************/

/**
 *  \brief Enable subdac filter
 */
#define ENABLE_SUBDAC_FILTER                0

/**
 *  \brief IRQ pin configuration for PPG sensor
 */
#define AFE_IRQ_PIN             9
#define AFE_IRQ_SLOT            0

#ifdef G4_EVA_WATCH_TARGET

#define AFE_CURRENT_IRQ_PORT    GPIO_PORT_C
#define AFE_CURRENT_IRQ         2
#define AFE_CURRENT_CTL_PORT    GPIO_PORT_C
#define AFE_CURRENT_CTL         3    

#elif defined(OVERCURRENT_PROTECTION_DBG)

#define AFE_CURRENT_IRQ_PORT    GPIO_PORT_A
#define AFE_CURRENT_IRQ         0
#define AFE_CURRENT_CTL_PORT    GPIO_PORT_B
#define AFE_CURRENT_CTL         9

#else

#define AFE_CURRENT_IRQ         9

#endif


/**
 *  \brief PPG maximum value: (2^19) - 1
 */
#define ADC_MAX                         (524287)
#define ADC_USED_FACTOR                 (2)                    /* used range = ADC_MAX >> ADC_USED_FACTOR */

#define PPG_HIGH   (90) /* 90% of used scale: */
#define PPG_LOW    (45) /* 45% of used scale: */

#define LED_MIN    (10) /* 10% */
#define LED_MAX    (60) /* 60% */

#if ENABLE_SUBDAC_FILTER
/**
 *  \brief Maximum value for subdac correction ( neg, pos)
 */
#define SUBDAC_DELTA_MAX                (65536)

#endif // #if ENABLE_SUBDAC_FILTER

/**
 *  \brief RAW PPG Buffer size: 6 bytes times 128 samples
 */     
#define PPG_RAW_BUFFER_SIZE               (768)

/**
 *  \brief Enable full scale led's power
 */
#define USE_FULL_RANGE                      1

/**
 *  \brief Fixed point conversion factor
 */
#define FREQ1HZ_SHIFT 8
#define FREQ1HZ (1<<FREQ1HZ_SHIFT)

/**
 *  \brief Fixed point conversion factor
 */
#define RTCSTS_TO_MEASUREMENTTIMESTAMP_SHIFT        (G4_UTILRTC_STSONESECOND_SHIFT-G4_MEASUREMENT_TIMESTAMP_ONESECOND_SHIFT)

/**
 *  \brief Structure in which the current settings of AFE are kept
 */
typedef struct _PPG_commonSettings_t
{
    uint8_t ledRange;
    
    /* pulse width */
    uint8_t pulseWidth;
} PPG_commonSettings_t;


/**
 *  \brief Structure for max30110 sample
 */
typedef struct _max30110_sample_t
{
    /* adc value */
    uint32_t  data;

    /* subdac value led */
    uint8_t   subdac;
    
} max30110_sample_t;


/**
 *  \brief Structure for max30110 data
 */
typedef struct _max30110_data_t
{
    /* max30110 fifo sample */
    max30110_sample_t ppg;
    
    max30110_sample_t ambient;

    uint8_t  adcGain;
    uint16_t ledPower;
    
#if ENABLE_SUBDAC_FILTER    
    /* Value of previous sample */
    int32_t ppg_d;

    /* Delta to be added to the samples */
    int32_t delta;

    /* Subdac value of previous sample */
    uint8_t subdac_d;

    /* Subdac first sample encounter */
    bool subdac_first;
    
    /* subdac filter count down counter to time changes */
    uint8_t subdac_cnt;
#endif // ENABLE_SUBDAC_FILTER    
} max30110_data_t;


/******************************************************************************/
/* Local data                                                                 */
/******************************************************************************/

/**
 *  \brief SPI handle
 */
//static g4_HalSpiHandle_t        sSpiHandle;

/**
 *  \brief Buffer in which the fifo data is retrieved
 *         maximum: 32 depth * 4 banks * 3 bytes/sample 
 *         384 bytes == 384/4=96 words
 */
static uint8_t                  sPpgBuffer[PPG_RAW_BUFFER_SIZE];

/**
 *  \brief PPG Ring Buffer Ptr
 */
static g4_UtilRingBuffer_t      *sPpgRingBuffer = 0;

/**
 *  \brief Keep the sensor settings
 */
static PPG_commonSettings_t     sCommonSettings;

/**
 *  \brief max30110 data for leds - both channels
 */
static max30110_data_t          sDataCh;

/**
 *  \brief The software gain factor
 */
static bool                     trigLoop;

/**
 *  \brief Time stamp of the latest sample
 */
static uint32_t                 sIrqTimeStamp=0;

/**
 *  \brief Fixed point representation of the frequency
 */
static uint32_t                 sFreq=25*FREQ1HZ;


#if ( defined(G4_EVA_WATCH_TARGET) || defined(OVERCURRENT_PROTECTION_DBG) )
static volatile bool                     sOverCurrentProtectionActivated = false;
#endif

/**
 *  \brief Controlloop parameters
 */
static g4_PPGControlLoopConfig_t    sControlLoopParams;

/**
 *  \brief PPG status
 */
static uint32_t sPpgStatus;

/******************************************************************************/
/* Exported data                                                              */
/******************************************************************************/

/**
 *  \brief Device identifier 
 */
uint8_t gPpgDevice_id[2];

/******************************************************************************/
/* Local function prototypes                                                  */
/******************************************************************************/

/** 
 *  \brief  Set MAX30110 led power.
 *  \param  percentage [IN]  : Normalized power.
 *  \return status
 */
static uint8_t SetLedPower(uint16_t value);

/** 
 *  \brief  Set MAX30110 led adc gain.
 *  \param  gain [IN]  : [0..3] , amplification = 2^gain ( or 1, 2, 4, 8 )
 *  \return status
 */
static uint8_t SetGain( uint8_t gain );

/** 
 *  \brief  Read a number of registers from the MAX30110 device via SPI.
 *  \param  Reg [IN]        : Start register address of MAX30110.
 *  \param  pValue [IN/OUT] : Pointer to byte buffer.
 *  \param  NumOfBytes [IN] : Number of bytes to read.
 *  \return void
 */
static void MAX30110_read( uint8_t reg, uint8_t* pValue, uint16_t numOfBytes );

/** 
 *  \brief  Write to one MAX30110 register.
 *  \param  Reg [IN]    : Register to be set in MAX30110.
 *  \param  Value [IN]  : New value of register.
 *  \return void
 */
static void MAX30110_writeReg( uint8_t reg, uint8_t value );

/** 
 *  \brief  
 *  \param  sample
 *  \param  buffer
 *  \return void
 */
static void GetSample(max30110_sample_t* sample, const uint8_t* const buffer);

/** 
 *  \brief  
 *  \param  ppg signal value
 *  \param  new led power output
 *  \param  new leds gain output
 *  \return void
 */
static void ControlLoop( uint32_t  ppg, uint16_t  *pwr, uint8_t *gain);
 
/** 
 *  \brief  
 *  \param  data
 *  \return void
 */
#if ENABLE_SUBDAC_FILTER
static void SubdacFilterInit(max30110_data_t* data);
#endif // ENABLE_SUBDAC_FILTER

/** 
 *  \brief  
 *  \param  data
 *  \return void
 */
#if ENABLE_SUBDAC_FILTER     
static void SubdacFilter(max30110_data_t* data);
#endif // ENABLE_SUBDAC_FILTER

/**
 *  \brief  PPG IRQ callback
 */
static void g4_HilPpgSensorIntf_IrqCallback(uint32_t pin);


#if ( defined(G4_EVA_WATCH_TARGET) || defined(OVERCURRENT_PROTECTION_DBG) )
/**
 *  \brief  LED overcurrent IRQ callback
 */
static void g4_HilPpgSensorIntf_OverCurrentIrqCallback(uint32_t pin);
#endif


//static g4_Gpio_edge_cb_t sExtern_IrqCallback;

/******************************************************************************/
/* Exported functions                                                         */
/******************************************************************************/


/** 
 *  \brief  Initialize PPG sensor
 *  \return void
 */
void g4_HilPpgSensorIntf_Init(void)
{
 /*   sExtern_IrqCallback = NULL;
    
    sSpiHandle = g4_HalSpi_Init( SPI_BUS_3,
                                 SPI_CPOL_LOW,
                                 SPI_CPHA_1EDGE,
                                 SPI_MSB_FIRST,
                                 SPI_SPEED_DIV32,
                                 PPG_CS_PORT,
                                 PPG_CS_PIN
                               );*/
    
    /* DEBUG*/
    //g4_Gpio_ConfigurePin( GPIO_PORT_B,
    //                       8,
    //                       PIN_CONFIGURATION_OUTPUT,
    //                       EDGE_NONE
    //                      );
    
    /* DEBUG*/
    //g4_Gpio_SetPinValue(GPIO_PORT_B, 8, PIN_VALUE_HIGH);
    
    /* Read device identification data before reset */
//    MAX30110_read(AFE_REV_ID, &gPpgDevice_id[0], 1);
    MAX30110_read(AFE_PART_ID, &gPpgDevice_id[1], 1);
    
    /* System control
     * b7-b5: -                              (0b000)
     * b4: Extern clock                        (0b0)
     * b3: Low power mode                      (0b0)
     * b2: FIFO enable                         (0b0)
     * b1: Power safe mode                     (0b0)
     * b0: reset                               (0b1)
     */
    MAX30110_writeReg(AFE_SYSTEM_CTRL, 0x01);
    
    /* Initialize iner structures just in case ( will be overwritten ) */
    sControlLoopParams.enabled  = true;
    sControlLoopParams.adcGain  = 2;
    sControlLoopParams.ledPower = 40;   /* ~4% */
    sControlLoopParams.thLow    = PPG_LOW; 
    sControlLoopParams.thHigh   = PPG_HIGH;
    sControlLoopParams.ledMin   = ((LED_MIN * 1023) / 100);
    sControlLoopParams.ledMax   = ((LED_MAX * 1023) / 100);
    
    sCommonSettings.pulseWidth  = 1;
    sCommonSettings.ledRange    = 0;
}


/******************************************************************************/


void g4_HilPpgSensorIntf_Config( g4_PpgSensorConfig_t* pPpgSensorConfig, bool setDefaults )
{
    if ( setDefaults )
    {
        /* set default (general) front-end settings, like led range / pulse width / ... */
        pPpgSensorConfig->ledRange            = 0;            /* 0=2*50mA, 1=2*100mA, 2=2*150mA, 3=2*200mA */
        pPpgSensorConfig->pulseWidth          = 1;            /* 0=52uS; 1=104uS; 2=206uS; 3=417uS;  */
        
        /* Set a default control loop parameters  */
        pPpgSensorConfig->controlLoopParams.enabled  = true;
        pPpgSensorConfig->controlLoopParams.adcGain  = 2;      /* amplification level x [1..3] where amplification = 2^x  */
        pPpgSensorConfig->controlLoopParams.ledPower = 4;     /* 4% of chosen scale */
        pPpgSensorConfig->controlLoopParams.thLow    = PPG_LOW;     /* min(%) */
        pPpgSensorConfig->controlLoopParams.thHigh   = PPG_HIGH;    /* max(%) */
        pPpgSensorConfig->controlLoopParams.ledMin   = LED_MIN;     /* min_led (%) */
        pPpgSensorConfig->controlLoopParams.ledMax   = LED_MAX;     /* max_led (%) */
    }
    
    sCommonSettings.pulseWidth  = pPpgSensorConfig->pulseWidth;
    sCommonSettings.ledRange    = pPpgSensorConfig->ledRange;
    
    sControlLoopParams.enabled  = pPpgSensorConfig->controlLoopParams.enabled;
    sControlLoopParams.thLow    = pPpgSensorConfig->controlLoopParams.thLow;
    sControlLoopParams.thHigh   = pPpgSensorConfig->controlLoopParams.thHigh;
    sControlLoopParams.ledMin   = (uint16_t) (((uint32_t)pPpgSensorConfig->controlLoopParams.ledMin   * 1023) / 100);  /* convert from % to min_led (0..1023) */
    sControlLoopParams.ledMax   = (uint16_t) (((uint32_t)pPpgSensorConfig->controlLoopParams.ledMax   * 1023) / 100);  /* convert from % to max_led (0..1023) */
    sControlLoopParams.adcGain  = pPpgSensorConfig->controlLoopParams.adcGain;
    sControlLoopParams.ledPower = (uint16_t) (((uint32_t)pPpgSensorConfig->controlLoopParams.ledPower * 1023) / 100);  /* convert from % to ledPower [0..1023] */
}


/******************************************************************************/


void g4_HilPpgSensorIntf_Start( g4_UtilRingBuffer_t* pRingBufferPpg)//, g4_Gpio_edge_cb_t pfExtern_IrqCallback )
{
    uint8_t regVal;
    
    sPpgRingBuffer = pRingBufferPpg;
    
//    sExtern_IrqCallback = pfExtern_IrqCallback;
    
    /* PPG status - used for debugging */
    sPpgStatus = 0;
    

#if ENABLE_SUBDAC_FILTER    
    SubdacFilterInit(&sDataCh);    
#endif // ENABLE_SUBDAC_FILTER      
    
    /* System control
     * b7-b5: -                              (0b000)
     * b4: Extern clock                        (0b1)
     * b3: Low power mode                      (0b1)
     * b2: FIFO enable                         (0b0)
     * b1: Power safe mode                     (0b1)
     * b0: reset                               (0b0)
     */
    MAX30110_writeReg(AFE_SYSTEM_CTRL, 0x1A);
    

    /* sensitivity setting */
    (void)SetGain( sControlLoopParams.adcGain );
    
    /* PPG configuration 2
     * b4-b3: Timing delay, default            (0b11)
     * b2-b0: Sample average, default          (0b000)
     */
  
    /* Led1 / Led2 Powers */
    (void)SetLedPower( sControlLoopParams.ledPower );
    
    /* Led range
     * b3-b2: LED2_RGE, 50 mA                  (0b00)
     * b1-b0: LED1_RGE, 50 mA                  (0b00)
     */
    uint8_t ledRange = ((sCommonSettings.ledRange << 4 ) & 0xF0 ) | ( sCommonSettings.ledRange & 0x0F );
    MAX30110_writeReg(AFE_LED_RANGE, ledRange );
    
    /* Fifo configuration
     * b5:     Almost full type irq            (0b0)
     * b4:     Rolls on full, stops            (0b0)
     * b3-b0: Almost full flag, (32 - 7) 25 samples in  (0b0111)
     */
    MAX30110_writeReg(AFE_FIFO_CFG, 0x07);

    /* Clear the interrupt */
    MAX30110_read(AFE_INT_STATUS1, &regVal, 1 );

    /* Set up various interrupts
     * b7: Almost full enable                  (1)
     */
    MAX30110_writeReg(AFE_INT_EN1, 0x80);
    
    /* Disable all for now */
    MAX30110_writeReg(AFE_INT_EN2, 0x00);

    /* Fifo data control 1
     * b7-b4: FD2 AMBIENT                   (0xC)
     * b3-b0: FD1 PPG_LED1&LED2             (0xD)
     */
    MAX30110_writeReg( AFE_FIFO_DATACTRL1, 0xCD );

    /* Fifo data control 2
     * b7-b4: FD4 none                      (0x0)
     * b3-b0: FD3 none                      (0x0)
     */
    MAX30110_writeReg( AFE_FIFO_DATACTRL2, 0x00 );
    
    MAX30110_read(AFE_INT_STATUS1, &regVal, 1 );

    /* System control
     * b3: LP_MODE              (1)
     * b2: FIFO_EN              (1)
     * b1: SHDN                 (0)
     * b0: RESET                (0)
     */
    MAX30110_writeReg(AFE_SYSTEM_CTRL, 0x1C);
   
//    g4_UtilRawPpgWriter_Start();
    
    /* set IRQ callback */
//    g4_Gpio_InstallEdgeIrqCallback(g4_HilPpgSensorIntf_IrqCallback, AFE_IRQ_PIN, AFE_IRQ_SLOT, (bool) true);
    
/*    g4_Gpio_ConfigurePin( GPIO_PORT_A,
                           AFE_IRQ_PIN,
                           PIN_CONFIGURATION_INPUT_FLOATING,
                           EDGE_FALLING
                          );
*/
#if ( defined(G4_EVA_WATCH_TARGET) || defined(OVERCURRENT_PROTECTION_DBG) )
    
    sOverCurrentProtectionActivated = false;
    
    g4_Gpio_ConfigurePin( AFE_CURRENT_IRQ_PORT,
                           AFE_CURRENT_IRQ,
                           PIN_CONFIGURATION_INPUT_FLOATING,
                           EDGE_RISING
                          );
    
    g4_Gpio_InstallEdgeIrqCallback(g4_HilPpgSensorIntf_OverCurrentIrqCallback, AFE_CURRENT_IRQ, 0, (bool) true);
    
    g4_Gpio_ConfigurePin( AFE_CURRENT_CTL_PORT,
                           AFE_CURRENT_CTL,
                           PIN_CONFIGURATION_INPUT_FLOATING,
                           EDGE_NONE
                               );
    
#else 
    
 /*   g4_Gpio_ConfigurePin( GPIO_PORT_B,
                           AFE_CURRENT_IRQ,
                           PIN_CONFIGURATION_INPUT_FLOATING,
                           EDGE_NONE
                          );
 */   
#endif

//    g4_HalSys_EnableSlowClock(SC_PPG);
    
}


/******************************************************************************/


void g4_HilPpgSensorIntf_Process(void)
{
/*    if ( g4_UtilRawPpgWriter_Busy() )
    {
        g4_UtilRawPpgWriter_Process();
    }*/
}


/******************************************************************************/


void g4_HilPpgSensorIntf_Stop(void)
{
//    g4_HalSys_DisableSlowClock(SC_PPG);

    /* remove IRQ callback */
//    g4_Gpio_InstallEdgeIrqCallback(NULL, AFE_IRQ_PIN, AFE_IRQ_SLOT, (bool) true);

    /* System control
     * b7-b5: -                              (0b000)
     * b4: Extern clock                        (0b0)
     * b3: Low power mode                      (0b0)
     * b2: FIFO enable                         (0b0)
     * b1: Power safe mode                     (0b0)
     * b0: reset                               (0b1)
     */
    MAX30110_writeReg(AFE_SYSTEM_CTRL, 0x01);
    
//    g4_UtilRawPpgWriter_Stop();
}


/******************************************************************************/


/**
 *  \brief Check PPG Sensor busy status
 */
bool g4_HilPpgSensorIntf_Busy(void)
{
//    return g4_UtilRawPpgWriter_Busy();
}

/******************************************************************************/


uint8_t g4_HilPpgSensorIntf_Identify(uint8_t* pData, uint8_t size)
{
    uint8_t bytes_read = 0;
    
    if ( size >=26 )
    {
        pData[0] = AFE_PPG_CFG1;
        MAX30110_read(AFE_PPG_CFG1, &pData[1], 1);
        pData[2] = AFE_LED1_PA;
        MAX30110_read(AFE_LED1_PA, &pData[3], 1);
        pData[4] = AFE_LED2_PA;
        MAX30110_read(AFE_LED2_PA, &pData[5], 1);
        pData[6] = AFE_LED_RANGE;
        MAX30110_read(AFE_LED_RANGE, &pData[7], 1);
        pData[8] = AFE_FIFO_CFG;
        MAX30110_read(AFE_FIFO_CFG, &pData[9], 1);
        pData[10] = AFE_INT_EN1;
        MAX30110_read(AFE_INT_EN1, &pData[11], 1);
        pData[12] = AFE_INT_EN2;
        MAX30110_read(AFE_INT_EN2, &pData[13], 1);
        pData[14] = AFE_FIFO_DATACTRL1;
        MAX30110_read(AFE_FIFO_DATACTRL1, &pData[15], 1);
        pData[16] = AFE_FIFO_DATACTRL2;
        MAX30110_read(AFE_FIFO_DATACTRL2, &pData[17], 1);
        pData[18] = AFE_SYSTEM_CTRL;
        MAX30110_read(AFE_SYSTEM_CTRL, &pData[19], 1);
        pData[20] = AFE_INT_STATUS1;
        MAX30110_read(AFE_INT_STATUS1, &pData[21], 1);
        pData[22] = AFE_LED_PILOT_PA;
        MAX30110_read(AFE_LED_PILOT_PA, &pData[23], 1);
        pData[24] = AFE_PART_ID;
        MAX30110_read(AFE_PART_ID, &pData[25], 1);
        
        if ( pData[25] != 0x20 )
        {
            bytes_read = 0;
        }
        else
        {
            bytes_read = 26;
        }
    }
    else
    {
        bytes_read = 0;
    }
    
    return bytes_read;
}


/******************************************************************************/


uint32_t g4_HilPpgSensorIntf_GetStatus(void)
{
    return sPpgStatus;
}

/******************************************************************************/


void g4_HilPpgSensorIntf_GetInterruptConfig(uint32_t* pin, uint32_t * slot)
{
  *pin = AFE_IRQ_PIN;
  *slot = AFE_IRQ_SLOT;
}



/******************************************************************************/
/* Local functions                                                            */
/******************************************************************************/
#if ( defined(G4_EVA_WATCH_TARGET) || defined(OVERCURRENT_PROTECTION_DBG) )
static void g4_HilPpgSensorIntf_OverCurrentIrqCallback(uint32_t pin)
{
    sOverCurrentProtectionActivated = true;
    
    g4_Gpio_ConfigurePin( AFE_CURRENT_CTL_PORT,
                           AFE_CURRENT_CTL,
                           PIN_CONFIGURATION_OUTPUT,
                           EDGE_NONE
                          );
    
    g4_Gpio_SetPinValue(AFE_CURRENT_CTL_PORT, AFE_CURRENT_CTL, PIN_VALUE_HIGH);
}

bool g4_HilPpgSensorIntf_OverCurrentCheck(void)
{
    return sOverCurrentProtectionActivated;
}
#endif


/******************************************************************************/


static void g4_HilPpgSensorIntf_IrqCallback(uint32_t pin)
{   
    uint32_t ts;    
    uint8_t statusReg = 0;
    uint8_t rdptr;
    uint8_t wrptr;
    uint8_t ovccnt;
    uint8_t numsamples = 0;
    int startpos = 0; 
    static g4_ppgMeasurement_t ppgData;
    uint32_t sampleTimeStep;
    uint32_t sampleTime;
    
    (void) pin;
        
//    ts = g4_UtilRtc_GetSTS();
    
    /* Clear the interrupt 2 */
    MAX30110_read(AFE_INT_STATUS2, &statusReg, 1);
    if ( 0x80 & statusReg )
    {
        sPpgStatus |= PPG_STATUS_FLAG_VDD_ANA_ERROR;
    }
    
    /* Clear the interrupt 1 */
    MAX30110_read(AFE_INT_STATUS1, &statusReg, 1);
    if ( 0x00 == statusReg )
    {
        sPpgStatus |= PPG_STATUS_FLAG_FAKE_INTERRUPT;
    }
    else if ( 0x08 & statusReg )
    {
        sPpgStatus |= PPG_STATUS_FLAG_LED_COMPB_ERROR;
    }
    else if ( 0x10 & statusReg )
    {
        sPpgStatus |= PPG_STATUS_FLAG_PROX_INT_ERROR;
    }
    else if ( 0x20 & statusReg )
    {
        sPpgStatus |= PPG_STATUS_FLAG_ALC_OVF_ERROR;
    }
    
    if ( statusReg & 0x80 )
    {
        /* Get the write pointer */
        MAX30110_read(AFE_FIFO_WRPTR, &wrptr, 1);

        /* Get the read pointer */
        MAX30110_read(AFE_FIFO_RDPTR, &rdptr, 1);

        /* Get the overflow status */
        MAX30110_read(AFE_FIFO_OVC, &ovccnt, 1);

        if (ovccnt == 0)
        {
            if (wrptr >= rdptr)                             // tbc: check if >= is correct
            {
                numsamples = wrptr - rdptr;
            }
            else
            {
                numsamples  = wrptr + 32 - rdptr;
            }
        }
        else
        {
            numsamples = 32;
            sPpgStatus |= PPG_STATUS_FLAG_OVERFLOW_COUNTER;
        }
        
       /* Now the number of samples is known, get the samples from the fifo
        * Amount of samples = numsamples * nr of banks * 3 bytes 
        */
        if ( numsamples > 0 )
        {
            if ( numsamples != 25 )
            {
                sPpgStatus |= PPG_STATUS_FLAG_SAMPES_CNT_NOT_25;
            }
            
            sIrqTimeStamp = ts;
            sampleTimeStep = (16/sFreq)*FREQ1HZ;
            sampleTime = sIrqTimeStamp - (sampleTimeStep*(numsamples-1));

            /* 2 banks */
            MAX30110_read( AFE_FIFO_DATA, sPpgBuffer, numsamples*6 );
            
            for (int samplecnt = 0; samplecnt < numsamples; samplecnt++)
            {
                uint32_t tmpValue;
                
                /* Startposition of the data for this sample in the fifo */
                startpos = 6 * samplecnt;

                /* get sample in the original 19bits format stored in fifo Maxim */
                GetSample( &sDataCh.ppg,     &sPpgBuffer[startpos    ] );
                GetSample( &sDataCh.ambient, &sPpgBuffer[startpos + 3] );
                
                #if ( ENABLE_SUBDAC_FILTER )
                /* Perform subdac filtering of the frame */
                SubdacFilter(&sDataCh);
                #endif // ENABLE_SUBDAC_FILTER

//                g4_UtilRawPpgWriter_Add(&sPpgBuffer[startpos], sDataCh.ledPower, sDataCh.adcGain );
                
//                ppgData.timeStamp = sampleTime >> RTCSTS_TO_MEASUREMENTTIMESTAMP_SHIFT;
                sampleTime += sampleTimeStep;                                     

                tmpValue    = ( sDataCh.ppg.data >> (4-ADC_USED_FACTOR) );                         /* ppg is stored taken gain factor into account */
                ppgData.ppg = (tmpValue > 32767) ? 32767 : (uint16_t)tmpValue;                     /* clip against 15 bit max*/   

                tmpValue    = ( sDataCh.ambient.data >> ((4-ADC_USED_FACTOR) + sDataCh.adcGain ) );/* ambient is stored always with gain 0 */
                tmpValue   += 3000;                                                                /* FXI 4.x.x needs ambient offset of 3000 */
                ppgData.amb = (tmpValue > 32767) ? 32767 : (uint16_t)tmpValue;                     /* clip against 15 bit max*/   

                ppgData.ppgLow = (uint8_t)( sDataCh.ppg.data & (0x0F >> (ADC_USED_FACTOR)) );    /* save the low bits that were discarded, for debugging purpose */

                tmpValue    = (sDataCh.ledPower / 10);                                             /* convert from [0..1023] to [0..102] */
                ppgData.pwr = (tmpValue > 100) ? 100 : (uint8_t)tmpValue;                          /* clip against 100 */

                ppgData.gain = (uint8_t) sDataCh.adcGain;

                (void) g4_UtilRingBuffer_Write(sPpgRingBuffer, &ppgData, sizeof(ppgData));
            }

            /* only perform the controlloop on the last sample */
            if (sControlLoopParams.enabled == true)
            {
                ControlLoop((uint32_t)sDataCh.ppg.data, &sControlLoopParams.ledPower, &sControlLoopParams.adcGain);
            }

            /* power changed detection: outside controlLoopEnabled check, therefore can be set externally while Control loop is disabled */
            if( sControlLoopParams.ledPower != sDataCh.ledPower )
            {
                /* take over the new calculated led power figure */
                (void)SetLedPower(sControlLoopParams.ledPower);

                #if ENABLE_SUBDAC_FILTER
                /* re-init subdac filter: jump directly */
                sDataCh.subdac_first = true;
                #endif
            }
            
            /* gain changed detection: outside controlLoopEnabled check, therefore can be set externally while Control loop is disabled */
            if (sControlLoopParams.adcGain != sDataCh.adcGain)
            {
                /* take over the new calculated gain figure */
                (void)SetGain( sControlLoopParams.adcGain );

                #if ENABLE_SUBDAC_FILTER
                /* re-init subdac filter: jump directly */
                sDataCh.subdac_first = true;
                #endif
            }
        }
    }
    
    if ( ( statusReg & 0x80 ) || ( statusReg & 0x01 ) )
    {
      /*  if ( sExtern_IrqCallback )
        {
            sExtern_IrqCallback(pin);
        }*/
    }
}


/******************************************************************************/


static void ControlLoop( uint32_t ppg, uint16_t *pwr, uint8_t *gain)
{
    /* preset new values with old value */
    uint32_t pwrNew  = *pwr;
    uint8_t  gainNew = *gain;

    /* calculate thresholds for full scale (gain=0)  */
    uint32_t thrHigh = ((((uint32_t)sControlLoopParams.thHigh) * (ADC_MAX >> ADC_USED_FACTOR)) / 100);
    uint32_t thrLow  = ((((uint32_t)sControlLoopParams.thLow ) * (ADC_MAX >> ADC_USED_FACTOR)) / 100);
    uint32_t target  = (thrLow + thrHigh) / 2;   /* put target in the middle of low-high thresholds */

    /* check if thresholds are hit */
    if ( (ppg > thrHigh) || (ppg < thrLow) || (trigLoop /*== true*/) ) 
    {
        trigLoop = false;
        
        /* based on this value, we are going to determine a new pwr setting on current gain scale */
        
        /* ppg/pwrOld = target/pwrNew, so: */
        /* pwrNew = (pwrOld * target) / ppg */
        pwrNew *= target;            /* new_power already filled with old_power */
        pwrNew /= (ppg + 1);         /* + 1 to prevent division by 0 */
        
        /* if power is low then we have "large" PPG signal */
        /* therfore we can better go to lower sensitivity with 2x led power */
        if ( pwrNew < sControlLoopParams.ledMin ) 
        {
          if ( gainNew > 1 )
          {
            gainNew -= 1;   /* switch to lower sensitivity/gain */
            pwrNew <<= 1;   /* multiply by 2 */
          }
        }
        /* if power is high then we have "small" PPG signal */
        /* therfore we can better go to higher sensitivity with 2x less led power */
        else if ( pwrNew >= sControlLoopParams.ledMax )
        {
          if ( gainNew < 3 )
          {
            gainNew += 1;   /* switch to higher sensitivity/gain */
            pwrNew >>= 1;   /* divide by 2 */
          }
        }
        else
        {
          //gainNew = 2;
        }

        /* clip the new led power to [20..1023] % */
        if (pwrNew > 1023)
        {
            pwrNew = 1023;
        }
        else if (pwrNew < 20)
        {
            pwrNew = 20;
        }

        /* clip the new gain to [1..3] % */
        if (gainNew > 3)
        {
            gainNew = 3;
        }
        else if (gainNew < 1)
        {
            gainNew = 1;
        }
    }

    /* Return the new gain value */
    *gain = gainNew;

    /* Return the new power value */
    *pwr = (uint16_t) pwrNew;
}


/******************************************************************************/


#if ENABLE_SUBDAC_FILTER
static void SubdacFilterInit(max30110_data_t* data)
{   
    /* Value of previous sample */
    data->ppg_d = 0;
    /* Subdac value of previous sample */
    data->subdac_d = 0;
    /* Subdac first sample encounter */
    data->subdac_first = true;
    /* subdac filter count down counter to time changes */
    data->subdac_cnt = 0;
    /* Offset to be added to sample after a subdac change */
    data->delta = 0;
}


/******************************************************************************/


static void SubdacFilter(max30110_data_t* data)
{
    int32_t result;
  
    /* Skip first sample to init _ld params */
    if ( data->subdac_first == true )
    {
        data->subdac_first = false;
        data->delta        = 0;
        data->subdac_cnt   = 15;
    }
    else
    {
        /* Check if a change in subdac code occured */
        if ( data->ppg.subdac != data->subdac_d )
        { 
            /* if change too quick jump to end result */
            if ( data->subdac_cnt > 0 )
            {
                data->delta = 0;
            }
            else
            {
                data->delta += (int32_t)(data->ppg.data) - data->ppg_d;
            }
        
            /* set count down counter */
            data->subdac_cnt = 15;  

            /* Clip */
            if ( data->delta > SUBDAC_DELTA_MAX )
            {
                data->delta = SUBDAC_DELTA_MAX;
            }
            else if ( data->delta < -SUBDAC_DELTA_MAX )
            {
                data->delta = -SUBDAC_DELTA_MAX;
            }
        }
    }
    
    /* decrease count down counter */
    if ( data->subdac_cnt > 0 )
    {
        data->subdac_cnt -= 1;
    }

    /* Keep the old value of ppg1 and subdac1 */
    data->ppg_d    = (int32_t)(data->ppg.data);
    data->subdac_d = data->ppg.subdac;

    /* Correct ppg1 */
    result = (int32_t)(data->ppg.data) - data->delta;
    if ( result < 0 )
    {
        data->ppg.data = 0;
        sPpgStatus |= PPG_STATUS_FLAG_ERR_SUBDAC_SET_ZERO;
    }
    else
    {
        if ( result > ADC_MAX )
        {
            data->ppg.data = ADC_MAX;       
        }
        else
        {
            data->ppg.data = (uint32_t)result;
        }  
    }
     
    /* Calculate new delta1 */
    data->delta = (int16_t)(data->delta * 0.95);
    
}
#endif // ENABLE_SUBDAC_FILTER

/******************************************************************************/


uint8_t SetGain( uint8_t value )
{
  uint8_t regval;

  /* PPG configuration 1
   * b7-b6: ADC range                        (0b01)   0=6uA, 1=12uA, 2=24uA, 3=48uA
   * b5-b2: PPG sample rate 25 Hz, 1 pulse   (0b0001), set PPG_SAMPLING_FREQ according to this settings
   * b1-b0: PPG led pulse width, 100 uS      (0b01)      
   */

  /* In max30110, 0 = highest gain, 3 = lowest gain, therefore : ((~gain) & 0x03) or (gain ^ 0x03); */
  /* convert 0..3 to 3..0 */
  regval = (((value ^ 0x03) << 6) & 0xC0) | 0x04 | (sCommonSettings.pulseWidth & 0x03);

  MAX30110_writeReg( AFE_PPG_CFG1, regval );
  
  trigLoop        = true;                          // sensitivity is changed, trigger loop again for fine tuning 
  sDataCh.adcGain = value;                         // store the used setting

  // Error code, dummy for now
  return 0;
}


/******************************************************************************/


static uint8_t SetLedPower( uint16_t value )
{
    uint8_t ledpower;

#if USE_FULL_RANGE
    /* full range used */
    ledpower = (uint8_t)(value >> 2);   /* 10bit to 8bit means shift 2(10-8) ; do not use 1024 !! */
#else  
    /* only use 40% of full range, if range = 50mA --> limit to 20mA
     * range of 40% = 0.40 * 256 = 100 = percentage
     * BE AWARE: this reduces peak current( good for battery life ) but will get less optimal signal for dark skin!!!
     */
    ledpower = (value / 10);  /* small error: runs from [0..102] */
#endif

    /* Led 1 Power adjust, steps of 0.2 mA */
    MAX30110_writeReg( AFE_LED1_PA, ledpower );

    /* Led 2 power adjust, steps of 0.2 mA */
    MAX30110_writeReg( AFE_LED2_PA, ledpower);

    sDataCh.ledPower = value;                      // store the used setting

    /* Error code, dummy for now */
    return 0;
}


/******************************************************************************/


static void GetSample(max30110_sample_t* sample, const uint8_t* const buffer)
{
    sample->data = (((uint32_t)buffer[0]&0x07) << 16) + (((uint32_t)buffer[1]) << 8) + ((uint32_t)buffer[2]);

    sample->subdac = (buffer[0]&0x38) >> 3;
}


/******************************************************************************/


static void MAX30110_read(uint8_t reg, uint8_t* pValue, uint16_t numOfBytes)
{
     
		MAX30110_spiSelecionaChip();
		
		MAX30110_leituraSPI(reg, pValue, numOfBytes);
		
		MAX30110_spiDeselecionaChip();

		return;

//    g4_HalSpi_Start(sSpiHandle);
    /* Pull the NSS pin low to enable the SPI bus to signal a transfer */
//    g4_HalSpi_SelectSlaveEnable(sSpiHandle);
    /* First write the register address we want to read */
//    g4_HalSpi_Transmit(sSpiHandle, &reg, 1);
    /* Second write the read command */
//    g4_HalSpi_Transmit(sSpiHandle, &opRead, 1);
    /* Receive value from register */
//    g4_HalSpi_Receive(sSpiHandle, pValue, numOfBytes);
    /* Signal a transfer finish by disabling the SPI bus line */
//    g4_HalSpi_SelectSlaveDisable(sSpiHandle);
//    g4_HalSpi_Stop(sSpiHandle);
}


/******************************************************************************/


static void MAX30110_writeReg(uint8_t reg, uint8_t value)
{
        
		MAX30110_spiSelecionaChip();

		MAX30110_escritaSPI(reg, &value, 1);
		
		MAX30110_spiDeselecionaChip();

		return;
	
	
		
//    g4_HalSpi_Start(sSpiHandle);
    /* Pull the NSS pin low to enable the SPI bus to signal a transfer */
//    g4_HalSpi_SelectSlaveEnable(sSpiHandle);
    /* First transmit register address we want to write */
//    g4_HalSpi_Transmit(sSpiHandle, &reg, 1);
    /* Second transmit write command */
//    g4_HalSpi_Transmit(sSpiHandle, &opWrite, 1);
    /* Third transmit data */
//    g4_HalSpi_Transmit(sSpiHandle, &value, 1);
    /* Signal a transfer finish by disabling the SPI bus line */
//    g4_HalSpi_SelectSlaveDisable(sSpiHandle);
//    g4_HalSpi_Stop(sSpiHandle);
}

/**
 * \}
 * End of file.
 */

