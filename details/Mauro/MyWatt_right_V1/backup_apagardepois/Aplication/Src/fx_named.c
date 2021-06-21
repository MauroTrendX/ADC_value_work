/**
 *  \file    fx_named.c
 *  \author  Philips Intellectual Property & Standards, The Netherlands.
 *  \brief   Fx library interface. Defines the interface to stream video
 *           images in order to retrieve pulserate information.
 *
 *  \remarks (c) Copyright 2017 Koninklijke Philips N.V. All rights reserved.
 *  This Software Library is provided to Assisfit Comercio e Assistencia Tecnica de Equipamentos de Ginastica EIRELI (ASSISFIT) for use under and subject to the terms of the Evaluation License Agreement dated 1 November 2018.
 *  VitalSigns Optical, Philips and its logo are trademarks of Koninklijke Philips N.V..
 *  All other names are the trademark or registered trademarks of their respective holders.
 */


#include "fx_named.h"


static const char metricSize0x00[]=
{  
/*00*/  0
/*01*/  ,0
/*02*/  ,0
/*03*/  ,sizeof(fx_MetricAge_t)
/*04*/  ,sizeof(fx_MetricProfile_t)
/*05*/  ,sizeof(fx_MetricHeight_t)
/*06*/  ,sizeof(fx_MetricWeight_t)
/*07*/  ,sizeof(fx_MetricRestheartRatePreference_t)
/*08*/  ,0
/*09*/  ,sizeof(fx_MetricSleepPreference_t)
};

static const char metricSize0x1E[]=
{  
/*1E:30*/   sizeof(fx_MetricTime_t)
/*1F:31*/  ,0
/*20:32*/  ,sizeof(fx_MetricAverageHeartRate_t)
/*21:33*/  ,sizeof(fx_MetricRestingHeartRate_t)
/*22:34*/  ,sizeof(fx_MetricSkinProximity_t)
/*23:35*/  ,sizeof(fx_MetricEnergyExpenditure_t)
/*24:36*/  ,sizeof(fx_MetricSpeed_t)
/*25:37*/  ,sizeof(fx_MetricCadence_t)
/*26:38*/  ,sizeof(fx_MetricActivityType_t)
/*27:39*/  ,sizeof(fx_MetricHeartBeats_t)
/*28:40*/  ,sizeof(fx_MetricVo2Max_t)
/*29:41*/  ,sizeof(fx_MetricCardioFitnessIndex_t)
/*2A:42*/  ,sizeof(fx_MetricRespirationRate_t)
/*2B:43*/  ,sizeof(fx_MetricAccRaw_t)
/*2C:44*/  ,sizeof(fx_MetricPpgRaw_t)
/*2D:45*/  ,sizeof(fx_MetricIntermittentHeartRate_t)
/*2E:46*/  ,sizeof(fx_MetricActivityCount_t)
/*2F:47*/  ,sizeof(fx_MetricWestPrivateData_t)
/*30:48*/  ,sizeof(fx_MetricSleep_t)
/*31:49*/  ,sizeof(fx_MetricMotionEvent_t)
};


FX_UINT16 FX_InitMetricHeader( fx_MetricHeader_t * hdr, FX_UINT08 metric_id)
{
  FX_UINT16 len = 0;

  hdr->id = metric_id;
  hdr->idx = 0;
  hdr->quality = 4;
  
  if (metric_id<(sizeof(metricSize0x00)/sizeof(metricSize0x00[0])))
  {
    len = metricSize0x00[metric_id];
  }
  else if (metric_id<30)
  {
  }
  else if (metric_id< (30+(sizeof(metricSize0x1E)/sizeof(metricSize0x1E[0]))))
  {
    len = metricSize0x1E[metric_id-0x1E];
  }
  
  if (len)
  {
    FX_UINT16 reclen = len-3;
  
    hdr->sizeL = (FX_UINT08)reclen;
    hdr->sizeH = reclen >> 8;
  }
  
  return len;
}


/**
 *  Raw PPG
 */
FX_UINT16 FX_InitMetricPpgRaw( fx_MetricPpgRaw_t * metric, FX_UINT08 bodyPosition )
{
  FX_UINT16 size = FX_InitMetricHeader(&metric->header, FXI_METRIC_ID_PPG);
  metric->data.bodyPosition = bodyPosition;
  metric->data.sampleFormat = FX_PPG_DEFAULT_SAMPLEFORMAT;
  metric->data.relLedPwr = 1;
  metric->data.relAdcGain = 1;
  return size;
}

void FX_UpdateMetricPpgWithSamples(fx_MetricPpgRaw_t * metric, const FX_SINT16 * ppgs, const FX_SINT16 * ambs)
{
  for (int i=0;i<FX_NUMBER_OF_PPG_SAMPLES_PER_METRIC;i++)
  {
    int ppg, amb;
    
    if (ppgs)
    {
      ppg = *ppgs++;
    }
    else
    {
      ppg=0;
    }
    metric->data.ppgAmb[i].ppgL = (FX_UINT08) ppg;
    metric->data.ppgAmb[i].ppgH = (FX_UINT08) ppg/256;
    
    if (ambs)
    {
      amb = *ambs++;
    }
    else
    {
      amb=0;
    }
    metric->data.ppgAmb[i].ambL = (FX_UINT08) amb;
    metric->data.ppgAmb[i].ambH = (FX_UINT08) amb/256;
  }
}


/**
 *  Raw ACC
 */
FX_UINT16 FX_InitMetricAccRaw( fx_MetricAccRaw_t * metric, FX_UINT08 bodyPosition )
{
  FX_UINT16 size = FX_InitMetricHeader(&metric->header, FXI_METRIC_ID_ACCELERATION);
  metric->data.bodyPosition = bodyPosition;
  metric->data.sampleFormat = FX_ACC_DEFAULT_SAMPLEFORMAT;
  return size;
}

void FX_UpdateMetricAccWithSamples(fx_MetricAccRaw_t * metric, const FX_SINT16 * accs)
{
  FX_UINT08 * pAcc =  metric->data.acc;
  int count = FX_NUMBER_OF_ACC_SAMPLES_PER_METRIC*3;
  
  while (count>0)
  {
    int acc = *accs;
    pAcc[0] = (FX_UINT08) acc;
    pAcc[1] = (FX_UINT08) acc/256;
    accs++;
    pAcc+=2;
    count--;
  }
}



