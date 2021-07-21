/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup nrf_adc_example main.c
 * @{
 * @ingroup nrf_adc_example
 * @brief ADC Example Application main file.
 *
 * This file contains the source code for a sample application using ADC.
 *
 * @image html example_board_setup_a.jpg "Use board setup A for this example."
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "utills.h"

#define SAMPLES_IN_BUFFER 5
/**
 * @brief Function for main application entry.
 */
//VARIABLES============================================================================================================================
uint32_t err_code_2;
static nrf_saadc_value_t ADC_sample;
	int16_t media =0;
	int contador;
volatile float var1=0;
//FUNCTION PROTOTYPES=================================================================================================================
void read_gauge_init(void);//prototype
void read_gauge(void);//prototype
void adc_mean(void);//prototype
//=====================================================================================================================================

void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{

//	    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
//    {
//        ret_code_t err_code;

//        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
//        APP_ERROR_CHECK(err_code);

//        int i;
////        NRF_LOG_INFO("ADC event number: %d", (int)m_adc_evt_counter);

//        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
//        {
//            NRF_LOG_INFO("%d", p_event->data.done.p_buffer[i]);
////					  var1=6*(p_event->data.done.p_buffer[i])*(0.6/1024);
////          	SUPER_LOG(var1,"ADC_sample");
//        }
////        m_adc_evt_counter++;
//    }
//	
	
}

void read_gauge_init(void)
{
	
   ret_code_t err_code;
    nrf_drv_saadc_config_t saadc_config;

    //Configure SAADC peripheral
    saadc_config.low_power_mode = false;                                                  
    saadc_config.resolution = NRF_SAADC_RESOLUTION_14BIT;                 
    saadc_config.oversample = NRF_SAADC_OVERSAMPLE_2X;   
    saadc_config.interrupt_priority = APP_IRQ_PRIORITY_LOW;                  
    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);//saadc init
    APP_ERROR_CHECK(err_code);
    
    //Configure CHANNEL
    nrf_saadc_channel_config_t cfg;
    cfg.reference = NRF_SAADC_REFERENCE_INTERNAL;                              
    cfg.gain = NRF_SAADC_GAIN1_6;                                              
    cfg.acq_time = NRF_SAADC_ACQTIME_10US;                                      
    cfg.mode = NRF_SAADC_MODE_DIFFERENTIAL;  
    cfg.pin_p = NRF_SAADC_INPUT_AIN5; //Select the input pin as P3,AIN5 pin maps to physical pin P3
    cfg.pin_n = NRF_SAADC_INPUT_AIN6;                                      
    cfg.resistor_p = NRF_SAADC_RESISTOR_DISABLED;                              
    cfg.resistor_n = NRF_SAADC_RESISTOR_DISABLED;                              
    cfg.burst = NRF_SAADC_BURST_ENABLED; 
    nrf_drv_saadc_channel_init(2,&cfg);//channel init
}

void read_gauge(void)
{
	float result=0;
	ret_code_t err_code;
	
	err_code=nrf_drv_saadc_sample_convert(2, &ADC_sample);
	APP_ERROR_CHECK(err_code);
	
//	NRF_LOG_INFO("ADC_sample : %d",ADC_sample);
	#ifdef SHOWADC_SAMPLE
	result=(0.6*ADC_sample)/(8192/6);
	SUPER_LOG(result,"resultado volts");
//	NRF_LOG_INFO("sample : %d",ADC_sample);
	#endif
	
	#ifdef SHOWMEASURES
	media=media+ADC_sample;
	contador++;
	if (contador>=10){
		media=media/contador;
		NRF_LOG_INFO("MEDIA %d",media);
		media=0;
		contador=0;
		}
	#endif
  
}

void adc_mean(void)
{
	
	media=media+ADC_sample;
 
	contador++;
	if (contador>=15){
		media=media/contador;
		//NRF_LOG_INFO("ADC_sample %d",ADC_sample);
		//NRF_LOG_INFO("media %d",media);
		media=0;
		contador=0;
		}
	
	
}
