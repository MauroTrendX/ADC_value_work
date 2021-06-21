__uint32_t ADS018_Init(void)
{
    __uint32_t  err_code = 0;
    mma8x5x_data_Type mma8x5x_dt;

	ADS018_Config_Pins();   // Configure INT2, LDOEN, RTS, CTS pins

    if (ADS018_CTS_READ != 0){
        ADS018_Uart_Config();
        ADS018_uart_on = true;
        ADS018_Print_Str("\r\n");
        ADS018_Print_Str("ADS018 UART ON\r\n");
#ifdef ADS018_DBG_OPER
        ADS018_Print_POWER_REGISTER();
        ADS018_Print_WDT_REGISTER();
#endif
        ADS018_Select_TRM_Session();  // UART connected => enter password to calibration or setup
    }
    else{
        ADS018_Uart_Unplug();
        ADS018_trm_session = 0;
    }
//    ADS018_trm_session = 0;

    do{
        err_code = pstorage_init(); // Init pstorage module
        APP_ERROR_CHECK(err_code);
        ADS018_PrintInt("ADS018_Init ps_storage_int err_code: %d\r\n",(int)err_code);
        if (err_code == NRF_SUCCESS){
            err_code = ADS018_Init_Setup(); // Init setup buffer from NV memory data
            ADS018_PrintInt("ADS018_Init_Setup err_code: %d\r\n",(int)err_code);
            pADS018_NV_buf = ADS018_Get_Setup();//just get setup for NV memory
            if (err_code == NRF_SUCCESS){
                //calibration variables
 /*OLHAAAAAA*/               ADS018_Cal_Set((ADS018_cal_Type *)&(pADS018_NV_buf->Cal), (float *)&ADS018_Cal_A, (float *)&ADS018_Cal_B, (int16_t *)&ADS018_Cal_ADC_Zero, (int16_t *)&ADS018_Cal_ADC_Delta);

        	    //factors and coeffs
                ADS018_sample_frequency = mma8x5x_DR/ADS018_prescaler_reload;
        	    ADS018_Sec_Limit        = (__uint32_t)ceil(ADS018_sample_frequency);
                ADS018_rotation_factor = 10*60* (int)ADS018_sample_frequency; // factor for rotation in rpmx10
                ADS018_Rotation_Min_N = 60*ADS018_sample_frequency/ADS018_ROTATION_MAX; // !< min rotation number of samples
                ADS018_Rotation_Max_N = 60*ADS018_sample_frequency/ADS018_ROTATION_MIN; // !< max rotation number of samples

        	    ADS018_Update_Factors();

        	    filter_type = 0; //fs=25Hz
        	    if (ADS018_sample_frequency > 26.0) filter_type = 1; //fs=50Hz
        	    //init high pass parameter
        	    filter_coef_num = (int16_t)((ADS018_sample_frequency - FILTER_HPF_CF) * 100);
        	    filter_coef_den = (int16_t)(ADS018_sample_frequency*100);

        	    //cycle timming
        	    ADS018_SCycle_Tout_Limit = (__uint32_t)(ADS018_Rotation_Max_N / ADS018_Cycle_S_n);
        	    ADS018_f_limit = (__uint32_t)(pADS018_NV_buf->Mean_Time * ADS018_sample_frequency / 1000);

                //show timing
        	    ADS018_Update_Show_Limits();
            }
            else{
        	    ADS018_Error = 2; // init setup error
            }
        }
        else{
    	    ADS018_Error = 1; // pstorge init error
        }


        if (ADS018_Error == 0){
            // Init ADC
            ADS018_NRF_Adc_Config(0); // init NRF ADC
            ADS018_Bat_Update(0);     // init BLE Battery Level Services, without interrupt
            ADS018_Meas_Exec(0,0); //init evaluation mean load
            ADS018_LDOEN_1;
            if (ads1120_Init((ads1120_setup_Type *)&ads1120_setup)){
            	ADS018_Print_Str("ADS018 ADC ADS1120 INIT OK\r\n");
            }
            else{
                ADS018_LDOEN_0;
                ADS018_Error = 3; // error on ads1120_Init
            }
        }

        if (ADS018_Error == 0){
	        // Init Accelerometer
            mma8x5x_setup.pregs->aslp_count = pADS018_NV_buf->Aslp_Count;
            aslp_count = MMA8X5X_ASLP_COUNT_10S;
	        if(mma8x5x_init((mma8x5x_setup_Type*)&mma8x5x_setup, (bool*)&ADS018_PowerON, aslp_count)){
	    	    if (mma8x5x_read_data((mma8x5x_data_Type *)&mma8x5x_dt)){
	    	    	ADS018_Print_Str("ADS018 ACCEL MMA8653 INIT OK\r\n");
	    	    }
	    	    else{
	    		    ADS018_Error = 5; // error on mma8x5x read
	    	    }
	        }
	        else ADS018_Error = 4; // error on mma8x5x_init
        }

        ADS018_PowerON_Init();

	    switch (ADS018_Error){
	        case  0: ADS018_Print_Str("ADS018 INIT OK\r\n"); break;
	        case  1: ADS018_Print_Str("ADS018 PSTORAGE INIT ERROR\r\n"); break;
	        case  2: ADS018_Print_Str("ADS018 SETUP INIT ERROR\r\n"); break;
	        case  3: ADS018_Print_Str("ADS018 ADC   ADS1120 INIT ERROR\r\n"); break;
	        case  4: ADS018_Print_Str("ADS018 ACCEL MMA8653 INIT ERROR\r\n"); break;
	        case  5: ADS018_Print_Str("ADS018 ACCEL MMA8653 READ ERROR\r\n"); break;
	        default: break;
	    };
    }while(ADS018_Error != 0);

//    while(1){__WFE();};   /*0,13mA*/
//    NRF_POWER->SYSTEMOFF = 1; /*0,13mA*/

    switch(ADS018_trm_session){
        case  0: // BLE / ANT  Operation
        	     ADS018_Session_Set(0);
        	     if (ADS018_PowerON){
#ifdef USE_ID_SETUP
        	    	 ADS018_Session_Set(ADS018_StartPosition(0));
#endif
        	     }
                 break;
        case  1: // Setup and Calibration on terminal
	             ADS018_Setup();
		         ADS018_Calibration();;;//antes dessa chamei
		         ADS018_Session_Set(0);
	             break;
        case  2: // Calibration on terminal
		         ADS018_Calibration();
		         ADS018_Session_Set(0);
                 break;
        case  3: // RUN ID SETUP
#ifdef USE_ID_SETUP
        	     ADS018_Session_Set(1);
#else
        	     ADS018_Print_Str("PEDAL ID SETUP disabled!\r\n");
        	     ADS018_Session_Set(0);
#endif
        	     break;
        case  4: // MYWATT BLE SERVICES
        	     ADS018_Session_Set(2);
        	     break;
        case  5: // DFU on BOOTLOADER
        	     ADS018_Session_Set(3);
        	     break;
        default: ADS018_Session_Set(0);
                 break;
    }
    ADS018_PrintInt("ADS018_Init selected session: %d\r\n",(int)ADS018_session);


    return ADS018_session;
}