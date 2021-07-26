#include "stdint.h"
#include "nrf_drv_spi.h"
#include "nrf_log.h"
#include "nrf_gpio.h"


#define ADS1120_WAIT_TRANSFER    while(ads1120PacketTransferComplete==false){}
#define ADS1220_CMD_RREG        0x20
#define ADS1120_SPI_TX_RX_BUF_LENGTH 16
#define ADS1120_RREG               0x20
#define ADS1120_WREG               0x40 
#define ADS1120_RDATA              0x10
#define ADS1120_START_SYNC         0x08
#define ADS1120_SPI_INSTANCE  0 /**< SPI instance index. */
	
static volatile uint8_t ADS1120_SPI_rx_data[ADS1120_SPI_TX_RX_BUF_LENGTH] = {0}; /**< A buffer for incoming data. */
static volatile uint8_t ADS1120_SPI_tx_data[ADS1120_SPI_TX_RX_BUF_LENGTH] = {0}; /**< A buffer with data to transfer. */  

void ads1120_Send_Byte(uint8_t bt);
void ads1120_send_receive(uint8_t len);
void ads1120_spi_init(void);
int16_t ads1120_RData(void);
uint8_t ads1120_Receive_Byte(void);

const nrf_drv_spi_t mADS1120Instance = NRF_DRV_SPI_INSTANCE(ADS1120_SPI_INSTANCE);  /**< SPI instance. */
static volatile bool ads1120PacketTransferComplete = false;

void ads1120_spi_event_handler(nrf_drv_spi_evt_t const * p_event,
                       void *                    p_context)
{
    ads1120PacketTransferComplete = true;
}

void ads1120_spi_init(void){
	  ret_code_t err_code;
    nrf_drv_spi_config_t ads1120_spi_config=NRF_DRV_SPI_DEFAULT_CONFIG;
    ads1120_spi_config.ss_pin   = 29;
    ads1120_spi_config.miso_pin = 30;
    ads1120_spi_config.mosi_pin = 21;
    ads1120_spi_config.sck_pin  = 18;
	  ads1120_spi_config.irq_priority=6;
	  ads1120_spi_config.frequency=SPI_FREQUENCY_FREQUENCY_K125;
    err_code=nrf_drv_spi_init(&mADS1120Instance, &ads1120_spi_config, ads1120_spi_event_handler, NULL);
	  switch(err_code)
		{
			case NRF_SUCCESS:
				NRF_LOG_INFO("ADS1120 SPI instance initialized");
				break;
			case NRF_ERROR_INVALID_STATE:
				NRF_LOG_INFO("Invalid State error, check to confirm there is no other instance beig used, if there is, please uninitiale it with nrf_drv_spi_uninit(&spi_instance)");
				break;
			default:
				NRF_LOG_INFO("something went wrong while initializing ADS1120 SPI, couldn't find an appropriate error code ");
				break;
		}

}


int16_t ads1120_RData(void)
{

    ADS1120_SPI_tx_data[0] = ADS1120_RDATA;
    ADS1120_SPI_tx_data[1] = 0;
    ADS1120_SPI_tx_data[2] = 0;
    ADS1120_SPI_tx_data[3] = 0;//ADS1120_POWERDOWN;
    ads1120PacketTransferComplete = false;
    ads1120_send_receive(4);
    ADS1120_WAIT_TRANSFER;
    return((int16_t)((ADS1120_SPI_rx_data[1]<<8)+ADS1120_SPI_rx_data[2]));

}



void ads1120_Read(uint8_t *pdestination, uint8_t n_bytes)
{
    int i;
    uint8_t *p = pdestination;
    for (i=0; i<n_bytes; i++) *p++ = ads1120_Receive_Byte();
}


void ads1120_Write(uint8_t *porigem, uint8_t n_bytes)
{
    int i;
    uint8_t *p = porigem;
    for (i=0; i<n_bytes; i++) ads1120_Send_Byte(*p++);
}



bool ads1120_Write_Reg(uint8_t *pregs, uint8_t addr, uint8_t nregs)
{
    if(((addr>=0) && (addr<=3)) && ((nregs>=0) && (nregs<=3))){
        uint8_t *p  = pregs;
        uint8_t cmd = ADS1120_WREG | ((addr & 0x03) << 2) | (nregs & 0x03);
    
      nrf_gpio_pin_set(12);//ADS1120_NCS_0 assert CS pin 
      ads1120_Send_Byte(cmd);
        ads1120_Write(p, nregs+1);
        nrf_gpio_pin_clear(12);//ADS1120_NCS_1; deassert CS pin 
        return true;
    }
    else{
     NRF_LOG_INFO("ADS1120 REG ADDRESS OUT OF RANGE");
        return false; 
    }
}

/**
 * @brief Function for configuring: reading ads1120 generic register, 
 * uint8_t *pregs-- pointer to the register you wanna read
 * uint8_t addr --address of the register you wanna read
 * uint8_t nregs --number of subsequent register to be read
 */
bool ads1120_Read_Reg(uint8_t *pregs, uint8_t addr, uint8_t nregs)
{
    if(((addr>=0) && (addr<=3)) && ((nregs>=0) && (nregs<=3))){
        uint8_t *p  = pregs;
      uint8_t cmd = ADS1120_RREG | ((addr & 0x03) << 2) | (nregs & 0x03);//the RREG cmd | the address | 2bytes (will always read two bytes)
    
        nrf_gpio_pin_set(12);//ADS1120_NCS_0 assert CS pin 
        ads1120_Send_Byte(cmd);
          ads1120_Read(p, nregs+1);
      nrf_gpio_pin_clear(12);//ADS1120_NCS_1; deassert CS pin 
        return true;
    }
    else{
          NRF_LOG_INFO("ADS1120 REG ADDRESS OUT OF RANGE");
        return false; 
    }
}


uint8_t ads1120_Receive_Byte(void)
{
    ADS1120_SPI_tx_data[0] = 0;
    ads1120PacketTransferComplete = false;
    ads1120_send_receive(1);
    ADS1120_WAIT_TRANSFER;
    return(ADS1120_SPI_rx_data[0]);
}

void ads1120_send_receive(uint8_t len)
{
        
    uint32_t err_code = nrf_drv_spi_transfer(&mADS1120Instance,
                                                             (uint8_t *)&ADS1120_SPI_tx_data[0],
                                                             len, 
                                                               (uint8_t *)&ADS1120_SPI_rx_data[0],
                                                             len);
    APP_ERROR_CHECK(err_code);
    if (err_code==NRF_SUCCESS){
        NRF_LOG_INFO("recebido %d por send_receive",ADS1120_SPI_rx_data[0]);
    }
    else {NRF_LOG_INFO("deu problema");}
}

void ads1120_Send_Byte(uint8_t bt)
{
    ret_code_t err_code;

    ADS1120_SPI_tx_data[0] = bt;
    ads1120PacketTransferComplete = false;
    ads1120_send_receive(1);
    ADS1120_WAIT_TRANSFER;
    
//#define TEST_STRING "Nordic"
//static uint8_t       m_tx_buf[] = TEST_STRING;           /**< TX buffer. */
//static uint8_t       m_rx_buf[sizeof(TEST_STRING) + 1];    /**< RX buffer. */
//static const uint8_t m_length = sizeof(m_tx_buf);        /**< Transfer length. */

//err_code=nrf_drv_spi_transfer(&mADS1120Instance,m_tx_buf,m_length,m_rx_buf,m_length)  ;
//APP_ERROR_CHECK(err_code);
//if (err_code==NRF_SUCCESS){
//      NRF_LOG_INFO("enviado");
//  }
//nrf_drv_spi_transfer(&instancia,buffer quero enviar,tamanho quero enviar,receber,tamanho receber)
}









