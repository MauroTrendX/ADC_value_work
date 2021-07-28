#include "nrf_fstorage.h"
#include "nrf_log.h"
#include "nrf_fstorage_sd.h"
#include "app_error.h"
#include "nrf_delay.h"


/* This is the address in flash where data will be read.*/
#define FLASH_ADDR_READ  0x56004//2000 0000
/* This is the address in flash were data will be written. */
#define FLASH_ADDR_WRITE  0x56004   
/* This is the address in flash were data will be erased. */
#define FLASH_ADDR_ERASE  0x56000   

static uint32_t number_novo;


void callback(nrf_fstorage_evt_t * p_evt);
void mwatt_flashinit(void);
uint32_t mwatt_flash_write (void);
uint32_t mwatt_flash_read (void);
static void print_bootloader_start_addr(void);

NRF_FSTORAGE_DEF(nrf_fstorage_t flash_instance) =
{
    .evt_handler    = callback,
    .start_addr     = 0x55000,
    .end_addr       = 0x77000,
};

void callback(nrf_fstorage_evt_t * p_evt)
{
	
    if (p_evt->result != NRF_SUCCESS)
    {
			  NRF_LOG_INFO("p_evt, result %d",p_evt->result);
        NRF_LOG_INFO("--> MAUROsEvent received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;

        default:
            break;
    }
		
}


void mwatt_flash_init(void)
{
	ret_code_t err_code;
	   err_code=nrf_fstorage_init(
        &flash_instance,       /* You fstorage instance, previously defined. */
        &nrf_fstorage_sd,   /* Name of the backend. */
        NULL                /* Optional parameter, backend-dependant. */
    );
	APP_ERROR_CHECK(err_code);
	NRF_LOG_INFO("FLASH INSTANCE..INITIALIZED");
	
}


uint32_t mwatt_flash_write (void)
{
/* This is the data to write in flash.
Because the fstorage interface is asynchrounous, the data must be kept in memory.
 */
static uint32_t number=0x256 ;
ret_code_t rc = nrf_fstorage_write
	(
    &flash_instance,   /* The instance to use. */
    FLASH_ADDR_WRITE,     /* The address in flash where to store the data. */
    &number,        /* A pointer to the data. */
    sizeof(number), /* Lenght of the data, in bytes. */
    NULL            /* Optional parameter, backend-dependent. */
    );
if (rc == NRF_SUCCESS)
{
    /* The operation was accepted.
       Upon completion, the NRF_FSTORAGE_WRITE_RESULT event
       is sent to the callback function registered by the instance. */
	NRF_LOG_INFO("sucesso");
	return rc;
	
}
else
{
    /* Handle error.*/
	NRF_LOG_INFO("falha");
	return rc;
}

}




uint32_t mwatt_flash_read (void)
{
/* This is the data to write in flash.
   Because the fstorage interface is asynchrounous, the data must be kept in memory.
 */
	  static uint32_t number;
    ret_code_t rc = nrf_fstorage_read(
    &flash_instance,   /* The instance to use. */
    FLASH_ADDR_READ,     /* The address in flash where to read data from. */
    &number,        /* A buffer to copy the data into. */
    sizeof(number)  /* Lenght of the data, in bytes. */
);
if (rc == NRF_SUCCESS)
{
  /* The operation was accepted. */
	NRF_LOG_INFO("Read \"%x\" from flash address \"%x\".", number, FLASH_ADDR_READ);
	NRF_LOG_INFO("sucesso_ler");
	return rc;
}
else
{
    /* Handle error.*/
	NRF_LOG_INFO("falha_ler");
	return rc;
}

}

void mwatt_flash_erase(void)
{
  uint32_t pages_to_erase = 1;
ret_code_t rc = nrf_fstorage_erase(
    &flash_instance,   /* The instance to use. */
    FLASH_ADDR_ERASE,     /* The address of the flash pages to erase. */
    pages_to_erase, /* The number of pages to erase. */
    NULL            /* Optional parameter, backend-dependent. */
);
if (rc == NRF_SUCCESS)
{
			NRF_LOG_INFO("data succEssfully erased");
}
else
{
    NRF_LOG_INFO("oooooops data couldn't be erased");
}

}


static void print_bootloader_start_addr(void)
{
    uint32_t bl_address;
    bl_address = *NRF_UICR->NRFFW;
    NRF_LOG_INFO("MAURO Bootloader start addr: 0x%08x", bl_address);
}

 void print_mwatt_flash_info(void)
{
    NRF_LOG_INFO("========| flash info |========");
    NRF_LOG_INFO("erase unit: \t%d bytes",      flash_instance.p_flash_info->erase_unit);
    NRF_LOG_INFO("program unit: \t%d bytes",    flash_instance.p_flash_info->program_unit);
    NRF_LOG_INFO("============MAURO==================");
	
}


uint32_t mwatt_write_anything (uint32_t value)
{
    ret_code_t rc;

//static uint32_t number=0x2AB ;
    
    mwatt_flash_erase();//erase values before doing anything
    nrf_delay_ms(2);//just some delay to allow complete erase.
	  number_novo=value;
    rc = nrf_fstorage_write
    (
    &flash_instance,   /* The instance to use. */
    FLASH_ADDR_WRITE,     /* The address in flash where to store the data. */
    &number_novo,        /* A pointer to the data. */
    sizeof(number_novo), /* Lenght of the data, in bytes. */
    NULL            /* Optional parameter, backend-dependent. */
    );
		
		
				if (rc == NRF_SUCCESS)
				{
								/* The operation was accepted.
								Upon completion, the NRF_FSTORAGE_WRITE_RESULT event
								is sent to the callback function registered by the instance. */
					      NRF_LOG_INFO("BOSCH");
								return rc;
				}
				else
				{
									/* Handle error.*/
									NRF_LOG_INFO("falha");
									return rc;
				}
				
				
				
}


uint32_t mwatt_read_anything(uint32_t address)
{
	  if ( (address & 0x3) == 0 )//check if address is word aligned
		{
    
	  nrf_delay_ms(2);
    static uint32_t number;
    ret_code_t rc = nrf_fstorage_read(
																										&flash_instance,   /* The instance to use. */
																										address,     /* The address in flash where to read data from. */
																										&number,        /* A buffer to copy the data into. */
																										sizeof(number)  /* Lenght of the data, in bytes. */
																										);
    if (rc == NRF_SUCCESS)
			{
					NRF_LOG_INFO("Read \"%d\" from flash address \"%x\".", number, address);
					return rc;
			}
    }
		else{
			    NRF_LOG_INFO("oops! Address is not 4 byte aligned.Check the address and try again"); 
		       }
}


void hex_to_int ()
{
	
	
	
	
}
