nrfutil settings generate --family NRF52 --application ..\Aplication\MDK-ARM\_build\nrf52832_xxaa.hex --application-version 1 --bootloader-version 0 --bl-settings-version 2 auxiliares\bootloader_setting.hex
nrfjprog --reset --program auxiliares\bootloader_setting.hex --family NRF52 --sectoranduicrerase
PAUSE