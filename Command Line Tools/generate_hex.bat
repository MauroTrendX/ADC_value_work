nrfutil settings generate --family NRF52 --application ..\Aplication\MDK-ARM\_build\nrf52832_xxaa.hex --application-version 1 --bootloader-version 0 --bl-settings-version 2 auxiliares\bootloader_setting.hex
mergehex --merge auxiliares\bootloader_setting.hex ..\Bootloader\MDK-ARM\_build\nrf52832_xxaa_s132.hex --output auxiliares\merge1.hex
mergehex --merge ..\components\softdevice\s332\hex\ANT_s332_nrf52_7.0.1.hex auxiliares\merge1.hex --output auxiliares\merge2.hex
mergehex --merge ..\Aplication\MDK-ARM\fxi.hex auxiliares\merge2.hex --output auxiliares\merge3.hex
mergehex --merge ..\Aplication\MDK-ARM\_build\nrf52832_xxaa.hex auxiliares\merge3.hex --output mybeat_2.hex
PAUSE