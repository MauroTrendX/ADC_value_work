nrfjprog --family nRF52 --eraseall
nrfjprog --reset --program mybeat_v2.hex --family NRF52 --sectoranduicrerase
nrfjprog --family nrf52 --rbp all
nrfjprog --pinreset
PAUSE