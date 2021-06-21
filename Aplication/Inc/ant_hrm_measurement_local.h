#include <stdint.h>
#include <stdbool.h>
#include "bsp.h"
#include "ant_hrm.h"
#include "hr_measurement.h"

typedef struct
{
    uint32_t          		time_since_last_hb;
    uint64_t          		fraction_since_last_hb;
    hr_measurement_state_t 	hr_measurement_state;
    hr_measurement_cfg_t	hr_measurement_cfg;
} ant_hrm_measurement_cb_t;

