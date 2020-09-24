#include "hr_measurement.h"

void hr_measurement_init(hr_measurement_state_t * p_state, hr_measurement_cfg_t * p_cfg){
	p_state->curr_val = p_cfg->start_val;
}

void hr_measurement_state_change(hr_measurement_state_t * p_state, uint32_t new_state){
	p_state->curr_val = new_state;
}

