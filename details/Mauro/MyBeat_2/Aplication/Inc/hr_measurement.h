#include <stdint.h>
#include <stdbool.h>

typedef struct{
	uint32_t min_val;
	uint32_t max_val;
	uint32_t start_val;
} hr_measurement_cfg_t;

typedef struct{
	uint32_t curr_val;
} hr_measurement_state_t;

void hr_measurement_init(hr_measurement_state_t * p_state, hr_measurement_cfg_t * p_cfg);
void hr_measurement_state_change(hr_measurement_state_t * p_state, uint32_t new_state);


