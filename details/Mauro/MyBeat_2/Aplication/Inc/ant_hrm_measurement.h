#include <stdint.h>
#include <stdbool.h>
#include "bsp.h"
#include "ant_hrm.h"
#include "ant_hrm_utils.h"
#include "ant_hrm_measurement_local.h"

/**@brief HRM measurement configuration structure. */
typedef struct
{
    ant_hrm_profile_t *	p_profile;     ///< Related profile.
    hr_measurement_cfg_t	measurement_cfg; ///< Measurement configuration.
} ant_hrm_measurement_cfg_t;


/**@brief HRM measurement structure. */
typedef struct
{
    ant_hrm_profile_t       * p_profile;    ///< Related profile.
    ant_hrm_measurement_cb_t    _cb;          ///< Internal control block.
} ant_hrm_measurement_t;


/**@brief Function for initializing the ANT HRM measurement instance.
 *
 * @param[in]  p_measurement    Pointer to the measurement instance.
 * @param[in]  p_config         Pointer to the measurement configuration structure.
 */
void ant_hrm_measurement_init(ant_hrm_measurement_t           * p_measurement,
                            ant_hrm_measurement_cfg_t const * p_config);

/**@brief Function for handling a measurement event.
 *
 * @details Based on this event, the transmitter data is updated with
 * respect to the value pointed by the ant_hrm_measurement_t instance on
 * main. This value is updated by ant_hrm_measurement_update() function.
 * @see ant_hrm_measurement_update().
 *
 * This function should be called in the HRM TX event handler.
 */
void ant_hrm_measurement_one_iteration(ant_hrm_measurement_t * p_measurement);

/**@brief Function for changing the heart rate value.
 *
 * @param[in]  p_simulator      Pointer to the simulator instance.
 */
void ant_hrm_measurement_update(ant_hrm_measurement_t * p_measurement, uint32_t new_val);
