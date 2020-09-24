#include <stdbool.h>
#include <stdint.h>
//#include "zonas_cardiacas.h"

typedef enum{
	LIST_ALL				= -1,	//for listing enabled metrics
	ALL						= 0,	//for getting all metrics updated in the last 'data available' event.
	AGE						= 0x03,
	PROFILE					= 0x04,
	HEIGHT					= 0x05,
	WEIGHT					= 0x06,
	RHR_PREFERENCE			= 0x07,
	SLEEP_PREFERENCE		= 0x09,
	TIME					= 0x1E,
	BODY_POSITION			= 0x1F,
	HEART_RATE				= 0x20,
	RESTING_HEART_RATE		= 0x21,
	SKIN_PROXIMITY			= 0x22,
	ENERGY_EXPENDITURE		= 0x23,
	SPEED					= 0x24,
	MOTION_CADENCE			= 0x25,
	ACTIVITY_TYPE			= 0x26,
	HEART_BEATS				= 0x27,
	VO2MAX					= 0x28,
	CARDIO_FITNESS_INDEX	= 0x29,
	RESPIRATION_RATE		= 0x2A,
	ACCELERATION			= 0x2B,
	PPG						= 0x2C,
	LOW_POWER_HEART_RATE	= 0x2D,
	ACTIVITY_COUNT			= 0x2E,
	WEST_PRIVATE_DATA		= 0x2F,
	SLEEP_SESSION			= 0x30,
	MOTION_EVENT			= 0x31,
	USER_ID      			= 0x32,          //@Including USER ID
	AEROBIC_THRESHOLD  		= 0x33,          //@Including AEROBIC THRESHOLD
	ANAEROBIC_THRESHOLD		= 0x34,          //@Including ANAEROBIC THRESHOLD
	NAME	      			= 0x35,          //@Including NAME
	FITNESS_INDEX      		= 0x36,           //@Including FITNESS INDEX
	FITST_NAME				= 0x37,
	LAST_NAME				= 0x38,
	EMAIL_ADDRESS			= 0x39,
	DATE_OF_BIRTH			= 0x3A,
	GENDER					= 0x3B,
	VO2_MAX					= 0x3C,
	HEART_RATE_MAX			= 0x3D
} module_metric_t;

typedef uint8_t hr_module_state_t;

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t age_value;
} hr_module_age_metric_t ;

/*** PROFILE METRIC - INPUT/PROPAGATED ***
 * Time-invariant properties of the person, fixed at birth
 * Incoming format: <Y_l><Y_h><M><D><S><H>
 * Y: 	Year of birth.
 * 		Range {0; 1900...2100};
 * 		value 0 means unspecified; other values reserved.
 * 	M:	Month of birth.
 * 		Range 1...12
 * 		value 0 means unspecified; other values are reserved.
 * 	D:	Day of birth
 * 		Range 0...31
 * 		value 0 means unspecified, other values are reserved.
 * 	S:	Sex
 * 		0: unspecified
 * 		1: Male
 * 		2: Female
 * 	H:	Handedness
 * 		0: unspecified
 * 		1: right-handed
 * 		2: left-handed
 * 		3: mixed-handed
 *
 * 	OBS: The date of birth fields shall either all be specified or
 * 	all be unspecified.
 */
typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint16_t year_of_birth;
	uint8_t month_of_birth;
	uint8_t day_of_birth;
	uint8_t gender;
	uint8_t handedness;
} hr_module_profile_metric_t;

/*** HEIGHT METRIC - INPUT/PROPAGATED ***
 * The height of the person.
 * Incoming format: <H>
 *
 * H:	Height [cm]
 * 		Range 0...255; value 0 means unspecified.
 */
typedef struct {
	uint8_t measurement_index;
	uint8_t quality_index;
	uint16_t height_value;
} hr_module_height_metric_t;

/*** WEIGHT METRIC - INPUT/PROPAGATED ***
 * The weight of the person
 * Incoming format: <W_l><W_h>
 *
 * W:	Weight [hg]
 * 		Range 0...2000; value 0 means unspecified; other values reserved.
 */
typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint16_t weight_value;
} hr_module_weight_metric_t;

/*** RESTING HEART RATE PREFERENCE METRIC - INPUT/PROPAGATED ***
 * The resting heart rate of the person.
 * Incoming format: <RHP>
 *
 * RHP:	Resting HR preference [BPM]
 * 		Range 30...120.
 */
typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t rhr_preference_value;
}hr_module_rhr_preference_metric_t;

/*** SLEEP PREFERENCE METRIC - INPUT/PROPAGATED ***
 * Information about the person in relation to his/her sleep
 * Incoming format: <I>
 *
 * I: Has intention to sleep
 * 		0: Unspecified
 * 		1: Yes
 * 		2: No
 * 		other: Reserved
 */
typedef struct{
	uint8_t quality_index;
	uint8_t sleep_preference_value;
} hr_module_sleep_preference_metric_t;

/*** TIME METRIC - INPUT/PROPAGATED ***
 * The time.
 * Incoming format: <U0><U1><U2><U3>
 *
 * U: 	Time [UTC] in seconds since January 1, 1970 (Unix/Posix time,
 * 		also known as 'epoch time').
 * 		Range 0...(2^32)-1
 */
typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint32_t time_value;
} hr_module_time_metric_t;

/*** BODY POSITION METRIC - INPUT/PROPAGATED ***
 * The body position at which the PPG and acceleration are sensed.
 * Incoming format: <BP>
 *
 * BP: Body position index.
 * 		0: Unspecified
 * 		1: Left wrist
 * 		2: Right wrist
 * 		3: Unspecified
 * 		other: Reserved
 */
typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t body_position_value;
} hr_module_body_position_metric_t;

/*** HEART RATE METRIC - EXTRACTED ***
 * The heart rate of the person.
 * Incoming format: <HR>
 *
 * HR: 	Heart rate [BPM]
 * 		Range 30...220.
 */
typedef struct{
	uint8_t quality_index;
	uint8_t measurement_index;
	uint8_t heart_rate_value;
} hr_module_heart_rate_metric_t;

/*** RESTING HEART RATE METRIC - EXTRACTED ***
 * The resting heart rate of the person.
 * Incoming format: <RHR>
 *
 * RHR:	Resting heart rate [BPM]
 * 		Range 30...120
 */
typedef uint8_t hr_module_rhr_metric_t;

/*** SKIN PROXIMITY METRIC - EXTRACTED ***
 * The proximity to the skin of the person
 * Incoming format: <P>
 *
 * P:	Proximity
 * 		0: on-skin
 * 		254: off-skin
 * 		255: unspecified
 * 		other: reserved
 */
typedef struct{
	uint8_t index;
	uint8_t quality;
	uint8_t skin_proximity_value;
} hr_module_skin_proximity_metric_t;

/*** ENERGY EXPEDITURE METRIC - EXTRACTED ***
 * The energy expenditure due to physical activity of the person
 * Incoming format: <E_l><E_h>
 *
 * E:	Energy expenditure [kcal/h]
 * 		Range 0 1200
 */
typedef uint16_t hr_module_energy_expediture_metric_t;

/*** SPEED METRIC - EXTRACTED ***
 * The motion speed of the person
 * Incoming format: <SP>
 *
 * SP:	Speed [0.1 m/s]
 * 		Range 0...150
 */
typedef uint8_t hr_module_speed_metric_t;

/*** CADENCE METRIC - EXTRACTED ***
 * The motion cadence of the person.
 * Incoming format: <C>
 *
 * C:	Cadence of the motion [1/min]
 * 		Range 20...120
 */
typedef uint8_t hr_module_motion_cadence_metric_t;

/*** ACTIVITY TYPE METRIC - EXTRACTED
 * The type of activity the person is performing
 * Incoming format: <AT>
 *
 * AT:	Activity type
 * 		0: unspecified
 * 		1: other
 * 		2: walk
 * 		4: run
 * 		6: cycle
 * 		other: reserved
 */
typedef uint8_t hr_module_activity_type_metric_t;

/**
 * data type for completion of the heart beats metric structure.
 */
typedef struct{
	uint8_t quality;
	uint32_t time;
	uint16_t reserved_data;
	uint8_t type;
} hr_module_heart_beat_data_t;

/*** HEART BEATS METRIC - EXTRACTED ***
 * The heart beats of the person at the specified body position.
 * Incoming format: <M><BP>
 * 					<Q0>
 * 					<T0,0><T1,0>
 * 					<T2,0><T3,0>
 * 					<E_l,0><E_h,0>
 * 					<TY0>
 * 					...
 * 					<Q[M-1]>
 * 					<T0,[M-1]><T1,[M-1]>
 * 					<T2,[M-1]><T3,[M-1]>
 * 					<E_L,[M-1]><E_H,[M-1]>
 * 					<TY[M-1]>
 * 	M:	Number of heart beats (0...5)
 * 	BP:	Body position index (refer to Body Position metric)
 * 	For all beats i = 0...M-1:
 * 	Qi:	Quality (0...4)
 * 	Ti: Time [ms]; range 0...(2^32)-1
 * 	Ei: reserved
 * 	TYi:Type - 	0x00: Normal beat in sequence
 * 				0xFF: Last beat in sequence
 * 	Heart beats are reported in sequence. The sequence of heart beats may
 * 	also be temporarily interrupted when no beats could be detected. This
 * 	is signaled by the heart beat type. The time between two consecutive
 * 	heart beats in a sequence can be determined by subtracting their times
 * 	(modulo 2^32)
 *
 * 	Note that this metric has an individual quality Qi per heart beat, in
 * 	addition to the overall quality index of the metric.
 */
typedef struct{
	uint8_t number_of_heart_beats;
	uint8_t body_position;
	hr_module_heart_beat_data_t beats[6];
} hr_module_heart_beats_metric_t;

/*** VO2MAX METRIC - EXTRACTED ***
 * 	The maximum oxygen uptake of the person
 * 	Incoming format: <VOM>
 *
 * 	VOM: VO2Max [ml/kg/min]
 * 		 Range 0...100
 */
typedef uint8_t hr_module_vo2max_metric_t;

/*** CARDIO FITNESS INDEX METRIC - EXTRACTED ***
 * The relative cardio fitness index of the person
 * Incoming format: <CFI>
 *
 * CFI:	Cardio Fitness Index
 * 		Range 0...100 - lowest -> highest fitness
 */
typedef uint8_t hr_module_cardio_fitness_index_metric_t;

/*** RESPIRATION RATE METRIC -EXTRACTED ***
 * The respiration rate of the person.
 * Incoming format: <RR>
 *
 * RR:	Respiration rate [0.25 breaths/min]
 * 		Range 20...80
 */
typedef uint8_t hr_module_rr_metric_t;

/*** ACCELERATION METRIC - EXTRACTED ***
 * The acceleration of the person at the specified body position
 * Incoming format: <BP><SF>
 * 					<X0,L><X0,H>
 * 					<Y0,L><Y0,H>
 * 					<Z0,L><Z0,H>
 * 					...
 * 					<X[M-1],L><X[M-1],H>
 * 					<Y[M-1],L><Y[M-1],H>
 * 					<Z[M-1],L><Z[M-1],H>
 *
 * 	BP:	Body position index (Refer to Body Position Metric)
 * 	SF: Sample format
 * 		0xFF:	M=8
 * 		other:	reserved
 * 	Xi:	Acceleration along X axis
 * 	Yi:	Acceleration along Y axis
 * 	Zi:	Acceleration along Z axis
 *
 * 	Accelerations are represented by signed 12-bit integer, range
 * 	-2048...2047, representing +/-8g. Consequently, one LSB step
 * 	corresponds to 1/256g.
 */
typedef struct {
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t body_position;
	uint8_t sample_format;
	int16_t x_accel[8];
	int16_t y_accel[8];
	int16_t z_accel[8];
} hr_module_acceleration_metric_t;

/*** PPG METRIC - EXTRACTED ***
 * The relative blood volume of the person at the specified body
 * position
 * Incoming format:	<BP><SF>
 * 					<L><G>
 * 					<P0,l><P0,h>
 * 					<A0,l><A0,h>
 * 					...
 * 					<P[M-1],l><P[M-1],h>
 * 					<A[M-1],l><A[M-1],h>
 *
 * 	BP:	Body position index (Refer to Body Position metric)
 * 	SF:	Sample format
 * 		0x7F:	M = 2
 * 		other: 	reserved
 * 	L:	Relative LED power [%]
 * 		0...1000: 	power
 * 		other:		reserved
 * 	G:	Relative ADC gain
 * 		0: gain factor 1x
 * 		1: gain factor 2x
 * 		2: gain factor 4x
 * 		3: gain factor 8x
 * 		other: reserved
 * 	Pi:	PPG (16-but signed)
 * 	Ai:	Ambient light (16-bit signed)
 *
 * 	The relative LED power is expressed relative to be maximum
 * 	that the hardware delivers. 0% means 'LED fully off'.
 */
typedef struct{
	uint8_t body_position_index;
	uint8_t sample_format;
	uint8_t relative_led_power;
	uint8_t relative_adc_gain;
	int16_t ppg[128];
	int16_t ambient_light[128];
} hr_module_ppg_metric_t;

/*** LOW POWER HEART RATE METRIC - EXTRACTED ***
 * The heart rate of the person, 1 sample per minute.
 * Incoming format: <LHR>
 *
 * LHR:	Low Power Heart Rate [BPM]
 * 		Range 30...220
 *
 * 	The Low Power Heart Rate is the short-term heart rate, but
 * 	represented periodically at each update interval (60 seconds).
 */
typedef uint8_t hr_module_low_power_hr_metric_t;

/*** ACTIVITY COUNT METRIC - EXTRACTED ***
 * The activity count of the person at the specified body position
 * Incoming format: <BP>
 * 					<AC_l><AC_h>
 *
 * 	BP:	Body Position Index (Refer to Body Position metric)
 * 	AC:	Activity Count [arbitraty unit]
 * 		Range 0...65535
 */
typedef struct{
	uint8_t body_position;
	uint16_t activity_count;
} hr_module_activity_count_metric_t;

/*** MOTION EVENT METRIC - EXTRACTED ***
 * 	A specific motion event.
 * 	Incoming format: 	<BP>
 * 						<M0><M1>...
 * 						<M[N-1]>
 *
 * 	BP: Body position index (Refer to Body Position metric)
 * 	Mi:	Data byte specifying the motion event.
 * 		Rage 0x00...0xFF.
 *
 * 	The semantics of the data bytes are specific to the
 * 	source of the metric.
 */
typedef struct{
	uint8_t body_position;
	uint8_t motion_event[256];
} hr_module_motion_event_metric_t;


typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t user_id_value;
} hr_module_user_id_metric_t ;					//@Including USER ID

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t aerobic_threshold_value;
} hr_module_aerobic_threshold_metric_t ;		//@Including AEROBIC THRESHOLD

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t anaerobic_threshold_value;
} hr_module_anaerobic_threshold_metric_t ;		//@Including ANAEROBIC THRESHOLD

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint32_t name_value;
} hr_module_name_metric_t ;						//@Including NAME

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t hr_zone_preference_calc_value;
} hr_module_hr_zone_preference_calc_metric_t ;			//@Including FITNESS INDEX


typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint64_t first_name_value;
} hr_module_first_name_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t last_name_value;
} hr_module_last_name_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t email_address_value;
} hr_module_email_address_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t date_of_birth_value;
} hr_module_date_of_birth_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t gender_value;
} hr_module_gender_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t VO2_max_value;
} hr_module_VO2_max_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t heart_rate_max_value;
} hr_module_heart_rate_max_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t resting_heart_rate_value;
} hr_module_resting_heart_rate_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t maximum_recommended_heart_rate_value;
} hr_module_maximum_recommended_heart_rate_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t sport_type_for_aerobic_and_anaerobic_thresholds_value;
} hr_module_sport_type_for_aerobic_and_anaerobic_thresholds_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t date_of_threshold_assessment_value;
} hr_module_date_of_threshold_assessment_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t waist_circumference_value;
} hr_module_waist_circumference_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t fat_burn_heart_rate_lower_limit_value;
} hr_module_fat_burn_heart_rate_lower_limit_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t fat_burn_heart_rate_upper_limit_value;
} hr_module_fat_burn_heart_rate_upper_limit_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t aerobic_heart_rate_lower_limit_value;
} hr_module_aerobic_heart_rate_lower_limit_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t aerobic_heart_rate_upper_limit_value;
} hr_module_aerobic_heart_rate_upper_limit_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t anaerobic_heart_rate_lower_limit_value;
} hr_module_anaerobic_heart_rate_lower_limit_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t five_zone_heart_rate_limits_value;
} hr_module_five_zone_heart_rate_limits_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t three_zone_heart_rate_limits_value;
} hr_module_three_zone_heart_rate_limits_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t two_zone_heart_rate_limit_value;
} hr_module_two_zone_heart_rate_limit_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t language_value;
} hr_module_language_metric_t ;			//@Including FITNESS INDEX

typedef struct{
	uint8_t measurement_index;
	uint8_t quality_index;
	uint8_t fitnes_index_value;
} hr_module_fitnes_index_metric_t ;			//@Including FITNESS INDEX


/**
 * structure to control which metrics will be enabled or disabled and
 * the type of value (default or user-defined) values to be used.
 */
typedef struct{
	bool profile_metric_set;
	bool rhr_preference_metric_set;
	bool sleep_preference_metric_set;
	bool time_metric_set;
	bool body_position_metric_set;
	bool user_id_metric_set;							//@Including USER ID
	bool hr_zone_preference_calc_metric_set;						//@Including FITNESS INDEX
	bool first_name_metric_set;
	bool last_name_metric_set;
	bool email_address_metric_set;
	bool age_metric_set;
	bool date_of_birth_metric_set;
	bool gender_metric_set;
	bool weight_metric_set;
	bool height_metric_set;
	bool VO2_max_metric_set;
	bool heart_rate_max_metric_set;
	bool resting_heart_rate_metric_set;
	bool maximum_recommended_heart_rate_metric_set;
	bool aerobic_threshold_metric_set;
	bool anaerobic_threshold_metric_set;
	bool sport_type_for_aerobic_and_anaerobic_thresholds_metric_set;
	bool date_of_threshold_assessment_metric_set;
	bool waist_circumference_metric_set;
	bool fat_burn_heart_rate_lower_limit_metric_set;
	bool fat_burn_heart_rate_upper_limit_metric_set;
	bool aerobic_heart_rate_lower_limit_metric_set;
	bool aerobic_heart_rate_upper_limit_metric_set;
	bool anaerobic_heart_rate_lower_limit_metric_set;
	bool five_zone_heart_rate_limits_metric_set;
	bool three_zone_heart_rate_limits_metric_set;
	bool two_zone_heart_rate_limit_metric_set;
	bool language_metric_set;
	bool fitnes_index_metric_set;

} hr_module_input_metrics_settings_t;

typedef struct{
		uint8_t zone1_threshold;
		uint8_t zone2_threshold;
		uint8_t zone3_threshold;
		uint8_t zone4_threshold;
		uint8_t zone5_threshold;
} hr_limits_t;

typedef enum{
	hr_zone_calc_select_RHR = 0,
	hr_zone_calc_select_SPIVI
}hr_zone_calc_select_t;

/**
 * structure to propagate information to main application layer.
 */
typedef struct{
	hr_module_state_t						state;
//	hr_module_age_metric_t					age;
	hr_module_profile_metric_t				profile;
//	hr_module_height_metric_t				height;
//	hr_module_weight_metric_t				weight;
	hr_module_rhr_preference_metric_t		rhr_preference;
	hr_module_sleep_preference_metric_t		sleep_preference;
	hr_module_time_metric_t					time;
	hr_module_body_position_metric_t		body_position;
	hr_module_heart_rate_metric_t			heart_rate;
	hr_module_rhr_metric_t					resting_hr;
	hr_module_skin_proximity_metric_t		skin_proximity;
	hr_module_energy_expediture_metric_t	energy_expediture;
	hr_module_speed_metric_t				speed;
	hr_module_motion_cadence_metric_t		motion_cadence;
	hr_module_activity_type_metric_t		activity_type;
	hr_module_heart_beats_metric_t			heart_beats;
	hr_module_vo2max_metric_t				vo2_max;
	hr_module_cardio_fitness_index_metric_t	cardio_fitness_index;
	hr_module_rr_metric_t					rr;
	hr_module_acceleration_metric_t			acceleration;
	hr_module_ppg_metric_t					ppg;
	hr_module_low_power_hr_metric_t			low_power_hr;
	hr_module_activity_count_metric_t		activity_count;
	hr_module_motion_event_metric_t			motion_event;
	hr_module_user_id_metric_t				user_id;					//@Including USER ID
//	hr_module_aerobic_threshold_metric_t	aerobic_threshold;			//@Including AEROBIC THRESHOLD
//	hr_module_anaerobic_threshold_metric_t	anaerobic_threshold;		//@Including ANAEROBIC THRESHOLD
//	hr_module_name_metric_t					name;						//@Including NAME
	hr_module_hr_zone_preference_calc_metric_t		hr_zone_preference_calc;				//@Including FITNESS INDEX
	hr_module_first_name_metric_t						first_name;
	hr_module_last_name_metric_t						last_name;
	hr_module_email_address_metric_t					email_address;
	hr_module_age_metric_t								age;
	hr_module_date_of_birth_metric_t					date_of_birth;
	hr_module_gender_metric_t							gender;
	hr_module_weight_metric_t							weight;
	hr_module_height_metric_t							height;
	hr_module_VO2_max_metric_t							VO2_max;
	hr_module_heart_rate_max_metric_t					heart_rate_max;
	hr_module_resting_heart_rate_metric_t				resting_heart_rate;
	hr_module_maximum_recommended_heart_rate_metric_t	maximum_recommended_heart_rate;
	hr_module_aerobic_threshold_metric_t				aerobic_threshold;
	hr_module_anaerobic_threshold_metric_t				anaerobic_threshold;
	hr_module_sport_type_for_aerobic_and_anaerobic_thresholds_metric_t	sport_type_for_aerobic_and_anaerobic_thresholds;
	hr_module_date_of_threshold_assessment_metric_t		date_of_threshold_assessment;
	hr_module_waist_circumference_metric_t				waist_circumference;
	hr_module_fat_burn_heart_rate_lower_limit_metric_t	fat_burn_heart_rate_lower_limit;
	hr_module_fat_burn_heart_rate_upper_limit_metric_t	fat_burn_heart_rate_upper_limit;
	hr_module_aerobic_heart_rate_lower_limit_metric_t	aerobic_heart_rate_lower_limit;
	hr_module_aerobic_heart_rate_upper_limit_metric_t	aerobic_heart_rate_upper_limit;
	hr_module_anaerobic_heart_rate_lower_limit_metric_t	anaerobic_heart_rate_lower_limit;
	hr_module_five_zone_heart_rate_limits_metric_t		five_zone_heart_rate_limits;
	hr_module_three_zone_heart_rate_limits_metric_t		three_zone_heart_rate_limits;
	hr_module_two_zone_heart_rate_limit_metric_t		two_zone_heart_rate_limit;
	hr_module_language_metric_t							language;
	hr_module_fitnes_index_metric_t							fitnes_index;
} hr_module_metric_info_t;





/**
 * structure to propagate information to main application layer.
 */
typedef struct{
	hr_module_rhr_preference_metric_t																							rhr_preference;
	hr_module_sleep_preference_metric_t																						sleep_preference;
	hr_module_time_metric_t																												time;
	hr_module_body_position_metric_t																							body_position;
	hr_module_input_metrics_settings_t 																						metric_settings_nv_buf;
	hr_limits_t																																		hr_SPIVI_zone_limits;
	hr_limits_t																																		hr_rhr_zone_limits;
	hr_zone_calc_select_t																													hr_zone_calc_select;
	hr_module_user_id_metric_t																										user_id;					//@Including USER ID
	hr_module_hr_zone_preference_calc_metric_t																		hr_zone_preference_calc;				//@Including FITNESS INDEX
	hr_module_first_name_metric_t																									first_name;
	hr_module_last_name_metric_t																									last_name;
	hr_module_email_address_metric_t																							email_address;
	hr_module_age_metric_t																												age;
	hr_module_date_of_birth_metric_t																							date_of_birth;
	hr_module_gender_metric_t																											gender;
	hr_module_weight_metric_t																											weight;
	hr_module_height_metric_t																											height;
	hr_module_VO2_max_metric_t																										VO2_max;
	hr_module_heart_rate_max_metric_t																							heart_rate_max;
	hr_module_resting_heart_rate_metric_t																					resting_heart_rate;
	hr_module_maximum_recommended_heart_rate_metric_t															maximum_recommended_heart_rate;
	hr_module_aerobic_threshold_metric_t																					aerobic_threshold;
	hr_module_anaerobic_threshold_metric_t																				anaerobic_threshold;
	hr_module_sport_type_for_aerobic_and_anaerobic_thresholds_metric_t						sport_type_for_aerobic_and_anaerobic_thresholds;
	hr_module_date_of_threshold_assessment_metric_t																date_of_threshold_assessment;
	hr_module_waist_circumference_metric_t																				waist_circumference;
	hr_module_fat_burn_heart_rate_lower_limit_metric_t														fat_burn_heart_rate_lower_limit;
	hr_module_fat_burn_heart_rate_upper_limit_metric_t														fat_burn_heart_rate_upper_limit;
	hr_module_aerobic_heart_rate_lower_limit_metric_t															aerobic_heart_rate_lower_limit;
	hr_module_aerobic_heart_rate_upper_limit_metric_t															aerobic_heart_rate_upper_limit;
	hr_module_anaerobic_heart_rate_lower_limit_metric_t														anaerobic_heart_rate_lower_limit;
	hr_module_five_zone_heart_rate_limits_metric_t																five_zone_heart_rate_limits;
	hr_module_three_zone_heart_rate_limits_metric_t																three_zone_heart_rate_limits;
	hr_module_two_zone_heart_rate_limit_metric_t																	two_zone_heart_rate_limit;
	hr_module_language_metric_t																										language;
	hr_module_fitnes_index_metric_t																								fitnes_index;
} hr_module_nv_buf_t;

typedef struct{
	hr_module_first_name_metric_t																									first_name;
	hr_module_last_name_metric_t																									last_name;
	hr_module_email_address_metric_t																							email_address;
	hr_module_age_metric_t																												age;
	hr_module_date_of_birth_metric_t																							date_of_birth;
	hr_module_gender_metric_t																											gender;
	hr_module_weight_metric_t																											weight;
	hr_module_height_metric_t																											height;
	hr_module_VO2_max_metric_t																										VO2_max;
	hr_module_heart_rate_max_metric_t																							heart_rate_max;
	hr_module_resting_heart_rate_metric_t																					resting_heart_rate;
	hr_module_maximum_recommended_heart_rate_metric_t															maximum_recommended_heart_rate;
	hr_module_aerobic_threshold_metric_t																					aerobic_threshold;
	hr_module_anaerobic_threshold_metric_t																				anaerobic_threshold;
	hr_module_sport_type_for_aerobic_and_anaerobic_thresholds_metric_t						sport_type_for_aerobic_and_anaerobic_thresholds;
	hr_module_date_of_threshold_assessment_metric_t																date_of_threshold_assessment;
	hr_module_waist_circumference_metric_t																				waist_circumference;
	hr_module_fat_burn_heart_rate_lower_limit_metric_t														fat_burn_heart_rate_lower_limit;
	hr_module_fat_burn_heart_rate_upper_limit_metric_t														fat_burn_heart_rate_upper_limit;
	hr_module_aerobic_heart_rate_lower_limit_metric_t															aerobic_heart_rate_lower_limit;
	hr_module_aerobic_heart_rate_upper_limit_metric_t															aerobic_heart_rate_upper_limit;
	hr_module_anaerobic_heart_rate_lower_limit_metric_t														anaerobic_heart_rate_lower_limit;
	hr_module_five_zone_heart_rate_limits_metric_t																five_zone_heart_rate_limits;
	hr_module_three_zone_heart_rate_limits_metric_t																three_zone_heart_rate_limits;
	hr_module_two_zone_heart_rate_limit_metric_t																	two_zone_heart_rate_limit;
	hr_module_language_metric_t																										language;
	hr_module_fitnes_index_metric_t																								fitnes_index;
} uds_buf_t;

typedef struct{
	hr_module_rhr_preference_metric_t				rhr_preference;
	hr_module_user_id_metric_t							user_id;					//@Including USER ID
} rus_buf_t;

typedef struct{
	bool profile;
	bool rhr_preference;
	bool sleep_preference;
	bool time;
	bool body_position;
	bool metric_settings_nv_buf;
	bool hr_SPIVI_zone_limits;
	bool hr_rhr_zone_limits;
	bool hr_zone_calc_select;
	bool user_id;						//@Including USER ID
	bool hr_zone_preference_calc;					//@Including FITNESS INDEX
	bool first_name;
	bool last_name;
	bool email_address;
	bool age;
	bool date_of_birth;
	bool gender;
	bool weight;
	bool height;
	bool VO2_max;
	bool heart_rate_max;
	bool resting_heart_rate;
	bool maximum_recommended_heart_rate;
	bool aerobic_threshold;
	bool anaerobic_threshold;
	bool sport_type_for_aerobic_and_anaerobic_thresholds;
	bool date_of_threshold_assessment;
	bool waist_circumference;
	bool fat_burn_heart_rate_lower_limit;
	bool fat_burn_heart_rate_upper_limit;
	bool aerobic_heart_rate_lower_limit;
	bool aerobic_heart_rate_upper_limit;
	bool anaerobic_heart_rate_lower_limit;
	bool five_zone_heart_rate_limits;
	bool three_zone_heart_rate_limits;
	bool two_zone_heart_rate_limit;
	bool language;
	bool fitnes_index;
} hr_module_nv_buf_update_flags_t;
/****************************************************************************/
/******************************************************************************/
/******************************************************************************/
uint8_t get_SPIVI_zone1_percentage_threshold(void);
void set_SPIVI_zone1_percentage_threshold(uint8_t);

uint8_t get_SPIVI_zone2_percentage_threshold(void);
void set_SPIVI_zone2_percentage_threshold(uint8_t);

uint8_t get_SPIVI_zone3_percentage_threshold(void);
void set_SPIVI_zone3_percentage_threshold(uint8_t);

uint8_t get_SPIVI_zone4_percentage_threshold(void);
void set_SPIVI_zone4_percentage_threshold(uint8_t);

uint8_t get_SPIVI_zone5_percentage_threshold(void);
void set_SPIVI_zone5_percentage_threshold(uint8_t);

/******************************************************************************/
uint8_t get_rhr_zone1_percentage_threshold(void);
void set_rhr_zone1_percentage_threshold(uint8_t);

uint8_t get_rhr_zone2_percentage_threshold(void);
void set_rhr_zone2_percentage_threshold(uint8_t);

uint8_t get_rhr_zone3_percentage_threshold(void);
void set_rhr_zone3_percentage_threshold(uint8_t);

uint8_t get_rhr_zone4_percentage_threshold(void);
void set_rhr_zone4_percentage_threshold(uint8_t);

uint8_t get_rhr_zone5_percentage_threshold(void);
void set_rhr_zone5_percentage_threshold(uint8_t);
/******************************************************************************/
//uint8_t max_hr_get(void);

uint8_t get_resting_heart_rate_value_threshold(void);
void set_resting_heart_rate_value_threshold(uint8_t value);

uint8_t get_user_id(void);
void set_user_id(uint8_t value);					//@Including USER ID

uint8_t get_hr_zone_preference_calc(void);
void set_hr_zone_preference_calc(uint8_t value);				//@Including FITNESS INDEX

uint64_t get_first_name_metric(void);
void set_first_name_metric(uint64_t value);

uint8_t get_last_name_metric(void);
void set_last_name_metric(uint8_t value);

uint8_t get_email_address_metric(void);
void set_email_address_metric(uint8_t value);

uint8_t get_age_metric(void);
void set_age_metric(uint8_t value);

uint8_t get_date_of_birth_metric(void);
void set_date_of_birth_metric(uint8_t value);

uint8_t get_gender_metric(void);
void set_gender_metric(uint8_t value);

uint16_t get_weight_metric(void);
void set_weight_metric(uint16_t value);

uint16_t get_height_metric(void);
void set_height_metric(uint16_t value);

uint8_t get_VO2_max_metric(void);
void set_VO2_max_metric(uint8_t value);

uint8_t get_heart_rate_max_metric(void);
void set_heart_rate_max_metric(uint8_t value);

uint8_t get_resting_heart_rate_metric(void);
void set_resting_heart_rate_metric(uint8_t value);

uint8_t get_maximum_recommended_heart_rate_metric(void);
void set_maximum_recommended_heart_rate_metric(uint8_t value);

uint8_t get_aerobic_threshold_metric(void);
void set_aerobic_threshold_metric(uint8_t value);

uint8_t get_anaerobic_threshold_metric(void);
void set_anaerobic_threshold_metric(uint8_t value);

uint8_t get_sport_type_for_aerobic_and_anaerobic_thresholds_metric(void);
void set_sport_type_for_aerobic_and_anaerobic_thresholds_metric(uint8_t value);

uint8_t get_date_of_threshold_assessment_metric(void);
void set_date_of_threshold_assessment_metric(uint8_t value);

uint8_t get_waist_circumference_metric(void);
void set_waist_circumference_metric(uint8_t value);

uint8_t get_fat_burn_heart_rate_lower_limit_metric(void);
void set_fat_burn_heart_rate_lower_limit_metric(uint8_t value);

uint8_t get_fat_burn_heart_rate_upper_limit_metric(void);
void set_fat_burn_heart_rate_upper_limit_metric(uint8_t value);

uint8_t get_aerobic_heart_rate_lower_limit_metric(void);
void set_aerobic_heart_rate_lower_limit_metric(uint8_t value);

uint8_t get_aerobic_heart_rate_upper_limit_metric(void);
void set_aerobic_heart_rate_upper_limit_metric(uint8_t value);

uint8_t get_anaerobic_heart_rate_lower_limit_metric(void);
void set_anaerobic_heart_rate_lower_limit_metric(uint8_t value);

uint8_t get_five_zone_heart_rate_limits_metric(void);
void set_five_zone_heart_rate_limits_metric(uint8_t value);

uint8_t get_three_zone_heart_rate_limits_metric(void);
void set_three_zone_heart_rate_limits_metric(uint8_t value);

uint8_t get_two_zone_heart_rate_limit_metric(void);
void set_two_zone_heart_rate_limit_metric(uint8_t value);

uint8_t get_language_metric(void);
void set_language_metric(uint8_t value);

uint8_t get_fitnes_index_metric(void);
void set_fitnes_index_metric(uint8_t value);

hr_module_metric_info_t *get_metrics(void);

void buff_init(void);
void buff_save(void);
void buff_load(void);
