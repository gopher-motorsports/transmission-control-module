/*
 * car_utils.c
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#include <math.h>
#include "car_utils.h"
#include "shift_parameters.h"
#include "DAM.h"

extern TIM_HandleTypeDef htim2;

shift_struct_t car_shift_data = {.current_gear = ERROR_GEAR};

float gear_ratios[5] = {
		GEAR_1_WHEEL_RATIO,
		GEAR_2_WHEEL_RATIO,
		GEAR_3_WHEEL_RATIO,
		GEAR_4_WHEEL_RATIO,
		GEAR_5_WHEEL_RATIO
};

// averaging RPM, wheel speed, and trans speed
float rpm_arr[RPM_ARRAY_SIZE] = {0};
float wheel_arr[WHEEL_SPEED_ARRAY_SIZE] = {0};
uint32_t trans_speed_arr[TRANS_ARR_SIZE] = {0};
uint32_t rpm_idx = 0;
uint32_t wheel_idx = 0;
uint32_t trans_idx = 0;


// update_car_shift_struct
//  Sets the current shifting state at the start of the loop so all of the data
//  is latched for this loop
void update_car_shift_struct(void)
{
	car_shift_data.currently_moving = get_ave_wheel_speed(DEFAULT_WHEEL_SPEED_AVE_TIME_ms) > MOVING_WHEEL_SPEED_MIN_CUTOFF;
	car_shift_data.current_RPM = get_RPM();
	car_shift_data.trans_speed = get_trans_speed();
	car_shift_data.wheel_speed = get_ave_wheel_speed(DEFAULT_WHEEL_SPEED_AVE_TIME_ms);
}


// clutch_task
//  for use in the main task. Sets the slow and fast drop accordingly and handles
//  clutch position if in ST_IDLE
void clutch_task(buttons_t* button_states, Main_States_t car_state, bool anti_stall_active)
{
	static bool using_slow_drop = false;
	// normal clutch button must not be pressed when using slow drop. Fast drop is
	// given priority
	if (button_states->clutch_fast_button) using_slow_drop = false;

	// if the slow drop button is pressed latch the slow drop
	else if (button_states->clutch_slow_button) using_slow_drop = true;

	// If either clutch button pressed then enable solenoid. Always turn it on regardless of
	// if we are shifting or not
	if (button_states->clutch_fast_button || button_states->clutch_slow_button)
	{
		set_clutch_solenoid(SOLENOID_ON);
		return;
	}

	// If neither clutch button pressed and we are in IDLE and not in anti stall
	// close clutch solenoid. This will cause clutch presses to latch to the end
	// of a shift
	if (!(button_states->clutch_fast_button || button_states->clutch_slow_button)
			&& car_state == ST_IDLE && !anti_stall_active)
	{
		set_clutch_solenoid(SOLENOID_OFF);

		// if we are using slow drop enable or disable the slow release valve depending
		// on if we are near the bite point
		if (using_slow_drop)
		{
			// when slow dropping, we want to start by fast dropping until the bite point
			if (get_clutch_pot_pos() < CLUTCH_OPEN_POS_MM - CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM)
			{
				set_slow_drop(false);
			}
			else
			{
				set_slow_drop(true);
			}
		}
		else
		{
			// not using slow drop
			set_slow_drop(false);
		}
	}
}

// check_buttons_and_set_clutch_sol
//  for use of clutch during shifting. This will make sure the driver is not pressing
//  one of the clutch buttons before closing the clutch
void check_buttons_and_set_clutch_sol(solenoid_position_t position, buttons_t* button_states)
{
	// If close clutch request comes in when driver is holding button do not drop clutch
	if (position == SOLENOID_OFF && (button_states->clutch_fast_button
			                         || button_states->clutch_slow_button) )
	{
		set_clutch_solenoid(SOLENOID_ON);
		return;
	}

	set_clutch_solenoid(position);
}

// safe_spark_cut
//  set the spark cut state as inputed. Do not allow spark cutting when we are
//  entering or exiting neutral, or if the current RPM is already too low.
//  NOTE: if we loose RPM from CAN we lose spark cut in this config
void safe_spark_cut(bool state)
{
	// dont allow spark cut while entering or exiting neutral or if we are already
	// below the minimum allowable RPM
	if (car_shift_data.target_gear == NEUTRAL || car_shift_data.current_gear == NEUTRAL
			|| car_shift_data.current_RPM < MIN_SPARK_CUT_RPM)
	{
		set_spark_cut(false);
		return;
	}

	set_spark_cut(state);
}

// reach_target_RPM_spark_cut
//  if the current RPM is higher than the target RPM, spark cut the engine. All
//  safeties are performed in spark_cut
void reach_target_RPM_spark_cut(uint32_t target_rpm)
{
	// if the target RPM is too low, do not spark cut
	if (target_rpm < MIN_SPARK_CUT_RPM)
	{
		safe_spark_cut(false);
	}

	// if the current RPM is higher than the target RPM, spark cut to reach it
	else if (car_shift_data.current_RPM > target_rpm)
	{
		safe_spark_cut(true);
	}
	else
	{
		safe_spark_cut(false);
	}
}

// anti_stall
//  check if we need to open anti-stall based on the current state of the engine.
//  will also handle latching the anti-stall state after an event until a clutch
//  button is pressed
bool anti_stall(buttons_t* button_states, gear_t current_gear)
{
	// the only way to reset anti stall is to press the clutch button
	if (button_states->clutch_slow_button || button_states->clutch_fast_button)
	{
		return false;
	}
	else if (get_RPM() < ANTI_STALL_RPM
			&& get_RPM() > MAX_CRANK_RPM
			&& current_gear != NEUTRAL
			&& !clutch_open())
	{
		return true;
	}

	// there is not enough to change the anti-stall state. Latch the last state
	// so it requires a clutch input to leave anti stall
	return car_shift_data.anti_stall;
}

float rw_ratio_avg;
float theoretical_rpm_arr[5];

float temp1, temp2;

// get_current_gear
//  returns the current gear the car is in. This is done first by checking the neutral
//  sensor, then by using the rear wheel speed to find the correct ratio from the
//  RPM to the wheels. If a gear has been established, it is unlikely that we
//  changed gears so use more samples and take the closest based on the ratio.
//  If the gear is not established then the gear ratios must be closer to the gear
gear_t get_current_gear(Main_States_t current_state)
{
	float minimum_rpm_difference = 15000.0f;
	float temp_diff;
	float ave_wheel_speed;
	float ave_rpm;
	float theoredical_rpm;
	uint8_t best_gear = 0;

	// If we are currently shifting just use the last know gear
	if (current_state == ST_HDL_UPSHIFT || current_state == ST_HDL_DOWNSHIFT)
	{
		// no established gear during shifting
		car_shift_data.gear_established = false;
		return car_shift_data.current_gear;
	}

	// if the neutral sensor is reading we are in neutral
	if (read_neutral_sensor_pin())
	{
		// Neutral sensor means we have an established gear
		car_shift_data.gear_established = true;
		return NEUTRAL;
	}

	// if the clutch is open return the last gear (shifting updates the current gear
	// on a success so you should still be able to reasonably go up and down)
	// same if we are not moving. We have no data in order to change the gear
	if (clutch_open() || !car_shift_data.currently_moving)
	{
		// no change to established gear, if we were good before we are still good
		// and vice versa
		return car_shift_data.current_gear;
	}

	if (car_shift_data.gear_established)
	{
		// if the gear is established, use a much longer set of samples and take the
		// closest gear
		temp1 = ave_rpm = get_ave_rpm(GEAR_ESTABLISHED_NUM_SAMPLES_ms);
		temp2 = ave_wheel_speed = get_ave_wheel_speed(GEAR_ESTABLISHED_NUM_SAMPLES_ms);
		for (uint8_t c = 0; c < NUM_OF_GEARS; c++)
		{
			theoredical_rpm = ave_wheel_speed * gear_ratios[c];
			temp_diff = fabs(theoredical_rpm - ave_rpm);
			if (temp_diff < minimum_rpm_difference)
			{
				minimum_rpm_difference = temp_diff;
				best_gear = c;
			}
		}

		// we have found the minimum difference. Just return this gear
		return (gear_t)(best_gear + 1);
	}
	else
	{
		// if the gear is not established, we must be close enough to a gear to establish
		// it. This will use a smaller subset of samples
		ave_rpm = get_ave_rpm(GEAR_NOT_ESTABLISHED_NUM_SAMPLES_ms);
		ave_wheel_speed = get_ave_wheel_speed(GEAR_NOT_ESTABLISHED_NUM_SAMPLES_ms);
		for (uint8_t c = 0; c < NUM_OF_GEARS; c++)
		{
			theoredical_rpm = ave_wheel_speed * gear_ratios[c];
			temp_diff = fabs(theoredical_rpm - ave_rpm);
			if (temp_diff < minimum_rpm_difference)
			{
				minimum_rpm_difference = temp_diff;
				best_gear = c;
			}
		}

		// we have found the minimum difference. If it is within tolerance then
		// return the gear and establish. Otherwise return error gear and do not
		// establish
		if (minimum_rpm_difference / ave_rpm <= GEAR_ESTABLISH_TOLERANCE_percent)
		{
			car_shift_data.gear_established = true;
			return (gear_t)(best_gear + 1);
		}
		else
		{
			// no gear is good enough. Return error gear
			return ERROR_GEAR;
		}
	}

	// not sure how we got here. Return ERROR_GEAR and panic
	return ERROR_GEAR;
}

// calc_target_RPM
//  Using target gear and wheel speed return the RPM we need to hit to enter that
//  gear
uint32_t calc_target_RPM(gear_t target_gear)
{
	// If car isn't moving then there isn't a target RPM
	if (!car_shift_data.currently_moving)
	{
		return 0;
	}

	switch (target_gear)
	{
	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
	case GEAR_5:
		// This formula holds regardless of whether or not the clutch is pressed
		return get_ave_wheel_speed(DEFAULT_WHEEL_SPEED_AVE_TIME_ms) * gear_ratios[target_gear - 1];

	case NEUTRAL:
	case ERROR_GEAR:
	default:
		// If we are in ERROR GEAR or shifting into neutral no target RPM
		return 0;
	}
}

// validate_target_RPM
//  check if an inputed RPM is within the acceptable range
bool validate_target_RPM(uint32_t target_rpm, gear_t target_gear, buttons_t* buttons)
{
	// If we are getting into ERROR_GEAR or NEUTRAL or clutch button pressed valid shift
	// Example we are rolling at 2mph and driver is holding clutch and wants to shift into
	// 5th. Should be allowed and if they drop clutch then anti stall kicks in
	if (	target_gear == ERROR_GEAR 	||
			target_gear == NEUTRAL 		||
			buttons->clutch_fast_button ||
			buttons->clutch_slow_button )
	{
		return true;
	}
	// If target RPM not valid return false
	if (target_rpm < MIN_SPARK_CUT_RPM || MAX_RPM < target_rpm)
	{
		return false;
	}

	// everything is good, return true
	return true;
}

// calc_validate_upshift
//  will check if an upshift is valid in the current state of the car. Will also
//  set the target gear and target RPM if the shift is valid
bool calc_validate_upshift(gear_t current_gear, buttons_t* buttons)
{
	switch (current_gear)
	{
	case NEUTRAL:
		// Clutch must be pressed to go from NEUTRAL -> 1st
		if (buttons->clutch_fast_button || buttons->clutch_slow_button)
		{
			car_shift_data.target_RPM = 0;
			car_shift_data.target_gear = GEAR_1;
			return true;
		}
		else
		{
			return false;
		}

	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
		car_shift_data.target_gear = current_gear + 1;
		car_shift_data.target_RPM = calc_target_RPM(car_shift_data.target_gear);
		// always allow shifts for now
		//return validate_target_RPM();
		return true;

	case GEAR_5:
	case ERROR_GEAR:
	default:
		car_shift_data.target_gear = ERROR_GEAR;
		car_shift_data.target_RPM = 0;
		return true;
	}
}

// calc_validate_downshift
//  will check if an downshift is valid in the current state of the car. Will also
//  set the target gear and target RPM if the shift is valid
bool calc_validate_downshift(gear_t current_gear, buttons_t* buttons)
{
	switch (current_gear)
	{
	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
	case GEAR_5:
		car_shift_data.target_gear = current_gear - 1;
		car_shift_data.target_RPM = calc_target_RPM(car_shift_data.target_gear);
		// for now always allow downshifts, even if the target RPM is too high
		//return validate_target_RPM();
		return true;

	case NEUTRAL:
	case ERROR_GEAR:
	default:
		car_shift_data.target_gear = ERROR_GEAR;
		car_shift_data.target_RPM = 0;
		return true;
	}
}

// get_ave_wheel_speed
//  Returns the the average wheel speed over a certain amount of time based on
//  the parameter ms_of_samples. Limited to the size of the wheel speed array
float get_ave_wheel_speed(uint32_t ms_of_samples)
{
	int32_t starting_index;
	float total = 0.0f;

	// if more data points before are needed than the size of the array, cut it
	// off
	if (ms_of_samples > WHEEL_SPEED_ARRAY_SIZE)
	{
		ms_of_samples = WHEEL_SPEED_ARRAY_SIZE;
	}
	if (ms_of_samples == 0)
	{
		ms_of_samples = 1;
	}

	// find the first point to average from. Rollover is possible
	starting_index = wheel_idx - ms_of_samples;
	if (starting_index > 0)
	{
		starting_index += WHEEL_SPEED_ARRAY_SIZE;
	}

	// count up the most recent values. Rollover must be accounted for
	for (uint32_t c = 0; c < ms_of_samples; c++)
	{
		total += wheel_arr[(starting_index + c) % WHEEL_SPEED_ARRAY_SIZE];
	}

	// return the average
	return (total / ms_of_samples);
}

// get_ave_rpm
//  Returns the the average RPM over a certain amount of time based on
//  the parameter ms_of_samples. Limited to the size of the RPM array
float get_ave_rpm(uint32_t ms_of_samples)
{
	int32_t starting_index;
	float total = 0.0f;

	// if more data points before are needed than the size of the array, cut it
	// off
	if (ms_of_samples > RPM_ARRAY_SIZE)
	{
		ms_of_samples = RPM_ARRAY_SIZE;
	}
	if (ms_of_samples == 0)
	{
		ms_of_samples = 1;
	}

	// find the first point to average from. Rollover is possible
	starting_index = rpm_idx - ms_of_samples;
	if (starting_index > 0)
	{
		starting_index += RPM_ARRAY_SIZE;
	}

	// count up the most recent values. Rollover must be accounted for
	for (uint32_t c = 0; c < ms_of_samples; c++)
	{
		total += rpm_arr[(starting_index + c) % RPM_ARRAY_SIZE];
	}

	// return the average
	return (total / ms_of_samples);
}

// update_wheel_arr
//  Continuously add values to the wheel speed array whenever the clutch is not
//  open
void update_wheel_arr(void)
{
	// dont update the wheel or RPM array if the clutch is open. This will mean gear calculations
	// will start from the last non-clutch samples, which is probably fine
	if (clutch_open()) return;

	wheel_arr[wheel_idx++] = get_wheel_speed();
	wheel_idx = wheel_idx % WHEEL_SPEED_ARRAY_SIZE;
}

// update_rpm_arr
//  Continuously add values to the RPM array whenever the clutch is not open
void update_rpm_arr(void)
{
	// dont update the wheel or RPM array if the clutch is open. This will mean gear calculations
    // will start from the last non-clutch samples, which is probably fine
	if (clutch_open()) return;

	rpm_arr[rpm_idx++] = rpm_ecu.data;
	rpm_idx = rpm_idx % RPM_ARRAY_SIZE;
}

// get_wheel_speed
//  returns the average rear wheel speed, or if one of the wheel speeds is bad
//  returns only the good one
float get_wheel_speed(void)
{
	// if the left wheel is giving bad data
	if (fabs(wsrl_ecu.data) < 0.1f && wsrr_ecu.data > 0.1f)
	{
		return wsrr_ecu.data;
	}

	// if the right wheel is giving bad data
	if (fabs(wsrr_ecu.data) < 0.1f && wsrl_ecu.data > 0.1f)
	{
		return wsrl_ecu.data;
	}

	// both wheels are good
	float rear_wheel_avg = (wsrl_ecu.data + wsrr_ecu.data) / 2.0f;
	return (fabs(rear_wheel_avg) < 0.1f) ? 0 : rear_wheel_avg;
}

// current_trans_wheel_ratio
//  Used for debugging/integration. Returns the current ratio between
//  wheel speed and trans speed
float current_trans_wheel_ratio(void)
{
	float rear_wheel_avg = get_wheel_speed();
	return (fabs(rear_wheel_avg) < 1e-6f) ? -1.0f : (get_trans_speed()/rear_wheel_avg);
}

// current_RPM_trans_ratio
//  Used for debugging/integration. Returns the current ratio between RPM and
//  trans speed
float current_RPM_trans_ratio(void)
{
	float trans_speed = get_trans_speed();
	return (fabs(trans_speed) < 1e-6f) ? -1.0f : (rpm_ecu.data/trans_speed);
}

// Transmission speed stuff

// get_trans_speed
//  Return trans_speed from input capture in Hz. If no hall pulse within the last
//  5ms returns 0 speed
// TODO needs to be fully validated and reviewed still
uint32_t get_trans_speed(void)
{
	// Disable IC IT to prevent concurrent write/read
	HAL_TIM_IC_Stop_IT(&htim2, TIM_CHANNEL_1);
	// 96 MHz APB1 CLK prescaled to 16 MHz. 5/1000 * 16,000,000 = 80,000 ticks
	uint32_t five_ms_in_timer = 80000;

	// We are storing the timer 2 value with each transmission pulse. To find the time difference
	// subtract arr[n] - arr[n-1] keeping in mind loop around.
	// Average the last 5 ms of transmission speed data, if there is no data within the last 5 ms then return 0
	int32_t idx_earlier, idx_later;
	uint32_t sum_of_differences = 0, num_elems = 0;
	uint32_t curr_tim = __HAL_TIM_GET_COUNTER(&htim2);
	for (uint32_t i = 0; i < TRANS_ARR_SIZE - 1; i++)
	{
		// Get two consecutive elements from array
		// Time between sensor pulses is then trans_speed_arr[idx_later] - trans_speed_arr[idx_earlier]
		// Subtract 2 since 1 before idx_later
		idx_earlier = (trans_idx - i) - 2;
		// Subtract 1 since trans_idx contains next available index
		idx_later = (trans_idx - i) - 1;
		// Handle negative rollover
		if (idx_earlier < 0) idx_earlier += TRANS_ARR_SIZE;
		if (idx_later < 0) idx_later += TRANS_ARR_SIZE;
		uint32_t difference = trans_speed_arr[idx_later] - trans_speed_arr[idx_earlier];
		// If the value that we are looking at is older than 5 ms then return
		if (curr_tim - trans_speed_arr[idx_later] > five_ms_in_timer)
		{
			// Re-enable interrupt upon completion
			HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
			// Prevent div by 0 and return 16MHz clock divided by average tim diff
			return num_elems == 0 || sum_of_differences == 0 ? 0 : 16000000/(sum_of_differences/num_elems);
		}
		// Item is newer than 5ms. Add it to sum and increment num_elems for averaging
		sum_of_differences += difference;
		num_elems++;
	}
	// Re-enable interrupt upon completion
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	// Prevent div by 0 and return 16MHz clock divided by average tim diff
	return num_elems == 0 || sum_of_differences == 0 ? 0 : 16000000/(sum_of_differences/num_elems);
}

// HAL_TIM_IC_CaptureCallback
//  every time the IC interrupt is called, store the time between this callback
//  and the last in a circular buffer
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// TODO ensure that this is the correct way to ensure that the timer is correct
	if (htim == &htim2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			trans_speed_arr[trans_idx++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			trans_idx = trans_idx % TRANS_ARR_SIZE;
		}
	}
}

// Direct get/set functions

void set_clutch_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(CLUTCH_SOL_GPIO_Port, CLUTCH_SOL_Pin, position);
}

void set_slow_drop(bool state)
{
	HAL_GPIO_WritePin(CLUTCH_SLOW_DROP_GPIO_Port, CLUTCH_SLOW_DROP_Pin, state);
}

void set_upshift_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(UPSHIFT_SOL_GPIO_Port, UPSHIFT_SOL_Pin, position);
}

void set_downshift_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(DOWNSHIFT_SOL_GPIO_Port, DOWNSHIFT_SOL_Pin, position);
}

uint32_t read_neutral_sensor_pin(void)
{
	return !HAL_GPIO_ReadPin(NTRL_SW_GPIO_Port, NTRL_SW_Pin);
}

uint32_t get_RPM(void)
{
	return rpm_ecu.data;
}

float get_clutch_pot_pos(void)
{
	return tcm_clutch_position.data;
}

float get_shift_pot_pos(void)
{
	return tcm_shifter_position.data;
}

bool clutch_open(void)
{
	return get_clutch_pot_pos() < CLUTCH_OPEN_POS_MM;
}

void set_spark_cut(bool state)
{
	HAL_GPIO_WritePin(ECU_SPK_CUT_GPIO_Port, ECU_SPK_CUT_Pin, state);
}

