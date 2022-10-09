/*
 * car_utils.c
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#include <math.h>
#include "car_utils.h"
#include "DAM.h"

gear_t car_gear;
shift_struct_t car_shift_data = {.current_gear = ERROR_GEAR};

extern TIM_HandleTypeDef htim2;

void update_car_shift_struct()
{
	car_shift_data.currently_moving = get_wheel_speed() > MOVING_WHEEL_SPEED_MIN_CUTOFF;
	car_shift_data.current_RPM = get_RPM();
	car_shift_data.trans_speed = get_trans_speed();
	car_shift_data.wheel_speed = get_wheel_speed();
	static uint32_t last_gear_update = 0;
	if (HAL_GetTick() - last_gear_update > 100)
	{
		last_gear_update = HAL_GetTick();
		car_shift_data.current_gear = get_current_gear();
	}

}


/* SOLENOID DEFINITIONS */
bool using_slow_drop = false;
bool pending_return_to_fast_drop = false;
uint32_t begin_tick_return_to_fast;

void clutch_task()
{
	if (car_buttons.clutch_fast_button)
	{
		using_slow_drop = false;
	}
	else if (car_buttons.clutch_slow_button)
	{
		using_slow_drop = true;
	}
	// If either clutch button pressed then enable solenoid
	if (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button)
	{
		set_clutch_solenoid(SOLENOID_ON);
	}
	// If neither clutch button pressed and we are in IDLE and not in anti stall close clutch solenoid
	if (!(car_buttons.clutch_fast_button || car_buttons.clutch_slow_button) && car_Main_State == ST_IDLE && !car_shift_data.anti_stall)
	{
		set_clutch_solenoid(SOLENOID_OFF);
	}
	// If we are waiting for return to fast then after 5s disable slow drop
	if (pending_return_to_fast_drop)
	{
		if (HAL_GetTick() - begin_tick_return_to_fast > 5000)
		{
			set_slow_drop(false);
			pending_return_to_fast_drop = false;
		}
	}
	else
	{
		set_slow_drop(false);
	}
}

void set_slow_drop(bool state)
{
	HAL_GPIO_WritePin(CLUTCH_SLOW_DROP_GPIO_Port, CLUTCH_SLOW_DROP_Pin, state);
}

void set_clutch_solenoid(solenoid_position_t position)
{
	// If close clutch request comes in when driver is holding button do not drop clutch
	if (position == SOLENOID_OFF && (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button) ) return;
	// If we are trying to close clutch and the last clutch button press was slow then enable slow drop
	if (position == SOLENOID_OFF && using_slow_drop)
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
		pending_return_to_fast_drop = true;
		begin_tick_return_to_fast = HAL_GetTick();
	}
	else
	{
		set_slow_drop(false);
	}
	HAL_GPIO_WritePin(CLUTCH_SOL_GPIO_Port, CLUTCH_SOL_Pin, position);
}

void set_upshift_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(UPSHIFT_SOL_GPIO_Port, UPSHIFT_SOL_Pin, position);
}

void set_downshift_solenoid(solenoid_position_t position)
{
	HAL_GPIO_WritePin(DOWNSHIFT_SOL_GPIO_Port, DOWNSHIFT_SOL_Pin, position);
}

void set_all_solenoids(solenoid_position_t position)
{
	set_downshift_solenoid(position);
	set_upshift_solenoid(position);
	set_clutch_solenoid(position);
}


void throttle_blip(bool state)
{
	// TODO - add global var configurable from steering wheel to disable blip
	if ((car_shift_data.target_gear == ERROR_GEAR || car_shift_data.target_gear == NEUTRAL || !car_shift_data.currently_moving) && state) return;
	if (get_clutch_pot_pos() > CLUTCH_OPEN_POS_MM && state) return;
	HAL_GPIO_WritePin(ECU_THROTTLE_BLIP_GPIO_Port, ECU_THROTTLE_BLIP_Pin, state);
	car_shift_data.throttle_blip = state;
}

void validate_throttle_blip()
{
	if (car_shift_data.throttle_blip && get_clutch_pot_pos() > CLUTCH_OPEN_POS_MM)
	{
		throttle_blip(false);
	}
}

void spark_cut(bool state)
{
	//TODO - add global var configurable from steering wheel to disable cut
	if (car_shift_data.target_gear == NEUTRAL || !car_shift_data.currently_moving)
	{
		HAL_GPIO_WritePin(ECU_SPK_CUT_GPIO_Port, ECU_SPK_CUT_Pin, false);
		return;
	}
	HAL_GPIO_WritePin(ECU_SPK_CUT_GPIO_Port, ECU_SPK_CUT_Pin, state);
}

void reach_target_RPM_spark_cut()
{
	// TODO - improve this section
	// Do not do anything if current gear is either ERROR or NEUTRAL or clutch is open as target RPM will be 0
	if (car_shift_data.target_gear == ERROR_GEAR || car_shift_data.target_gear == NEUTRAL || !car_shift_data.currently_moving)
	{
		return;
	}

	if (	car_shift_data.current_RPM < car_shift_data.target_RPM * (1.0 - TARGET_RPM_TOLERANCE) ||
			car_shift_data.current_RPM < MIN_RPM)
	{
		spark_cut(false);
	}
	if (	car_shift_data.current_RPM > car_shift_data.target_RPM * (1.0 + TARGET_RPM_TOLERANCE) &&
			car_shift_data.current_RPM > MIN_RPM)
	{
		spark_cut(true);
	}
}

void anti_stall()
{
	if (car_buttons.clutch_slow_button || car_buttons.clutch_fast_button)
	{
		car_shift_data.anti_stall = false;
	}
	if ((get_RPM() < MIN_RPM && get_RPM() > 500 && car_shift_data.current_gear != NEUTRAL && !clutch_open()) || car_shift_data.anti_stall)
	{
		set_clutch_solenoid(SOLENOID_ON);
		car_shift_data.anti_stall = true;
	}
}

uint32_t read_neutral_sensor_pin()
{
	return !HAL_GPIO_ReadPin(NTRL_SW_GPIO_Port, NTRL_SW_Pin);
}


/* GEARS */
float rpm_arr[250];
float wheel_arr[250];
uint32_t rpm_idx = 0, wheel_idx = 0;
// Get current gear
//float gear_ratios[5] = {270.0f, 215.0f, 185.0f, 165.0f, 150.0f};
float gear_ratios[5] = {141.7f, 112.9f, 97.1f, 86.6f, 78.7f};
float tolerance = 0.07f;

float rw_ratio_avg;
float theoretical_rpm_arr[5];
gear_t get_current_gear()
{
	// If we are shifting latch last known gear
	if (car_Main_State == ST_HDL_UPSHIFT || car_Main_State == ST_HDL_DOWNSHIFT)
	{
		return car_shift_data.current_gear;
	}
	// TEMP REPLACEMENT
	if (car_shift_data.currently_moving && car_Main_State == ST_IDLE)
	{
		float wheel_sum = 0.0f, rpm_sum = 0.0f;
		for (int i = 0; i < 100; i++)
		{
			wheel_sum += wheel_arr[i];
			rpm_sum += rpm_arr[i];
		}
		float wheel_avg  = wheel_sum / 1000.0f;
		float rpm_avg = rpm_sum / 1000.0f;

		// TEMP DELETE
		rw_ratio_avg = rpm_avg/wheel_avg;
		for (int i = 0; i < 5; i++)
		{
			theoretical_rpm_arr[i] = wheel_avg * gear_ratios[i];
		}
		// END OF TEMP

		for (int i = 0; i < 5; i++)
		{
			float theoretical_RPM = wheel_avg * gear_ratios[i];
			if (	theoretical_RPM * (1.0 - tolerance) < rpm_avg &&
					rpm_avg  < theoretical_RPM * (1.0 + tolerance))
			{
				car_shift_data.gear_established = true;
				return (i + 1);
			}
		}

		if (read_neutral_sensor_pin())
		{
			car_shift_data.gear_established = true;
			return NEUTRAL;
		}
		// Not precise match up - could either be due to currently shifting or due to bad shift
		return ERROR_GEAR;
	}

	// If not moving then check whether or not we have established current gear. If not then
	// this is due to startup of the module and not having known position
	if (!car_shift_data.gear_established)
	{
		// If we are not moving, the only way to establish our current gear is with neutral sensor
		if (read_neutral_sensor_pin())
		{
			car_shift_data.gear_established = true;
			return NEUTRAL;
		}
	}

	// If gear is established simply return current gear. This is a dumb counter since we
	// can't correct during a failed shift due to not moving. This will be incremented and
	// decremented based on successful shifts. Default startup gear is ERROR_GEAR which will
	// be returned by this function until position has been established by either neutral
	// sensor or by starting to move. If a failed shift occurs we will return to ERROR_GEAR
	// until position can be re-established
	return car_shift_data.current_gear;




	// END TEMP REPLACEMENT
	// TODO - if trans speed broken then validate with engine RPM instead
	// If moving calculate current gear based on wheel + trans speed
//	if (car_shift_data.currently_moving && car_Main_State == ST_IDLE)
//	{
//		for (int i = 0; i < 5; i++)
//		{
//			float theoretical_trans_spd = car_shift_data.wheel_speed * gear_ratios[i] * WHEEL_TO_TRANS_RATIO;
//			if (	theoretical_trans_spd * (1.0 - tolerance) < car_shift_data.trans_speed &&
//					car_shift_data.trans_speed < theoretical_trans_spd * (1.0 + tolerance))
//			{
//				car_shift_data.gear_established = true;
//				return (i + 1);
//			}
//		}
//
//		if (read_neutral_sensor_pin())
//		{
//			car_shift_data.gear_established = true;
//			return NEUTRAL;
//		}
//		// Not precise match up - could either be due to currently shifting or due to bad shift
//		return ERROR_GEAR;
//	}
//
//	// If not moving then check whether or not we have established current gear. If not then
//	// this is due to startup of the module and not having known position
//	if (!car_shift_data.gear_established)
//	{
//		// If we are not moving, the only way to establish our current gear is with neutral sensor
//		if (read_neutral_sensor_pin())
//		{
//			car_shift_data.gear_established = true;
//			return NEUTRAL;
//		}
//	}
//
//	// If gear is established simply return current gear. This is a dumb counter since we
//	// can't correct during a failed shift due to not moving. This will be incremented and
//	// decremented based on successful shifts. Default startup gear is ERROR_GEAR which will
//	// be returned by this function until position has been established by either neutral
//	// sensor or by starting to move. If a failed shift occurs we will return to ERROR_GEAR
//	// until position can be re-established
//	return car_shift_data.current_gear;
}

// Get current RPM
uint32_t get_RPM()
{
	return rpm_ecu.data;
}

float get_clutch_pot_pos()
{
	return tcm_clutch_position.data;
}

float get_shift_pot_pos()
{
	return tcm_shifter_position.data;
}

bool clutch_open()
{
	return get_clutch_pot_pos() < CLUTCH_OPEN_POS_MM;
}

uint32_t trans_speed_arr[TRANS_ARR_SIZE] = {0};
uint32_t trans_idx = 0;
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	// TODO - ensure that this is the correct way to ensure that the timer is correct
	if (htim == &htim2)
	{
		if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			trans_speed_arr[trans_idx++] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			trans_idx = trans_idx % TRANS_ARR_SIZE;
		}
	}
}

// Return trans_speed from input capture in Hz. If no hall pulse within the last 5ms returns 0 speed
uint32_t get_trans_speed()
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

void update_wheel_arr()
{
	if (clutch_open()) return;
	static float last_wheel_val = 0;
	float current_wheel_speed = get_wheel_speed();
	if (last_wheel_val == current_wheel_speed) return;
	wheel_arr[wheel_idx++] = current_wheel_speed;
	wheel_idx = wheel_idx % 100;
}

void update_rpm_arr()
{
	if (clutch_open()) return;
	static float last_rpm_val = 0;
	if (last_rpm_val == rpm_ecu.data) return;
	rpm_arr[rpm_idx++] = rpm_ecu.data;
	rpm_idx = rpm_idx % 100;
}

float get_wheel_speed()
{
	// Handle bad data channel
	if (fabs(wsrl_ecu.data) < 0.1f && fabs(wsrr_ecu.data) > 0.1f)
	{
		return (fabs(wsrr_ecu.data) < 0.1f) ? 0 : wsrr_ecu.data;
	}
	if (fabs(wsrr_ecu.data) < 0.1f && fabs(wsrl_ecu.data) > 0.1f)
	{
		return (fabs(wsrl_ecu.data) < 0.1f) ? 0 : wsrl_ecu.data;
	}
	float rear_wheel_avg = (wsrl_ecu.data + wsrr_ecu.data)/2.0f;
	return (fabs(rear_wheel_avg) < 0.1f) ? 0 : rear_wheel_avg;
}

// Used for debugging/integration. Returns the current ratio between wheel speed/trans speed
float current_trans_wheel_ratio()
{
	float rear_wheel_avg = (wsrl_ecu.data + wsrr_ecu.data)/2.0f;
	return (fabs(rear_wheel_avg) < 1e-6f) ? -1.0f : get_trans_speed()/rear_wheel_avg;
}

float current_RPM_trans_ratio()
{
	float trans_speed = get_trans_speed();
	return (fabs(trans_speed) < 1e-6f) ? -1.0f : rpm_ecu.data/trans_speed;
}

// Returns target RPM. Returns 0 if neutral or shifting or error gear
uint32_t calc_target_RPM()
{
	// If car isn't moving then there isn't a target RPM
	if (!car_shift_data.currently_moving)
	{
		return 0;
	}
	switch (car_shift_data.target_gear)
	{
	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
	case GEAR_5:
		// TODO - this formula isn't correct update it
		// This formula holds regardless of whether or not the clutch is pressed
		return get_wheel_speed() * gear_ratios[car_shift_data.target_gear-1];
		break;

	case NEUTRAL:
	case ERROR_GEAR:
		// If we are in ERROR GEAR or shifting into neutral no target RPM
		return 0;
		break;
	}

	return 0;
}

bool validate_target_RPM()
{
	// If we are getting into ERROR_GEAR or NEUTRAL or clutch button pressed valid shift
	// Example we are rolling at 2mph and driver is holding clutch and wants to shift into
	// 5th. Should be allowed and if they drop clutch then anti stall kicks in
	if (	car_shift_data.target_gear == ERROR_GEAR 	||
			car_shift_data.target_gear == NEUTRAL 		||
			car_buttons.clutch_fast_button 				||
			car_buttons.clutch_slow_button 	)
	{
		return true;
	}
	// If target RPM not valid return false
	if (car_shift_data.target_RPM < MIN_RPM || MAX_RPM < car_shift_data.target_RPM)
	{
		car_logs.FS_Total++;
		car_logs.F_RPM++;
		return false;
	}

	return true;
}


bool calc_validate_upshift()
{
	switch (car_shift_data.current_gear)
	{
	case NEUTRAL:
		// Clutch must be pressed to go from NEUTRAL -> 1st
		if (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button)
		{
			car_shift_data.target_gear = GEAR_1;
		}
		else
		{
			return false;
		}
		break;

	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
		car_shift_data.target_gear = car_shift_data.current_gear++;
		break;

	case GEAR_5:
	case ERROR_GEAR:
		// If in ERROR_GEAR or GEAR_5 only allow shift if clutch override
		car_shift_data.target_gear = ERROR_GEAR;
		break;
	}

	car_shift_data.target_RPM = calc_target_RPM();
	return validate_target_RPM();
}

bool calc_validate_downshift()
{
	switch (car_shift_data.current_gear)
	{
	case GEAR_1:
	case GEAR_2:
	case GEAR_3:
	case GEAR_4:
	case GEAR_5:
		car_shift_data.target_gear = car_shift_data.current_gear--;
		break;

	case NEUTRAL:
	case ERROR_GEAR:
		car_shift_data.target_gear = ERROR_GEAR;
		break;
	}

	calc_target_RPM();
	return validate_target_RPM();
}


