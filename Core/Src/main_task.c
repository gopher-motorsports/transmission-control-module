/*
 * main_task.c
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */
#include "global_vars.h"
#include "main_task.h"
#include "DAM.h"
//#include "car_utils.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

extern bool using_slow_drop;

int init_main_task()
{
	init_can(&hcan1, TCM_ID, BXTYPE_MASTER);

	return 0;
}

// TEMP DELETE
float trans_wheel_ratio, RPM_trans_ratio, temp_ws, temp_ts;
float trans_wheel_ratio_arr[5000];
uint32_t tw_arr_idx = 0;
float RPM_trans_ratio_arr[5000];
uint32_t rt_arr_idx = 0;
float rt_avg, tw_avg, rw_ratio;

// DO NOT DELETE
int main_task()
{
	update_and_queue_param_u8(&sw_upshift, car_buttons.upshift_button);
	update_and_queue_param_u8(&sw_downshift, car_buttons.downshift_button);
	update_and_queue_param_u8(&sw_clutch_fast, car_buttons.clutch_fast_button);
	update_and_queue_param_u8(&sw_clutch_slow, car_buttons.clutch_slow_button);
	update_and_queue_param_u8(&sw_aero_front, car_buttons.aero_front_button);
	update_and_queue_param_u8(&sw_aero_rear, car_buttons.aero_rear_button);
	update_and_queue_param_u8(&tcm_neutral, (uint8_t)read_neutral_sensor_pin());

	update_and_queue_param_u32(&tcm_target_rpm, car_shift_data.target_RPM);
	update_and_queue_param_u8(&tcm_current_gear, car_shift_data.current_gear);
	update_and_queue_param_u8(&tcm_target_gear, car_shift_data.target_gear);
	update_and_queue_param_u8(&tcm_currently_moving, car_shift_data.currently_moving);
	update_and_queue_param_u8(&tcm_successful_shift, car_shift_data.successful_shift);
	update_and_queue_param_u32(&tcm_trans_rpm, car_shift_data.trans_speed);
	update_and_queue_param_u8(&tcm_throttle_blip, car_shift_data.throttle_blip);
	update_and_queue_param_u8(&tcm_using_clutch, car_shift_data.using_clutch);
	update_and_queue_param_u8(&tcm_anti_stall, car_shift_data.anti_stall);

	// check for the lap timer signal
	update_and_queue_param_u8(&tcm_lap_timer, !HAL_GPIO_ReadPin(LAP_TIM_9_GPIO_Port, LAP_TIM_9_Pin));

	// Update shift struct with relevant data
	update_car_shift_struct();

	// Validate that if we are blipping that the clutch is open
	validate_throttle_blip();

	// Heartbeat LED
	static uint32_t last_heartbeat;
	if (HAL_GetTick() - last_heartbeat > 500)
	{
		last_heartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
	}

	// Anti Stall
	anti_stall();

	// Update Display
	static uint32_t last_display_update;
	if (HAL_GetTick() - last_display_update > 100)
	{
		last_display_update = HAL_GetTick();
		send_display_data();
	}


	// Open clutch if requested by driver always and close clutch if button not pressed and IDLE
	clutch_task();

	switch (car_Main_State)
	{
	case ST_IDLE:
		set_all_solenoids(SOLENOID_OFF);
		spark_cut(false);
		if (car_buttons.pending_upshift)
		{
			car_buttons.pending_upshift = 0;
			if (calc_validate_upshift())
			{
				using_slow_drop = car_buttons.clutch_slow_button;
				car_Main_State = ST_HDL_UPSHIFT;
				car_Upshift_State = ST_U_BEGIN_SHIFT;
			}
			return 0;
		}
		if (car_buttons.pending_downshift)
		{
			car_buttons.pending_downshift = 0;
			if (calc_validate_downshift())
			{
				using_slow_drop = car_buttons.clutch_slow_button;
				car_Main_State = ST_HDL_DOWNSHIFT;
				car_Downshift_State = ST_D_BEGIN_SHIFT;
			}
			return 0;
		}
		break;
	case ST_HDL_UPSHIFT:
		car_shift_data.target_RPM = calc_target_RPM();
		run_upshift_sm();
		break;
	case ST_HDL_DOWNSHIFT:
		car_shift_data.target_RPM = calc_target_RPM();
		run_downshift_sm();
		break;
	}

	trans_wheel_ratio = current_trans_wheel_ratio();
	RPM_trans_ratio = current_RPM_trans_ratio();
	temp_ts = get_trans_speed();
	temp_ws = get_wheel_speed();
	trans_wheel_ratio_arr[tw_arr_idx++] = trans_wheel_ratio;
	RPM_trans_ratio_arr[rt_arr_idx++] = RPM_trans_ratio;
	if (tw_arr_idx == 5000)
	{
		float tw_sum = 0.0f, rt_sum = 0.0f;
		for (int i = 0; i < 5000; i++)
		{
			tw_sum += trans_wheel_ratio_arr[i];
			rt_sum += RPM_trans_ratio_arr[i];
		}
		tw_avg = tw_sum/5000;
		rt_avg = rt_sum/5000;


	}
	tw_arr_idx = tw_arr_idx % 5000;
	rt_arr_idx = rt_arr_idx % 5000;

	update_wheel_arr();
	update_rpm_arr();

	return 0;
}

void run_upshift_sm()
{
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	switch (car_Upshift_State)
	{
	case ST_U_BEGIN_SHIFT:
		begin_shift_tick = HAL_GetTick();
		set_upshift_solenoid(SOLENOID_ON);
		car_logs.TOTAL_SHIFTS++;
		car_shift_data.using_clutch = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);
		car_shift_data.clutch_override = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);
		car_shift_data.failed_enter_gear = false;
		car_shift_data.successful_shift = true;
		car_Upshift_State = ST_U_LOAD_SHIFT_LVR;
		break;

	case ST_U_LOAD_SHIFT_LVR:
		if (HAL_GetTick() - begin_shift_tick > SHIFT_LEVER_PRELOAD_TIME_MS)
		{
			spark_cut(true);
			begin_exit_gear_tick = HAL_GetTick();
			car_Upshift_State = ST_U_EXIT_GEAR;
		}
		break;

	case ST_U_EXIT_GEAR:
		// Reach target RPM ensures that our target gear is not neutral and that clutch isn't used
		reach_target_RPM_spark_cut();
#if (!TIME_BASED_SHIFTING_ONLY)
		if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM)
		{
			if (car_shift_data.clutch_override && !(HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_TIMEOUT_MS - 10))
			{
				break;
			}
			car_Upshift_State = ST_U_ENTER_GEAR;
			begin_enter_gear_tick = HAL_GetTick();
		}
#endif
		if (HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_TIMEOUT_MS)
		{
			if (!car_shift_data.using_clutch)
			{
				// Try returning spark for 10ms
				begin_exit_gear_tick = HAL_GetTick();
				spark_cut(false);
				car_logs.F_U_EXIT_NO_CLUTCH++;
				car_Upshift_State = ST_U_SPARK_RETURN;

			}
			else
			{
				// FAILED with clutch. Stop shift
				car_logs.FS_Total++;
				car_logs.F_U_EXIT_WITH_CLUTCH++;
				car_shift_data.successful_shift = false;
				car_Upshift_State = ST_U_FINISH_SHIFT;
			}
		}
		break;

	case ST_U_SPARK_RETURN:
		if (HAL_GetTick() - begin_exit_gear_tick > SPARK_RETURN_MS)
		{
			// Spark Return didn't release gear. Start using clutch
			spark_cut(false);
			set_clutch_solenoid(SOLENOID_ON);
			begin_exit_gear_tick = HAL_GetTick();
			car_shift_data.using_clutch = true;
			car_logs.F_U_EXIT_NO_CLUTCH_AND_SPARK_RETURN++;
			car_Upshift_State = ST_U_EXIT_GEAR;
		}
#if (!TIME_BASED_SHIFTING_ONLY)
		if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM)
		{
			// If spark return successfully releases then continue
			spark_cut(true);
			car_Upshift_State = ST_U_ENTER_GEAR;
			begin_enter_gear_tick = HAL_GetTick();
		}
#endif
		break;

	case ST_U_ENTER_GEAR:
		reach_target_RPM_spark_cut();
#if (!TIME_BASED_SHIFTING_ONLY)
		if (get_shift_pot_pos() > UPSHIFT_ENTER_POS_MM)
		{
			if (car_shift_data.clutch_override && !(HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_TIMEOUT_MS - 10))
			{
				break;
			}
			car_Upshift_State = ST_U_FINISH_SHIFT;
		}
#endif
		if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_TIMEOUT_MS)
		{
			if (car_shift_data.using_clutch)
			{
				// FAILED with clutch. Stop shift
				car_logs.FS_Total++;
				car_logs.F_U_ENTER_WITH_CLUTCH++;
				car_shift_data.successful_shift = false;
				car_Upshift_State = ST_U_FINISH_SHIFT;
			}
			else
			{
				// Retry with clutch
				car_shift_data.using_clutch = true;
				begin_enter_gear_tick = HAL_GetTick();
				car_logs.F_U_ENTER_NO_CLUTCH++;
			}
		}
		break;

	case ST_U_FINISH_SHIFT:
		// set_clutch verifies that clutch button not depressed
		set_clutch_solenoid(SOLENOID_OFF);
		set_upshift_solenoid(SOLENOID_OFF);
		set_downshift_solenoid(SOLENOID_OFF);
		spark_cut(false);
		car_shift_data.using_clutch = false;
		if (car_shift_data.successful_shift)
		{
			car_shift_data.current_gear = car_shift_data.target_gear;
		}
		else
		{
			car_shift_data.current_gear = ERROR_GEAR;
			car_shift_data.gear_established = false;
		}
		car_Main_State = ST_IDLE;
		break;
	}
}



void run_downshift_sm()
{
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_hold_clutch_tick;
	switch (car_Downshift_State)
	{
	case ST_D_BEGIN_SHIFT:
		begin_shift_tick = HAL_GetTick();
		set_downshift_solenoid(SOLENOID_ON);
		set_clutch_solenoid(SOLENOID_ON);
		car_logs.TOTAL_SHIFTS++;
		car_shift_data.using_clutch = true;
		car_shift_data.clutch_override = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);
		car_shift_data.failed_enter_gear = false;
		car_shift_data.throttle_blip = false;
		car_shift_data.successful_shift = true;
		car_Downshift_State = ST_D_LOAD_SHIFT_LVR;
		break;

	case ST_D_LOAD_SHIFT_LVR:
		if (	(HAL_GetTick() - begin_shift_tick > SHIFT_LEVER_PRELOAD_TIME_MS) )//&&
				//(get_clutch_pot_pos() < CLUTCH_OPEN_POS_MM) )
		{
			throttle_blip(true);
			begin_exit_gear_tick = HAL_GetTick();
			car_Downshift_State = ST_D_EXIT_GEAR;

		}
		// Note: If sensor bad and we don't detect clutch opening it can still
		// theoretically shift as we are pushing on both the downshift and clutch
		// solenoid. The driver would have to blip manually
		if ( HAL_GetTick() - begin_shift_tick > CLUTCH_OPEN_TIMEOUT_MS)
		{
			car_logs.F_CLUTCH_OPEN++;
			car_shift_data.successful_shift = false;
			car_Downshift_State = ST_D_FINISH_SHIFT;
		}
		break;

	case ST_D_EXIT_GEAR:
#if (!TIME_BASED_SHIFTING_ONLY)
		if (get_shift_pot_pos() < DOWNSHIFT_EXIT_POS_MM)
		{
			if (car_shift_data.clutch_override && !(HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_TIMEOUT_MS - 10))
			{
				break;
			}
			car_Downshift_State = ST_D_ENTER_GEAR;
			begin_enter_gear_tick = HAL_GetTick();
		}
#endif
		if (HAL_GetTick() - begin_exit_gear_tick > DOWNSHIFT_EXIT_TIMEOUT_MS)
		{
			// FAILED with clutch. Stop shift
			car_logs.FS_Total++;
			car_logs.F_D_EXIT++;
			car_shift_data.successful_shift = false;
			car_Downshift_State = ST_D_FINISH_SHIFT;
		}
		break;

	case ST_D_ENTER_GEAR:
		reach_target_RPM_spark_cut();
#if (!TIME_BASED_SHIFTING_ONLY)
		if (get_shift_pot_pos() < DOWNSHIFT_ENTER_POS_MM)
		{
			if (car_shift_data.clutch_override && !(HAL_GetTick() - begin_enter_gear_tick > DOWNSHIFT_ENTER_TIMEOUT_MS - 10))
			{
				break;
			}
			car_Downshift_State = ST_D_FINISH_SHIFT;
		}
#endif
		if (HAL_GetTick() - begin_enter_gear_tick > DOWNSHIFT_ENTER_TIMEOUT_MS)
		{
			car_logs.FS_Total++;
			car_logs.F_D_ENTER++;
			car_shift_data.successful_shift = false;
			car_Downshift_State = ST_D_FINISH_SHIFT;
			car_shift_data.failed_enter_gear = true;
		}
		break;

	case ST_D_FINISH_SHIFT:
		// set_clutch verifies that clutch button not depressed
		set_clutch_solenoid(car_shift_data.failed_enter_gear ? SOLENOID_ON : SOLENOID_OFF);
		set_downshift_solenoid(SOLENOID_OFF);
		set_upshift_solenoid(SOLENOID_OFF);
		throttle_blip(false);
		car_shift_data.using_clutch = false;
		begin_hold_clutch_tick = HAL_GetTick();
		if (car_shift_data.failed_enter_gear)
		{
			car_shift_data.current_gear = ERROR_GEAR;
			car_shift_data.gear_established = false;
			car_Downshift_State = ST_D_HOLD_CLUTCH;
		}
		else
		{
			car_Main_State = ST_IDLE;
		}
		break;

	case ST_D_HOLD_CLUTCH:
		if (HAL_GetTick() - begin_hold_clutch_tick > FAILED_ENTER_CLUTCH_CLOSE_TIMEOUT_MS)
		{
			set_clutch_solenoid(SOLENOID_OFF);
			car_shift_data.current_gear = ERROR_GEAR;
			car_shift_data.gear_established = false;
			car_Main_State = ST_IDLE;
		}
		break;
	}
}

