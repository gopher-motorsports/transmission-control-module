/*
 * main_task.c
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */
#include "main_task.h"
#include "DAM.h"
#include "acm.h"
#include "car_utils.h"
#include "display.h"
#include "shift_parameters.h"

static void run_upshift_sm(void);
static void run_downshift_sm(void);

extern CAN_HandleTypeDef hcan1;

static Main_States_t car_Main_State = ST_IDLE;
static Upshift_States_t car_Upshift_State;
static Downshift_States_t car_Downshift_State;

int init_main_task(void)
{
	init_can(&hcan1, TCM_ID, BXTYPE_MASTER);
	acm_init();

	return 0;
}


int main_task(void)
{
	// Heartbeat LED
	static uint32_t last_heartbeat = 0;
	if (HAL_GetTick() - last_heartbeat >= HEARTBEAT_LED_TIME_ms)
	{
		last_heartbeat = HAL_GetTick();
		HAL_GPIO_TogglePin(HEARTBEAT_GPIO_Port, HEARTBEAT_Pin);
	}

	// update all the internal values for logging
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

	// acm go brrrrr
	run_acm();

	// Update shift struct with relevant data. This is to latch all of the
	// important data for this go around the state machine
	update_car_shift_struct();

	// gear calculation handling
	update_wheel_arr();
	update_rpm_arr();

	// only check the gear every 25ms
	static uint32_t last_gear_update = 0;
	if (HAL_GetTick() - last_gear_update >= GEAR_UPDATE_TIME_ms)
	{
		last_gear_update = HAL_GetTick();
		car_shift_data.current_gear = get_current_gear(car_Main_State);
	}

	// Anti Stall
	//anti_stall(); // disabling anti-stall for now (mike says we dont really need it)

	// Update Display
	static uint32_t last_display_update = 0;
	if (HAL_GetTick() - last_display_update >= DISPLAY_UPDATE_TIME_ms)
	{
		last_display_update = HAL_GetTick();
		send_display_data();
	}

	// handle the clutch based on the state of the car and the buttons
	clutch_task(&car_buttons, car_Main_State, car_shift_data.anti_stall);

	switch (car_Main_State)
	{
	case ST_IDLE:
		// in idle state, make sure we are not spark cutting and not pushing
		// the solenoid
		set_upshift_solenoid(SOLENOID_OFF);
		set_downshift_solenoid(SOLENOID_OFF);
		set_spark_cut(false);

		// start a downshift if there is one pending. This means that a new
		// shift can be queued during the last shift
		if (car_buttons.pending_downshift)
		{
			car_buttons.pending_downshift = 0;
			if (calc_validate_downshift(car_shift_data.current_gear, &car_buttons))
			{
				car_Main_State = ST_HDL_DOWNSHIFT;
				car_Downshift_State = ST_D_BEGIN_SHIFT;
			}
		}

		// same for upshift. Another shift can be queued
		if (car_buttons.pending_upshift)
		{
			car_buttons.pending_upshift = 0;
			if (calc_validate_upshift(car_shift_data.current_gear, &car_buttons))
			{
				car_Main_State = ST_HDL_UPSHIFT;
				car_Upshift_State = ST_U_BEGIN_SHIFT;
			}
		}
		break;

	case ST_HDL_UPSHIFT:
		run_upshift_sm();
		break;

	case ST_HDL_DOWNSHIFT:
		run_downshift_sm();
		break;
	}

	return 0;
}

void run_upshift_sm(void)
{
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_exit_gear_spark_return_tick;
	static uint32_t finish_shift_start_tick;
	static uint32_t spark_cut_start_time;

	// calculate the target RPM at the start of each
	car_shift_data.target_RPM = calc_target_RPM(car_shift_data.target_gear);

	switch (car_Upshift_State)
	{
	case ST_U_BEGIN_SHIFT:
		// at the beginning of the upshift, start pushing on the shift lever
		// to "preload". This means that there will be force on the shift
		// lever when the spark is cut and the gear can immediately disengage

		begin_shift_tick = HAL_GetTick(); // Log that first begin shift tick
		set_upshift_solenoid(SOLENOID_ON); // start pushing upshift

		// if the clutch button is pressed during the start of the shift,
		// keep holding the clutch for the entire shift.
		car_shift_data.using_clutch = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);
		car_shift_data.clutch_override = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);

		// reset information about the shift
		car_shift_data.failed_enter_gear = false;
		car_shift_data.successful_shift = true;

		// move on to waiting for the "preload" time to end
		car_Upshift_State = ST_U_LOAD_SHIFT_LVR;
		break;

	case ST_U_LOAD_SHIFT_LVR:
		// wait for the preload time to be over
		if (HAL_GetTick() - begin_shift_tick > UPSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS)
		{
			// preload time is over. Start spark cutting to disengage the
			// gears and moving the RPM to match up with the next gearS
			spark_cut_start_time = HAL_GetTick(); // Begin timer for making sure we shift for a minimum amount of time
			begin_exit_gear_tick = HAL_GetTick();
			check_and_spark_cut(true);

			// move on to waiting to exit gear
			car_Upshift_State = ST_U_EXIT_GEAR;
		}
		break;

	case ST_U_EXIT_GEAR:
		// wait until the threshold for the lever leaving the last gear has
		// been met or the timeout hits. During this time spark should be cut
		// to change engine driving the wheels to the wheels driving the
		// engine. If all goes well, the gear should be left at that midpoint

		// right now just leave spark cut on for this section of code instead
		// of trying to actively rev match. Rev matching should be cutting
		// spark anyway as we have not left the lower gear yet
		//reach_target_RPM_spark_cut();
		check_and_spark_cut(true);

		// wait for the shift lever to reach the leaving threshold
		if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM)
		{
			// the shifter position is above the threshold to exit. The
			// transmission is now in a false neutral position
			car_Upshift_State = ST_U_ENTER_GEAR;
			begin_enter_gear_tick = HAL_GetTick();
			break;
		}

		// the shift lever has not moved far enough. Check if it has been
		// long enough to timeout yet
		if (HAL_GetTick() - begin_exit_gear_tick > UPSHIFT_EXIT_TIMEOUT_MS)
		{
			// the shift lever did not exit the previous gear. Attempt to
			// return spark to attempt to disengage
			begin_exit_gear_spark_return_tick = HAL_GetTick();
			check_and_spark_cut(false);
			car_Upshift_State = ST_U_SPARK_RETURN;
		}
		break;

	case ST_U_SPARK_RETURN:
		// A common cause of a failed disengagement is the transition from
		// the engine driving the wheels to the wheels driving the engine
		// too quickly, meaning there is no way to exit the gear while
		// the spark is cut. In order to leave the gear we must return spark
		// briefly
		check_and_spark_cut(false);

		// the shifter has moved above the threshold of exiting the gear. Spark
		// must be cut again to reach the correct RPM for the next gear. If
		// enough time has passed return spark anyway
		if (get_shift_pot_pos() > UPSHIFT_EXIT_POS_MM ||
				HAL_GetTick() - begin_exit_gear_spark_return_tick > UPSHIFT_EXIT_SPARK_RETURN_MS)
		{
			// If spark return successfully releases then continue onto the
			// next phase of the shift. If it was not successful (timeout) move
			// on anyway
			check_and_spark_cut(true);
			car_Upshift_State = ST_U_ENTER_GEAR;
			begin_enter_gear_tick = HAL_GetTick();
		}
		break;

	case ST_U_ENTER_GEAR:
		// we are in a false neutral and waiting to enter the gear. Likely
		// the RPMs will need to drop a little more to make it to the next
		// gear

		// right now the code is always spark cutting. This section it is
		// less desirable to always spark cut because the revs may drop below
		// what they need to be, but it is ok as they will come back up
		// when this section is over
		//reach_target_RPM_spark_cut();
		check_and_spark_cut(true);

		// check if the shifter position is above the threshold to complete
		// a shift
		if (get_shift_pot_pos() > UPSHIFT_ENTER_POS_MM)
		{
			// shift position says we are done shifting
			car_Upshift_State = ST_U_FINISH_SHIFT;
			finish_shift_start_tick = HAL_GetTick();
			break;
		}

		// check if we are out of time for this state
		if (HAL_GetTick() - begin_enter_gear_tick > UPSHIFT_ENTER_TIMEOUT_MS)
		{
			// at this point the shift was probably not successful. Note this so we
			// dont increment the gears and move on
			car_shift_data.successful_shift = false;
			car_Upshift_State = ST_U_FINISH_SHIFT;
			finish_shift_start_tick = HAL_GetTick();
		}
		break;

	case ST_U_FINISH_SHIFT:
		// wrapping up the upshift. First make sure we have been been cutting
		// spark for long enough (to prevent a case where the shifter position
		// is stuck in the success region from stopping shifting), then work
		// on ending the shift while still pushing on the shift lever

		// make sure enough time has passed from the beginning of the shift
		if (HAL_GetTick() - spark_cut_start_time < UPSHIFT_MIN_TIME)
		{
			// not enough time has passed. Keep spark cutting for a bit longer
			check_and_spark_cut(true);
			break;
		}
		else
		{
			// enough time has passed to finish the shift. Call the shift
			// over but keep pushing as there is no downside other than the
			// extra time the shift lever will take to return, preventing from
			// starting another shift
			check_and_spark_cut(false);
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

			// check if we can disable the solenoid and return to to idle
			if (HAL_GetTick() - finish_shift_start_tick >= UPSHIFT_EXTRA_PUSH_TIME)
			{
				// done with the upshift state machine
				set_upshift_solenoid(SOLENOID_OFF);
				car_Main_State = ST_IDLE;
			}
		}
		break;
	}
}


// TODO still need to look into this in depth
void run_downshift_sm(void)
{
	static uint32_t begin_shift_tick;
	static uint32_t begin_exit_gear_tick;
	static uint32_t begin_enter_gear_tick;
	static uint32_t begin_hold_clutch_tick;
	static uint32_t shifting_timeout_timer; // TODO rename

	car_shift_data.target_RPM = calc_target_RPM(car_shift_data.target_gear);

	switch (car_Downshift_State)
	{
	case ST_D_BEGIN_SHIFT:
		begin_shift_tick = HAL_GetTick();
		set_downshift_solenoid(SOLENOID_ON);
		set_clutch_solenoid(SOLENOID_ON);
		car_shift_data.using_clutch = true;
		car_shift_data.clutch_override = (car_buttons.clutch_fast_button || car_buttons.clutch_slow_button);
		car_shift_data.failed_enter_gear = false;
		car_shift_data.throttle_blip = false;
		car_shift_data.successful_shift = true;
		car_Downshift_State = ST_D_LOAD_SHIFT_LVR;
		break;

	case ST_D_LOAD_SHIFT_LVR:
		if ((HAL_GetTick() - begin_shift_tick > DOWNSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS))
		{
			shifting_timeout_timer = HAL_GetTick();
			begin_exit_gear_tick = HAL_GetTick();
			car_Downshift_State = ST_D_EXIT_GEAR;

		}
		// Note: If sensor bad and we don't detect clutch opening it can still
		// theoretically shift as we are pushing on both the downshift and clutch
		// solenoid. The driver would have to blip manually
		if ( HAL_GetTick() - begin_shift_tick > CLUTCH_OPEN_TIMEOUT_MS)
		{
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
			car_shift_data.successful_shift = false;
			car_Downshift_State = ST_D_FINISH_SHIFT;
		}
		break;

	case ST_D_ENTER_GEAR:
		reach_target_RPM_spark_cut(car_shift_data.target_RPM);
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
			car_shift_data.successful_shift = false;
			car_Downshift_State = ST_D_FINISH_SHIFT;
			car_shift_data.failed_enter_gear = true;
		}
		break;

	case ST_D_FINISH_SHIFT:
		// winding down the downshift.
		if (HAL_GetTick() - shifting_timeout_timer > DOWNSHIFT_MIN_TIME)
		{
			// set_clutch verifies that clutch button not depressed
			set_clutch_solenoid(car_shift_data.failed_enter_gear ? SOLENOID_ON : SOLENOID_OFF);
			set_downshift_solenoid(SOLENOID_OFF);
			set_upshift_solenoid(SOLENOID_OFF);
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

