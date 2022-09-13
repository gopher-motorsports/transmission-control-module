/*
 * car_utils.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_CAR_UTILS_H_
#define INC_CAR_UTILS_H_


#include "global_vars.h"
#include <stdbool.h>

typedef enum
{
	NEUTRAL,
	GEAR_1,
	GEAR_2,
	GEAR_3,
	GEAR_4,
	GEAR_5,
	ERROR_GEAR
} gear_t;

typedef struct shift_struct
{
	uint32_t current_RPM;
	uint32_t target_RPM;

	uint32_t trans_speed;

	float wheel_speed;

	gear_t current_gear;
	gear_t target_gear;

	// Is the car moving?
	bool currently_moving;
	// Gear established - Used for determining gear upon startup
	bool gear_established;
	// Are we currently throttle blipping?
	bool throttle_blip;
	// Are we using the clutch for this shift?
	bool using_clutch;
	// Did we fail to enter target gear?
	bool failed_enter_gear;
	// Has the shift been successful
	bool successful_shift;
	// Anti Stall
	bool anti_stall;
	// Using clutch override
	bool clutch_override;
} shift_struct_t;


typedef enum
{
	SOLENOID_OFF,
	SOLENOID_ON

} solenoid_position_t;

void update_car_shift_struct();

void clutch_task();

void set_slow_drop(bool state);

void set_clutch_solenoid(solenoid_position_t position);

void set_upshift_solenoid(solenoid_position_t position);

void set_downshift_solenoid(solenoid_position_t position);

void set_all_solenoids(solenoid_position_t position);

void throttle_blip(bool state);

void validate_throttle_blip();

void spark_cut(bool state);

void reach_target_RPM_spark_cut();

void anti_stall();

uint32_t read_neutral_sensor_pin();

gear_t get_current_gear();

uint32_t get_RPM();

uint32_t get_target_RPM();

void update_wheel_arr();

void update_rpm_arr();

float get_clutch_pot_pos();

float get_shift_pot_pos();

bool clutch_open();

uint32_t get_trans_speed();

float get_wheel_speed();

float current_trans_wheel_ratio();

float current_RPM_trans_ratio();


uint32_t calc_target_RPM();

bool validate_target_RPM();

bool calc_validate_upshift();

bool calc_validate_downshift();


extern gear_t car_gear;
extern shift_struct_t car_shift_data;




#endif /* INC_CAR_UTILS_H_ */
