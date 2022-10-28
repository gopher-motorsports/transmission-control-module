/*
 * shift_parameters.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_SHIFT_PARAMETERS_H_
#define INC_SHIFT_PARAMETERS_H_

// shifting defines for both upshift and downshift
#define LEVER_NEUTRAL_POS_MM 37.0f
#define LEVER_NEUTRAL_TOLERANCE 0.8
#define TARGET_RPM_TOLERANCE 0.03f

// upshift defines
#define UPSHIFT_MIN_TIME 70 // for an upshift this does not include the preload time. This is used if the shift position sensor is wrong and the state machine goes through each state too quickly. If it is wrong in the other direction the timeouts for each state will be used
#define UPSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS 70 // long preload to get lots of load on the shift lever
#define UPSHIFT_EXIT_TIMEOUT_MS 25 // short time to exit. If the shift lever wasn't pushed far enough quickly it is probably stuck
#define UPSHIFT_EXIT_SPARK_RETURN_MS 20 // we dont want the spark return to be too long
#define UPSHIFT_ENTER_TIMEOUT_MS 30 // data shows this does not take very long, so have a pretty short timeout here.
#define UPSHIFT_EXTRA_PUSH_TIME 50
#define UPSHIFT_EXIT_POS_MM 42.8f
#define UPSHIFT_ENTER_POS_MM 47.3f

// downshift defines
#define DOWNSHIFT_MIN_TIME 70
#define DOWNSHIFT_SHIFT_LEVER_PRELOAD_TIME_MS 50
#define DOWNSHIFT_EXIT_TIMEOUT_MS 75
#define DOWNSHIFT_ENTER_TIMEOUT_MS 100
#define FAILED_ENTER_CLUTCH_CLOSE_TIMEOUT_MS 200
#define DOWNSHIFT_EXTRA_PUSH_TIME 50 // TODO add this
#define DOWNSHIFT_EXIT_POS_MM 34.0f
#define DOWNSHIFT_ENTER_POS_MM 31.0f

// clutch defines
#define CLUTCH_OPEN_POS_MM 27.0f
#define CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM 3.0f
#define CLUTCH_OPEN_TIMEOUT_MS 300

// transmission, wheel speed, and RPM array defines
#define RPM_ARRAY_SIZE 250
#define WHEEL_SPEED_ARRAY_SIZE RPM_ARRAY_SIZE
#define TRANS_ARR_SIZE 100
#define TRANS_TO_ENGINE_RATIO 7.5f
#define WHEEL_TO_TRANS_RATIO 20.0f

// gear establishing and calculating defines
#define DEFAULT_WHEEL_SPEED_AVE_TIME_ms 25
#define GEAR_NOT_ESTABLISHED_NUM_SAMPLES_ms 50
#define GEAR_ESTABLISHED_NUM_SAMPLES_ms RPM_ARRAY_SIZE
#define GEAR_ESTABLISH_TOLERANCE_percent 0.03f
#define NUM_OF_GEARS 5
#define GEAR_1_WHEEL_RATIO 141.7f
#define GEAR_2_WHEEL_RATIO 112.9f
#define GEAR_3_WHEEL_RATIO 97.1f
#define GEAR_4_WHEEL_RATIO 86.6f
#define GEAR_5_WHEEL_RATIO 78.7f

// RPM cutoffs
#define MAX_RPM 14000
#define MIN_SPARK_CUT_RPM 3000
#define ANTI_STALL_RPM 1700
#define MAX_CRANK_RPM 700

// misc defines
#define MOVING_WHEEL_SPEED_MIN_CUTOFF 2.0f
#define BUTTON_DEBOUNCE_MS 20

#endif /* INC_SHIFT_PARAMETERS_H_ */
