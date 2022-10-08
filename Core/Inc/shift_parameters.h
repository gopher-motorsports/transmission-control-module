/*
 * shift_parameters.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_SHIFT_PARAMETERS_H_
#define INC_SHIFT_PARAMETERS_H_

#define TIME_BASED_SHIFTING_ONLY 0

#define SHIFT_LEVER_PRELOAD_TIME_MS 50

#define LEVER_NEUTRAL_POS_MM 37.0f

#define LEVER_NEUTRAL_TOLERANCE 0.8


#define SHIFTING_TIMEOUT_MS 100

#define UPSHIFT_EXIT_TIMEOUT_MS 85

#define UPSHIFT_ENTER_TIMEOUT_MS 25

#define UPSHIFT_EXIT_POS_MM 42.8f

#define UPSHIFT_ENTER_POS_MM 43.5f


#define DOWNSHIFT_EXIT_TIMEOUT_MS 50

#define DOWNSHIFT_ENTER_TIMEOUT_MS 100

#define DOWNSHIFT_EXIT_POS_MM 30.0f

#define DOWNSHIFT_ENTER_POS_MM 28.5f


#define TARGET_RPM_TOLERANCE 0.1f

#define CLUTCH_OPEN_POS_MM 27.0f

#define CLUTCH_SLOW_DROP_FAST_TO_SLOW_EXTRA_MM 3.0f

#define CLUTCH_OPEN_TIMEOUT_MS 300

#define SPARK_RETURN_MS 10

#define FAILED_ENTER_CLUTCH_CLOSE_TIMEOUT_MS 500



#endif /* INC_SHIFT_PARAMETERS_H_ */
