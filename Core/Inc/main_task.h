/*
 * main_task.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_MAIN_TASK_H_
#define INC_MAIN_TASK_H_

#include <stdint.h>

typedef enum
{
	ST_IDLE,
	ST_HDL_UPSHIFT,
	ST_HDL_DOWNSHIFT
} Main_States_t;

typedef enum
{
	ST_U_BEGIN_SHIFT,
	ST_U_LOAD_SHIFT_LVR,
	ST_U_EXIT_GEAR,
	ST_U_SPARK_RETURN,
	ST_U_ENTER_GEAR,
	ST_U_FINISH_SHIFT,
} Upshift_States_t;

typedef enum
{
	ST_D_BEGIN_SHIFT,
	ST_D_LOAD_SHIFT_LVR,
	ST_D_EXIT_GEAR,
	ST_D_ENTER_GEAR,
	ST_D_FINISH_SHIFT,
	ST_D_HOLD_CLUTCH
} Downshift_States_t;

typedef struct
{
	// Total shifts
	uint32_t TOTAL_SHIFTS;

	// Failed Clutch Disengagement
	uint32_t F_CLUTCH_OPEN;

	// Total number of failed shifts
	uint32_t FS_Total;

	// Number of shifts that can't be executed due to RPM
	uint32_t F_RPM;

	// Number of failed exits and enters
	uint32_t F_D_EXIT;
	uint32_t F_D_ENTER;

	// Number of failed gear exits upshift
	uint32_t F_U_EXIT_NO_CLUTCH;
	uint32_t F_U_EXIT_NO_CLUTCH_AND_SPARK_RETURN;
	uint32_t F_U_EXIT_WITH_CLUTCH;

	// Number of failed gear enters upshift
	uint32_t F_U_ENTER_NO_CLUTCH;
	uint32_t F_U_ENTER_WITH_CLUTCH;

	uint32_t F_RETURN_LEVER;
} logs_t;


int init_main_task(void);
int main_task(void);

#define HEARTBEAT_LED_TIME_ms 500
#define DISPLAY_UPDATE_TIME_ms 100
#define GEAR_UPDATE_TIME_ms 25


#endif /* INC_MAIN_TASK_H_ */
