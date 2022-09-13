/*
 * logs.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_LOGS_H_
#define INC_LOGS_H_

#include <stdint.h>

typedef struct logs
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


extern logs_t car_logs;

#endif /* INC_LOGS_H_ */
