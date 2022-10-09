/*
 * state_machines.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_STATE_MACHINES_H_
#define INC_STATE_MACHINES_H_

#include "global_vars.h"

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
	//ST_U_SPARK_RETURN,
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


extern Main_States_t car_Main_State;
extern Upshift_States_t car_Upshift_State;
extern Downshift_States_t car_Downshift_State;

#endif /* INC_STATE_MACHINES_H_ */
