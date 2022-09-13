/*
 * state_machines.c
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */
#include "state_machines.h"

Main_States_t car_Main_State = ST_IDLE;
Upshift_States_t car_Upshift_State;
Downshift_States_t car_Downshift_State;
