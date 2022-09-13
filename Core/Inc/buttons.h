/*
 * buttons.h
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#ifndef INC_BUTTONS_H_
#define INC_BUTTONS_H_

#include <stdint.h>
#include "global_vars.h"

typedef struct buttons
{
	uint8_t upshift_button;
	uint8_t pending_upshift;

	uint8_t downshift_button;
	uint8_t pending_downshift;

	uint8_t clutch_fast_button;

	uint8_t clutch_slow_button;

	uint8_t aero_front_button;

	uint8_t aero_rear_button;
} buttons_t;


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN);

extern buttons_t car_buttons;

#endif /* INC_BUTTONS_H_ */
