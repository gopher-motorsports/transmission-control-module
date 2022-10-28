/*
 * buttons.c
 *
 *  Created on: Apr 24, 2022
 *      Author: sebas
 */

#include "buttons.h"
#include "main.h"
#include "shift_parameters.h"

buttons_t car_buttons;

uint32_t last_upshift, last_downshift;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN)
{
	// If it doesn't belong to the absolute encoder check other inputs
	switch (GPIO_PIN)
	{
	case CLUTCH_SLOW_DROP_Pin:
		car_buttons.clutch_slow_button = !HAL_GPIO_ReadPin(CLUTCH_SLOW_BTN_GPIO_Port, CLUTCH_SLOW_BTN_Pin);
		break;

	case CLUTCH_FAST_BTN_Pin:
			car_buttons.clutch_fast_button = HAL_GPIO_ReadPin(CLUTCH_FAST_BTN_GPIO_Port, CLUTCH_FAST_BTN_Pin);
			break;

	case UPSHIFT_BTN_Pin:
		car_buttons.upshift_button = !HAL_GPIO_ReadPin(UPSHIFT_BTN_GPIO_Port, UPSHIFT_BTN_Pin);
		// Set flag on edge with debounce
		if (HAL_GetTick() - last_upshift > BUTTON_DEBOUNCE_MS && car_buttons.upshift_button)
		{
			last_upshift = HAL_GetTick();
			car_buttons.pending_upshift = 1;
		}
		break;

	case DOWNSHIFT_BTN_Pin:
		car_buttons.downshift_button = !HAL_GPIO_ReadPin(DOWNSHIFT_BTN_GPIO_Port, DOWNSHIFT_BTN_Pin);
		// Set flag on edge with debounce
		if (HAL_GetTick() - last_downshift > BUTTON_DEBOUNCE_MS && car_buttons.downshift_button)
		{
			last_downshift = HAL_GetTick();
			car_buttons.pending_downshift = 1;
		}
		break;



	case AERO_FRONT_BTN_Pin:
		// This specific button is normally closed as opposed to normally open like the other buttons.
		// Thus we do not need to invert the GPIO read
		car_buttons.aero_front_button = 		!HAL_GPIO_ReadPin(AERO_FRONT_BTN_GPIO_Port, AERO_FRONT_BTN_Pin);
		break;

	case AERO_REAR_BTN_Pin:
		car_buttons.aero_rear_button = 			!HAL_GPIO_ReadPin(AERO_REAR_BTN_GPIO_Port, AERO_REAR_BTN_Pin);
		break;
	}
	return;
}
