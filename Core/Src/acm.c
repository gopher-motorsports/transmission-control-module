
#include "main.h"
#include "buttons.h"

extern TIM_HandleTypeDef htim8;

#define HIGH_REAR_ANGLE 3700
#define LOW_REAR_ANGLE 2550 // high downforce or low downforce position?

int angle = HIGH_REAR_ANGLE;

void acm_init(void)
{
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	TIM8->CCR3 = HIGH_REAR_ANGLE;
}


void run_acm(void)
{
	if (car_buttons.aero_rear_button)
	{
		TIM8->CCR3 = LOW_REAR_ANGLE;
	}
	else
	{
		TIM8->CCR3 = HIGH_REAR_ANGLE;
		//TIM8->CCR3 = angle;
	}
}


