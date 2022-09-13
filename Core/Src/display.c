/*
 * display.c
 *
 *  Created on: May 12, 2022
 *      Author: sebas
 */

#include "display.h"
#include "global_vars.h"
extern CAN_HandleTypeDef hcan2;

int send_display_data()
{
	uint8_t display_data_1[8] = {0};
	uint8_t display_data_2[4] = {0};

	CAN_TxHeaderTypeDef tx_header;
	uint32_t tx_mailbox_num;

	// TODO - figure out how to handle clutch display. What happens in the case of sensor failure?
	// Clutch Data
	display_data_1[7] = clutch_open();
	// DRS(Aux1) status
	display_data_1[3] = (uint8_t)car_buttons.aero_rear_button;
	// What gear are we currently in
	display_data_1[1] = car_shift_data.current_gear;

	// tx_header for display_data_1
	tx_header.IDE = CAN_ID_STD;
	tx_header.TransmitGlobalTime = DISABLE;
	tx_header.StdId = 0x200;
	tx_header.DLC = 8;
	tx_header.RTR = 0;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, display_data_1, &tx_mailbox_num);


	uint32_t front_brake_temp = 69;
	uint32_t rear_brake_temp = 420;

	display_data_2[1] = (uint8_t)front_brake_temp;
	display_data_2[0] = (uint8_t)(front_brake_temp >> 8);

	display_data_2[3] = (uint8_t)rear_brake_temp;
	display_data_2[2] = (uint8_t)(rear_brake_temp >> 8);

	// tx_header for display_data_2
	tx_header.IDE = CAN_ID_STD;
	tx_header.TransmitGlobalTime = DISABLE;
	tx_header.StdId = 0x201;
	tx_header.DLC = 4;
	tx_header.RTR = 0;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, display_data_2, &tx_mailbox_num);

	return 0;
}


