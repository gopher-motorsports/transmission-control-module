/*
 * display.c
 *
 *  Created on: May 12, 2022
 *      Author: sebas
 */

#include "display.h"
#include "main.h"
#include "buttons.h"
#include "car_utils.h"
#include "GopherCAN.h"

extern CAN_HandleTypeDef hcan2;

int send_display_data()
{
	uint8_t display_data_1[8] = {0};

	CAN_TxHeaderTypeDef tx_header;
	uint32_t tx_mailbox_num;

	// Clutch Data
	display_data_1[7] = clutch_open();
	// DRS(Aux1) status
	display_data_1[3] = tcm_lap_timer.data;
	// What gear are we currently in
	display_data_1[1] = car_shift_data.current_gear;

	// tx_header for display_data_1
	tx_header.IDE = CAN_ID_STD;
	tx_header.TransmitGlobalTime = DISABLE;
	tx_header.StdId = 0x200;
	tx_header.DLC = 8;
	tx_header.RTR = 0;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, display_data_1, &tx_mailbox_num);

	return 0;
}

void send_lap_time_data(uint32_t last_lap_time, uint32_t fastest_lap_time)
{
	uint8_t display_data_1[8] = {0};

	CAN_TxHeaderTypeDef tx_header;
	uint32_t tx_mailbox_num;

	display_data_1[3] = (last_lap_time&0x000000FF);
	display_data_1[2] = (last_lap_time&0x0000FF00) >> 8;
	display_data_1[1] = (last_lap_time&0x00FF0000) >> 16;
	display_data_1[0] = (last_lap_time&0xFF000000) >> 24;
	display_data_1[7] = (fastest_lap_time&0x000000FF);
	display_data_1[6] = (fastest_lap_time&0x0000FF00) >> 8;
	display_data_1[5] = (fastest_lap_time&0x00FF0000) >> 16;
	display_data_1[4] = (fastest_lap_time&0xFF000000) >> 24;

	// tx_header for display_data_1
	tx_header.IDE = CAN_ID_STD;
	tx_header.TransmitGlobalTime = DISABLE;
	tx_header.StdId = 0x201;
	tx_header.DLC = 8;
	tx_header.RTR = 0;

	HAL_CAN_AddTxMessage(&hcan2, &tx_header, display_data_1, &tx_mailbox_num);

}


