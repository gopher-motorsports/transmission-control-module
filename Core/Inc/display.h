/*
 * display.h
 *
 *  Created on: May 12, 2022
 *      Author: sebas
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include <stdint.h>

int send_display_data();
void send_lap_time_data(uint32_t last_lap_time, uint32_t fastest_lap_time);



#endif /* INC_DISPLAY_H_ */
