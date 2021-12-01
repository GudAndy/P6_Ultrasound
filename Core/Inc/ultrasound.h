/*
 * ultrasound.h
 *
 *  Created on: Nov 30, 2021
 *      Author: Andrew
 */

#ifndef INC_ULTRASOUND_H_
#define INC_ULTRASOUND_H_

#include "stm32l4xx_hal.h"
#include "main.h"

#define usTIM htim4
#define echoTIM htim3

extern uint8_t echoNum;
extern uint32_t edge1, edge2;
extern uint8_t read_done;
extern const float speed_of_sound;

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

void usDelay(uint32_t uSec);
uint32_t read_ultrasound(void);



#endif /* INC_ULTRASOUND_H_ */
