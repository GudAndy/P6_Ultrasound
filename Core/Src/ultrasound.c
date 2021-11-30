/*
 * ultrasound.c
 *
 *  Created on: Nov 30, 2021
 *      Author: Andrew
 */
#include "ultrasound.h"

uint8_t echoNum = 0;
uint32_t edge1 = 0, edge2 = 0;
uint8_t read_done = 0;
const float speed_of_sound = 0.343/2;
uint8_t tst = 0;

void usDelay(uint32_t uSec){
    usTIM.Instance->ARR = uSec - 1;
    usTIM.Instance->EGR = 1;
    usTIM.Instance->SR &= ~TIM_SR_UIF_Msk;
    usTIM.Instance->CR1 |= 1;
    while((usTIM.Instance->SR & TIM_SR_UIF_Msk) != 1);
    usTIM.Instance->SR &= ~TIM_SR_UIF_Msk;

}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if (echoNum == 0){
        edge1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        echoNum = 1;
    } else if (echoNum == 1){
        edge2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);

        echoNum = 0;
        read_done = 1;
    }
}


/**
  * @brief Function triggers a measurement with the ultrasound
  * @param None
  * @retval Distance in mm
  */
uint32_t read_ultrasound(void){
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	usDelay(3);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
	usDelay(10);
	HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);
	HAL_TIM_IC_Start_IT(&echoTIM, TIM_CHANNEL_1);
	uint32_t start = HAL_GetTick();
	do {
		if (read_done == 1) break;
	} while (HAL_GetTick() - start < 100);
	read_done = 0;
	HAL_TIM_IC_Stop_IT(&echoTIM, TIM_CHANNEL_1);
	uint32_t dist = (edge2 - edge1) * speed_of_sound;
	return dist;
}

