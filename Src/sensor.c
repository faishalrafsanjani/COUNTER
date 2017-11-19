/*
 * sensor.c
 *
 *  Created on: 9 Nov 2017
 *      Author: tinova
 */

#include "sensor.h"

uint8_t TombolM(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(TombolM_GPIO_Port,TombolM_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(TombolM_GPIO_Port,TombolM_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(TombolM_GPIO_Port,TombolM_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
uint8_t TombolP(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(TombolP_GPIO_Port,TombolP_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(TombolP_GPIO_Port,TombolP_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(TombolP_GPIO_Port,TombolP_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
uint8_t Sensor1(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(Sensor1_GPIO_Port,Sensor1_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
uint8_t Sensor2(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(Sensor2_GPIO_Port,Sensor2_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
uint8_t Sensor3(void)
{
	uint8_t FlagDetect=0;
	if(HAL_GPIO_ReadPin(Sensor3_GPIO_Port,Sensor3_Pin)== GPIO_PIN_RESET) {
		HAL_Delay(20);
		if(HAL_GPIO_ReadPin(Sensor3_GPIO_Port,Sensor3_Pin)== GPIO_PIN_RESET) {
			while(HAL_GPIO_ReadPin(Sensor3_GPIO_Port,Sensor3_Pin)== GPIO_PIN_RESET);
			FlagDetect=1;
		}
	}
	return FlagDetect;
}
