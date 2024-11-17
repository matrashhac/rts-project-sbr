/*
 * myComfyPrint.h
 *
 *  Created on: Nov 17, 2024
 *      Author: alex
 */

#ifndef INC_MYCOMFYPRINT_H_
#define INC_MYCOMFYPRINT_H_

#include "main.h"
#include <string.h>


extern UART_HandleTypeDef huart2;

uint8_t buff[2048] = { 0 };

void myComfyPrint(const char* string){
	strncpy((char*) buff, string, sizeof(buff));
	HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
}

#endif /* INC_MYCOMFYPRINT_H_ */
