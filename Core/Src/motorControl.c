/*
 * motorControl.c
 *
 *  Created on: Nov 17, 2024
 *      Author: alex
 */

#include "motorControl.h"


extern TIM_HandleTypeDef htim3;


void motor_init(){
	//initialize PWM
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
}

void motor_control(uint8_t direction, uint16_t speed){
  //direction 0=forward, 1=backward, everything else leaves the current direction
  //speed 0=off, 255=max

  //Error Handling
  if(speed<0) speed=0;
  if(speed>MOTOR_MAX_SPEED) speed=MOTOR_MAX_SPEED;

  //set motor direction
  switch(direction){
  case MOTOR_DIR_FORWARDS:
	  HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_RESET);
	  HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_Port, MOTOR3_DIR_Pin, GPIO_PIN_RESET);
	  break;
  case MOTOR_DIR_BACKWARDS:
	  HAL_GPIO_WritePin(MOTOR2_DIR_GPIO_Port, MOTOR2_DIR_Pin, GPIO_PIN_SET);
	  HAL_GPIO_WritePin(MOTOR3_DIR_GPIO_Port, MOTOR3_DIR_Pin, GPIO_PIN_SET);
	  break;
  default:
	  //do nothing
	  break;
  }

  //set motor speed
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, speed);
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, speed);

}
