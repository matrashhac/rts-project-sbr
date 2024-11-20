/*
 * motorControl.h
 *
 *  Created on: Nov 15, 2024
 *      Author: alex
 */

#ifndef INC_MOTORCONTROL_H_
#define INC_MOTORCONTROL_H_

#include "main.h"
#include <stdint.h>

#define MOTOR_MAX_SPEED 8399	//is just the values i have currently set up on my microcontroller

#define MOTOR_DIR_FORWARDS 0
#define MOTOR_DIR_BACKWARDS 1

void motor_init();
void motor_control(uint8_t direction, uint16_t speed);

#endif /* INC_MOTORCONTROL_H_ */
