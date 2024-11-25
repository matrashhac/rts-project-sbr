/*
 * mpu6050.c
 *
 *  Created on: Nov 15, 2024
 *      Author: alex
 */

#include "mpu6050.h"
#include "main.h"
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

void mpu6050_init(){

	uint8_t buff[256] = { 0 };

	//check if MPU is ready
	  if(HAL_I2C_IsDeviceReady(&hi2c1, (DEVICE_ADDRESS<<1) +0, 1, 100) == HAL_OK){
		  strcpy((char*) buff, "The device is ready\r\n");
		  HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
	  }
	  else{
		  strcpy((char*) buff, "The device is not ready. Check the cables\r\n");
		  HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
	  }

	  //configure gyroscope
	  uint8_t gyroscope_range = FS_GYRO_1000;	//set gyroscope range to 500 °/s
	  if(HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1) +0, REG_CONFIG_GYRO, 1, &gyroscope_range, sizeof(gyroscope_range), 100) == HAL_OK){
		  strcpy((char*) buff, "gyroscope range successfully set\r\n");
		  HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
	  }
	  else{
		  strcpy((char*) buff, "failed setting gyroscope range\r\n");
		  HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
	  }

	  //configure accelerometer
	  uint8_t accelerometer_range = FS_ACC_2G;	//set gyroscope range to 500 °/s
	  if(HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1) +0, REG_CONFIG_ACC, 1, &accelerometer_range, sizeof(accelerometer_range), 100) == HAL_OK){
		  strcpy((char*) buff, "accelerometer range successfully set\r\n");
		  HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
	  }
	  else{
		  strcpy((char*) buff, "failed setting accelerometer range\r\n");
		  HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
	  }

	  //set low pass filter
	  uint8_t lpf_setting = LPF_44;
	  HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1) +0, REG_LOW_PASS_FILTER, 1, &lpf_setting, sizeof(lpf_setting), 100);

	  //enable interrupt to signal that data is ready to be read
	  uint8_t interrupt_signal = INT_DATA_READY;
	  HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1) +0, REG_INTERRUPT, 1, &interrupt_signal, sizeof(interrupt_signal), 100);

	  //deactivate sleep
	  uint8_t asdf_flags = 0b00001000;	//set the '1' also to '0' if you want the temperature sensor to be enabled
	  if(HAL_I2C_Mem_Write(&hi2c1, (DEVICE_ADDRESS<<1) +0, REG_USR_CTRL, 1, &asdf_flags, sizeof(asdf_flags), 100) == HAL_OK){
		  strcpy((char*) buff, "disablin sleep successfully\r\n");
		  HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
	  }
	  else{
		  strcpy((char*) buff, "error disabling sleep\r\n");
		  HAL_UART_Transmit(&huart2, buff, strlen((char*) buff), HAL_MAX_DELAY);
	  }

}



void mpu6050_read(mpu6050_values_t* values){

	  uint8_t buff[6] = { 0 };
	  HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS<<1) +1, REG_ACC_DATA, 1, buff, sizeof(buff), 100);
	  values->acc_x = (uint16_t) (buff[0]<<8) + buff[1];
	  values->acc_y = (uint16_t) (buff[2]<<8) + buff[3];
	  values->acc_z = (uint16_t) (buff[4]<<8) + buff[5];

	  HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS<<1) +1, REG_GYRO_DATA, 1, buff, sizeof(buff), 100);
	  values->gyro_x = (uint16_t) (buff[0]<<8) + buff[1];
	  values->gyro_y = (uint16_t) (buff[2]<<8) + buff[3];
	  values->gyro_z = (uint16_t) (buff[4]<<8) + buff[5];

	  //apply offsets
	  values->gyro_y += GYRO_Y_OFFSET_1000;

}
