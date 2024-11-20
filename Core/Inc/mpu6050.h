/*
 * mpu6050.h
 *
 *  Created on: Nov 15, 2024
 *      Author: alex
 */

#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>

#define DEVICE_ADDRESS 0x68

#define FS_GYRO_250 0
#define FS_GYRO_500 8
#define FS_GYRO_1000 16
#define FS_GYRO_2000 24

#define FS_ACC_2G 0
#define FS_ACC_4G 8
#define FS_ACC_8G 9
#define FS_ACC_16G 10

#define LPF_260 0
#define LPF_184 1
#define LPF_94 2
#define LPF_44 3
#define LPF_21 4
#define LPF_10 5
#define LPF_5 6

#define REG_CONFIG_GYRO 27
#define REG_CONFIG_ACC 28
#define REG_USR_CTRL 107
#define REG_ACC_DATA 59
#define REG_GYRO_DATA 67
#define REG_LOW_PASS_FILTER 26

#define GYRO_Y_OFFSET_500 22
#define GYRO_Y_OFFSET_1000 12



typedef struct{
	int16_t gyro_x, gyro_y, gyro_z;
	int16_t acc_x, acc_y, acc_z;
}mpu6050_values_t;

void mpu6050_init();
void mpu6050_read(mpu6050_values_t* values);

#endif /* INC_MPU6050_H_ */
