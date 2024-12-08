#ifndef RINGBUFFER_H
#define RINGBUFFER_H

#include <stdint.h>
#include "mpu6050.h"

#define RINGBUFFER_SIZE 8

int16_t readRbCounter = 0, writeRbCounter = 0;
mpu6050_values_t ringBuffer[RINGBUFFER_SIZE];

int16_t getSizeOfRingBuffer(){
	return RINGBUFFER_SIZE;
}

void writeToRingBuffer(mpu6050_values_t newData){
	ringBuffer[writeRbCounter] = newData;

	if(writeRbCounter >= RINGBUFFER_SIZE-1){
		writeRbCounter = 0;
	}
	else{
		writeRbCounter++;
	}
}

mpu6050_values_t readFromRingBuffer(void){
	mpu6050_values_t tempData = ringBuffer[readRbCounter];

	if(readRbCounter >= RINGBUFFER_SIZE){
		readRbCounter = 0;
	}
	else{
		readRbCounter++;
	}

	return tempData;
}


#endif RINGBUFFER_H
