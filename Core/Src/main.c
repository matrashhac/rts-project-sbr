/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mpu6050.h"
#include "motorControl.h"
#include "myComfyPrint.h"
#include "ringbuffer.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG 180.0/M_PI
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* Definitions for Task_ReadMPU605 */
osThreadId_t Task_ReadMPU605Handle;
const osThreadAttr_t Task_ReadMPU605_attributes = {
  .name = "Task_ReadMPU605",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_CalcValues */
osThreadId_t Task_CalcValuesHandle;
const osThreadAttr_t Task_CalcValues_attributes = {
  .name = "Task_CalcValues",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_CtrlMotor */
osThreadId_t Task_CtrlMotorHandle;
const osThreadAttr_t Task_CtrlMotor_attributes = {
  .name = "Task_CtrlMotor",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for mpu6050_values_queue */
osMessageQueueId_t mpu6050_values_queueHandle;
const osMessageQueueAttr_t mpu6050_values_queue_attributes = {
  .name = "mpu6050_values_queue"
};
/* Definitions for motor_value_queue */
osMessageQueueId_t motor_value_queueHandle;
const osMessageQueueAttr_t motor_value_queue_attributes = {
  .name = "motor_value_queue"
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
void task_read_mpu6050(void *argument);
void task_calculate_motor_values(void *argument);
void task_control_motors(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void task_calculate_motor_values(void* parameter);
void task_read_mpu6050(void* parameter);
void task_motor_control(void* parameter);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  //initialize mpu6050
  HAL_Delay(1000);
  while(mpu6050_init() != 0){
	  HAL_Delay(1000);
	  //try until it worked
  }

  //initialize motors
  motor_init();



  //while loop to wait until robot is in upright position
  //calculate angle from acceleration values
  float acc_angle_temp=1;
  static uint8_t done_already = 0;
  char myString[256] = { 0 };
  mpu6050_values_t mpu_values;

  mpu6050_read(&mpu_values);

  if(done_already == 0){
	  while(acc_angle_temp != 0){
		  acc_angle_temp = atan2(mpu_values.acc_x, mpu_values.acc_z)*RAD_TO_DEG;	//*(180/M_PI) to convert from radian to degree
		  if(isnan(acc_angle_temp)){
			  sprintf((char*) myString, "acc_angle_temp is NaN\r\n");
			  myComfyPrint(myString);
		  }
		  else{
			  sprintf((char*) myString, "acc_angle_temp*100 = %d\r\n", (int) (acc_angle_temp*100));	//apparently floating numbers are disabled in print funcitons by defautl if using newlib
			  myComfyPrint(myString);
		  }

		  mpu6050_read(&mpu_values);
	  }
	  done_already = 1;
	  mpu6050_read(&mpu_values);
  }
  sprintf((char*) myString, "Acceleration X = %d\r\nAcceleration Y = %d\r\nAcceleration Z = %d\r\n", mpu_values.acc_x, mpu_values.acc_y, mpu_values.acc_z);
  myComfyPrint(myString);
  sprintf((char*) myString, "Gyroscope X = %d\r\nGyroscope Y = %d\r\nGyroscope Z = %d\r\n", mpu_values.gyro_x, mpu_values.gyro_y, mpu_values.gyro_z);
  myComfyPrint(myString);


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of mpu6050_values_queue */
  mpu6050_values_queueHandle = osMessageQueueNew (8, sizeof(mpu6050_values_t), &mpu6050_values_queue_attributes);

  /* creation of motor_value_queue */
  motor_value_queueHandle = osMessageQueueNew (1, sizeof(int32_t), &motor_value_queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_ReadMPU605 */
  Task_ReadMPU605Handle = osThreadNew(task_read_mpu6050, NULL, &Task_ReadMPU605_attributes);

  /* creation of Task_CalcValues */
  Task_CalcValuesHandle = osThreadNew(task_calculate_motor_values, NULL, &Task_CalcValues_attributes);

  /* creation of Task_CtrlMotor */
  Task_CtrlMotorHandle = osThreadNew(task_control_motors, NULL, &Task_CtrlMotor_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 8400-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|MOTOR2_DIR_Pin|MOTOR3_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin MOTOR2_DIR_Pin MOTOR3_DIR_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|MOTOR2_DIR_Pin|MOTOR3_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MPU6050_INT_Pin */
  GPIO_InitStruct.Pin = MPU6050_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(MPU6050_INT_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == MPU6050_INT_Pin){
	  //do stuff when mpu6050 says data is ready for reading
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_task_read_mpu6050 */

/* USER CODE END Header_task_read_mpu6050 */
void task_read_mpu6050(void *argument)
{
  /* USER CODE BEGIN 5 */

	uint8_t read_mpu_buff[6] = { 0 };
	mpu6050_values_t mpu_values_temp;

	for(;;)
	{

		//make sure there is room in queue
		while(osMessageQueueGetSpace(mpu6050_values_queueHandle) == 0){
			//busy waiting
		}

		//read accelerometer data
		HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS<<1) +1, REG_ACC_DATA, 1, read_mpu_buff, sizeof(read_mpu_buff), 100);
		mpu_values_temp.acc_x = (int16_t) (read_mpu_buff[0]<<8) + read_mpu_buff[1];
		mpu_values_temp.acc_y = (int16_t) (read_mpu_buff[2]<<8) + read_mpu_buff[3];
		mpu_values_temp.acc_z = (int16_t) (read_mpu_buff[4]<<8) + read_mpu_buff[5];

		//read gyroscope data
		HAL_I2C_Mem_Read(&hi2c1, (DEVICE_ADDRESS<<1) +1, REG_GYRO_DATA, 1, read_mpu_buff, sizeof(read_mpu_buff), 100);
		mpu_values_temp.gyro_x = (int16_t) (read_mpu_buff[0]<<8) + read_mpu_buff[1];
		mpu_values_temp.gyro_y = (int16_t) (read_mpu_buff[2]<<8) + read_mpu_buff[3];
		mpu_values_temp.gyro_z = (int16_t) (read_mpu_buff[4]<<8) + read_mpu_buff[5];

		//apply offsets
		mpu_values_temp.gyro_y += GYRO_Y_OFFSET_1000;

		//send values to queue
		osMessageQueuePut(mpu6050_values_queueHandle, &mpu_values_temp, 0, osWaitForever);

	  }

  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_task_calculate_motor_values */
/**
* @brief Function implementing the Task_CalcValues thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_calculate_motor_values */
void task_calculate_motor_values(void *argument)
{
  /* USER CODE BEGIN task_calculate_motor_values */

	  float acc_angle;
	  float gyro_rate, gyro_angle;
	  uint32_t current_time=(osKernelGetTickCount()*1000 / osKernelGetTickFreq()), last_time = current_time;
	  float current_angle=0, last_angle=0;
	  float alpha = 0.9934;
	  float target_angle=0;
	  float deviation, deviation_sum=0;
	  float Kp=12, Ki=0, Kd=0;
	  float motor_value_in_percent;
	  int32_t motor_value;
	  mpu6050_values_t mpu6050_temp_values;
	  mpu6050_values_t mpu_values;



  /* Infinite loop */
  for(;;)
  {

	  //wait for there to be values in the queue
	  while(osMessageQueueGetCount(mpu6050_values_queueHandle) == 0){
		  //busy waiting
	  }

	  //read values from queue
	  osMessageQueueGet(mpu6050_values_queueHandle, &mpu_values, NULL, osWaitForever);


	  //calculate angle from acceleration values
	  acc_angle = atan2(mpu_values.acc_x, mpu_values.acc_z)*RAD_TO_DEG;	//*(180/M_PI) to convert from radian to degree

	  //calculate angle from gyroscope
	  gyro_rate = mpu_values.gyro_y * (1000.0/INT16_MAX);	//500, because that is the range that the mpu6050 is set to currently
	  last_time = current_time;
	  current_time = ((osKernelGetTickCount()*1000) / osKernelGetTickFreq());
	  gyro_angle = gyro_angle + gyro_rate*((float)(current_time-last_time)/1000.0);

	  //combine both values into one
	  last_angle = current_angle;
	  current_angle = alpha * (gyro_angle) + (float) (1-alpha) * acc_angle;


	  //PID
	  deviation = current_angle - target_angle;
	  deviation_sum = deviation_sum + deviation;
	  //add constraining of max value of deviation_sum here

	  motor_value_in_percent = Kp*deviation + Ki*deviation_sum*((float)(current_time-last_time)/1000) - Kd*(current_angle-last_angle)/((float)(current_time-last_time)/1000);
	  motor_value = (motor_value_in_percent * MOTOR_MAX_SPEED) /100;


	  //if there is already a message in the queue, remove it
	  if(osMessageQueueGetSpace(motor_value_queueHandle) < 1){
		  int16_t temp;
		  osMessageQueueGet(motor_value_queueHandle, &temp, NULL, osWaitForever);
	  }
	  //send motor value to queue
	  osMessageQueuePut(motor_value_queueHandle, &motor_value, 0, osWaitForever);

  }
  /* USER CODE END task_calculate_motor_values */
}

/* USER CODE BEGIN Header_task_control_motors */
/**
* @brief Function implementing the Task_CtrlMotor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_task_control_motors */
void task_control_motors(void *argument)
{
  /* USER CODE BEGIN task_control_motors */

	int32_t motor_value;

  /* Infinite loop */
  for(;;)
  {

	  //wait for a value
	  while(osMessageQueueGetCount(motor_value_queueHandle) == 0){
		  //busy waiting
	  }

	  //get motor value
	  osMessageQueueGet(motor_value_queueHandle, &motor_value, NULL, osWaitForever);

	  if(motor_value >= 0){
		  motor_control(MOTOR_DIR_FORWARDS, (uint16_t) abs(motor_value));
	  }
	  else if(motor_value < 0){
		  motor_control(MOTOR_DIR_BACKWARDS, (uint16_t) abs(motor_value));
	  }

  }
  /* USER CODE END task_control_motors */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
