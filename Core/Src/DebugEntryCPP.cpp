/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "RobotSpecificDefines.hpp"
//#include "TLE5205.hpp"
#include "Configuration.h"
#include "vl53l1x.hpp"
#include "I-BUS.hpp"
#include "L9960T.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim15;
extern UART_HandleTypeDef huart7;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern uint8_t _data_receiveL[3];
extern uint8_t _data_receiveR[3];
extern DMA_HandleTypeDef hdma_uart7_rx;

void EmStop(void);

uint8_t pRx_Data[40] = {0};


//TLE5205 MOTOR_CONTROLLERS[] = {[MOTOR_LEFT] = TLE5205(MOTOR_LEFT, &htim2, TIM_CHANNEL_1), [MOTOR_RIGHT] = TLE5205(MOTOR_RIGHT, &htim2, TIM_CHANNEL_2)};
CommManager MainCommManager;
L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, TIM_CHANNEL_1, &htim3),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, TIM_CHANNEL_3, &htim4)};

IBus RxController(&huart7, EmStop, pRx_Data, &hdma_uart7_rx);
//VL53L1X vl53l1x = VL53L1X(&hi2c1, &MainCommManager);
MessageInfoTypeDef MsgInfo = {0};
uint16_t distance = 0;

void InitControllers(void);


void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	MainCommManager.MsgReceivedCB(hspi);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
	MainCommManager.MsgReceivedCB(hspi);
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
	MainCommManager.MsgReceivedCB(hspi);
}

void main_cpp(void * pvParameters )
{
	MainCommManager.AttachCommInt(&hspi2);
	InitControllers();
	MOTOR_CONTROLLERS[MOTOR_LEFT].Enable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Enable();
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
 	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);

 	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
 	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, pRx_Data, 40);
	__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
 	 while(1)
	{
 		 HAL_Delay(1000);
 		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
 	 	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);
 	 	HAL_Delay(1000);
 		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
 	 	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
		if(huart7.RxState == HAL_UART_STATE_READY)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart7, pRx_Data, 40);
			__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
		}
	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TOF_GPIO_5_Pin) {
  }
}

void EmStop(void)
{

}

void InitControllers(void)
{
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Init();
	while(MOTOR_CONTROLLERS[MOTOR_LEFT].CheckIfControllerInitializedOk() != HAL_OK)
	{}
	while(MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckIfControllerInitializedOk() != HAL_OK)
	{}

}

#ifdef __cplusplus
}
#endif
