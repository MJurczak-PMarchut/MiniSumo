/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "RobotSpecificDefines.hpp"
#include "L9960T.hpp"
#include "Configuration.h"
#include "vl53l1x.hpp"
#include "I-BUS.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart7;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t _data_receiveL[3];
extern uint8_t _data_receiveR[3];
extern SPI_HandleTypeDef hspi2;
extern DMA_HandleTypeDef hdma_uart7_rx;

void EmStop(void);

uint8_t pRx_Data[40] = {0};

CommManager MainCommManager;
Moto MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, TIM_CHANNEL_1, &htim3),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, TIM_CHANNEL_3, &htim4)};

IBus RxController(&huart7, EmStop, pRx_Data, &hdma_uart7_rx);
//VL53L1X vl53l1x = VL53L1X(&hi2c1, &MainCommManager);
MessageInfoTypeDef MsgInfo = {0};
uint16_t distance = 0;

void main_cpp(void * pvParameters)
{
	uint16_t motor_l = 0;
	uint16_t motor_r = 0;
	float motor_l_ratio, motor_r_ratio;
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MainCommManager.AttachCommInt(&hspi2);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init(0);
	while(MOTOR_CONTROLLERS[MOTOR_LEFT].CheckIfControllerInitializedOk() != HAL_OK)
	{}
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Init(0);
	while(MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckIfControllerInitializedOk() != HAL_OK)
	{}
	MOTOR_CONTROLLERS[MOTOR_LEFT].Enable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Enable();
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
 	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(0);
 	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(0);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, pRx_Data, 40);
	__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
	while(1)
	{
		if(RxController.GetConnectionStatus() == HAL_OK){
			if(RxController.GetAxisValue(1) > 500)
			{
				motor_l = ((RxController.GetAxisValue(1) - 500) * 2);
				motor_r = ((RxController.GetAxisValue(1) - 500) * 2);
				if(RxController.GetAxisValue(0) < 500)
				{
					motor_l_ratio = (float)RxController.GetAxisValue(0)/500;
					motor_r_ratio = 1;
				}
				else
				{
					motor_r_ratio = (RxController.GetAxisValue(0) - 500);
					motor_r_ratio = (500 - motor_r_ratio)/500;
					motor_l_ratio = 1;
				}
				motor_l = motor_l * motor_l_ratio;
				motor_r = motor_r * motor_r_ratio;

				MOTOR_CONTROLLERS[MOTOR_LEFT ].SetMotorDirection(MOTOR_DIR_FORWARD);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
	//			MOTOR_CONTROLLERS[MOTOR_LEFT ].SetMotorPowerPWM(motor_l);
			}
			else
			{
				motor_l = ((500 - RxController.GetAxisValue(1)) * 2);
				motor_r = ((500 - RxController.GetAxisValue(1)) * 2);
				if(RxController.GetAxisValue(0) < 500)
				{
					motor_r_ratio = (RxController.GetAxisValue(0) - 500);
					motor_r_ratio = (500 - motor_r_ratio)/500;
					motor_l_ratio = 1;
				}
				else
				{
					motor_l_ratio = (float)RxController.GetAxisValue(0)/500;
					motor_r_ratio = 1;
				}
				motor_l = motor_l * motor_l_ratio;
				motor_r = motor_r * motor_r_ratio;

				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);
			}
		}
		else
		{
		}
		MOTOR_CONTROLLERS[MOTOR_LEFT ].SetMotorPowerPWM(motor_l);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(motor_r);
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
//	  vl53l1x.GetDistance(TOF5);
  }
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
}


void EmStop(void)
{
//	MOTOR_CONTROLLERS[MOTOR_LEFT].EmergencyStop();
//	MOTOR_CONTROLLERS[MOTOR_RIGHT].EmergencyStop();
}


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
	if(huart->Instance == UART7)
	{
		RxController.ProcessRxDataCB();
	}
}

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

#ifdef __cplusplus
}
#endif
