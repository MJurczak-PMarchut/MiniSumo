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
extern UART_HandleTypeDef huart7;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern uint8_t _data_receiveL[3];
extern uint8_t _data_receiveR[3];
extern DMA_HandleTypeDef hdma_uart7_rx;

void EmStop(void);

uint8_t pRx_Data[40] = {0};

void RxL9960TCompletedCB(struct MessageInfoTypeDef* MsgInfo);
void TxL9960TCompletedCB(struct MessageInfoTypeDef* MsgInfo);

//TLE5205 MOTOR_CONTROLLERS[] = {[MOTOR_LEFT] = TLE5205(MOTOR_LEFT, &htim2, TIM_CHANNEL_1), [MOTOR_RIGHT] = TLE5205(MOTOR_RIGHT, &htim2, TIM_CHANNEL_2)};
CommManager MainCommManager;
L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, RxL9960TCompletedCB, TxL9960TCompletedCB),
		[MOTOR_RIGHT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, RxL9960TCompletedCB, TxL9960TCompletedCB)};

IBus RxController(&huart7, EmStop, pRx_Data, &hdma_uart7_rx);
//VL53L1X vl53l1x = VL53L1X(&hi2c1, &MainCommManager);
MessageInfoTypeDef MsgInfo = {0};
uint16_t distance = 0;



void main_cpp(void)
{
	MainCommManager.AttachCommInt(&hspi2);
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(0);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(0);
	HAL_UARTEx_ReceiveToIdle_DMA(&huart7, pRx_Data, 40);
	__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
	while(1)
	{

		if(huart7.RxState == HAL_UART_STATE_READY)
		{
			HAL_UARTEx_ReceiveToIdle_DMA(&huart7, pRx_Data, 40);
			__HAL_DMA_DISABLE_IT(&hdma_uart7_rx, DMA_IT_HT);
		}
	}
}


//void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
//	{
//		MOTOR_CONTROLLERS[MOTOR_LEFT].TimCBPulse();
//	}
//	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
//	{
//		MOTOR_CONTROLLERS[MOTOR_RIGHT].TimCBPulse();
//	}
//}
//
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	MOTOR_CONTROLLERS[MOTOR_LEFT].TimCB();
//	MOTOR_CONTROLLERS[MOTOR_RIGHT].TimCB();
//}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TOF_GPIO_5_Pin) {
//	  vl53l1x.GetDistance(TOF5);
  }
}

void RxL9960TCompletedCB(struct MessageInfoTypeDef* MsgInfo)
{
	if(MsgInfo->context & (1<<MOTOR_LEFT))
	{

	}
	else if(MsgInfo->context & (1<<MOTOR_RIGHT))
	{

	}
}

void TxL9960TCompletedCB(struct MessageInfoTypeDef* MsgInfo)
{
	if(MsgInfo->context & (1<<MOTOR_LEFT))
	{
		if((uint16_t)MsgInfo->pRxData & (INIT_SEQUENCE_CONTEXT << 2))
		{
			MOTOR_CONTROLLERS[MOTOR_LEFT].Init();
		}
	}
	else if(MsgInfo->context & (1<<MOTOR_RIGHT))
	{

	}
}

void EmStop(void)
{

}

#ifdef __cplusplus
}
#endif
