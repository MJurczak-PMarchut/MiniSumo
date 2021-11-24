/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "CommManager.hpp"
#include "ServoControl.hpp"
#include "RobotSpecificDefines.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;

CommManager CommunicationManager;
ServoControl Servos(&htim2);

extern UART_HandleTypeDef huart4;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart4_tx;

void main_cpp(void)
{
//	MessageInfoTypeDef msg = {0};
//	msg.
//	CommunicationManager.AttachCommInt(&huart4, &hdma_uart4_rx, &hdma_uart4_tx);
//	CommunicationManager.PushCommRequestIntoQueue(MsgInfo)
	Servos.AttachServo(SERVO03_GPIO_Port, SERVO03_Pin);
	Servos.AttachServo(SERVO04_GPIO_Port, SERVO04_Pin);
	Servos.AttachServo(SERVO12_GPIO_Port, SERVO12_Pin);
	Servos.AttachServo(SERVO13_GPIO_Port, SERVO13_Pin);
	Servos.AttachServo(SERVO14_GPIO_Port, SERVO14_Pin);
	Servos.StartServos();
	while(1)
	{
		for(uint16_t u16Angle = 0; u16Angle < 1900; u16Angle += 80)
		{
			for(uint8_t u8Servo_No = 0; u8Servo_No < Servos.GetNoOfAttchedServos(); u8Servo_No++)
			{
				Servos.SetServoValue(u8Servo_No, u16Angle);
			}
			HAL_Delay(30);
		}
		for(uint16_t u16Angle = 1900; u16Angle > 150; u16Angle -= 150)
		{
			for(uint8_t u8Servo_No = 0; u8Servo_No < Servos.GetNoOfAttchedServos(); u8Servo_No++)
			{
				Servos.SetServoValue(u8Servo_No, u16Angle);
			}
			HAL_Delay(30);
		}
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		Servos.ServoControlCBHalfPulse();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM2)
	{
		Servos.ServoControlCBUpdate();
	}
}

#ifdef __cplusplus
}
#endif
