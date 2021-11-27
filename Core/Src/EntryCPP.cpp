/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "RobotSpecificDefines.hpp"
#include "L9960T.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef MOTOR_LEFT_TIMER;
extern TIM_HandleTypeDef MOTOR_RIGHT_TIMER;
extern SPI_HandleTypeDef hspi2;
uint32_t a = 0;
uint16_t b[2] = {0};
uint16_t c[2] = {0};

L9960T MOTOR_CONTROLLERS[2] = {[MOTOR_LEFT] = L9960T(MOTOR_LEFT), [MOTOR_RIGHT] = L9960T(MOTOR_RIGHT)};

void main_cpp(void)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT ].AttachPWMTimerAndChannel(&MOTOR_LEFT_TIMER, MOTOR_LEFT_TIM_CHANNEL);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].AttachPWMTimerAndChannel(&MOTOR_RIGHT_TIMER, MOTOR_RIGHT_TIM_CHANNEL);
	MOTOR_CONTROLLERS[MOTOR_LEFT ].SetMotorPowerPWM(500);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(500);
	MOTOR_CONTROLLERS[MOTOR_LEFT ].SetMotorDirection(MOTOR_DIR_FORWARD);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
	MOTOR_CONTROLLERS[MOTOR_LEFT ].Enable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Enable();
	b[0] = 0x2000;
	HAL_GPIO_WritePin(MD_CS_1_GPIO_Port, MD_CS_1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MD_CS_2_GPIO_Port, MD_CS_2_Pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(MD_CS_1_GPIO_Port, MD_CS_1_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)b, (uint8_t *)c, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MD_CS_1_GPIO_Port, MD_CS_1_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(MD_CS_2_GPIO_Port, MD_CS_2_Pin, GPIO_PIN_RESET);
	HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)b, (uint8_t *)c, 1, HAL_MAX_DELAY);
	HAL_GPIO_WritePin(MD_CS_2_GPIO_Port, MD_CS_2_Pin, GPIO_PIN_SET);
	b[0] = 0xF004;
	while(1)
	{
		HAL_GPIO_WritePin(MD_CS_1_GPIO_Port, MD_CS_1_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)b, (uint8_t *)c, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(MD_CS_1_GPIO_Port, MD_CS_1_Pin, GPIO_PIN_SET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(MD_CS_2_GPIO_Port, MD_CS_2_Pin, GPIO_PIN_RESET);
		HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)b, (uint8_t *)c, 1, HAL_MAX_DELAY);
		HAL_GPIO_WritePin(MD_CS_2_GPIO_Port, MD_CS_2_Pin, GPIO_PIN_SET);
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
}

#ifdef __cplusplus
}
#endif
