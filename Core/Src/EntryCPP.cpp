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
extern TIM_HandleTypeDef MOTOR_LEFT_TIMER_HANDLE;
extern TIM_HandleTypeDef MOTOR_RIGHT_TIMER_HANDLE;

L9960T MOTOR_CONTROLLERS[] = {[MOTOR_LEFT] = L9960T(MOTOR_LEFT), [MOTOR_RIGHT] = L9960T(MOTOR_RIGHT)};

void main_cpp(void)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT ].AttachPWMTimerAndChannel(MOTOR_LEFT_TIMER_HANDLE, MOTOR_LEFT_TIM_CHANNEL);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].AttachPWMTimerAndChannel(MOTOR_RIGHT_TIMER_HANDLE, MOTOR_RIGHT_TIM_CHANNEL);
	MOTOR_CONTROLLERS[MOTOR_LEFT ].SetMotorPowerPWM(500);
	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(500);
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
