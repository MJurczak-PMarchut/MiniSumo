/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "RobotSpecificDefines.hpp"
#include "L9960T.hpp"
#include "../../SumoDrivers/ToF_sensors/core/vl53l1x.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef MOTOR_LEFT_TIMER_HANDLE;
extern TIM_HandleTypeDef MOTOR_RIGHT_TIMER_HANDLE;

L9960T MOTOR_CONTROLLERS[] = {[MOTOR_LEFT] = L9960T(MOTOR_LEFT), [MOTOR_RIGHT] = L9960T(MOTOR_RIGHT)};
CommManager MainCommManager = CommManager();
VL53L1X vl53l1x = VL53L1X(&hi2c1, &MainCommManager);
MessageInfoTypeDef MsgInfo = {0};
uint16_t distance = 0;

void main_cpp(void)
{
//	MOTOR_CONTROLLERS[MOTOR_LEFT ].AttachPWMTimerAndChannel(MOTOR_LEFT_TIMER_HANDLE, MOTOR_LEFT_TIM_CHANNEL);
//	MOTOR_CONTROLLERS[MOTOR_RIGHT].AttachPWMTimerAndChannel(MOTOR_RIGHT_TIMER_HANDLE, MOTOR_RIGHT_TIM_CHANNEL);
//	MOTOR_CONTROLLERS[MOTOR_LEFT ].SetMotorPowerPWM(500);
//	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(500);
	MsgInfo.uCommInt.hi2c = &hi2c1;
	MsgInfo.eCommType = COMM_INT_I2C_TX;
	MsgInfo.context = 0x21;
	HAL_GPIO_WritePin(XSHUT_5_GPIO_Port, XSHUT_5_Pin, GPIO_PIN_SET);
	VL53L1X_SensorInit(TOF_DEFAULT_ADDRESS, &MainCommManager, &MsgInfo);
	VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDRESS, TOF5_Addr, &MainCommManager, &MsgInfo);
	VL53L1X_StartRanging(TOF5_Addr, &MainCommManager, &MsgInfo);
	while(1)
	{
		HAL_Delay(100);
		VL53L1X_GetDistance(TOF5_Addr, &distance, &MainCommManager, &MsgInfo);
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == TOF_GPIO_5_Pin) {
	  vl53l1x.GetDistance(TOF5);
  }
}

void HAL_I2C_MasterTxCpltCallback (I2C_HandleTypeDef * hi2c)
{
}

void HAL_I2C_MasterRxCpltCallback (I2C_HandleTypeDef * hi2c)
{
}

#ifdef __cplusplus
}
#endif
