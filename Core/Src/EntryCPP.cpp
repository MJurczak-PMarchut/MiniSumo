/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"
#include "RobotSpecificDefines.hpp"
#include "TLE5205.hpp"
#include "Configuration.h"
#include "../../SumoDrivers/ToF_sensors/core/vl53l1x.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern TIM_HandleTypeDef htim2;
extern I2C_HandleTypeDef hi2c1;
extern uint8_t _data_receiveL[3];
extern uint8_t _data_receiveR[3];

TLE5205 MOTOR_CONTROLLERS[] = {[MOTOR_LEFT] = TLE5205(MOTOR_LEFT, &htim2, TIM_CHANNEL_1), [MOTOR_RIGHT] = TLE5205(MOTOR_RIGHT, &htim2, TIM_CHANNEL_2)};
CommManager MainCommManager = CommManager();
//VL53L1X vl53l1x = VL53L1X(&hi2c1, &MainCommManager);
MessageInfoTypeDef MsgInfo = {0};
uint16_t distance = 0;

void main_cpp(void)
{
	uint16_t distance_left, distance_right;
	int is_started = 0;
	int last_direction = 0; //0 is left
	MsgInfo.uCommInt.hi2c = &hi2c1;
	MsgInfo.eCommType = COMM_INT_I2C_TX;
	MsgInfo.context = 0x21;


	HAL_GPIO_WritePin(XSHUT_5_GPIO_Port, XSHUT_5_Pin, GPIO_PIN_SET);
	VL53L1X_SensorInit(TOF_DEFAULT_ADDRESS, &MainCommManager, &MsgInfo);
	VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDRESS, TOF5_Addr, &MainCommManager, &MsgInfo);

	HAL_GPIO_WritePin(XSHUT_6_GPIO_Port, XSHUT_6_Pin, GPIO_PIN_SET);
	VL53L1X_SensorInit(TOF_DEFAULT_ADDRESS, &MainCommManager, &MsgInfo);
	VL53L1X_SetI2CAddress(TOF_DEFAULT_ADDRESS, TOF0_Addr, &MainCommManager, &MsgInfo);

	VL53L1X_StartRanging(TOF5_Addr, &MainCommManager, &MsgInfo);
	VL53L1X_StartRanging(TOF0_Addr, &MainCommManager, &MsgInfo);


	while(1)
	{
		while (HAL_GPIO_ReadPin(START_SW_GPIO_Port, START_SW_Pin) == GPIO_PIN_RESET)
		{
			MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(0);
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(0);
			HAL_Delay(1);
		}

		if(is_started == 0)
		{
			MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
			HAL_Delay(250);
			is_started = 1;
		}

	    MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
	    MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
		HAL_Delay(60);
		VL53L1X_GetDistance(TOF5_Addr, &distance, &MainCommManager, &MsgInfo);
		VL53L1X_GetDistance(TOF0_Addr, &distance, &MainCommManager, &MsgInfo);
		distance_left = (_data_receiveL[0] << 8) | (_data_receiveL[1]);
		distance_right = (_data_receiveR[0] << 8) | (_data_receiveR[1]);

		if(distance_left < 200)
		{
			if(distance_right < 200)
			{
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
			}
			else
			{
				last_direction = 0;
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
			}
		}
		else if(distance_right < 200)
		{
			MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
			MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);
			last_direction = 1;
		}
		else
		{
			if(last_direction == 0)
			{
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
			}
			else if(last_direction == 1)
			{
				MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
				MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);
			}
		}
	}
}


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
	{
		MOTOR_CONTROLLERS[MOTOR_LEFT].TimCBPulse();
	}
	else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
	{
		MOTOR_CONTROLLERS[MOTOR_RIGHT].TimCBPulse();
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	MOTOR_CONTROLLERS[MOTOR_LEFT].TimCB();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].TimCB();
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

#ifdef __cplusplus
}
#endif
