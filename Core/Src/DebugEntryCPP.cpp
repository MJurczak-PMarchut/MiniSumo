/*
 * EntryCPP.cpp
 *
 *  Created on: Nov 21, 2021
 *      Author: Mateusz
 */
#include "EntryCPP.hpp"

#include "Configuration.h"
#include "I-BUS/I-BUS.hpp"
#include "L9960T.hpp"
#include "RobotSpecificDefines.hpp"
#include "LineDetectors.hpp"
#include "vl53l5cx.hpp"
#include "algo.hpp"
#include "VL53L1X_api.hpp"
#include "ToFSensor.hpp"
#ifdef __cplusplus
extern "C" {
#endif

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern UART_HandleTypeDef huart7;
extern I2C_HandleTypeDef hi2c1;
extern SPI_HandleTypeDef hspi2;
extern uint8_t _data_receiveL[3];
extern uint8_t _data_receiveR[3];
extern DMA_HandleTypeDef hdma_uart7_rx;

void EmStop(void);

uint32_t FL, FR, BL, BR;

uint8_t pRx_Data[40] = {0};


//TLE5205 MOTOR_CONTROLLERS[] = {[MOTOR_LEFT] = TLE5205(MOTOR_LEFT, &htim2, TIM_CHANNEL_1), [MOTOR_RIGHT] = TLE5205(MOTOR_RIGHT, &htim2, TIM_CHANNEL_2)};
CommManager MainCommManager;
L9960T MOTOR_CONTROLLERS[] = {
		[MOTOR_LEFT] = L9960T(MOTOR_LEFT, &hspi2, &MainCommManager, TIM_CHANNEL_1, &htim3),
		[MOTOR_RIGHT] = L9960T(MOTOR_RIGHT, &hspi2, &MainCommManager, TIM_CHANNEL_3, &htim4)};

IBus RxController(&huart7, EmStop, pRx_Data, &hdma_uart7_rx);
LineDetectors LDLineDetectors(4);
//VL53L1X vl53l1x = VL53L1X(&hi2c1, &MainCommManager);
VL53L5CX Sensors[] ={ VL53L5CX(FRONT_LEFT, &MainCommManager, &hi2c1), VL53L5CX(FRONT_LEFT, &MainCommManager, &hi2c1), VL53L5CX(FRONT_LEFT, &MainCommManager, &hi2c1) };
MessageInfoTypeDef MsgInfo = {0};
uint16_t distance = 0;
HAL_StatusTypeDef transmit_status = HAL_ERROR;

void InitControllers(void);
void InitLineDetectors(void);

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

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	MainCommManager.MsgReceivedCB(hi2c);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	MainCommManager.MsgReceivedCB(hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c){
	MainCommManager.MsgReceivedCB(hi2c);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
	MainCommManager.MsgReceivedCB(hi2c);
}

void InitSensors(void)
{
	ToF_Sensor::StartSensorTask();
}

void main_cpp(void * pvParameters )
{
	MainCommManager.AttachCommInt(&hspi2);
	MainCommManager.AttachCommInt(&hi2c1);
	InitControllers();
	InitLineDetectors();
	MOTOR_CONTROLLERS[MOTOR_LEFT].Enable();
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Enable();
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
 	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);

 	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(0);
 	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(0);
 	 while(1)
	{
 		vTaskDelay(1);
 		algorithm();

	}
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == TOF_GPIO_6_Pin || GPIO_Pin == TOF_GPIO_5_Pin || GPIO_Pin == TOF_GPIO_4_Pin || GPIO_Pin == TOF_GPIO_3_Pin || GPIO_Pin == TOF_GPIO_2_Pin)
	{
		ToF_Sensor::EXTI_Callback_func(GPIO_Pin);
	}
}

void EmStop(void)
{

}

void InitControllers(void)
{
	HAL_GPIO_WritePin(MD_NDIS_GPIO_Port, MD_NDIS_Pin, GPIO_PIN_SET);
	MOTOR_CONTROLLERS[MOTOR_LEFT].Init(0);
	while(MOTOR_CONTROLLERS[MOTOR_LEFT].CheckIfControllerInitializedOk() != HAL_OK)
	{}
	MOTOR_CONTROLLERS[MOTOR_RIGHT].Init(0);
	while(MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckIfControllerInitializedOk() != HAL_OK)
	{}

}

void InitLineDetectors(void)
{
	LineDetectors_t LD_List[2] = {[0] = LD_FRONT_LEFT, [1] = LD_FRONT_RIGHT};
	LDLineDetectors.AttachDetectors(&hadc1, LD_List, 2);
	LD_List[0] = LD_BACK_RIGHT;
	LD_List[1] = LD_BACK_LEFT;
	LDLineDetectors.AttachDetectors(&hadc2, LD_List, 2);
}

#ifdef __cplusplus
}
#endif
