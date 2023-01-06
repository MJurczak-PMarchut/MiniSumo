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
#ifdef ROBOT_MT_V1
#include "miniTomi.hpp"
#elif ROBOT_IS_TOMISLAV

#endif


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


uint8_t pRx_Data[40] = {0};

CommManager MainCommManager;

#ifdef ROBOT_MT_V1
	miniTomi ROBOT = miniTomi();
#elif ROBOT_IS_TOMISLAV

#endif
uint16_t distance = 0;
HAL_StatusTypeDef transmit_status = HAL_ERROR;

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

}

void main_cpp(void * pvParameters)
{
	MainCommManager.AttachCommInt(&hspi2);
	MainCommManager.AttachCommInt(&hi2c1);
	HAL_Delay(10);
	ToF_Sensor::StartSensorTask();
//	ROBOT.run();
}


void EmStop(void)
{

}

#ifdef __cplusplus
}
#endif
