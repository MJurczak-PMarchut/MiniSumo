#include "algo.hpp"
#include <stdio.h>
extern L9960T MOTOR_CONTROLLERS[];
extern VL53L5CX Sensors[];

#define SUMO_MODEL 1

int starter_counter = 0;
int starter_off_counter = 0;
bool starter_on = false;
uint8_t sumo_sensor_count[] = {3,2};
uint8_t sumo1_sensor_orientation[] = {1,2,3};
uint8_t sumo2_sensor_orientation[] = {0,0};
uint8_t default_y = 2;
int16_t detection_range = 250;
int16_t critical_distance = 80; //distance for PELNA KURWA
bool checkdata = false;

uint8_t get_sensor_orientation(uint8_t sensor)
{
	if (SUMO_MODEL == 1)
		return sumo1_sensor_orientation[sensor];
	return sumo2_sensor_orientation[sensor];
}

uint8_t translate_x(uint8_t sensor, uint8_t x, uint8_t y)
{
	uint8_t orientation = get_sensor_orientation(sensor);
	switch(orientation)
	{
	case 0:
		return x;
	case 1:
		return y;
	case 2:
		return 3-x;
	case 3:
		return 3-y;
	}
	return 0;
}

uint8_t translate_y(uint8_t sensor, uint8_t x, uint8_t y)
{
	uint8_t orientation = get_sensor_orientation(sensor);
	switch(orientation)
	{
	case 0:
		return y;
	case 1:
		return 3-x;
	case 2:
		return 3-y;
	case 3:
		return x;
	}
	return 0;
}

int16_t read_distance(int sensor, int x, int y)
{
	uint8_t x_translated = translate_x(sensor, x, y);
	uint8_t y_translated = translate_y(sensor, x, y);
	return Sensors[sensor].GetDataFromSensor(x, y);
}

int8_t search_for_opponent(int8_t last_position)
{
	int16_t dist_l = read_distance(0, 2, 2);
	int16_t dist_r = read_distance(2, 2, 2);
	int16_t dist_m = read_distance(1, 2, 2);

	MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
	if (dist_m > 0 && dist_m < detection_range)
	{
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
		return 0;
	}
	else if (dist_l > 0 && dist_l < detection_range)
	{
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(0);
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
		last_position = -2;
		return -1;
	}
	else if (dist_r > 0 && dist_r < detection_range)
	{
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(999);
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(0);
		last_position = -2;
		return 1;
	}
	return last_position;

}

void drive(int PWM_Right, int PWM_Left, bool R_forward, bool L_forward)
{
	if (R_forward)
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_FORWARD);
	else
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorDirection(MOTOR_DIR_BACKWARD);
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(PWM_Right);
	if (L_forward)
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_FORWARD);
	else
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorDirection(MOTOR_DIR_BACKWARD);
	MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(PWM_Left);
}


void algorithm()
{
	starter_on = (bool)HAL_GPIO_ReadPin(STARTER_Port, STARTER_Pin);
	if(checkdata)
	{
		MOTOR_CONTROLLERS[MOTOR_RIGHT].CheckControllerState();
		MOTOR_CONTROLLERS[MOTOR_LEFT].CheckControllerState();
	}
	if(starter_on)
	{

		int8_t opponent_direction;
		static int8_t last_position = -6;
		opponent_direction = search_for_opponent(last_position);
		last_position = opponent_direction;


	}
	else
	{
		MOTOR_CONTROLLERS[MOTOR_RIGHT].SetMotorPowerPWM(0);
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(0);
	}

}
