#include "algo.hpp"
#include <stdio.h>
extern L9960T MOTOR_CONTROLLERS[];
extern VL53L5CX Sensors[];


void algorithm()
{

	int16_t distance = Sensors[0].GetDataFromSensor(3, 0);
	printf("distance");
	if(distance < 1000)
	{
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(999);
	}
	else
	{
		MOTOR_CONTROLLERS[MOTOR_LEFT].SetMotorPowerPWM(500);
	}


}
