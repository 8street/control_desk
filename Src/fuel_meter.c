/*
 * fuel_meter.c
 *
 *  Created on: 31 џэт. 2019 у.
 *      Author: OASP
 */

#include "fuel_meter.h"
#include "tim.h"

uint16_t FuelMeter;

void FuelMeter_UserInit(void)
{

	htim11.Instance->CCR1 = 5000;
	FuelMeter = 5000;
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);

}

void FuelMeter_Update(void)
{

	FuelMeter = FuelMeter + 200;


//	FuelMeter = DataFromUSBhost.Fuel;

	//limiter
	if(FuelMeter > 10000)  FuelMeter = 0;
//	if(FuelMeter > 10000)  FuelMeter = 10000;

	//update PWM
	htim11.Instance->CCR1 = FuelMeter;
}
