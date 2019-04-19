/*
 * tm1637.c
 *
 *  Created on: 28 дек. 2018 г.
 *      Author: OASP
 */


#include "usbd_custom_hid_if.h"

extern I2C_HandleTypeDef hi2c2;
//extern volatile myReportStruct Desk;


I2C_HandleTypeDef* TM_i2cPtr;



void TM1637_UserInit(void)
{
	TM_i2cPtr = &hi2c2;

	uint16_t dev_addr = 0;
	uint8_t test_send1 = 0x02;
	uint8_t test_send[] = {0x03, 0xee, 0x3f, 0x9c, 0x7a};
	uint8_t test_send2 = 0x51;

	HAL_I2C_Master_Transmit(TM_i2cPtr, dev_addr, &test_send1, 1, PERIPHERAL_TIMEOUT);

	HAL_I2C_Master_Transmit(TM_i2cPtr, dev_addr, test_send, 5, PERIPHERAL_TIMEOUT);

	HAL_I2C_Master_Transmit(TM_i2cPtr, dev_addr, &test_send2, 1, PERIPHERAL_TIMEOUT);



}

void TM1637_Update(void)
{

/*
	if(TM_i2cPtr->State == HAL_I2C_STATE_READY)
	{


	}
*/
}
