/*
 * ADS1118.c
 *
 *  Created on: 11 дек. 2018 г.
 *      Author: OASP
 */

#include "ADS1118.h"
#include "usbd_custom_hid_if.h"


//variables
extern SPI_HandleTypeDef hspi2;
extern volatile myInReportStruct DataToUSBhost;

static uint8_t switcher1 = 0;
static uint8_t switcher2 = 0;

SPI_HandleTypeDef* ADS_spiPtr;
GPIO_TypeDef* ADS1_Port_SS_pin;
uint16_t ADS1_SS_pin;
GPIO_TypeDef* ADS2_Port_SS_pin;
uint16_t ADS2_SS_pin;

//private function declaration
void ADS1_CS(GPIO_PinState PinState);
void ADS2_CS(GPIO_PinState PinState);

//functions
void ADS_UserInit(void)
{

	ADS_spiPtr = &hspi2;

	ADS1_Port_SS_pin = ADS1_CS_GPIO_Port;
	ADS1_SS_pin = ADS1_CS_Pin;

	ADS2_Port_SS_pin = ADS2_CS_GPIO_Port;
	ADS2_SS_pin = ADS2_CS_Pin;


	//4 options. 4th - reserve
	for (int i = 0; i < 4; i++)
	{
		//ADS1.
		ADS1[i].DataIn.SS = 0b1; // start conversion
		ADS1[i].DataIn.MUX = 0b001; // A0 A3
		ADS1[i].DataIn.PGA = 0b001; //+-4.096v
		ADS1[i].DataIn.MODE = 0b0; //0 - continius / 1 - single shot mode
		ADS1[i].DataIn.DR = 0b111; //860 sps
		ADS1[i].DataIn.TS_MODE = 0b0; //ADC temperature sensor mode
		ADS1[i].DataIn.PULL_UP_EN = 0b1; // Pull up resistor
		ADS1[i].DataIn.NOP = 0b01; // Valid data
		ADS1[i].DataIn.Reserved = 0b1; // res 1 write

		//ADS2
		ADS2[i].DataIn.SS = 0b1; // start conversion
		ADS2[i].DataIn.MUX = 0b001; // A0 A3
		ADS2[i].DataIn.PGA = 0b001; //+-4.096v
		ADS2[i].DataIn.MODE = 0b0; //0 - continius / 1 - single shot mode
		ADS2[i].DataIn.DR = 0b111; //860 sps
		ADS2[i].DataIn.TS_MODE = 0b0; //ADC temperature sensor mode
		ADS2[i].DataIn.PULL_UP_EN = 0b1; // Pull up resistor
		ADS2[i].DataIn.NOP = 0b01; // Valid data
		ADS2[i].DataIn.Reserved = 0b1; // res 1 write
	}

	//various mux settings
	ADS1[0].DataIn.MUX = 0b001; // A0 A3
	ADS1[1].DataIn.MUX = 0b010; // A1 A3
	ADS1[2].DataIn.MUX = 0b011; // A2 A3
	ADS1[3].DataIn.MUX = 0b001; // reserve

	ADS2[0].DataIn.MUX = 0b001; // A0 A3
	ADS2[1].DataIn.MUX = 0b010; // A1 A3
	ADS2[2].DataIn.MUX = 0b011; // A2 A3
	ADS2[3].DataIn.MUX = 0b001; // reserve

	// write settings
	ADS1_CS(GPIO_PIN_RESET);
	HAL_SPI_Transmit(ADS_spiPtr, ( uint8_t * ) &( ADS1[0].DataIn ), 1, PERIPHERAL_TIMEOUT);
    ADS1_CS(GPIO_PIN_SET);

	ADS2_CS(GPIO_PIN_RESET);
	HAL_SPI_Transmit(ADS_spiPtr, ( uint8_t * ) &( ADS2[0].DataIn ), 1, PERIPHERAL_TIMEOUT);
	ADS2_CS(GPIO_PIN_SET);
}
///////////////////////////////////////////////////
void ADS1118_Update(void)
{
	//ADS1
	if (ADS_spiPtr->State == HAL_SPI_STATE_READY)
	{
		ADS1_CS(GPIO_PIN_SET); //needs for normal operation. not delete this
		ADS1_CS(GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(SPI2_MISO_GPIO_Port, SPI2_MISO_Pin) == GPIO_PIN_RESET) //MISO pin. is ADS RDY (Ready)
		{
			switch (switcher1) // 4 options MUX, 2-3th - reserve
			{
				case 0:
					HAL_SPI_TransmitReceive_IT(ADS_spiPtr, (uint8_t *)  &( ADS1[0].DataIn ), (uint8_t *) &(ADS1[1].VoltageOut), 1);
					//HAL_SPI_TransmitReceive(ADS_spiPtr, (uint8_t *)  &( ADS1[0].DataIn ), (uint8_t *) &(ADS1[1].VoltageOut), 1, 1000);
					break;
				case 1:
				    HAL_SPI_TransmitReceive_IT(ADS_spiPtr, (uint8_t *)  &( ADS1[1].DataIn ), (uint8_t *) &(ADS1[0].VoltageOut), 1);
					//HAL_SPI_TransmitReceive(ADS_spiPtr, (uint8_t *)  &( ADS1[1].DataIn ), (uint8_t *) &(ADS1[0].VoltageOut), 1,1000);
					break;
				case 2:
					//HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t *)  &( ADS1[0].DataIn ), (uint8_t *) &(ADS1[2].VoltageOut), 2);
					//reserve
					break;
				case 3:
					//reserve
					break;
				default:
					switcher1 = 0;
					break;
			}
			switcher1++;
			if(switcher1 > 1) switcher1 = 0;
		}
		else
		{
			ADS1_CS(GPIO_PIN_SET);
		}
		//ADS1_CS(GPIO_PIN_SET);  // for blocking transmit functions
	}

	//ADS2
	if (ADS_spiPtr->State == HAL_SPI_STATE_READY)
	{
		ADS2_CS(GPIO_PIN_SET);
		ADS2_CS(GPIO_PIN_RESET);
		if(HAL_GPIO_ReadPin(SPI2_MISO_GPIO_Port, SPI2_MISO_Pin) == GPIO_PIN_RESET) //MISO pin. ADS READY
		{
			switch (switcher2)
			{
				case 0:
					HAL_SPI_TransmitReceive_IT(ADS_spiPtr, (uint8_t *)  &( ADS2[0].DataIn ), (uint8_t *) &(ADS2[1].VoltageOut), 1);
					break;
				case 1:
					HAL_SPI_TransmitReceive_IT(ADS_spiPtr, (uint8_t *)  &( ADS2[1].DataIn ), (uint8_t *) &(ADS2[0].VoltageOut), 1);
					break;
				case 2:
					//HAL_SPI_TransmitReceive_IT(&hspi2, (uint8_t *)  &( ADS2[0].DataIn ), (uint8_t *) &(ADS2[2].VoltageOut), 1);
					//reserve
					break;
				case 3:
					//reserve
					break;
				default:
					switcher2 = 0;
					break;
			}
			switcher2++;
			if(switcher2 > 1)
				switcher2 = 0;
		}
		else
		{
			ADS2_CS(GPIO_PIN_SET);
		}
	}

	// Convert the voltage in data
	// * 2*4096/3300 //2 is full range scale
	//(4096 is 4.096 volt of ADC reference)
	// (3300 is 3.300 volt of Power supply)
	int32_t axis_x = 65535 - ADS1[0].VoltageOut * 2048 / 825;
	int32_t axis_y = ADS1[1].VoltageOut * 2048 / 825;

	//limiters
	if(axis_x > 64535) axis_x = 0xFFFF;
	if(axis_x < 1000) axis_x = 0;

	if(axis_y > 64535) axis_y = 0xFFFF;
	if(axis_y < 1000) axis_y = 0;

	// Desk report
	DataToUSBhost.X = (uint16_t)axis_x;
	DataToUSBhost.Y = (uint16_t)axis_y;

	int32_t axis_z = 65535 - ADS2[0].VoltageOut * 2048 / 825; // (3300 is 3.300 volt of Power supply)
	int32_t axis_rz = ADS2[1].VoltageOut * 2048 / 825;

	//limiters
	if(axis_z > 64535) axis_z = 0xFFFF;
	if(axis_z < 1000) axis_z = 0;

	if(axis_rz > 64535) axis_rz = 0xFFFF;
	if(axis_rz < 1000) axis_rz = 0;

	DataToUSBhost.Z = (uint16_t)axis_z;
	DataToUSBhost.Rz = (uint16_t)axis_rz;
}


void ADS1_CS(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(ADS1_Port_SS_pin, ADS1_SS_pin, PinState);
}

void ADS2_CS(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(ADS2_Port_SS_pin, ADS2_SS_pin, PinState);
}

void ADS1118_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if (hspi->Instance == ADS_spiPtr->Instance) //if SPI2
	{
		//set all CS
		ADS1_CS(GPIO_PIN_SET);
		ADS2_CS(GPIO_PIN_SET);
	}
}
