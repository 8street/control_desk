/*
 * mcp23017.c
 *
 *  Created on: 15 дек. 2018 г.
 *      Author: Николай
 */

#include "mcp23017.h"
#include "usbd_custom_hid_if.h"

extern I2C_HandleTypeDef hi2c1;
extern volatile myInReportStruct DataToUSBhost;


I2C_HandleTypeDef* MCP_i2cPtr;
uint8_t UpdateFlag[4] = {0};


uint32_t SwitchButtons1(uint32_t input);
uint32_t SwitchButtons2(uint32_t input);

void MCP_UserInit(void)
{

	MCP_i2cPtr = &hi2c1;
	//Addressing
	MCP23017[0].Address = 0b01000000;//0
	MCP23017[1].Address = 0b01000010;//1
	MCP23017[2].Address = 0b01000100;//2
	MCP23017[3].Address = 0b01001000;//3

	//0-2 - input 3 - output
	for(int i = 0; i < 3; i++)
	{
		MCP23017[i].Register.IODIRA = 0xFF; // Direction 1 - input 0 - output
		MCP23017[i].Register.IODIRB = 0xFF; // 0xFF default
		MCP23017[i].Register.IPOLA = 0x00; // Input polarity 1 - opposite 0 - same
		MCP23017[i].Register.IPOLB = 0x00; //
		MCP23017[i].Register.GPINTENA = 0xFF; // Interrupt on change event 1 - enable 0 - disable
		MCP23017[i].Register.GPINTENB = 0xFF; //
		MCP23017[i].Register.DEFVALA = 0x00; //  Default value for interrupt
		MCP23017[i].Register.DEFVALB = 0x00; //
		MCP23017[i].Register.INTCONA = 0x00; // Interrupt on change control 1 - pin compared against DEFVAL 0 - pin compared against previous value
		MCP23017[i].Register.INTCONB = 0x00; //
		MCP23017[i].Register.IOCON1 = 0b01000010; // Configuration register
		MCP23017[i].Register.IOCON2 = 0b01000010; // INT mirror 1, INTPOL 1
		MCP23017[i].Register.GPPUA = 0xFF; // PULL-UP resistor 1 - enable 0 - disable
		MCP23017[i].Register.GPPUB = 0xFF; //
		MCP23017[i].Register.INTFA = 0x00; // Interrupt flag 1 - caused interrupt 0 - interrupt not pending
		MCP23017[i].Register.INTFB = 0x00; //
		MCP23017[i].Register.INTCAPA = 0x00; // Interrupt capture value register 1 - high 0 - low
		MCP23017[i].Register.INTCAPB = 0x00; //
		MCP23017[i].Register.GPIO_A = 0xFF; // GPIO value of the port 1 - high 0 - low
		MCP23017[i].Register.GPIO_B = 0xFF; //
		MCP23017[i].Register.OLATA = 0x00; // Output Latch modifies the pins in output mode
		MCP23017[i].Register.OLATB = 0x00; //
	}

	//3 - output
	MCP23017[3].Register.IODIRA = 0x00; // Direction 1 - input 0 - output
	MCP23017[3].Register.IODIRB = 0x00; // 0xFF default
	MCP23017[3].Register.IPOLA = 0x00; // Input polarity 1 - opposite 0 - same
	MCP23017[3].Register.IPOLB = 0x00; //
	MCP23017[3].Register.GPINTENA = 0x00; // Interrupt on change event 1 - enable 0 - disable
	MCP23017[3].Register.GPINTENB = 0x00; //
	MCP23017[3].Register.DEFVALA = 0x00; //  Default value for interrupt
	MCP23017[3].Register.DEFVALB = 0x00; //
	MCP23017[3].Register.INTCONA = 0x00; // Interrupt on change control 1 - pin compared against DEFVAL 0 - pin compared against previous value
	MCP23017[3].Register.INTCONB = 0x00; //
	MCP23017[3].Register.IOCON1 = 0b01000010; // Configuration register
	MCP23017[3].Register.IOCON2 = 0b01000010; // INT mirror 1, INTPOL 1
	MCP23017[3].Register.GPPUA = 0x00; // PULL-UP resistor 1 - enable 0 - disable
	MCP23017[3].Register.GPPUB = 0x00; //
	MCP23017[3].Register.INTFA = 0x00; // Interrupt flag 1 - caused interrupt 0 - interrupt not pending
	MCP23017[3].Register.INTFB = 0x00; //
	MCP23017[3].Register.INTCAPA = 0x00; // Interrupt capture value register 1 - high 0 - low
	MCP23017[3].Register.INTCAPB = 0x00; //
	MCP23017[3].Register.GPIO_A = 0x00; // GPIO value of the port 1 - high 0 - low
	MCP23017[3].Register.GPIO_B = 0x00; //
	MCP23017[3].Register.OLATA = 0x00; // Output Latch modifies the pins in output mode
	MCP23017[3].Register.OLATB = 0x00; //

	///I2C send MCP settings
	for(int i = 0; i < 3; i++)
	{
		HAL_I2C_Mem_Write(MCP_i2cPtr, MCP23017[i].Address,  0x00, 1, &(MCP23017[i].Register.IODIRA), 14, PERIPHERAL_TIMEOUT);
	}
}

void MCP23017_Update(void)
{
	// Address of the registers in MCP
	uint16_t regAddressGPIOA = 0x12;
	uint16_t regAddressOLATA = 0x14;

	//Set update flag for MCP3 if change the registers OLAT
	static uint16_t tmp;
	if (tmp != (MCP23017[3].Register.OLATB << 8 | MCP23017[3].Register.OLATA))
		UpdateFlag[3] = 1;

	tmp = MCP23017[3].Register.OLATB << 8 | MCP23017[3].Register.OLATA;


	//Update all MCP over I2C
	if(MCP_i2cPtr->State == HAL_I2C_STATE_READY && HAL_GPIO_ReadPin(MCP1_EXTI5_GPIO_Port, MCP1_EXTI5_Pin))
	{//read inputs if external MCP interrupt pending
			HAL_I2C_Mem_Read_IT(MCP_i2cPtr, MCP23017[0].Address, regAddressGPIOA, 1, &(MCP23017[0].Register.GPIO_A), 2);
	}
	if(MCP_i2cPtr->State == HAL_I2C_STATE_READY && HAL_GPIO_ReadPin(MCP2_EXTI6_GPIO_Port, MCP2_EXTI6_Pin))
	{//read inputs if external MCP interrupt pending
			HAL_I2C_Mem_Read_IT(MCP_i2cPtr, MCP23017[1].Address, regAddressGPIOA, 1, &(MCP23017[1].Register.GPIO_A), 2);
	}
	if(MCP_i2cPtr->State == HAL_I2C_STATE_READY && HAL_GPIO_ReadPin(MCP3_EXTI7_GPIO_Port, MCP3_EXTI7_Pin))
	{//read inputs if external MCP interrupt pending
			HAL_I2C_Mem_Read_IT(MCP_i2cPtr, MCP23017[2].Address, regAddressGPIOA, 1, &(MCP23017[2].Register.GPIO_A), 2);
	}
	if(MCP_i2cPtr->State == HAL_I2C_STATE_READY && UpdateFlag[3])
	{//write outputs if change the data
			HAL_I2C_Mem_Write_IT(MCP_i2cPtr, MCP23017[3].Address, regAddressOLATA, 1, &(MCP23017[3].Register.OLATA), 2);
			MCP23017[3].Register.GPIO_A = MCP23017[3].Register.OLATA;
			MCP23017[3].Register.GPIO_B = MCP23017[3].Register.OLATB;
			UpdateFlag[3] = 0;
	}

	// USB report update
	// Buttons1 =  GPIOB1.GPIOA1.GPIOB0.GPIOA0
	uint32_t MCP_buttons1 = ~(MCP23017[1].Register.GPIO_B << 24 | MCP23017[1].Register.GPIO_A << 16 | MCP23017[0].Register.GPIO_B << 8 | MCP23017[0].Register.GPIO_A);
	// Buttons2 = reserve.reserve.GPIOB2.GPIOA2
	uint32_t MCP_buttons2 = ~( 0xFFFF0000 | MCP23017[2].Register.GPIO_B << 8 | MCP23017[2].Register.GPIO_A); // 0xFFFF 0000 - is reserve
	DataToUSBhost.Buttons1 = MCP_buttons1;
	//Desk.Buttons2 = SwitchButtons2(MCP_buttons2);
}

// function made from switch to momentary buttons with time of press
uint32_t SwitchButtons1(uint32_t input)
{
	static uint32_t tmp;
	static uint32_t retVal;
	static uint32_t timer;

	if(tmp != input)
	{
		retVal = retVal | (tmp ^ input); // ^ - XOR
	    timer = 10000;// initial value
		tmp = input;
	}

	timer--;

	if(timer < 1)
	{
		retVal = 0;
	}
	return retVal;
}

// function made from switch to momentary buttons
uint32_t SwitchButtons2(uint32_t input)
{
	static uint32_t tmp;
	static uint32_t retVal;
	static uint32_t timer;

	if(tmp != input)
	{
		retVal = retVal | (tmp ^ input);
	    timer = 10000;// initial value
		tmp = input;
	}

	timer--;

	if(timer < 1)
	{
		retVal = 0;
	}
	return retVal;
}

// Callback from interrupt
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//this not properly work
	/*
	// set flag for read GPIO from MCP23017
	switch (GPIO_Pin)
	{
		case GPIO_PIN_5:
			UpdateFlag[0] = 1;
		break;
		case GPIO_PIN_6:
			UpdateFlag[1] = 1;
		break;
		case GPIO_PIN_7:
			UpdateFlag[2] = 1;
		break;
		case GPIO_PIN_8:

		break;
		default:

		break;
	}
	*/
}
