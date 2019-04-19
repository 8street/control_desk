/*
* max7219.c
 *
 *  Created on: 20 дек. 2018 г.
 *      Author: OASP
 */

#include "max7219.h"
#include "usbd_custom_hid_if.h"
#include <math.h>

//extern variables
extern SPI_HandleTypeDef hspi1;
//extern volatile myReportStruct Desk;

typedef enum
{
  OFF = 0,
  ON
}MAX7219_State;

//variables
SPI_HandleTypeDef* MAX7219_spiPtr;
GPIO_TypeDef* Port_SS_pin;
uint16_t SS_pin;
uint16_t ennumerator;


// Private function declaration
uint32_t CharToSendbufer(char *buf0, char *buf1, char *buf2, char *buf3, uint8_t digit);
uint8_t IntToDigit(int32_t value, uint8_t digit, uint8_t dot);
uint32_t Int32ToSendbufer(int32_t value0, int32_t value1, int32_t value2, int32_t value3, uint8_t digit);
uint32_t Int32dotToSendbufer(int32_t value0, int32_t value1, int32_t value2, int32_t value3,
							uint8_t dot0, uint8_t dot1, uint8_t dot2, uint8_t dot3,
							uint8_t digit);
uint32_t FloatToSendbuffer(float value0, float value1, float value2, float value3,
		uint8_t digitAfterDot0, uint8_t digitAfterDot1, uint8_t digitAfterDot2, uint8_t digitAfterDot3,
		uint8_t digit);
uint8_t charToDigit(char *buf, uint8_t digit);

void MAX7219_CS(GPIO_PinState PinState);

void MAX7219_DecodeMode(uint8_t decode);
void MAX7219_Intensity(uint8_t intensity);
void MAX7219_ScanLimit(uint8_t limit);
void MAX7219_DisplayTest(uint8_t st);
void MAX7219_Enable(uint8_t st);
void MAX7219_Send_IT(uint8_t regAddr, uint32_t send_buffer);
void MAX7219_Send(uint8_t regAddr, uint8_t buffer);

//////////////////////functions////////////////////////////////////////
void MAX7219_UserInit(void)
{
	MAX7219_spiPtr = &hspi1;

	Port_SS_pin = MAX1_CS_GPIO_Port;
	SS_pin = MAX1_CS_Pin;

	//HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_SET);

	MAX7219_Enable(ON);
	MAX7219_DisplayTest(OFF);
	MAX7219_ScanLimit(7);
	MAX7219_Intensity(0x03); //0x0F - max
	MAX7219_DecodeMode(0xFF); // code B for all digits

	//nulling all digit
	for(int digit = 1; digit <= 8; digit++)
	{
		MAX7219_Send(digit, 0);
	}
}

char LED_Display0[10] = "00000000";
char LED_Display1[10] = "00000000";
char LED_Display2[10] = "00000000";
char LED_Display3[10] = "00000000";

//test variables
int32_t v0 = 0;
int32_t v1 = 0;
int32_t v2 = 0;
int32_t v3 = 0;

float value0 = 0.0F;
float value1 = 123.456F;
float value2 = -123456.0F;
float value3 = -123.456F;

void MAX7219_Update(void)
{
	//static uint8_t updateFlag;
	static uint8_t updateFlag[8] = {1, 1, 1, 1, 1, 1, 1, 1};
	uint32_t send_buf;    // send command buffer for 4 LCD
	static uint8_t digit; // number of update digit

    //set update flags
//	if(!updateFlag[0] && !updateFlag[1] && !updateFlag[2]&& !updateFlag[3]&& !updateFlag[4]&& !updateFlag[5]&& !updateFlag[6]&& !updateFlag[7]) // no delay send
	if(MAX7219DelayUpdate)
	{
		MAX7219DelayUpdate = 0;
		for(int i=0; i < 8; i++)
		{
			updateFlag[i] = 1;
		}

		//update all value
	    //test
	    //sprintf(display0, "%.4f", value0);
/*
		v0 = Desk.X;
		v1 = Desk.Y;
		v2 = Desk.Z;
		v3 = Desk.Rz;

	    value0 = (float)Desk.X;
	    value1 = (float)Desk.Y;
	    value2 = (float)Desk.Z;
	    value3 = (float)Desk.Rz;
	    */
 	}

	//send digit if SPI ready
	if(updateFlag[digit] && (MAX7219_spiPtr->State == HAL_SPI_STATE_READY))
	{
		//send_buf = Int32ToSendbufer(v0, v1, v2, v3, digit); // send int value
		//send_buf = FloatToSendbuffer(value0, value1, value2, value3, 2, 2, 2, 2, digit); // send float value with dot
		send_buf = CharToSendbufer(LED_Display0, LED_Display1, LED_Display2, LED_Display3, digit); // send array of char
		MAX7219_Send_IT(digit + 1, send_buf);
		updateFlag[digit] = 0;
	}
	digit++;
	if(digit > 7) digit = 0;
}

///////////////////.

uint32_t Int32ToSendbufer(int32_t value0, int32_t value1, int32_t value2, int32_t value3, uint8_t digit)
{
	uint8_t v0 = IntToDigit(value0, digit, 0xFF);
	uint8_t v1 = IntToDigit(value1, digit, 0xFF);
	uint8_t v2 = IntToDigit(value2, digit, 0xFF);
	uint8_t v3 = IntToDigit(value3, digit, 0xFF);
	return (v0 << 24) | (v1 << 16) | (v2 << 8) | v3;
}

uint32_t Int32dotToSendbufer(int32_t value0, int32_t value1, int32_t value2, int32_t value3,
							uint8_t dot0, uint8_t dot1, uint8_t dot2, uint8_t dot3,
							uint8_t digit)
{
	uint8_t v0 = IntToDigit(value0, digit, dot0);
	uint8_t v1 = IntToDigit(value1, digit, dot1);
	uint8_t v2 = IntToDigit(value2, digit, dot2);
	uint8_t v3 = IntToDigit(value3, digit, dot3);
	return (v0 << 24) | (v1 << 16) | (v2 << 8) | v3;
}

uint32_t FloatToSendbuffer(float value0, float value1, float value2, float value3,
		uint8_t digitAfterDot0, uint8_t digitAfterDot1, uint8_t digitAfterDot2, uint8_t digitAfterDot3,
		uint8_t digit)
{
	//convert to int32
	int32_t v0 = (int32_t)(value0 * powf(10, digitAfterDot0));
	int32_t v1 = (int32_t)(value1 * powf(10, digitAfterDot1));
	int32_t v2 = (int32_t)(value2 * powf(10, digitAfterDot2));
	int32_t v3 = (int32_t)(value3 * powf(10, digitAfterDot3));

	//autoset digitAfterDot if display overflow
	while( (v0 > 99999999 || v0 < -9999999) && digitAfterDot0 > 0)
	{
		v0 = v0 / 10;
		digitAfterDot0--;
	}
	while( (v1 > 99999999 || v1 < -9999999) && digitAfterDot1 > 0)
	{
		v1 = v1 / 10;
		digitAfterDot1--;
	}
	while( (v2 > 99999999 || v2 < -9999999) && digitAfterDot2 > 0)
	{
		v2 = v2 / 10;
		digitAfterDot2--;
	}
	while( (v3 > 99999999 || v3 < -9999999) && digitAfterDot3 > 0)
	{
		v3 = v3 / 10;
		digitAfterDot3--;
	}

	return Int32dotToSendbufer(v0, v1, v2, v3,
			digitAfterDot0, digitAfterDot1, digitAfterDot2, digitAfterDot3,
			digit);
}

uint8_t IntToDigit(int32_t value, uint8_t digit, uint8_t dot)
{
	//Function parse the value and get one digit of it. And set dot, blank or minus on the display if need
	//Example value = "123456789", digit = "1", then retVal = "8"
	// value = "123456789", digit = "0", then retVal = "9"

	if(value > 99999999 || value < -9999999) return 8 | 128; //display overflow. LCD set 8.8.8.8.8.8.8.8.

	uint8_t retVal = 0;
	uint8_t i = 0;
	int32_t bufVal = value;

	if (bufVal < 0) bufVal = bufVal * (-1);

	do
	{
		if (i == digit)
		{
			retVal = bufVal % 10;
		}
		bufVal = bufVal / 10;
		i++;
	}while (bufVal);

	if(dot == digit && dot > 0) retVal = 0b10000000 | retVal; // dot on the display "."
	if (digit >= i) retVal = 0b1111; // blank on display " "
	if (value < 0 && i == digit) retVal = 0b1010; // minus on display "-"

	return retVal;
}

uint32_t CharToSendbufer(char *buf0, char *buf1, char *buf2, char *buf3, uint8_t digit)
{
	uint8_t v0 = charToDigit(buf0, digit);
	uint8_t v1 = charToDigit(buf1, digit);
	uint8_t v2 = charToDigit(buf2, digit);
	uint8_t v3 = charToDigit(buf3, digit);
	return (v0 << 24) | (v1 << 16) | (v2 << 8) | v3;
}

uint8_t charToDigit(char *buf, uint8_t digit)
{
	//Function parse array chars and get one digit of it. And set dot, blank or minus on the display if need
	//Example buf[] = "12345678", digit = 1, then retVal = 7
	uint8_t dot_pos = 0;
	uint8_t dot = 0;

	uint16_t len = strlen(buf);
	uint8_t shiftFlag = 0;
	uint8_t minus = 0;
	uint8_t minus_pos = 0;

//	if (len > 8) return 8 | 128; //overflow. LCD set all digit "8." with dot

	char *end; //
	char out; //

	int i;

	//find '-', ',', '.' in array buf[]
	for (i = len - 1; i >= 0; i--)
	{
		if (buf[i] == '-')
		{
			minus_pos = len - 1 - i;
			minus = 1;
		}
		else if (buf[i] == '.' || buf[i] == ',')
		{
			dot_pos = len - 1 - i;
			dot = 1;
			len--;
		}
	}

	if (dot && digit < dot_pos)	shiftFlag = 1; // shift one digit if dot exist because dot '.' no need room in LCD symbol

	out = buf[len - digit - 1 + shiftFlag];

	uint16_t retVal = 0;
/*
	if(out == 'E')
	{
		retVal = 0b1011; // E on display
	}else if(out == 'H')
	{
		retVal = 0b1100; // H on display
	}else if(out == 'L')
	{
		retVal = 0b1101; // L on display
	}else if(out == 'P')
	{
		retVal = 0b1110; // P on display
	}else{*/
		retVal = strtol(&out, &end, 10); //convert to int

		if (digit >= len) retVal = 0b1111; // blank on display " "
		if (minus && minus_pos == digit) retVal = 0b1010;// minus on display "-"
		if (dot && dot_pos == digit) retVal = retVal | 0b10000000; // dot on display "."
	//}

	return retVal;
}

void MAX7219_CS(GPIO_PinState PinState)
{
	HAL_GPIO_WritePin(Port_SS_pin, SS_pin, PinState);
}

void MAX7219_DecodeMode(uint8_t decode)
{
	MAX7219_Send(0x09, decode);
}

void MAX7219_Intensity(uint8_t intensity)
{
	MAX7219_Send(0x0A, intensity);
}


void MAX7219_ScanLimit(uint8_t limit)
{
	MAX7219_Send(0x0B, limit);
}

void MAX7219_DisplayTest(uint8_t st)
{
	MAX7219_Send(0x0F, st);
}

void MAX7219_Enable(uint8_t st)
{
	MAX7219_Send(0x0C, st);
}


void MAX7219_Send_IT(uint8_t regAddr, uint32_t send_buffer)
{
	//1x LCD receive uint8_t regAddr and uint8_t command
	//4x LCD receive 4*(1x LCD)
	//Non blocking function. All 4 register address set same. Command is send_buffer
	//send_buffer is four byte for four LCD command
	static uint16_t sendArray[4];
	sendArray[0] = (regAddr << 8) | *((uint8_t *)(&send_buffer));
	sendArray[1] = (regAddr << 8) | *(((uint8_t *)(&send_buffer))+1);
	sendArray[2] = (regAddr << 8) | *(((uint8_t *)(&send_buffer))+2);
	sendArray[3] = (regAddr << 8) | *(((uint8_t *)(&send_buffer))+3);

	MAX7219_CS(OFF);
	HAL_SPI_Transmit_IT(MAX7219_spiPtr, (uint8_t *)sendArray, 4); //size 4 = 4 word. SPI in 16 bit mode
}

void MAX7219_Send(uint8_t regAddr, uint8_t buffer)
{
	//Blocking function. All 4 register address set same. All 4 command set same

	uint16_t sendData = (regAddr << 8) | buffer;
	uint16_t sendArray[4];
	//uint8_t receiveArray[8];

	sendArray[0] = sendData;
	sendArray[1] = sendData;
	sendArray[2] = sendData;
	sendArray[3] = sendData;

	MAX7219_CS(ON);//not need but for reliable
	MAX7219_CS(OFF);
	HAL_SPI_Transmit(MAX7219_spiPtr, (uint8_t *)sendArray, 4, PERIPHERAL_TIMEOUT); // size = 4 because sends 16bit word
	MAX7219_CS(ON);
}

/**
  * @brief Tx Transfer completed callback.
  * @param  hspi pointer to a SPI_HandleTypeDef structure that contains
  *               the configuration information for SPI module.
  * @retval None
  */
void MAX7219_TxRxCpltCallback(SPI_HandleTypeDef *hspi)
{
	if(hspi->Instance == MAX7219_spiPtr->Instance)
	{
		//on CS after complete IT transaction
		MAX7219_CS(ON);
	}
}

