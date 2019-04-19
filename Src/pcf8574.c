/*
 * pcf8574.c
 *
 *  Created on: 28 дек. 2018 г.
 *      Author: OASP
 */

#include "pcf8574.h"
#include "usbd_custom_hid_if.h"
#include <stdlib.h>

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)
#define BUFFER_LEN 100

extern I2C_HandleTypeDef hi2c2;
//extern volatile myReportStruct Desk;

I2C_HandleTypeDef* PCF_i2cPtr;
uint16_t PCF_Address;
uint8_t LCD_X;
uint8_t LCD_Y;
uint8_t LCD_DELAY_MS;
uint8_t Backlight;
uint16_t SendBufer[BUFFER_LEN] = {0};
volatile uint8_t BuferLen = 0;
uint32_t UpdateTime;

void LCD_Update_IT(uint16_t lcd_addr, uint16_t *buffer);
void LCD_Send_IT(uint16_t lcd_addr, uint16_t data);

void LCD_Cursor(uint16_t lcd_addr, uint8_t x, uint8_t y);
void LCD_Clear(uint16_t lcd_addr);
void LCD_CursorHome(uint16_t lcd_addr);
void LCD_SendCommand(uint16_t lcd_addr, uint8_t cmd);
void LCD_SendData(uint16_t lcd_addr, uint8_t data);
void LCD_SendString(uint16_t lcd_addr, char *str);
void LCD_Init(uint16_t lcd_addr);
void LCD_SendInternal(uint16_t lcd_addr, uint8_t data, uint8_t flags);


void PCF8574_UserInit(void)
{
	PCF_i2cPtr = &hi2c2;
	PCF_Address = 0b01001110;// addr = 111
	LCD_X = 20;
	LCD_Y = 4;
	LCD_DELAY_MS = 5;
	Backlight = 1;
	PCF8574DelayUpdateFlag = 0;
	BuferLen = 0;

    LCD_Init(PCF_Address);

    LCD_Cursor(PCF_Address, 0, 0);
    LCD_SendString(PCF_Address, "StarShip ControlDesk");
    LCD_SendString(PCF_Address, "  Welcome Captain   ");
    LCD_SendString(PCF_Address, "StarFleet Pride You ");
    LCD_SendString(PCF_Address, "  Glory To Mankind  ");

   // HAL_Delay(100);

   // LCD_Clear(PCF_Address);
}

void PCF8574_Update(void)
{
	//char tmp[20];

	//if buffer is empty, set new data in buffer
	if(BuferLen == 0)
	{
		//LCD_Clear_IT();

		//LCD_Cursor_IT(0, 0);
		//sprintf(tmp, "Throttle:%d     ", Desk.Throttle);
		//LCD_AddString_IT((char *)dataReceive);

		//sprintf(tmp, "Slider:%d     ", Desk.Slider);
		//LCD_Cursor_IT(0, 1);
		//LCD_AddString_IT(tmp);


	}

	//send buffer to LCD
	LCD_Update_IT(PCF_Address, SendBufer);
}


void LCD_Cursor_IT(uint8_t x, uint8_t y)
{
	uint8_t sendData = x;

	if(y == 1)
	{
		sendData = sendData + 40;
	}
	else if(y == 2)
	{
		sendData = sendData + 20;
	}
	else if(y == 3)
	{
		sendData = sendData + 60;
	}

	if(sendData >= 80) return;
	sendData = sendData | 0b10000000;
	LCD_AddComand_IT(sendData);
}

void LCD_CursorHome_IT(void)
{
	LCD_AddComand_IT(0b00000010);
}

void LCD_Clear_IT(void)
{
	LCD_AddComand_IT(0b00000001);
}


void LCD_Update_IT(uint16_t lcd_addr, uint16_t *buffer)
{
	//Update PCF over I2C
	if((PCF_i2cPtr->State == HAL_I2C_STATE_READY) && ((Timer13 - UpdateTime) >= LCD_DELAY_MS) && BuferLen)
	{
		BuferLen--;
		LCD_Send_IT(lcd_addr, buffer[0]);
		PCF8574DelayUpdateFlag = 0;

		//shift all array to left at one element
		for(uint8_t i = 0; i < BuferLen; i++)
		{
			buffer[i] = buffer[i + 1];
		}
	}
}

void LCD_AddComand_IT(uint8_t cmd)
{
	if(BuferLen >= BUFFER_LEN) return;
    SendBufer[BuferLen] = (Backlight << 9) | cmd;
	BuferLen++;
}
void LCD_AddData_IT(uint8_t data)
{
	if(BuferLen >= BUFFER_LEN) return;
    SendBufer[BuferLen] = (Backlight << 9) | (1 << 8) | data; //  (1 << 8) is RS pin
	BuferLen++;
}

void LCD_AddString_IT(char *str)
{
    while(*str) {
    	LCD_AddData_IT((uint8_t)(*str));
        str++;
    }
}

void LCD_Send_IT(uint16_t lcd_addr, uint16_t data)
{

//PIN_RS    (1 << 0)
//PIN_EN    (1 << 2)
//BACKLIGHT (1 << 3)

	uint8_t rs = (data >> 8) & 1;
	uint8_t backlight = ((data >> 9) & 1) << 3;
	uint8_t pin_enable = PIN_EN;

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    static uint8_t data_arr[4];
    data_arr[0] = up|rs|backlight|pin_enable;
    data_arr[1] = up|rs|backlight;
    data_arr[2] = lo|rs|backlight|pin_enable;
    data_arr[3] = lo|rs|backlight;

    HAL_I2C_Master_Transmit_IT(PCF_i2cPtr, lcd_addr, data_arr, 4);
}

void LCD_Clear(uint16_t lcd_addr)
{
    LCD_SendCommand(lcd_addr, 0b00000001);
}

void LCD_Cursor(uint16_t lcd_addr, uint8_t x, uint8_t y)
{
	uint8_t sendData = x + y * LCD_X;
	if(sendData > 80) return;
	sendData = sendData | 0b10000000;
    LCD_SendCommand(lcd_addr, sendData);
}

void LCD_CursorHome(uint16_t lcd_addr)
{
    LCD_SendCommand(lcd_addr, 0b00000010);
}


void LCD_Init(uint16_t lcd_addr)
{
    // 4-bit mode, 2 lines, 5x8 format
    LCD_SendCommand(lcd_addr, 0b00110000);
    // display & cursor home (keep this!)
    LCD_CursorHome(lcd_addr);
    // display on, right shift, underline off, blink off
    LCD_SendCommand(lcd_addr, 0b00001100);
    // clear display (optional here)
    LCD_Clear(lcd_addr);
}

void LCD_SendInternal(uint16_t lcd_addr, uint8_t data, uint8_t flags)
{

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;

    HAL_I2C_Master_Transmit(PCF_i2cPtr, lcd_addr, data_arr, sizeof(data_arr), PERIPHERAL_TIMEOUT);
    HAL_Delay(LCD_DELAY_MS);
}

void LCD_SendCommand(uint16_t lcd_addr, uint8_t cmd)
{
    LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint16_t lcd_addr, uint8_t data)
{
    LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_SendString(uint16_t lcd_addr, char *str)
{
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}

void PCF8574_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == PCF_i2cPtr->Instance)
	{
		UpdateTime = Timer13;
	}
}
