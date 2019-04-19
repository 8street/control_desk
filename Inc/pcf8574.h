/*
 * pcf8574.h
 *
 *  Created on: 28 дек. 2018 г.
 *      Author: OASP
 */

#ifndef PCF8574_H_
#define PCF8574_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

volatile uint8_t PCF8574DelayUpdateFlag;

void LCD_Cursor_IT(uint8_t x, uint8_t y);
void LCD_CursorHome_IT(void);
void LCD_Clear_IT(void);
void LCD_AddComand_IT(uint8_t cmd);
void LCD_AddData_IT(uint8_t data);
void LCD_AddString_IT(char *str);

void PCF8574_UserInit(void);
void PCF8574_Update(void);
void PCF8574_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c);

#ifdef __cplusplus
}
#endif
#endif /* PCF8574_H_ */
