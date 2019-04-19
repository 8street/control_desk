/*
 * max7219.h
 *
 *  Created on: 20 дек. 2018 г.
 *      Author: OASP
 */

#ifndef MAX7219_H_
#define MAX7219_H_
#ifdef __cplusplus
 extern "C" {
#endif

#include "main.h"

 char LED_Display0[10];
 char LED_Display1[10];
 char LED_Display2[10];
 char LED_Display3[10];

uint8_t MAX7219DelayUpdate;

void MAX7219_UserInit(void);
void MAX7219_Update(void);
void MAX7219_TxRxCpltCallback(SPI_HandleTypeDef *hspi);


#ifdef __cplusplus
}
#endif
#endif /* MAX7219_H_ */
