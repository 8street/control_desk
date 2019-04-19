


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ADS1118_H
#define __ADS1118_H
#ifdef __cplusplus
 extern "C" {
#endif


#include "main.h"

#pragma pack(push, 1)// располагает переменные в памяти структуры один за другим
typedef struct _ADSdataTypeDef
{
	uint8_t Reserved:1; //Always write 1h
	uint8_t NOP:2; // No operation
	uint8_t PULL_UP_EN:1; //Pullup enable DOUT
	uint8_t TS_MODE:1; // Temperature sensor mode
	uint8_t DR:3; //Data rate
	uint8_t MODE:1; // Device operating mode
	uint8_t PGA:3; // Programmable gain amplifier configuration
	uint8_t MUX:3; // Input multiplexer configuration
	uint8_t SS:1; // Single-shot conversion start
    }ADSdataTypeDef;
#pragma pack(pop)

typedef struct _ADS_HandleTypeDef
{
    ADSdataTypeDef DataIn;
    int16_t VoltageOut;
}ADS_HandleTypeDef;


ADS_HandleTypeDef ADS1[4];
ADS_HandleTypeDef ADS2[4];



void ADS_UserInit(void);
void ADS1118_Update(void);
void ADS1118_TxRxCpltCallback(SPI_HandleTypeDef *hspi);

//void SPIsend(uint16_t data);
//uint16_t SPIreceve(void);


#ifdef __cplusplus
}
#endif
#endif /*__ ADS1118_H */
