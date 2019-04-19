/*
 * mcp23017.h
 *
 *  Created on: 15 дек. 2018 г.
 *      Author: Николай
 */

#ifndef MCP23017_H_
#define MCP23017_H_
#ifdef __cplusplus
 extern "C" {
#endif



#include "main.h"

// strongly for IOCON.BANK = 0

#pragma pack(push, 8)// располагает переменные в памяти структуры один за другим
typedef struct _MCPregistersTypeDef
{
	uint8_t IODIRA; // Direction 1 - input 0 - output
	uint8_t IODIRB; // 0xFF default
	uint8_t IPOLA; // Input polarity 1 - opposite 0 - same
	uint8_t IPOLB; //
	uint8_t GPINTENA; // Interrupt on change event 1 - enable 0 - disable
	uint8_t GPINTENB; //
	uint8_t DEFVALA; //  Default value for interrupt
	uint8_t DEFVALB; //
	uint8_t INTCONA; // Interrupt on change control 1 - pin compared against DEFVAL 0 - pin compared against previous value
	uint8_t INTCONB; //
	uint8_t IOCON1; // Configuration register
	uint8_t IOCON2; //
	uint8_t GPPUA; // PULL-UP resistor 1 - enable 0 - disable
	uint8_t GPPUB; //
	uint8_t INTFA; // Interrupt flag 1 - caused interrupt 0 - interrupt not pending
	uint8_t INTFB; //
	uint8_t INTCAPA; // Interrupt capture value register 1 - high 0 - low
	uint8_t INTCAPB; //
	uint8_t GPIO_A; // GPIO value of the port 1 - high 0 - low
	uint8_t GPIO_B; //
	uint8_t OLATA; // Output Latch modifies the pins in output mode
	uint8_t OLATB; //
    }MCPregistersTypeDef;
#pragma pack(pop)


typedef struct _MCP_HandleTypeDef
{
    uint16_t Address;
	MCPregistersTypeDef Register;
}MCP_HandleTypeDef;


MCP_HandleTypeDef MCP23017[4];


// Functions declaration
void MCP_UserInit(void);
void MCP23017_Update(void);




#ifdef __cplusplus
}
#endif
#endif /* MCP23017_H_ */
