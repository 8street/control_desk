/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */
#include "pcf8574.h"
#include "max7219.h"
/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */

/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */

	//AXIS
	0x05, 0x01,                    // USAGE_PAGE (Generic Desktop)
	0x09, 0x04,                    // USAGE (Joystick)
	0xa1, 0x01,                    // COLLECTION (Application)28
	0x05, 0x02,                    //   USAGE_PAGE (Simulation Controls)
	0x09, 0xbb,                    //   USAGE (Throttle)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
	0x75, 0x10,                    //   REPORT_SIZE (16)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x85, 0x01,					   //   REPORT_ID (1)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
	0x09, 0x01,                    //   USAGE (Pointer)
	0xa1, 0x00,                    //   COLLECTION (Physical)
	0x09, 0x30,                    //     USAGE (X)
	0x09, 0x31,                    //     USAGE (Y)
	0x95, 0x02,                    //     REPORT_COUNT (2)
	0x81, 0x02,                    //     INPUT (Data,Var,Abs)
	0xc0,                          //   END_COLLECTION
	0x05, 0x01,                    //   USAGE_PAGE (Generic Desktop)
	0x09, 0x32,                    //   USAGE (Z)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x09, 0x33,                    //   USAGE (Rx)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x09, 0x34,                    //   USAGE (Ry)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x09, 0x35,                    //   USAGE (Rz)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)
	0x09, 0x36,                    //   USAGE (Slider)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)

	//HAT
	0x09, 0x39,                    //   USAGE (Hat switch)
	0x15, 0x01,                    //   LOGICAL_MINIMUM (1)
	0x25, 0x08,                    //   LOGICAL_MAXIMUM (8)
	0x35, 0x00,                    //   PHYSICAL_MINIMUM (0)
	0x46, 0x0e, 0x01,              //   PHYSICAL_MAXIMUM (270)
	0x65, 0x14,                    //   UNIT (Eng Rot:Angular Pos)
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)

	//Buttons
	0x05, 0x09,                    //   USAGE_PAGE (Button)
	0x19, 0x01,                    //   USAGE_MINIMUM (Button 1)
	0x29, 0x40,                    //   USAGE_MAXIMUM (Button 64)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x25, 0x01,                    //   LOGICAL_MAXIMUM (1)
	0x75, 0x01,                    //   REPORT_SIZE (1)
	0x95, 0x40,                    //   REPORT_COUNT (64)
	0x55, 0x00,                    //   UNIT_EXPONENT (0)
	0x65, 0x00,                    //   UNIT (None)
	0x81, 0x02,                    //   INPUT (Data,Var,Abs)

	//LEDs
	0x85, 0x02,					   // REPORT_ID (2)
	0x05, 0x08,                    // USAGE_PAGE (LEDs)
	0x09, 0x4B,                    // USAGE (Generic Indicator)
	0x95, 0x40,                    // REPORT_COUNT (16)
	0x91, 0x02,                    // OUTPUT (Data,Var,Abs)
	0xc0,                          // END_COLLECTION

	//LCD Displays
    0x05, 0x14,                    // USAGE_PAGE (Alphnumeric Display)
	0x09, 0x01,                    // USAGE (Alphanumeric Display)
	0x15, 0x00,                    // LOGICAL_MINIMUM (0)
	0xa1, 0x02,                    // COLLECTION (Logical)
/*
	0x75, 0x08,                    //   REPORT_SIZE (8)
	0x95, 0x01,                    //   REPORT_COUNT (1)
	0x25, 0x02,                    //   LOGICAL_MAXIMUM (2)
	0x09, 0x2d,                    //   USAGE (Display Status)
	0xa1, 0x02,                    //   COLLECTION (Logical)
	0x09, 0x2e,                    //     USAGE (Stat Not Ready)
	0x09, 0x2f,                    //     USAGE (Stat Ready)
	0x09, 0x30,                    //     USAGE (Err Not a loadable character)
	0x81, 0x40,                    //     INPUT (Data,Ary,Abs,Null)
	0xc0,                          //   END_COLLECTION
*/
	0x09, 0x32,                    //   USAGE (Cursor Position Report)
	0xa1, 0x02,                    //   COLLECTION (Logical)
	0x85, 0x04,                    //     REPORT_ID (4)
	0x75, 0x08,                    //     REPORT_SIZE (8)
	0x95, 0x01,                    //     REPORT_COUNT (1)
	0x25, 0x13,                    //     LOGICAL_MAXIMUM (19)
	0x09, 0x34,                    //     USAGE (Column)
	0xb1, 0x22,                    //     FEATURE (Data,Var,Abs,NPrf)
	0x25, 0x03,                    //     LOGICAL_MAXIMUM (3)
	0x09, 0x33,                    //     USAGE (Row)
	0x91, 0x22,                    //     OUTPUT (Data,Var,Abs,NPrf)
	0xc0,                          //   END_COLLECTION

	0x09, 0x2b,                    //   USAGE (Character Report)
	0xa1, 0x02,                    //   COLLECTION (Logical)
	0x85, 0x05,                    //     REPORT_ID (5)
	0x95, 0x14,                    //     REPORT_COUNT (20)
	0x26, 0xFF, 0x00,              //     LOGICAL_MAXIMUM (255)
	0x09, 0x2c,                    //     USAGE (Display Data)
	0x92, 0x02, 0x01,              //     OUTPUT (Data,Var,Abs,Buf)
	0xc0,                          //   END_COLLECTION


    0x09, 0x24,                    // USAGE (Display Control Report)
    0x85, 0x06,                    // REPORT_ID (6)
    0x95, 0x01,                    // REPORT_COUNT (1)
    0x91, 0x22,                    // OUTPUT (Data,Var,Abs,NPrf)
	0xc0,                          // END_COLLECTION

	//LED Displays
    0x05, 0x14,                    // USAGE_PAGE (Alphnumeric Display)
	0x09, 0x01,                    // USAGE (Alphanumeric Display)
	0x15, 0x00,                    // LOGICAL_MINIMUM (0)
	0xa1, 0x02,                    // COLLECTION (Logical)

	0x09, 0x2b,                    //   USAGE (Character Report)
	0xa1, 0x02,                    //   COLLECTION (Logical)
	0x85, 0x07,                    //     REPORT_ID (7)
	0x75, 0x08,                    //     REPORT_SIZE (8)
	0x95, 0x28,                    //     REPORT_COUNT (40)
	0x26, 0xFF, 0x00,              //     LOGICAL_MAXIMUM (255)
	0x09, 0x2c,                    //     USAGE (Display Data)
	0x92, 0x02, 0x01,              //     OUTPUT (Data,Var,Abs,Buf)
	0xc0,                          //   END_COLLECTION

	//Other DATA
    0x06, 0x00, 0xff,              // USAGE_PAGE (Generic Desktop)
    0x09, 0x01,                    // USAGE (Vendor Usage 1)
    0xa1, 0x01,                    // COLLECTION (Application)
    0x85, 0x08,                    //   REPORT_ID (8)
    0x09, 0x01,                    //   USAGE (Vendor Usage 1)
	0x15, 0x00,                    //   LOGICAL_MINIMUM (0)
	0x27, 0xff, 0xff, 0x00, 0x00,  //   LOGICAL_MAXIMUM (65535)
    0x75, 0x10,                    //   REPORT_SIZE (16)
    0x95, 0x0A,                    //   REPORT_COUNT (10)
    0x91, 0x82,                    //   OUTPUT (Data,Var,Abs,Vol)

/*
	0x85, 0x06,                    //   REPORT_ID (6)
	0x09, 0x3b,                    //   USAGE (Font Report)
	0xa1, 0x02,                    //   COLLECTION (Logical)
	0x15, 0x00,                    //     LOGICAL_MINIMUM (0)
	0x25, 0x7e,                    //     LOGICAL_MAXIMUM (126)
	0x75, 0x08,                    //     REPORT_SIZE (8)
	0x95, 0x01,                    //     REPORT_COUNT (1)
	0x09, 0x2c,                    //     USAGE (Display Data)
	0x91, 0x02,                    //     OUTPUT (Data,Var,Abs)
	0x95, 0x05,                    //     REPORT_COUNT (5)
	0x09, 0x3c,                    //     USAGE (Font Data)
	0x92, 0x02, 0x01,              //     OUTPUT (Data,Var,Abs,Buf)
	0xc0,                          //   END_COLLECTION
*/
  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
  uint8_t dataReceiveArray[USBD_CUSTOMHID_OUTREPORT_BUF_SIZE];
  USBD_CUSTOM_HID_HandleTypeDef     *hhid = (USBD_CUSTOM_HID_HandleTypeDef*)hUsbDeviceFS.pClassData;

  for (uint8_t i = 0; i < USBD_CUSTOMHID_OUTREPORT_BUF_SIZE; i++)
  {
	dataReceiveArray[i] = hhid->Report_buf[i];
  }

  if (dataReceiveArray[0] == 2) //report ID 2 leds
  {


  }

  if (dataReceiveArray[0] == 4) //report ID 4 cursor position
  {
	  LCD_Cursor_IT(dataReceiveArray[1], dataReceiveArray[2]);
  }
  if (dataReceiveArray[0] == 5) //report ID 5 display data
  {
	  for (uint8_t i = 1; i < 21; i++)
	  {
		LCD_AddData_IT(dataReceiveArray[i]);
	  }
  }
  if (dataReceiveArray[0] == 6) //report ID 6 display control report
  {
		LCD_AddComand_IT(dataReceiveArray[1]);
  }
  if (dataReceiveArray[0] == 7) //report ID 7 LED displays data
  {
	  for (uint8_t i = 0; i < 10; i++)
	  {
		  LED_Display0[i] =(char)dataReceiveArray[i + 1];
		  LED_Display1[i] =(char)dataReceiveArray[i + 11];
		  LED_Display2[i] =(char)dataReceiveArray[i + 21];
		  LED_Display3[i] =(char)dataReceiveArray[i + 31];
		  MAX7219DelayUpdate = 1;
	  }
  }
  if (dataReceiveArray[0] == 8) //report ID 8 Other DATA
  {
	  memcpy(&DataFromUSBhost, dataReceiveArray, sizeof(DataFromUSBhost));
  }

  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
/*
static int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}
*/
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */
void HID_REPORT_Init(void)
{
	  DataToUSBhost.ReportID = 1;
	  DataToUSBhost.Throttle = 0xFFFF/2;
	  DataToUSBhost.X = 0xFFFF/2;
	  DataToUSBhost.Y = 0xFFFF/2;
	  DataToUSBhost.Z = 0xFFFF/2;
	  DataToUSBhost.Rx = 0xFFFF/2;
	  DataToUSBhost.Ry = 0xFFFF/2;
	  DataToUSBhost.Rz = 0xFFFF/2;
	  DataToUSBhost.Slider = 0xFFFF/2;
	  DataToUSBhost.Hat = 0;
	  DataToUSBhost.Buttons1 = 0;
	  DataToUSBhost.Buttons2 = 0;
}

void USB_Update(void)
{

	USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, (uint8_t *) &DataToUSBhost, sizeof(DataToUSBhost));

}
/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

