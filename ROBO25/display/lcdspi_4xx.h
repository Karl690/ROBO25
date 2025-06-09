#pragma once
#include <stdint.h>
#include "main.h"
#include "pinout.h"
#if defined(FK_407) || defined(CORE_407I) || defined(CORE_407Z)
#include "LcdDefinitions.h"

/**
  * @brief  SPI handle Structure definition
  */
typedef struct
{
	SPI_TypeDef*          SpiHandle;
  //addit ional pin information to complete the lcd interface
  uint32_t					PinType_RST;
  uint32_t					PinType_CS;
  uint32_t					PinType_RS;
  uint32_t					PinType_SCK;
  uint32_t					PinType_MOSI;

  uint16_t                   RST_Pin;
  GPIO_TypeDef*              RST_Port;
  uint16_t                   CS_Pin;
  GPIO_TypeDef*              CS_Port;
  uint16_t                   RS_Pin;
  GPIO_TypeDef*              RS_Port;
  uint16_t                   BKLT_Pin;
  GPIO_TypeDef*              BKLT_Port;
  //lcd specific information
  uint16_t					 xOffset;
  uint16_t					 yOffset;
  uint16_t                   width;
  uint16_t                   height;
  uint16_t                   orientation; //landscape , portrait,degrees 0-360
  uint16_t                   controller;
  uint16_t                   InitState;  //0=not initialized   1=ok and ready
  uint16_t*				 	ColorTable;
} SPI_LCD_HandleTypeDef;

typedef enum {
	COLOR_MODE_NORMAL,
	COLOR_MODE_INVERT,
}color_mode_t;


extern SPI_LCD_HandleTypeDef* LCDHandler;
#define LCD_RS_SET      LCDHandler->RS_Port->BSRR = LCDHandler->RS_Pin
#define LCD_RS_CLR    	LCDHandler->RS_Port->BSRR = (uint32_t)LCDHandler->RS_Pin << 16
#define LCD_CS_CLR    	LCDHandler->CS_Port->BSRR = (uint32_t)LCDHandler->CS_Pin << 16
#define LCD_CS_SET    	LCDHandler->CS_Port->BSRR = LCDHandler->CS_Pin
#define LCD_RST_CLR    	LCDHandler->RST_Port->BSRR = (uint32_t)LCDHandler->RST_Pin << 16
#define LCD_RST_SET    	LCDHandler->RST_Port->BSRR = LCDHandler->RST_Pin

#define LCD_BLK_CLR    	LCDHandler->BKLT_Port->BSRR = (uint32_t)LCDHandler->BKLT_Pin << 16
#define LCD_BLK_SET    	LCDHandler->BKLT_Port->BSRR = LCDHandler->BKLT_Pin
#define SPI_Drv 		(LCDHandler->SpiHandle)

void SetupMyLCD(SPI_LCD_HandleTypeDef* LcdHandler, int SPIIndex);

int32_t LCD_SPI_Senddata(uint8_t* pdata,uint32_t length, uint32_t timeout);
int32_t LCD_SPI_Recvdata(uint8_t* pdata,uint32_t length);
#endif

