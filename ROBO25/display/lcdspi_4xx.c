#include "main.h"
#include "pinout.h"
#include "lcdspi_4xx.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
SPI_LCD_HandleTypeDef* LCDHandler;
//DMA_HandleTypeDef hdma_SPI_tx;

void SetupMyLCD(SPI_LCD_HandleTypeDef* LcdHandler, int SPIIndex)
{
	LCDHandler = LcdHandler;
	LcdHandler->InitState = 0;

	//Setup the Clock for SPI.
	switch (SPIIndex) 
	{	case 1:
		RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
		RCC->APB2RSTR &= ~RCC_APB2ENR_SPI1EN;
		break;
	case 2:
		RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;
		RCC->APB1RSTR &= ~RCC_APB1ENR_SPI2EN;
		break;
	case 3:
		RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
		RCC->APB1RSTR &= ~RCC_APB1ENR_SPI3EN;
		break;
	}

	switch (SPIIndex)
	{
	case 0:break;//return false;
	case 1:LcdHandler->SpiHandle = SPI1;
//		LcdHandler->PinType_RST =  SPI1_LCD_RST;
//		LcdHandler->PinType_CS =   SPI1_LCD_CS;
//		LcdHandler->PinType_RS =   SPI1_LCD_RS;
//		LcdHandler->PinType_SCK =  SPI1_LCD_SCK;
//		LcdHandler->PinType_MOSI = SPI1_LCD_MOSI;
		break;
	case 2:LcdHandler->SpiHandle = SPI2;
//		LcdHandler->PinType_RST = SPI2_LCD_RST;
//		LcdHandler->PinType_CS = SPI2_LCD_CS;
//		LcdHandler->PinType_RS = SPI2_LCD_RS;
//		LcdHandler->PinType_SCK = SPI2_LCD_SCK;
//		LcdHandler->PinType_MOSI = SPI2_LCD_MOSI;
		break;
	case 3:LcdHandler->SpiHandle = SPI3;
		LcdHandler->PinType_RST = SPI3_LCD_RST;
		LcdHandler->PinType_CS = SPI3_LCD_CS;
		LcdHandler->PinType_RS = SPI3_LCD_RS;
		LcdHandler->PinType_SCK = SPI3_LCD_SCK;
		LcdHandler->PinType_MOSI = SPI3_LCD_MOSI;
		break;
	//put in code to catch a default as false and return;
	  default : return;
	}
	LcdHandler->RST_Pin   = pinExtractPinMask(LcdHandler->PinType_RST);     LcdHandler->RST_Port    = pinExtractPortPtr(LcdHandler->PinType_RST);
	LcdHandler->CS_Pin   = pinExtractPinMask(LcdHandler->PinType_CS);     LcdHandler->CS_Port    = pinExtractPortPtr(LcdHandler->PinType_CS);
	LcdHandler->RS_Pin   = pinExtractPinMask(LcdHandler->PinType_RS);     LcdHandler->RS_Port    = pinExtractPortPtr(LcdHandler->PinType_RS);
	pinInit(LcdHandler->PinType_RS);
	pinInit(LcdHandler->PinType_CS);
	pinInit(LcdHandler->PinType_RST);
	pinInit(LcdHandler->PinType_SCK);
	pinInit(LcdHandler->PinType_MOSI);

	//reset the display so it will be ready to run
	LCD_RST_CLR;

	SPI_InitTypeDef SPI_InitStruct;
	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
	SPI_InitStruct.SPI_Direction = SPI_Direction_1Line_Tx; // 2Lines_FullDuplex;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;	
	SPI_InitStruct.SPI_CRCPolynomial = 0;
	
	SPI_Init(LcdHandler->SpiHandle, &SPI_InitStruct);	
	SPI_Cmd(LcdHandler->SpiHandle, ENABLE);
	LCD_RST_SET;
}
#define SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->SR) & (__FLAG__)) == (__FLAG__))
int32_t LCD_SPI_Senddata(uint8_t* pdata, uint32_t length, uint32_t timeout)
{
	
	uint64_t tickStart = tickCount;
	uint16_t i = 0;
	
	if (length == 1)
	{
		*((__IO uint8_t *)&SPI_Drv->DR) = *pdata;
		i++;
		pdata++;
	}
	while (i < length)
	{	
		if (SPI_GET_FLAG(SPI_Drv, SPI_FLAG_TXE))
		{
			*((__IO uint8_t *)&SPI_Drv->DR) = *pdata;
			i++;
			pdata++;
		}
		else {
			if (((tickCount - tickStart) >=  timeout))
			{
				return 0;
			}
		}		
	}
	while ((SPI_GET_FLAG(SPI_Drv, SPI_FLAG_BSY) ? SET : RESET) != RESET)
	{
		if (tickCount - tickStart >=  timeout) break;
	}
	return 0;
}


int32_t LCD_SPI_Recvdata(uint8_t* pdata,uint32_t length)
{
	int32_t result;
	LCD_CS_CLR;
	//result = HAL_SPI_Receive(SPI_Drv,pdata,length,500);
	LCD_CS_SET;
	return result;
}

