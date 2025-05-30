#if 0	// NOT READY FOR PRIME TIME
#ifdef ADD_ON_SPI_DISPLAY
/**
 ******************************************************************************
 * @file    ssd1306.c
 * @author  MCD Application Team
 * @version V1.1.1
 * @date    24-November-2014
 * @brief   This file includes the driver for SSD1306
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

// HEAVILY MODIFIED FOR Hyrel3D

////////////////////////////////////////////////////////////////////////////////
// includes
////////////////////////////////////////////////////////////////////////////////

#include "main.h"
#include "lcd.h"

byte LCD_GRAM[128][8];	//local framebuffer

////////////////////////////////////////////////////////////////////////////////
// local defines
////////////////////////////////////////////////////////////////////////////////

#define SSD1306_NOP                0x00 /* No Operation: NOP */
#define SSD1306_SWRESET            0x01 /* Software reset: SWRESET */
#define SSD1306_READ_DISPLAY_ID    0x04 /* Read Display ID: RDDID */
#define SSD1306_RDDST              0x09 /* Read Display Statu: RDDST */
#define SSD1306_RDDPM              0x0A /* Read Display Power: RDDPM */
#define SSD1306_RDDMADCTL          0x0B /* Read Display: RDDMADCTL */
#define SSD1306_RDDCOLMOD          0x0C /* Read Display Pixel Format: RDDCOLMOD */
#define SSD1306_RDDIM              0x0D /* Read Display Image: RDDIM */
#define SSD1306_RDDSM              0x0E /* Read Display Signal: RDDSM */
#define SSD1306_SPLIN              0x10 /* Sleep in & booster off: SLPIN */
#define SSD1306_SLEEP_OUT          0x11 /* Sleep out & booster on: SLPOUT */
#define SSD1306_PTLON              0x12 /* Partial mode on: PTLON */
#define SSD1306_NORMAL_MODE_ON     0x13 /* Partial off (Normal): NORON */
#define SSD1306_DINVOFF            0x20 /* Display inversion off: INVOFF */
#define SSD1306_DINVON             0x21 /* Display inversion on: INVON */
#define SSD1306_GAMMA              0x26 /* Gamma curve select: GAMSET */
#define SSD1306_DISPLAY_OFF        0x28 /* Display off: DISPOFF */
#define SSD1306_DISPLAY_ON         0x29 /* Display on: DISPON */
#define SSD1306_COLUMN_ADDR        0x2A /* Column address set: CASET */
#define SSD1306_ROW_ADDR           0x2B /* Row address set: RASET */
#define SSD1306_GRAM               0x2C /* Memory write: RAMWR */
#define SSD1306_RGBSET             0x2D /* LUT for 4k,65k,262k color: RGBSET */
#define SSD1306_RAMRD              0x2E /* Memory read: RAMRD*/
#define SSD1306_PLTAR              0x30 /* Partial start/end address set: PTLAR */
#define SSD1306_TEOFF              0x34 /* Tearing effect line off: TEOFF */
#define SSD1306_TEON               0x35 /* Tearing effect mode set & on: TEON */
#define SSD1306_MAC                0x36 /* Memory data access control: MADCTL */
#define SSD1306_IDMOFF             0x38 /* Idle mode off: IDMOFF */
#define SSD1306_IDMON              0x39 /* Idle mode on: IDMON */
#define SSD1306_PIXEL_FORMAT       0x3A /* Interface pixel format: COLMOD */
#define SSD1306_FRMCTR1            0xB1 /* In normal mode (Full colors): FRMCTR1 */
#define SSD1306_FRMCTR2            0xB2 /* In Idle mode (8-colors): FRMCTR2 */
#define SSD1306_FRMCTR3            0xB3 /* In partial mode + Full colors: FRMCTR3 */
#define SSD1306_INV_CTRL           0xB4 /* Display inversion control: INVCTR */
#define SSD1306_POWER1             0xC0 /* Power control setting: PWCTR1 */
#define SSD1306_POWER2             0xC1 /* Power control setting: PWCTR2 */
#define SSD1306_POWER3             0xC2 /* In normal mode (Full colors): PWCTR3 */
#define SSD1306_POWER4             0xC3 /* In Idle mode (8-colors): PWCTR4 */
#define SSD1306_POWER5             0xC4 /* In partial mode + Full colors: PWCTR5 */
#define SSD1306_VCOM1              0xC5 /* VCOM control 1: VMCTR1 */
#define SSD1306_VCOM2              0xC7 /* Set VCOM offset control: VMOFCTR */
#define SSD1306_WRID2              0xD1 /* Set LCM version code: WRID2 */
#define SSD1306_WRID3              0xD2 /* Customer Project code: WRID3 */
#define SSD1306_NVMCTR1            0xD9 /* NVM control status: NVCTR1 */
#define SSD1306_READ_ID1           0xDA /* Read ID1: RDID1 */
#define SSD1306_READ_ID2           0xDB /* Read ID2: RDID2 */
#define SSD1306_READ_ID3           0xDC /* Read ID3: RDID3 */
#define SSD1306_NVCTR2             0xDE /* NVM Read Command: NVCTR2 */
#define SSD1306_NVCTR3             0xDF /* NVM Write Command: NVCTR3 */
#define SSD1306_PGAMMA             0xE0 /* Set Gamma adjustment (+ polarity): GAMCTRP1 */
#define SSD1306_NGAMMA             0xE1 /* Set Gamma adjustment (- polarity): GAMCTRN1 */

#define SSD1306_DELAY              SSD1306_NOP    // overload no-op as key for inserting a delay in init seq

#ifndef SWAP_UINT16
#define SWAP_UINT16(a, b)                                                      \
  {                                                                            \
	uint16_t t = a;                                                            \
	a = b;                                                                     \
	b = t;                                                                     \
  }
#endif

////////////////////////////////////////////////////////////////////////////////
// externally referenced
////////////////////////////////////////////////////////////////////////////////

extern void     LCD_SPI_Write8(byte wdata);
extern void     LCD_SPI_WriteArray8(byte *wdata, int numBytes);
extern void     LCD_SPI_Write16(uint16_t wdata);
extern void     LCD_SPI_WriteReg8(uint8_t reg);
extern void     LCD_SPI_ReadArray8(byte *rdata, int numBytes);
extern uint16_t LCD_SPI_Read16(void);
extern void     LCD_SendRegDataSequence(byte seq[], uint16_t numBytes, byte delayReg);

////////////////////////////////////////////////////////////////////////////////
// forward declarations
////////////////////////////////////////////////////////////////////////////////

void            ssd1306_InitPanelStruct(LCD_panelStruct *panelPtr, byte index);
void            ssd1306_InitDriver(void);
void            ssd1306_ReadID(byte *rdata);
void            ssd1306_ReadStatus(byte *rdata);
void            ssd1306_ReadColorMode(byte *rdata);
void            ssd1306_DisplayOn(void);
void            ssd1306_DisplayOff(void);
void            ssd1306_DisplayInvert(uint16_t invert);
void			ssd1306_DisplayFramebuffer(void);
void            ssd1306_SetRotation(byte rotation);

uint32_t        ssd1306_SetCursor(LCD_addr_t x, LCD_addr_t y);
uint32_t        ssd1306_SetDisplayWindow(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height);
void            ssd1306_WritePixel(LCD_addr_t x, LCD_addr_t y, LCD_color_t color);
LCD_color_t     ssd1306_ReadPixel(LCD_addr_t x, LCD_addr_t y);

void            ssd1306_EnableGraphicsRamAccess(void);

////////////////////////////////////////////////////////////////////////////////
// variable declarations
////////////////////////////////////////////////////////////////////////////////

LCD_panelStruct *tftp;

LCD_driverStruct _ssd1306 =
{
		LCD_DRIVER_SSD1306,
		ssd1306_InitPanelStruct,
		ssd1306_InitDriver,
		ssd1306_ReadID,
		ssd1306_ReadStatus,
		ssd1306_ReadColorMode,
		ssd1306_DisplayOn,
		ssd1306_DisplayOff,
		ssd1306_DisplayInvert,
		ssd1306_DisplayFramebuffer,
		ssd1306_SetRotation,
		ssd1306_SetCursor,
		ssd1306_SetDisplayWindow,
		ssd1306_WritePixel,
		ssd1306_ReadPixel,
};

//byte ssd1306InitSeq[] = {
//		0,      // placeholder for total number of bytes
//		SSD1306_SWRESET,         0,
//		SSD1306_DELAY,           1,  100,                // 100ms delay
//		SSD1306_DISPLAY_OFF,     0,
//		SSD1306_FRMCTR1,         3,  0x01, 0x2C, 0x2D,
//		SSD1306_FRMCTR2,         3,  0x01, 0x2C, 0x2D,
//		SSD1306_FRMCTR3,         6,  0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
//		SSD1306_INV_CTRL,        1,  0x07,
//		SSD1306_POWER1,          3,  0xA2, 0x02, 0x84,
//		SSD1306_POWER2,          1,  0xC5,
//		SSD1306_POWER3,          2,  0x0A, 0x00,
//		SSD1306_POWER4,          2,  0x8A, 0x2A,
//		SSD1306_POWER5,          2,  0x8A, 0xEE,
//		SSD1306_VCOM1,           1,  0x0E,
//		SSD1306_DINVOFF,         0,
//		SSD1306_MAC,             1,  0xC0,
//		SSD1306_PIXEL_FORMAT,    1,  0x05,
//		SSD1306_COLUMN_ADDR,     4,  0x00, 0x00, 0x00, 0x7F,
//		SSD1306_ROW_ADDR,        4,  0x00, 0x00, 0x00, 0x9F,
//		SSD1306_PGAMMA,          16, 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
//		SSD1306_NGAMMA,          16, 0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
//		SSD1306_NORMAL_MODE_ON,  1,  0x00,
//		SSD1306_SLEEP_OUT,       0,
//		SSD1306_DELAY,           1,  200,
//		SSD1306_GRAM,            0,
//};

////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////
void ssd1306_updateOffsetsAndFriends(void)
{
	byte value = 0;
	int my = 0;
	int mx = 0;
	int mv = 0;
	int ml = 0;
	int bgr = tftp->flipRGB;
	int mh = 0;

	switch(tftp->rotation)
	{
	case LCD_ROTATION_0:        my=1; mx=0; mv=0; break;
	case LCD_ROTATION_90:       my=1; mx=1; mv=1; break;
	case LCD_ROTATION_180:      my=0; mx=1; mv=0; break;
	case LCD_ROTATION_270:      my=0; mx=0; mv=1; break;
	case LCD_ROTATION_0_MX:     my=1; mx=1; mv=0; break;
	case LCD_ROTATION_90_MX:    my=1; mx=0; mv=1; break;
	case LCD_ROTATION_180_MX:   my=0; mx=0; mv=0; break;
	case LCD_ROTATION_270_MX:   my=0; mx=1; mv=1; break;
	default:                    my=1; mx=0; mv=0; break;    //LCD_ROTATION_0
	}

	//MY:MX:MV:ML:BGR:MH:0:0
	value |= (my  << 7);
	value |= (mx  << 6);
	value |= (mv  << 5);
	value |= (ml  << 4);
	value |= (bgr << 3);
	value |= (mh  << 2);

	tftp->xyMirrorRgbCtrl = value;
	
	tftp->ofsX = 0;
	tftp->ofsY = 0;
	
	if (mv)
	{
		tftp->width  = tftp->defaultHeight;
		tftp->height = tftp->defaultWidth;
	}
	else
	{
		tftp->width  = tftp->defaultWidth;
		tftp->height = tftp->defaultHeight;
	}

	tftp->maxXaddr = tftp->ofsX + tftp->width - 1;
	tftp->maxYaddr = tftp->ofsY + tftp->height - 1;
}

void ssd1306_InitPanelStruct(LCD_panelStruct *panelPtr, byte index)
{
	tftp = panelPtr;
	tftp->index = index;
	switch(tftp->index)
	{
	case LCD_TFT_MCUDEV_TFT_096:            // SS7735(s) 128x160 w/ 2,2 offset
		tftp->defaultWidth = 128;
		tftp->defaultHeight = 64;
		tftp->flipRGB = 0;
		break;
	default:
		break;
	}
	ssd1306_updateOffsetsAndFriends();

	// common to all flavors
	tftp->maxSpiFrequency = 6666666; // 6.66MHz .. spec is 1 / 150ns
	tftp->supports16BitSpi = 1;
	tftp->hasMISO = 0;
	tftp->hasResetPin = 1;
}

void LCD_WR_Byte(u8 dat, u8 cmd)
{
	BB_PIN_LCD_D_CB = cmd;
	LCD_CS_ENABLE;
	for(int i=0; i<8; i++)
	{
		LCD_CLK_LOW;
		if (dat & 0x80) {
			LCD_SDO_HIGH;
		} else {
			LCD_SDO_LOW;
		}
		LCD_CLK_HIGH;
		dat<<=1;
	}
	LCD_CS_DISABLE;
	LCD_DATA_ENABLE;
}

void ssd1306_InitDriver(void)
{
	if (tftp->hasResetPin)
	{
		LCD_RST_ENABLE;
		delayMsec(10);
		LCD_RST_DISABLE;
		delayMsec(130);
	}

	//RESTORE LCD_SendRegDataSequence(ssd1306InitSeq, sizeof(ssd1306InitSeq), SSD1306_DELAY);
#define LCD_CMD 0
#define LCD_DATA 1
	LCD_RST_ENABLE;
	delayMsec(100);
	LCD_RST_DISABLE;
	LCD_WR_Byte(0xAE, LCD_CMD);
	LCD_WR_Byte(0xD5, LCD_CMD);
	LCD_WR_Byte(80, LCD_CMD);  //[3:0],;[7:4],
	LCD_WR_Byte(0xA8, LCD_CMD);
	LCD_WR_Byte(0X3F, LCD_CMD);//(1/64)
	LCD_WR_Byte(0xD3, LCD_CMD);
	LCD_WR_Byte(0X00, LCD_CMD);

	LCD_WR_Byte(0x40, LCD_CMD);// [5:0],.

	LCD_WR_Byte(0x8D, LCD_CMD);
	LCD_WR_Byte(0x14, LCD_CMD);
	LCD_WR_Byte(0x20, LCD_CMD);
	LCD_WR_Byte(0x02, LCD_CMD);//[1:0],;
	LCD_WR_Byte(0xA1, LCD_CMD);//, bit0:0, 0->0;1, 0->127;
	LCD_WR_Byte(0xC0, LCD_CMD);//;bit3:0,;1, COM[N-1]->COM0;N:
	LCD_WR_Byte(0xDA, LCD_CMD);
	LCD_WR_Byte(0x12, LCD_CMD);//[5:4]

	LCD_WR_Byte(0x81, LCD_CMD);
	LCD_WR_Byte(0xEF, LCD_CMD);//1~255;
	LCD_WR_Byte(0xD9, LCD_CMD);
	LCD_WR_Byte(0xf1, LCD_CMD);//[3:0], PHASE 1;[7:4], PHASE 2;
	LCD_WR_Byte(0xDB, LCD_CMD);
	LCD_WR_Byte(0x30, LCD_CMD);//[6:4] 000, 0.65*vcc;001, 0.77*vcc;011, 0.83*vcc;
	LCD_WR_Byte(0xA4, LCD_CMD);//;bit0:1,;0,;
	LCD_WR_Byte(0xA6, LCD_CMD);//;bit0:1,;0,
	LCD_WR_Byte(0xAF, LCD_CMD);
	delayMsec(100);
	ssd1306_SetRotation(tftp->rotation);
}

void ssd1306_ReadID(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(SSD1306_READ_DISPLAY_ID);
		LCD_SPI_ReadArray8(rdata, 5);
	}
}

void ssd1306_ReadStatus(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(SSD1306_RDDST);
		LCD_SPI_ReadArray8(rdata, 5);
	}
}

void ssd1306_ReadColorMode(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(SSD1306_RDDCOLMOD);
		LCD_SPI_ReadArray8(rdata, 2);
	}
}

void ssd1306_DisplayOn(void)
{
//	LCD_SPI_WriteReg8(SSD1306_DISPLAY_ON);
//	ssd1306_EnableGraphicsRamAccess();
	LCD_WR_Byte(0X8D, LCD_CMD);  //SET
	LCD_WR_Byte(0X14, LCD_CMD);  //DCDC ON
	LCD_WR_Byte(0XAF, LCD_CMD);  //DISPLAY ON
}

void ssd1306_DisplayOff(void)
{
//	LCD_SPI_WriteReg8(SSD1306_DISPLAY_OFF);
//	ssd1306_EnableGraphicsRamAccess();
	LCD_WR_Byte(0X8D, LCD_CMD);  //SET
	LCD_WR_Byte(0X10, LCD_CMD);  //DCDC OFF
	LCD_WR_Byte(0XAE, LCD_CMD);  //DISPLAY OFF
}

void ssd1306_DisplayInvert(uint16_t invert)
{
	LCD_SPI_WriteReg8((invert == 1) ? SSD1306_DINVON : SSD1306_DINVOFF);
	ssd1306_EnableGraphicsRamAccess();
}

void ssd1306_DisplayFramebuffer(void)
{
	u8 i, n;
	for(i=0; i<8; i++)
	{
		LCD_WR_Byte (0xb0+i, LCD_CMD);
		LCD_WR_Byte (0x00, LCD_CMD);
		LCD_WR_Byte (0x10, LCD_CMD);
		for(n=0; n<128; n++)
		{
			LCD_WR_Byte(LCD_GRAM[n][i], LCD_DATA);
		}
	}
}

void ssd1306_SetRotation(byte rotation)
{
	tftp->rotation = rotation;
	ssd1306_updateOffsetsAndFriends();
	LCD_SPI_WriteReg8(SSD1306_MAC);
	LCD_SPI_Write8(tftp->xyMirrorRgbCtrl);


	ssd1306_EnableGraphicsRamAccess();
}

uint32_t ssd1306_SetCursor(LCD_addr_t x, LCD_addr_t y)
{   // app is responsible to insure x,y is within prior setting of active window

	if ((x>=0) && (x<tftp->width) && (y>=0) && (y<tftp->height))
	{
//		LCD_SPI_WriteReg8(SSD1306_COLUMN_ADDR);
//		LCD_SPI_Write16(tftp->ofsX + x);            // lower addr
//		LCD_SPI_Write16(tftp->maxXaddr);            // upper addr
//		LCD_SPI_WriteReg8(SSD1306_ROW_ADDR);
//		LCD_SPI_Write16(tftp->ofsY + y);            // lower addr
//		LCD_SPI_Write16(tftp->maxYaddr);            // upper addr
//		ssd1306_EnableGraphicsRamAccess();
		return(1);
	}
	else
	{
		return(0);  //off display
	}
}

uint32_t ssd1306_SetDisplayWindow(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height)
{   // clip to screen and return the actual area

	uint16_t x1 = iFitWithinRange(x+width-1, 0, tftp->width);
	x = iFitWithinRange(x, 0, tftp->width);

	uint16_t y1 = iFitWithinRange(y+height-1, 0, tftp->height);
	y = iFitWithinRange(y, 0, tftp->height);

	LCD_SPI_WriteReg8(SSD1306_COLUMN_ADDR);
	LCD_SPI_Write16(tftp->ofsX + x);                // lower addr
	LCD_SPI_Write16(tftp->ofsX + x1);               // upper addr
	LCD_SPI_WriteReg8(SSD1306_ROW_ADDR);
	LCD_SPI_Write16(tftp->ofsY + y);                // lower addr
	LCD_SPI_Write16(tftp->ofsY + y1);               // upper addr
	ssd1306_EnableGraphicsRamAccess();
	return((uint32_t)(x1 - x + 1) * (uint32_t)(y1 - y + 1));
}

void ssd1306_WritePixel(LCD_addr_t x, LCD_addr_t y, LCD_color_t color)
{
//	if (ssd1306_SetCursor(x, y))
//	{   //x, y was on the display
//		LCD_SPI_Write16(color);
//	}

	int yBytePos;
	int yBitPos;
	int yBitMask;

	if (ssd1306_SetCursor(x, y))
	{   //x, y was on the display
		yBytePos = 7 - (y / 8);
		yBitPos = y % 8;
		yBitMask = 1 << (7 - yBitPos);
		if (color)	// any non-zero color is written as "on"
			LCD_GRAM[x][yBytePos] |= yBitMask;
		else
			LCD_GRAM[x][yBytePos] &= ~yBitMask;
	}
}

LCD_color_t ssd1306_ReadPixel(LCD_addr_t x, LCD_addr_t y)
{
	uint16_t rdata = 0;
	if (tftp->hasMISO)
	{
		if (ssd1306_SetCursor(x, y))
		{   //x, y was on the display
			rdata = LCD_SPI_Read16();
		}
	}
	else
	{
		rdata = 0;
	}
	return(rdata);
}

////////////////////////////////////////////////////////////////////////////////

void ssd1306_EnableGraphicsRamAccess(void)
{
	LCD_SPI_WriteReg8(SSD1306_GRAM); // restore default access to graphics ram
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif //ADD_ON_SPI_DISPLAY

#endif //0
