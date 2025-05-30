#ifdef ADD_ON_SPI_DISPLAY
/**
 ******************************************************************************
 * @file    st7735.c
 * @author  MCD Application Team
 * @version V1.1.1
 * @date    24-November-2014
 * @brief   This file includes the driver for ST7735
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

////////////////////////////////////////////////////////////////////////////////
// local defines
////////////////////////////////////////////////////////////////////////////////

#define ST7735_NOP                0x00 /* No Operation: NOP */
#define ST7735_SWRESET            0x01 /* Software reset: SWRESET */
#define ST7735_READ_DISPLAY_ID    0x04 /* Read Display ID: RDDID */
#define ST7735_RDDST              0x09 /* Read Display Statu: RDDST */
#define ST7735_RDDPM              0x0A /* Read Display Power: RDDPM */
#define ST7735_RDDMADCTL          0x0B /* Read Display: RDDMADCTL */
#define ST7735_RDDCOLMOD          0x0C /* Read Display Pixel Format: RDDCOLMOD */
#define ST7735_RDDIM              0x0D /* Read Display Image: RDDIM */
#define ST7735_RDDSM              0x0E /* Read Display Signal: RDDSM */
#define ST7735_SPLIN              0x10 /* Sleep in & booster off: SLPIN */
#define ST7735_SLEEP_OUT          0x11 /* Sleep out & booster on: SLPOUT */
#define ST7735_PTLON              0x12 /* Partial mode on: PTLON */
#define ST7735_NORMAL_MODE_ON     0x13 /* Partial off (Normal): NORON */
#define ST7735_DINVOFF            0x20 /* Display inversion off: INVOFF */
#define ST7735_DINVON             0x21 /* Display inversion on: INVON */
#define ST7735_GAMMA              0x26 /* Gamma curve select: GAMSET */
#define ST7735_DISPLAY_OFF        0x28 /* Display off: DISPOFF */
#define ST7735_DISPLAY_ON         0x29 /* Display on: DISPON */
#define ST7735_COLUMN_ADDR        0x2A /* Column address set: CASET */
#define ST7735_ROW_ADDR           0x2B /* Row address set: RASET */
#define ST7735_GRAM               0x2C /* Memory write: RAMWR */
#define ST7735_RGBSET             0x2D /* LUT for 4k,65k,262k color: RGBSET */
#define ST7735_RAMRD              0x2E /* Memory read: RAMRD*/
#define ST7735_PLTAR              0x30 /* Partial start/end address set: PTLAR */
#define ST7735_TEOFF              0x34 /* Tearing effect line off: TEOFF */
#define ST7735_TEON               0x35 /* Tearing effect mode set & on: TEON */
#define ST7735_MAC                0x36 /* Memory data access control: MADCTL */
#define ST7735_IDMOFF             0x38 /* Idle mode off: IDMOFF */
#define ST7735_IDMON              0x39 /* Idle mode on: IDMON */
#define ST7735_PIXEL_FORMAT       0x3A /* Interface pixel format: COLMOD */
#define ST7735_FRMCTR1            0xB1 /* In normal mode (Full colors): FRMCTR1 */
#define ST7735_FRMCTR2            0xB2 /* In Idle mode (8-colors): FRMCTR2 */
#define ST7735_FRMCTR3            0xB3 /* In partial mode + Full colors: FRMCTR3 */
#define ST7735_INV_CTRL           0xB4 /* Display inversion control: INVCTR */
#define ST7735_POWER1             0xC0 /* Power control setting: PWCTR1 */
#define ST7735_POWER2             0xC1 /* Power control setting: PWCTR2 */
#define ST7735_POWER3             0xC2 /* In normal mode (Full colors): PWCTR3 */
#define ST7735_POWER4             0xC3 /* In Idle mode (8-colors): PWCTR4 */
#define ST7735_POWER5             0xC4 /* In partial mode + Full colors: PWCTR5 */
#define ST7735_VCOM1              0xC5 /* VCOM control 1: VMCTR1 */
#define ST7735_VCOM2              0xC7 /* Set VCOM offset control: VMOFCTR */
#define ST7735_WRID2              0xD1 /* Set LCM version code: WRID2 */
#define ST7735_WRID3              0xD2 /* Customer Project code: WRID3 */
#define ST7735_NVMCTR1            0xD9 /* NVM control status: NVCTR1 */
#define ST7735_READ_ID1           0xDA /* Read ID1: RDID1 */
#define ST7735_READ_ID2           0xDB /* Read ID2: RDID2 */
#define ST7735_READ_ID3           0xDC /* Read ID3: RDID3 */
#define ST7735_NVCTR2             0xDE /* NVM Read Command: NVCTR2 */
#define ST7735_NVCTR3             0xDF /* NVM Write Command: NVCTR3 */
#define ST7735_PGAMMA             0xE0 /* Set Gamma adjustment (+ polarity): GAMCTRP1 */
#define ST7735_NGAMMA             0xE1 /* Set Gamma adjustment (- polarity): GAMCTRN1 */

#define ST7735_DELAY              ST7735_NOP    // overload no-op as key for inserting a delay in init seq

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

void            st7735_InitPanelStruct(LCD_panelStruct *panelPtr, byte index);
void            st7735_InitDriver(void);
void            st7735_ReadID(byte *rdata);
void            st7735_ReadStatus(byte *rdata);
void            st7735_ReadColorMode(byte *rdata);
void            st7735_DisplayOn(void);
void            st7735_DisplayOff(void);
void            st7735_DisplayInvert(uint16_t invert);
void			st7735_DisplayFramebuffer(void);
void            st7735_SetRotation(byte rotation);

uint32_t        st7735_SetCursor(LCD_addr_t x, LCD_addr_t y);
uint32_t        st7735_SetDisplayWindow(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height);
void            st7735_WritePixel(LCD_addr_t x, LCD_addr_t y, LCD_color_t color);
LCD_color_t     st7735_ReadPixel(LCD_addr_t x, LCD_addr_t y);

void            st7735_EnableGraphicsRamAccess(void);

////////////////////////////////////////////////////////////////////////////////
// variable declarations
////////////////////////////////////////////////////////////////////////////////

LCD_panelStruct *tftp;

const LCD_driverStruct _st7735 =
{
		LCD_DRIVER_ST7735,
		st7735_InitPanelStruct,
		st7735_InitDriver,
		st7735_ReadID,
		st7735_ReadStatus,
		st7735_ReadColorMode,
		st7735_DisplayOn,
		st7735_DisplayOff,
		st7735_DisplayInvert,
		st7735_DisplayFramebuffer,
		st7735_SetRotation,
		st7735_SetCursor,
		st7735_SetDisplayWindow,
		st7735_WritePixel,
		st7735_ReadPixel,
};

const byte st7735InitSeq[] = {
		0,      // placeholder for total number of bytes
		ST7735_SWRESET,         0,
		ST7735_DELAY,           1,  100,                // 100ms delay
		ST7735_DISPLAY_OFF,     0,
		ST7735_FRMCTR1,         3,  0x01, 0x2C, 0x2D,
		ST7735_FRMCTR2,         3,  0x01, 0x2C, 0x2D,
		ST7735_FRMCTR3,         6,  0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D,
		ST7735_INV_CTRL,        1,  0x07,
		ST7735_POWER1,          3,  0xA2, 0x02, 0x84,
		ST7735_POWER2,          1,  0xC5,
		ST7735_POWER3,          2,  0x0A, 0x00,
		ST7735_POWER4,          2,  0x8A, 0x2A,
		ST7735_POWER5,          2,  0x8A, 0xEE,
		ST7735_VCOM1,           1,  0x0E,
		ST7735_DINVOFF,         0,
		ST7735_MAC,             1,  0xC0,
		ST7735_PIXEL_FORMAT,    1,  0x05,
		ST7735_COLUMN_ADDR,     4,  0x00, 0x00, 0x00, 0x7F,
		ST7735_ROW_ADDR,        4,  0x00, 0x00, 0x00, 0x9F,
		ST7735_PGAMMA,          16, 0x02, 0x1c, 0x07, 0x12, 0x37, 0x32, 0x29, 0x2d, 0x29, 0x25, 0x2B, 0x39, 0x00, 0x01, 0x03, 0x10,
		ST7735_NGAMMA,          16, 0x03, 0x1d, 0x07, 0x06, 0x2E, 0x2C, 0x29, 0x2D, 0x2E, 0x2E, 0x37, 0x3F, 0x00, 0x00, 0x02, 0x10,
		ST7735_NORMAL_MODE_ON,  1,  0x00,
		ST7735_SLEEP_OUT,       0,
		ST7735_DELAY,           1,  200,
		ST7735_GRAM,            0,
};

////////////////////////////////////////////////////////////////////////////////
// functions
////////////////////////////////////////////////////////////////////////////////

void st7735_updateOffsetsAndFriends(void)
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
	case LCD_ROTATION_0:        my=0; mx=0; mv=0; break;
	case LCD_ROTATION_90:       my=0; mx=1; mv=1; break;
	case LCD_ROTATION_180:      my=1; mx=1; mv=0; break;
	case LCD_ROTATION_270:      my=1; mx=0; mv=1; break;
	case LCD_ROTATION_0_MX:     my=0; mx=1; mv=0; break;
	case LCD_ROTATION_90_MX:    my=1; mx=1; mv=1; break;
	case LCD_ROTATION_180_MX:   my=1; mx=0; mv=0; break;
	case LCD_ROTATION_270_MX:   my=0; mx=0; mv=1; break;
	default:                    my=0; mx=0; mv=0; break;    //LCD_ROTATION_0
	}

	//MY:MX:MV:ML:BGR:MH:0:0
	value |= (my  << 7);
	value |= (mx  << 6);
	value |= (mv  << 5);
	value |= (ml  << 4);
	value |= (bgr << 3);
	value |= (mh  << 2);

	tftp->xyMirrorRgbCtrl = value;

	if (tftp->index == LCD_TFT_MCUDEV_TFT_144)
	{
		switch(tftp->rotation)
		{
		case LCD_ROTATION_0:        tftp->ofsX=2; tftp->ofsY=1;  break;
		case LCD_ROTATION_90:       tftp->ofsX=1; tftp->ofsY=2;  break;
		case LCD_ROTATION_180:      tftp->ofsX=2; tftp->ofsY=3;  break;
		case LCD_ROTATION_270:      tftp->ofsX=3; tftp->ofsY=2;  break;
		case LCD_ROTATION_0_MX:     tftp->ofsX=2; tftp->ofsY=1;  break;
		case LCD_ROTATION_90_MX:    tftp->ofsX=3; tftp->ofsY=2;  break;
		case LCD_ROTATION_180_MX:   tftp->ofsX=2; tftp->ofsY=3;  break;
		case LCD_ROTATION_270_MX:   tftp->ofsX=1; tftp->ofsY=2;  break;
		default:                    tftp->ofsX=2; tftp->ofsY=1;  break; //LCD_ROTATION_0
		}
	}
	else if (tftp->index == LCD_TFT_MCUDEV_TFT_180)
	{
		switch(tftp->rotation)
		{
		case LCD_ROTATION_0:        tftp->ofsX=2; tftp->ofsY=1;  break;
		case LCD_ROTATION_90:       tftp->ofsX=1; tftp->ofsY=2;  break;
		case LCD_ROTATION_180:      tftp->ofsX=2; tftp->ofsY=1;  break;
		case LCD_ROTATION_270:      tftp->ofsX=1; tftp->ofsY=2;  break;
		case LCD_ROTATION_0_MX:     tftp->ofsX=2; tftp->ofsY=1;  break;
		case LCD_ROTATION_90_MX:    tftp->ofsX=1; tftp->ofsY=2;  break;
		case LCD_ROTATION_180_MX:   tftp->ofsX=2; tftp->ofsY=1;  break;
		case LCD_ROTATION_270_MX:   tftp->ofsX=1; tftp->ofsY=2;  break;
		default:                    tftp->ofsX=2; tftp->ofsY=1;  break; //LCD_ROTATION_0
		}
	}
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

void st7735_InitPanelStruct(LCD_panelStruct *panelPtr, byte index)
{
	tftp = panelPtr;
	tftp->index = index;
	switch(tftp->index)
	{
	case LCD_TFT_MCUDEV_TFT_180:            // SS7735(s) 128x160 w/ 2,2 offset
		tftp->defaultWidth = 128;
		tftp->defaultHeight = 160;
		tftp->flipRGB = 0;
		break;
	case LCD_TFT_MCUDEV_TFT_144:            // SS7735(s) 128x128 w/ screwy offsets
		tftp->defaultWidth = 128;
		tftp->defaultHeight = 128;
		tftp->flipRGB = 1;
		break;
	default:
		break;
	}
	tftp->rotation = LCD_ROTATION_DEFAULT;
	st7735_updateOffsetsAndFriends();

	// common to all flavors
	tftp->maxSpiFrequency = 6666666; // 6.66MHz .. spec is 1 / 150ns
	tftp->supports16BitSpi = 1;
	tftp->hasMISO = 0;
	tftp->hasResetPin = 1;
}

void st7735_InitDriver(void)
{
	if (tftp->hasResetPin)
	{
		LCD_RST_ENABLE;
		delayMsec(10);
		LCD_RST_DISABLE;
		delayMsec(130);
	}
	LCD_SendRegDataSequence((byte *)&st7735InitSeq[0], sizeof(st7735InitSeq), ST7735_DELAY);
	st7735_SetRotation(tftp->rotation);
}

void st7735_ReadID(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(ST7735_READ_DISPLAY_ID);
		LCD_SPI_ReadArray8(rdata, 5);
	}
}

void st7735_ReadStatus(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(ST7735_RDDST);
		LCD_SPI_ReadArray8(rdata, 5);
	}
}

void st7735_ReadColorMode(byte *rdata)
{
	if (tftp->hasMISO)
	{
		LCD_SPI_WriteReg8(ST7735_RDDCOLMOD);
		LCD_SPI_ReadArray8(rdata, 2);
	}
}

void st7735_DisplayOn(void)
{
	LCD_SPI_WriteReg8(ST7735_DISPLAY_ON);
	st7735_EnableGraphicsRamAccess();
}

void st7735_DisplayOff(void)
{
	LCD_SPI_WriteReg8(ST7735_DISPLAY_OFF);
	st7735_EnableGraphicsRamAccess();
}

void st7735_DisplayInvert(uint16_t invert)
{
	LCD_SPI_WriteReg8((invert == 1) ? ST7735_DINVON : ST7735_DINVOFF);
	st7735_EnableGraphicsRamAccess();
}

void st7735_DisplayFramebuffer(void)
{
	;
}

void st7735_SetRotation(byte rotation)
{
	tftp->rotation = rotation;
	st7735_updateOffsetsAndFriends();

	LCD_SPI_WriteReg8(ST7735_MAC);
	LCD_SPI_Write8(tftp->xyMirrorRgbCtrl);
	st7735_EnableGraphicsRamAccess();
}

uint32_t st7735_SetCursor(LCD_addr_t x, LCD_addr_t y)
{   // app is responsible to enure x,y is within prior setting of active window

	if ((x>=0) && (x<tftp->width) && (y>=0) && (y<tftp->height))
	{
		LCD_SPI_WriteReg8(ST7735_COLUMN_ADDR);
		LCD_SPI_Write16(tftp->ofsX + x);            // lower addr
		LCD_SPI_Write16(tftp->maxXaddr);            // upper addr
		LCD_SPI_WriteReg8(ST7735_ROW_ADDR);
		LCD_SPI_Write16(tftp->ofsY + y);            // lower addr
		LCD_SPI_Write16(tftp->maxYaddr);            // upper addr
		st7735_EnableGraphicsRamAccess();
		return(1);
	}
	else
	{
		return(0);  //off display
	}
}

uint32_t st7735_SetDisplayWindow(LCD_addr_t x, LCD_addr_t y, uint16_t width, uint16_t height)
{   // clip to screen and return the actual area

	uint16_t x1 = iFitWithinRange(x+width-1, 0, tftp->width);
	x = iFitWithinRange(x, 0, tftp->width);

	uint16_t y1 = iFitWithinRange(y+height-1, 0, tftp->height);
	y = iFitWithinRange(y, 0, tftp->height);

	LCD_SPI_WriteReg8(ST7735_COLUMN_ADDR);
	LCD_SPI_Write16(tftp->ofsX + x);                // lower addr
	LCD_SPI_Write16(tftp->ofsX + x1);               // upper addr
	LCD_SPI_WriteReg8(ST7735_ROW_ADDR);
	LCD_SPI_Write16(tftp->ofsY + y);                // lower addr
	LCD_SPI_Write16(tftp->ofsY + y1);               // upper addr
	st7735_EnableGraphicsRamAccess();
	return((uint32_t)(x1 - x + 1) * (uint32_t)(y1 - y + 1));
}

void st7735_WritePixel(LCD_addr_t x, LCD_addr_t y, LCD_color_t color)
{
	if (st7735_SetCursor(x, y))
	{   //x, y was on the display
		LCD_SPI_Write16(color);
	}
}

LCD_color_t st7735_ReadPixel(LCD_addr_t x, LCD_addr_t y)
{
	uint16_t rdata = 0;
	if (tftp->hasMISO)
	{
		if (st7735_SetCursor(x, y))
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

void st7735_EnableGraphicsRamAccess(void)
{
	LCD_SPI_WriteReg8(ST7735_GRAM); // restore default access to graphics ram
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
#endif //ADD_ON_SPI_DISPLAY
