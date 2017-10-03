/*********************************************************************
This is an Arduino library for our Monochrome SHARP Memory Displays

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1393

These displays use SPI to communicate, 3 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
 *********************************************************************/

#include "TSharpMem.h"
#include <avr/pgmspace.h>
#include <limits.h>
#include "pins_arduino.h"
#include "wiring_private.h"
#include <SPI.h>

/**************************************************************************
    Sharp Memory Display Connector
    -----------------------------------------------------------------------
    Pin   Function        Notes
    ===   ==============  ===============================
      1   VIN             3.3-5.0V (into LDO supply)
      2   3V3             3.3V out
      3   GND
      4   SCLK            Serial Clock
      5   MOSI            Serial Data Input
      6   CS              Serial Chip Select
      9   EXTMODE         COM Inversion Select (Low = SW clock/serial)
      7   EXTCOMIN        External COM Inversion Signal
      8   DISP            Display On(High)/Off(Low)

 **************************************************************************/
#define SPICLOCK 5000000

#define SHARPMEM_BIT_WRITECMD   (0x01)
#define SHARPMEM_BIT_VCOM       (0x02)
#define SHARPMEM_BIT_CLEAR      (0x04)
#define TOGGLE_VCOM             do { _sharpmem_vcom = _sharpmem_vcom ? 0x00 : SHARPMEM_BIT_VCOM; } while(0);

byte sharpmem_buffer[(SHARPMEM_LCDWIDTH * SHARPMEM_LCDHEIGHT) / 8];

/* ************* */
/* CONSTRUCTORS  */
/* ************* */
TSharpMem::TSharpMem(uint8_t clk, uint8_t mosi, uint8_t ss) :
		Adafruit_GFX(SHARPMEM_LCDWIDTH, SHARPMEM_LCDHEIGHT) {
	_clk = clk;
	_mosi = mosi;
	_ss = ss;

	// Set the vcom bit to a defined state
	_sharpmem_vcom = SHARPMEM_BIT_VCOM;

}

void TSharpMem::begin() {
	pinMode(_ss, OUTPUT);
	SPI.begin();
	setRotation(1);
}

/**************************************************************************/
/*!
    @brief  Sends a single byte in pseudo-SPI.
 */
/**************************************************************************/
void TSharpMem::sendbyte(uint8_t data) 
{
	writedata8_cont(data);

}

void TSharpMem::sendbyte_last(uint8_t data) 
{
	writedata8_last(data);//
}


/* ************** */
/* PUBLIC METHODS */
/* ************** */

// 1<<n is a costly operation on AVR -- table usu. smaller & faster
static const uint8_t PROGMEM
set[] = {  1,  2,  4,  8,  16,  32,  64,  128 },
clr[] = { 0xFE, 0xFD, 0xFB, 0xF7, 0xEF, 0xDF, 0xBF, 0x7F };

/**************************************************************************/
/*! 
    @brief Draws a single pixel in image buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)
 */
/**************************************************************************/
void TSharpMem::drawPixel(int16_t x, int16_t y, uint16_t color) 
{
	if((x < 0) || (x >= _width) || (y < 0) || (y >= _height)) return;

	switch(rotation) {
	case 1:
		adagfxswap(x, y);
		x = WIDTH  - 1 - x;
		break;
	case 2:
		x = WIDTH  - 1 - x;
		y = HEIGHT - 1 - y;
		break;
	case 3:
		adagfxswap(x, y);
		y = HEIGHT - 1 - y;
		break;
	}

	if(color) {
		sharpmem_buffer[(y*SHARPMEM_LCDWIDTH + x) / 8] |=
				pgm_read_byte(&set[x & 7]);
	} else {
		sharpmem_buffer[(y*SHARPMEM_LCDWIDTH + x) / 8] &=
				pgm_read_byte(&clr[x & 7]);
	}
}

/**************************************************************************/
/*! 
    @brief Gets the value (1 or 0) of the specified pixel from the buffer

    @param[in]  x
                The x position (0 based)
    @param[in]  y
                The y position (0 based)

    @return     1 if the pixel is enabled, 0 if disabled
 */
/**************************************************************************/
uint8_t TSharpMem::getPixel(uint16_t x, uint16_t y)
{
	if((x >= _width) || (y >= _height)) return 0; // <0 test not needed, unsigned

	switch(rotation) {
	case 1:
		adagfxswap(x, y);
		x = WIDTH  - 1 - x;
		break;
	case 2:
		x = WIDTH  - 1 - x;
		y = HEIGHT - 1 - y;
		break;
	case 3:
		adagfxswap(x, y);
		y = HEIGHT - 1 - y;
		break;
	}

	return sharpmem_buffer[(y*SHARPMEM_LCDWIDTH + x) / 8] &
			pgm_read_byte(&set[x & 7]) ? 1 : 0;
}

/**************************************************************************/
/*! 
    @brief Clears the screen
 */
/**************************************************************************/
void TSharpMem::clearDisplay() 
{
	resetBuffer() ;
	// Send the clear screen command rather than doing a HW refresh (quicker)
	SPI.beginTransaction(SPISettings(SPICLOCK, LSBFIRST, SPI_MODE0));
	digitalWriteFast(_ss, HIGH);
	sendbyte(_sharpmem_vcom | SHARPMEM_BIT_CLEAR);
	sendbyte_last(0x00);
	TOGGLE_VCOM;
	digitalWriteFast(_ss, LOW);
	SPI.endTransaction();
}

void TSharpMem::resetBuffer() 
{
	memset(sharpmem_buffer, 0xff, (SHARPMEM_LCDWIDTH * SHARPMEM_LCDHEIGHT) / 8);
}

/**************************************************************************/
/*! 
    @brief Renders the contents of the pixel buffer on the LCD
 */
/**************************************************************************/
void TSharpMem::writeWhole(void) 
{
	uint16_t i, totalbytes, currentline, oldline;
	totalbytes = (SHARPMEM_LCDWIDTH * SHARPMEM_LCDHEIGHT) / 8;

	// Send the write command
	SPI.beginTransaction(SPISettings(SPICLOCK, LSBFIRST, SPI_MODE0));
	digitalWriteFast(_ss, HIGH);

	sendbyte(SHARPMEM_BIT_WRITECMD | _sharpmem_vcom);
	TOGGLE_VCOM;

	// Send the address for line 1
	oldline = currentline = 1;
	sendbyte(currentline);

	// Send image buffer
	for (i=0; i<totalbytes; i++)
	{
		sendbyte(sharpmem_buffer[i]);
		currentline = ((i+1)/(SHARPMEM_LCDWIDTH/8)) + 1;
		
		if(currentline != oldline)
		{
			// Send end of line and address bytes
			sendbyte_last(0x00);

			if (currentline <= SHARPMEM_LCDHEIGHT)
			{
				sendbyte(currentline);
			}
			oldline = currentline;
		}
	}

	// Send another trailing 8 bits for the last line
	sendbyte(0x00);
	sendbyte_last(0x00);

	digitalWriteFast(_ss, LOW);
	SPI.endTransaction();
}

