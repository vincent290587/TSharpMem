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

#ifndef _TSharpMem_
#define _TSharpMem_


#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif
#include <Adafruit_GFX.h>

#define adagfxswap(a, b) { int16_t t = a; a = b; b = t; }

// LCD Dimensions
#define SHARPMEM_LCDWIDTH       (400)
#define SHARPMEM_LCDHEIGHT      (240) 

class TSharpMem : public Adafruit_GFX {
public:
	TSharpMem(uint8_t clk, uint8_t mosi, uint8_t ss);
	void begin(void);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	uint8_t getPixel(uint16_t x, uint16_t y);
	void clearDisplay(void);

	void resetBuffer(void) ;
	void writeWhole(void);

protected:
	uint8_t _ss, _clk, _mosi;
	volatile uint8_t *dataport, *clkport;
	uint8_t datapinmask, clkpinmask;
	uint8_t _sharpmem_vcom;

	void sendbyte(uint8_t data);
	void sendbyte_last(uint8_t data);
private:
	uint8_t pcs_data, pcs_command;
	void waitFifoNotFull(void) {
		uint32_t sr;
		uint32_t tmp __attribute__((unused));
		do {
			sr = KINETISK_SPI0.SR;
			if (sr & 0xF0) tmp = KINETISK_SPI0.POPR;  // drain RX FIFO
		} while ((sr & (15 << 12)) > (3 << 12));
	}
	void waitTransmitComplete(uint32_t mcr) __attribute__((always_inline)) {
		uint32_t tmp __attribute__((unused));
		while (1) {
			uint32_t sr = KINETISK_SPI0.SR;
			if (sr & SPI_SR_EOQF) break;  // wait for last transmit
			if (sr &  0xF0) tmp = KINETISK_SPI0.POPR;
		}
		KINETISK_SPI0.SR = SPI_SR_EOQF;
		SPI0_MCR = mcr;
		while (KINETISK_SPI0.SR & 0xF0) {
			tmp = KINETISK_SPI0.POPR;
		}
	}
	void writedata8_cont(uint8_t c) __attribute__((always_inline)) {
		KINETISK_SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
		waitFifoNotFull();
	}
	void writedata8_last(uint8_t c) __attribute__((always_inline)) {
		uint32_t mcr = SPI0_MCR;
		KINETISK_SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	}
};

#endif
