//
// OV7670 library
//
#pragma once
#ifndef _OV7670_H_
#define _OV7670_H_

#include "sccb.h"

#define OV7670_ADDR				0x42

class OV7670 {
public:
	OV7670();
	~OV7670();
	void setupSCCB(GPIO_TypeDef* sccbPort, uint16_t clkPin, uint16_t dataPin);
	void setupHw(GPIO_TypeDef* dataPort, GPIO_TypeDef* clkPort, uint16_t vsyncPin, uint16_t rclkPin, uint16_t wrEnPin, uint16_t wrRstPin);
	uint16_t init();
	void resetCounters();
	void captureFrame();
	void readLine(uint16_t* line, uint16_t width);

protected:
	SCCB sccb;
	GPIO_TypeDef* dataPort;

	GPIO_TypeDef* clkPort;
	uint16_t vsyncPin;
	uint16_t rclkPin;
	uint16_t wrEnPin;
	uint16_t wrRstPin;

private:
	void reset();
	void setWindow(unsigned int startx, unsigned int starty, unsigned int width, unsigned int height);
	void controlWrite(uint8_t addr, uint8_t pBuffer);
	uint8_t controlRead(uint8_t addr);
	void clockDelay(uint16_t i);
	void readClockTick();
};


#endif
