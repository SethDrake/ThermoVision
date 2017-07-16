
#include "stm32f10x_exti.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "misc.h"
#include "delay.h"
#include "sccb.h"
#include "ov7670_reg.h"
#include "ov7670.h"

using namespace std;

OV7670::OV7670() {

}

OV7670::~OV7670() {

}

void OV7670::setupSCCB(GPIO_TypeDef* sccbPort, uint16_t clkPin, uint16_t dataPin) {
	this->sccb.setup(sccbPort, clkPin, dataPin);
	this->sccb.setSlaveAddr(OV7670_ADDR);
}

void OV7670::setupHw(GPIO_TypeDef* dataPort, GPIO_TypeDef* clkPort, uint16_t vsyncPin, uint16_t rclkPin, uint16_t wrEnPin, uint16_t wrRstPin) {
	this->dataPort = dataPort;
	/* init data port to OV7670 */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(dataPort, &GPIO_InitStructure);
	
	this->clkPort = clkPort;
	this->vsyncPin = vsyncPin;
	this->rclkPin = rclkPin;
	this->wrEnPin = wrEnPin;
	this->wrRstPin = wrRstPin;

	/* init clock port to OV7670 */
	GPIO_InitStructure.GPIO_Pin = vsyncPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(clkPort, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = rclkPin | wrEnPin | wrRstPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(clkPort, &GPIO_InitStructure);
	
	clkPort->BRR = rclkPin;
	clkPort->BRR = wrEnPin;
	clkPort->BSRR = wrRstPin;
}

uint16_t OV7670::init()
{
	unsigned char i;
	uint8_t retval;
	uint8_t tmp;
	uint16_t cameraId = 0x0000;

	reset();

	retval = controlRead(REG_PID);
	if (retval != 0x76) {           // check camera pid (0x0a)
		return 0;
	}
	cameraId = retval;
	cameraId = cameraId << 8;


	retval = controlRead(REG_VER);
	if (retval == 0x00) {           // check camera ver (0x0b)
		return 0;
	}
	cameraId |= retval;

	controlWrite(REG_COM7, COM7_RGB | COM7_FMT_QVGA);
	for (i = 0; i < sizeof(ov7670_fmt_rgb565) / sizeof(ov7670_fmt_rgb565[0]); i++) {
		controlWrite(ov7670_fmt_rgb565[i][0], ov7670_fmt_rgb565[i][1]);
	}
	for (i = 0; i < sizeof(ov7670_qvga) / sizeof(ov7670_qvga[0]); i++) {
		controlWrite(ov7670_qvga[i][0], ov7670_qvga[i][1]);
	}

	return cameraId;
}

void OV7670::reset(void) {
	controlWrite(0x12, 0x80);                  // RESET CAMERA
	DelayManager::DelayMs(200);
}


void OV7670::setWindow(unsigned int startx, unsigned int starty, unsigned int width, unsigned int height)
{
	unsigned int endx, endy;// "v*2"
	unsigned char temp_reg1, temp_reg2;
	unsigned char temp = 0;

	endx = (startx + width);
	endy = (starty + height + height);// "v*2"
	temp_reg1 = controlRead(REG_VREF);
	temp_reg1 &= 0xf0;
	temp_reg2 = controlRead(REG_HREF);
	temp_reg2 &= 0xc0;

	// Horizontal
	temp = temp_reg2 | ((endx & 0x7) << 3) | (startx & 0x7);
	controlWrite(REG_HREF, temp);
	temp = (startx & 0x7F8) >> 3;
	controlWrite(REG_HSTART, temp);
	temp = (endx & 0x7F8) >> 3;
	controlWrite(REG_HSTOP, temp);

	// Vertical
	temp = temp_reg1 | ((endy & 0x3) << 2) | (starty & 0x3);
	controlWrite(REG_VREF, temp);
	temp = starty >> 2;
	controlWrite(REG_VSTART, temp);
	temp = endy >> 2;
	controlWrite(REG_VSTOP, temp);
}

void OV7670::captureFrame()
{
	resetCounters();
	//wait for vsync high
	while (!GPIO_ReadInputDataBit(clkPort, vsyncPin));
	//enable write to FIFO
	clkPort->BSRR = wrEnPin;
	//wait for vsync low
	while (GPIO_ReadInputDataBit(clkPort, vsyncPin));
	//wait for vsync hight
	while (!GPIO_ReadInputDataBit(clkPort, vsyncPin));
	//disable write to FIFO
	clkPort->BRR = wrEnPin;
}


void OV7670::readLine(uint16_t* line, uint16_t width)
{
	uint16_t data = 0;
	line += (width - 1);
	for (uint16_t i = 0; i < width; i++) {
		readClockTick();
		data = (dataPort->IDR & 0xFF) << 8;
		readClockTick();
		data |= (dataPort->IDR & 0xFF);

		*line = data;
		line--;
	}
}


// write to camera
void OV7670::controlWrite(uint8_t addr, uint8_t pBuffer)
{
	sccb.writeRegister(addr, pBuffer);
}

// read from camera
uint8_t OV7670::controlRead(uint8_t addr)
{
	uint8_t data = 0;
	sccb.readRegister(addr, &data);
    return data;
}


void OV7670::clockDelay(uint16_t i)
{
	while (i)
	{
		i--;
	}
}

void OV7670::readClockTick()
{
	clkPort->BSRR = rclkPin;
	//clockDelay(0);
	clkPort->BRR = rclkPin;
	//clockDelay(0);
}

void OV7670::resetCounters()
{
	// reset raddr & wraddr
	clkPort->BRR = wrRstPin;
	readClockTick();
	readClockTick();
	clkPort->BSRR = wrRstPin;
	clockDelay(10);
}



