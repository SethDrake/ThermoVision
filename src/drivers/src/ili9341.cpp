
#include <stdio.h>
#include <stdarg.h>
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_spi.h"
#include "stm32f10x_dma.h"
#include "stm32f10x_rcc.h"
#include "delay.h"
#include "fonts.h"
#include "ili9341.h"

ILI9341::ILI9341() {
	isDataSending = 0;
	manualCsControl = 0;
	disableDMA = 0;

	color = WHITE;
	bgColor = BLACK;
}

ILI9341::~ILI9341() {
}

void ILI9341::setupHw(SPI_TypeDef* spi, uint16_t spiPrescaler, GPIO_TypeDef* controlPort, uint16_t rsPin, uint16_t resetPin, uint16_t csPin) {
	this->spi = spi;
	this->spiPrescaler = spiPrescaler;
	this->controlPort = controlPort;
	this->rsPin = rsPin;
	this->resetPin = resetPin;
	this->csPin = csPin;

	/* RS, RESET & CS pins of ILI9341 */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = rsPin | resetPin | csPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(controlPort, &GPIO_InitStructure);
}

void ILI9341::setSPISpeed(uint16_t prescaler) {
	SPI_Cmd(spi, DISABLE);
	spi->CR1 &= ~SPI_CR1_BR; // Clear SPI baud rate bits
	spi->CR1 |= prescaler; // Set SPI baud rate bits
	SPI_Cmd(spi, ENABLE);
}

void ILI9341::init(void) {

	setSPISpeed(SPI_BaudRatePrescaler_16);
	switchCs(0);
	switchReset(0);
	DelayManager::DelayMs(5);
	switchReset(1);

	sendCmd(ILI9341_SOFT_RESET_REG); //software reset
	DelayManager::DelayMs(5);
	sendCmd(ILI9341_DISPLAYOFF_REG); // display off

	sendCmd(ILI9341_POWERCTL1_REG);    	//Power control
	sendData(0x23);   	//VRH[5:0]

	sendCmd(ILI9341_POWERCTL2_REG);    	//Power control
	sendData(0x10);   	//SAP[2:0];BT[3:0]

	sendCmd(ILI9341_VCOMCTL1_REG);    	//VCM control
	sendData(0x3E); //Contrast
	sendData(0x28);

	sendCmd(ILI9341_VCOMCTL2_REG);    	//VCM control2
	sendData(0x86);

	sendCmd(ILI9341_MEMACCESS_REG);    	// Memory Access Control
	sendData(0x48);  	//my,mx,mv,ml,BGR,mh,0.0


	sendCmd(ILI9341_PIXFORMATSET_REG);
	sendData(0x55);

	sendCmd(ILI9341_FRAMECTL_NOR_REG);
	sendData(0x00);
	sendData(0x18);

	sendCmd(ILI9341_FUNCTONCTL_REG);    	// Display Function Control
	sendData(0x08);
	sendData(0x82);
	sendData(0x27);

	sendCmd(ILI9341_ENABLE_3G_REG);    	// 3Gamma Function Disable
	sendData(0x00);

	sendCmd(ILI9341_GAMMASET_REG);    	//Gamma curve selected
	sendData(0x01);

	sendCmd(ILI9341_POSGAMMACORRECTION_REG);    	//Set Gamma
	sendData(0x0F);
	sendData(0x31);
	sendData(0x2B);
	sendData(0x0C);
	sendData(0x0E);
	sendData(0x08);
	sendData(0x4E);
	sendData(0xF1);
	sendData(0x37);
	sendData(0x07);
	sendData(0x10);
	sendData(0x03);
	sendData(0x0E);
	sendData(0x09);
	sendData(0x00);


	sendCmd(ILI9341_NEGGAMMACORRECTION_REG);    	//Set Gamma
	sendData(0x00);
	sendData(0x0E);
	sendData(0x14);
	sendData(0x03);
	sendData(0x11);
	sendData(0x07);
	sendData(0x31);
	sendData(0xC1);
	sendData(0x48);
	sendData(0x08);
	sendData(0x0F);
	sendData(0x0C);
	sendData(0x31);
	sendData(0x36);
	sendData(0x0F);

	sendCmd(ILI9341_SLEEP_OUT_REG);    	//Exit Sleep
	DelayManager::DelayMs(100);
	sendCmd(ILI9341_DISPLAYON_REG);   	//Display on
	DelayManager::DelayMs(100);

	//sendCmd(ILI9341_WRITEBRIGHT_REG);   	//Change brightness
	//sendData(0x50);
	switchCs(1);
	setSPISpeed(this->spiPrescaler);
}


void ILI9341::enable(short on) {
	switchCs(0);
	if (on == 0) {
		sendCmd(ILI9341_DISPLAYOFF_REG);
	} else {
		sendCmd(ILI9341_DISPLAYON_REG);
	}
	switchCs(1);
	DelayManager::DelayMs(100);
}


void ILI9341::sleep(short on){
	switchCs(0);
	if (on == 0) {
		sendCmd(ILI9341_SLEEP_OUT_REG);
	} else {
		sendCmd(ILI9341_SLEEP_ENTER_REG);
	}
	switchCs(1);
	DelayManager::DelayMs(100);
}

void ILI9341::clear(uint16_t color)
{
	fillScreen(TFT_MIN_X, TFT_MIN_Y, TFT_MAX_X, TFT_MAX_Y, color);
}

void ILI9341::setCol(int StartCol, int EndCol)
{
	sendCmd(ILI9341_COLADDRSET_REG);    // Column Command address
	sendWords(StartCol, EndCol);
}

void ILI9341::setPage(int StartPage, int EndPage)
{
	sendCmd(ILI9341_PAGEADDRSET_REG);   // Page Command address
	sendWords(StartPage, EndPage);
}

void ILI9341::setWindow(int startX, int startY, int stopX, int stopY)
{
	setPage(startX, stopX);
	setCol(startY, stopY);
}

void ILI9341::fillScreen(uint16_t xstart, uint16_t ystart, uint16_t xstop, uint16_t ystop, uint16_t color)
{
  uint32_t pixels;
  pixels = (xstop - xstart + 1) * (ystop - ystart + 1);
  while (this->isDataSending); //wait until all data wasn't sended
  switchCs(0);   // CS=0;
  setWindow(xstart, ystart, xstop, ystop);
  sendCmd(ILI9341_MEMORYWRITE_REG);

  switchRs(1);

  if (!disableDMA) {
	  manualCsControl = 1;
	  if (pixels > 65535) {
		  initDMAforSendSPI(&color, 65535, 1);
		  DMATXStart();
		  pixels -= 65535;
		  while (this->isDataSending); //wait until all data wasn't sended
	  }
	  manualCsControl = 0;
	  initDMAforSendSPI(&color, pixels + 2, 1);
	  DMATXStart();
	  while (this->isDataSending); //wait until all data wasn't sended
  }
  else {
	  SPI_DataSizeConfig(spi, SPI_DataSize_16b);
	  while (pixels) {
		  sendWord16bitMode(color);
		  pixels--;
	  }
	  SPI_DataSizeConfig(spi, SPI_DataSize_8b);
	  switchCs(1);   // CS=1;
  }
}

void ILI9341::pixelDraw(int16_t xpos, int16_t ypos, uint16_t color)
{
	while (this->isDataSending); //wait until all data wasn't sended
	switchCs(0);   // CS=0;
	setWindow(xpos, ypos, xpos, ypos);
	sendCmd(ILI9341_MEMORYWRITE_REG);
	sendWord(color);
	switchCs(1);   // CS=1;
}

void ILI9341::lineDraw(int16_t ypos, uint16_t* line,  uint32_t size)
{
	while (this->isDataSending); //wait until all data wasn't sended
	switchCs(0);   // CS=0;
	setWindow(0, ypos, size-1, ypos);
	sendCmd(ILI9341_MEMORYWRITE_REG);
	switchRs(1);

	if (!disableDMA) {
		initDMAforSendSPI(line, size, 0);
		DMATXStart();
	} else {
		uint32_t i;
		SPI_DataSizeConfig(spi, SPI_DataSize_16b);
		for (i = 0; i < size; i++) {
			sendWord16bitMode(line[i]);
		}
		SPI_DataSizeConfig(spi, SPI_DataSize_8b);
		switchCs(1);   // CS=1;
	}
}

void ILI9341::putChar(uint16_t x, uint16_t y, uint8_t chr, uint16_t charColor, uint16_t bkgColor) {
	uint8_t i, j, k;

	switchCs(0);   // CS=0
	setWindow(x, y, x+8, y+12);
	sendCmd(ILI9341_MEMORYWRITE_REG);

	SPI_DataSizeConfig(spi, SPI_DataSize_16b);
	for (i=0;i<7;i++)
		for (k=2;k>0;k--) {
		   uint8_t chl=NewBFontLAT[((chr-0x20)*14 + i+ 7*(k-1))];
		   if (k==2) chl=chl<<2;// else chl=chl;
		   uint8_t h;
		   if (k==2) h=6; else h=7;
		   for (j=0;j<h;j++) {
			   uint16_t color;
			  if (chl & 0x80) color=charColor; else color=bkgColor;
			  chl = chl<<1;
			  sendWord16bitMode(color);
		   }
	}
	for (j=0;j<13;j++) {
		sendWord16bitMode(bkgColor);
	}
	SPI_DataSizeConfig(spi, SPI_DataSize_8b);
	switchCs(1);   // CS=1;
}

void ILI9341::putString(const char str[], uint16_t x, uint16_t y, uint16_t charColor, uint16_t bkgColor) {
	while (this->isDataSending); //wait until all data wasn't sended
	while (*str != 0) {
		putChar(x, y, *str, charColor, bkgColor);
		x += 8;
		str++;
	}
}

void ILI9341::printf(uint16_t x, uint16_t y, uint16_t charColor, uint16_t bkgColor, const char *format, ...) {
	char buf[320];
	va_list args;
	va_start(args, format);
	vsprintf(buf, format, args);
	putString(buf, x, y, charColor, bkgColor);
}

void ILI9341::printf(uint16_t x, uint16_t y, const char *format, ...)
{
	char buf[320];
	va_list args;
	va_start(args, format);
	vsprintf(buf, format, args);
	putString(buf, x, y, color, bgColor);
}

uint32_t ILI9341::readID() {
	uint32_t data;
	switchCs(0);   // CS=0
	sendCmd(ILI9341_READID1_REG);
	switchRs(1);
	spi->DR = 0x00; //ignore first byte
	data = readByte();
	switchCs(1);   // CS=1
	data <<= 8;

	switchCs(0);   // CS=0
	sendCmd(ILI9341_READID2_REG);
	switchRs(1);
	spi->DR = 0x00; //ignore first byte
	data |= readByte();
	switchCs(1);   // CS=1
	data <<= 8;

	switchCs(0);   // CS=0
	sendCmd(ILI9341_READID3_REG);
	switchRs(1);
	spi->DR = 0x00; //ignore first byte
	data |= readByte();
	switchCs(1);   // CS=1
	data <<= 8;

	switchCs(0);   // CS=0
	sendCmd(ILI9341_READID4_REG);
	switchRs(1);
	spi->DR = 0x00; //ignore first byte
	data |= readByte();
	switchCs(1);   // CS=1
	
	return data;
}


uint8_t ILI9341::readByte()
{
	uint8_t data = 0x00;
	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_RXNE) == RESET);
	data = spi->DR;
	return data;
}

void ILI9341::sendByteInt(uint8_t byte)
{
	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET);
	spi->DR = byte;
}

void ILI9341::sendWordInt(uint16_t data)
{
	while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_TXE) == RESET);
	spi->DR = data;
}

void ILI9341::sendCmd(uint8_t index)
{
	switchRs(0);
	sendByteInt(index);
    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_BSY) != RESET);
}

void ILI9341::sendData(uint8_t data)
{
	switchRs(1);
	sendByteInt(data);
    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_BSY) != RESET);
}

void ILI9341::sendWord(uint16_t data)
{
	switchRs(1);
	sendByteInt(data >> 8);
	sendByteInt(data & 0xFF);
    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_BSY) != RESET);
}

void ILI9341::sendWord16bitMode(uint16_t data)
{
	switchRs(1);
	sendWordInt(data);
    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_BSY) != RESET);
}

void ILI9341::sendWords(uint16_t data1, uint16_t data2)
{
	switchRs(1);
	sendByteInt(data1 >> 8);
	sendByteInt(data1 & 0xFF);
	sendByteInt(data2 >> 8);
	sendByteInt(data2 & 0xFF);
    while (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_BSY) != RESET);
}

void ILI9341::initDMAforSendSPI(uint16_t* buffer, uint32_t size, uint8_t singleColor)
{
	DMA_InitTypeDef  DMA_InitStructure;
	//channel3 for SPI1_TX, channel5 for SPI2_TX	
	DMA_DeInit(((spi == SPI1) ? DMA1_Channel3 : DMA1_Channel5));
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(spi->DR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = size;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = singleColor ? DMA_MemoryInc_Disable : DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(((spi == SPI1) ? DMA1_Channel3 : DMA1_Channel5), &DMA_InitStructure);
}

void ILI9341::DMATXStart()
{
	SPI_DataSizeConfig(spi, SPI_DataSize_16b);
	//channel3 for SPI1_TX, channel5 for SPI2_TX
	DMA_Cmd(((spi == SPI1) ? DMA1_Channel3 : DMA1_Channel5), ENABLE);
	// Enable the SPI TX DMA request
	SPI_I2S_DMACmd(spi, SPI_I2S_DMAReq_Tx, ENABLE);
	DMA_ITConfig(((spi == SPI1) ? DMA1_Channel3 : DMA1_Channel5), DMA_IT_TC, ENABLE);
	this->isDataSending = 1;
}

void ILI9341::DMATXInterrupt()
{
	DMA_ClearITPendingBit((spi == SPI1) ? DMA1_IT_TC3 : DMA1_IT_TC5);
	DMA_Cmd(((spi == SPI1) ? DMA1_Channel3 : DMA1_Channel5), DISABLE);
	SPI_DataSizeConfig(spi, SPI_DataSize_8b);
	if (!manualCsControl) {
		switchCs(1);   // CS=1;
	}
	this->isDataSending = 0;
}


uint8_t ILI9341::IsDataSending()
{
	return this->isDataSending;
}


void ILI9341::setDisableDMA(uint8_t isDisable)
{
	this->disableDMA = isDisable; 
}


void ILI9341::setColor(uint16_t color, uint16_t bgColor)
{
	this->color = color;
	this->bgColor = bgColor;
}

void ILI9341::switchCs(short BitVal)
{
	if (BitVal != Bit_RESET) {
		controlPort->BSRR = csPin;
	} else {
		controlPort->BRR = csPin;
	}
}

void ILI9341::switchRs(short BitVal)
{
	if (BitVal != Bit_RESET) {
		controlPort->BSRR = rsPin;
	} else {
		controlPort->BRR = rsPin;
	}
}

void ILI9341::switchReset(short BitVal)
{
	if (BitVal != Bit_RESET) {
		controlPort->BSRR = resetPin;
	} else {
		controlPort->BRR = resetPin;
	}
}
