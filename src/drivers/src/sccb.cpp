
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "sccb.h"

using namespace std;

SCCB::SCCB() {

}

SCCB::~SCCB() {

}

void SCCB::setup(GPIO_TypeDef* port, uint16_t clkPin, uint16_t dataPin){

	this->port = port;
	this->clkPin = clkPin;
	this->dataPin = dataPin;

	/* init SCCB port*/
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = clkPin | dataPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);
}

void SCCB::setup(GPIO_TypeDef* port, const uint8_t slaveAddress, uint16_t clkPin, uint16_t dataPin) {
	setup(port, clkPin, dataPin);
	this->setSlaveAddr(slaveAddress);
}

void SCCB::setSlaveAddr(const uint8_t slaveAddress) {
	this->slaveAddress = slaveAddress;
}


uint8_t SCCB::readRegister(const uint8_t registerAddress, uint8_t* value) {
	SCCBGenerateSTART();
	if (SCCBSendByte(slaveAddress)) {
		if (SCCBSendByte(registerAddress)) {
			SCCBGenerateSTOP();
			SCCBGenerateSTART();
			if (SCCBSendByte(slaveAddress | 0x01)) {
				*value = SCCBReadByte();
				SCCBSendNAck();
				SCCBGenerateSTOP();
				return 1;
			} else {
				SCCBGenerateSTOP();
				return 0;
			}
		} else {
			SCCBGenerateSTOP();
			return 0;
		}
	} else {
		SCCBGenerateSTOP();
		return 0;
	}
}

uint8_t SCCB::writeRegister(const uint8_t registerAddress, const uint8_t value) {
	SCCBGenerateSTART();
	if (SCCBSendByte(slaveAddress)) {
		if (SCCBSendByte(registerAddress)) {
			if (SCCBSendByte(value)) {
				SCCBGenerateSTOP();
				return 1;
	        } else {
	        	SCCBGenerateSTOP();
	        	return 0;
	        }
	    } else {
	    	SCCBGenerateSTOP();
	        return 0;
	    }
	} else {
		SCCBGenerateSTOP();
	    return 0;
	}
}




void SCCB::SCCBSwitchClock(short val)
{
	if (val != 0) {
		port->BSRR = clkPin;
	} else {
		port->BRR = clkPin;
	}
}

void SCCB::SCCBSwitchData(short val)
{
	if (val != 0) {
		port->BSRR = dataPin;
	} else {
		port->BRR = dataPin;
	}
}

void SCCB::SCCBDelay(void)
{
	uint16_t i = 1275;
	while (i)
	{
		i--;
	}
}

void SCCB::SCCBSwitchDataPinMode(short outMode) {
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = dataPin;
	if (outMode != 0) {
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	}
	else {
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	}
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);
}

void SCCB::SCCBGenerateSTART(void)
{
	SCCBSwitchData(1);
	SCCBDelay();
	SCCBSwitchClock(1);
	SCCBDelay();
	SCCBSwitchData(0);
	SCCBDelay();
	SCCBSwitchClock(0);
	SCCBDelay();
}

void SCCB::SCCBGenerateSTOP(void)
{
	SCCBSwitchData(0);
	SCCBDelay();
	SCCBSwitchClock(1);
	SCCBDelay();
	SCCBSwitchData(1);
	SCCBDelay();
}

void SCCB::SCCBSendNAck(void)
{
	SCCBSwitchData(1);
	SCCBDelay();
	SCCBSwitchClock(1);
	SCCBDelay();
	SCCBSwitchClock(0);
	SCCBDelay();
	SCCBSwitchData(0);
	SCCBDelay();
}

uint8_t SCCB::SCCBReadByte(void)
{
	uint8_t data = 0;
	uint8_t i;
	SCCBSwitchDataPinMode(0); //switch data pin to IN mode
	for (i = 8; i > 0; i--) {
		SCCBDelay();
		SCCBSwitchClock(1);
		SCCBDelay();
		data = data << 1;
		if (GPIO_ReadInputDataBit(port, dataPin)) {
			data++;
		}
		SCCBSwitchClock(0);
		SCCBDelay();
	}
	SCCBSwitchDataPinMode(1); //switch data pin to OUT mode
	return data;
}

uint8_t SCCB::SCCBSendByte(uint8_t data)
{
	uint8_t sucess = 0;
	uint8_t i;

	for (i = 0; i < 8; i++) {
		if ((data << i) & 0x80) {
			SCCBSwitchData(1);
		}
		else {
			SCCBSwitchData(0);
		}
		SCCBDelay();
		SCCBSwitchClock(1);
		SCCBDelay();
		SCCBSwitchClock(0);
		SCCBDelay();
	}
	SCCBDelay();
	SCCBSwitchDataPinMode(0); //switch data pin to IN mode
	SCCBDelay();
	SCCBSwitchClock(1);
	SCCBDelay();
	if (GPIO_ReadInputDataBit(port, dataPin)) {
		sucess = 0;
	}
	else {
		sucess = 1;
	}
	SCCBSwitchClock(0);
	SCCBDelay();
	SCCBSwitchDataPinMode(1); //switch data pin to OUT mode

	return sucess;
}
