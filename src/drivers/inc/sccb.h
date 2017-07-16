//
// SCCB library
//
#pragma once
#ifndef _SCCB_H_
#define _SCCB_H_

using namespace std;


class SCCB {
public:
	SCCB();
	~SCCB();
	void setup(GPIO_TypeDef* port, uint16_t clkPin, uint16_t dataPin);
	void setup(GPIO_TypeDef* port, const uint8_t slaveAddress, uint16_t clkPin, uint16_t dataPin);
	void setSlaveAddr(const uint8_t slaveAddress);
	uint8_t readRegister(const uint8_t registerAddress, uint8_t* value);
	uint8_t writeRegister(const uint8_t registerAddress, const uint8_t value);

protected:
	GPIO_TypeDef* port;
	uint16_t clkPin;
	uint16_t dataPin;
	uint8_t slaveAddress;

private:
	void SCCBSwitchClock(short val);
	void SCCBSwitchData(short val);
	static void SCCBDelay(void);
	void SCCBSwitchDataPinMode(short outMode);
	void SCCBGenerateSTART(void);
	void SCCBGenerateSTOP(void);
	void SCCBSendNAck(void);
	uint8_t SCCBReadByte(void);
	uint8_t SCCBSendByte(uint8_t data);
};


#endif
