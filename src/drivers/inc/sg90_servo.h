//
// CG90 servo control library
//
#pragma once
#ifndef _SG90_SERVO_H_
#define _SG90_SERVO_H_

#include "stm32f10x.h"

using namespace std;


class SG90Servo {
public:
	SG90Servo();
	~SG90Servo();
	void setupPort(GPIO_TypeDef* port, uint16_t pwmPin);
	void setup(TIM_TypeDef* TIMx, short channelNum);
	void setServoAngle(uint8_t angle);

protected:
	GPIO_TypeDef* port;
	uint16_t pwmPin;
	TIM_TypeDef* TIMx;
	short channelNum;

	void setupChannel(short channelNum);
private:
	enum
	{
		SERVO_SHORTEST_PULSE = 60,
		SERVO_MAX_ANGLE = 120
	};
};


#endif