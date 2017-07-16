
#pragma once
#ifndef __DELAY_H_
#define __DELAY_H_

#include "stm32f10x.h"

using namespace std;

class DelayManager {
private:
	DelayManager();
	~DelayManager();
	
	static volatile uint32_t timingDelay;
	static volatile uint64_t sysTickCount;

public:
	static void DelayMs(volatile uint32_t nTime);
	static void DelayUs(volatile uint32_t nTime);
	static void Delay(volatile uint32_t nTime);
	static void TimingDelay_Decrement();
	static void SysTickIncrement();
	static uint64_t GetSysTickCount();
};

#endif
