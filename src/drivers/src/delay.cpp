
#include "delay.h"
#include "stm32f10x_rcc.h"

using namespace std;

volatile uint32_t DelayManager::timingDelay;
volatile uint64_t DelayManager::sysTickCount;

DelayManager::DelayManager() {
	this->sysTickCount = 0;
	this->timingDelay = 0;
}

DelayManager::~DelayManager() {

}

void DelayManager::Delay(volatile uint32_t nTime){
  timingDelay = nTime;
  while(DelayManager::timingDelay > 0);
}

void DelayManager::TimingDelay_Decrement(void){
  if (timingDelay > 0){
    timingDelay--;
  }
}

void DelayManager::SysTickIncrement(void) {
	sysTickCount++;
	TimingDelay_Decrement();
}

uint64_t DelayManager::GetSysTickCount() {
	return sysTickCount;
}

void DelayManager::DelayMs(volatile uint32_t nTime)
{
	volatile uint32_t nCount;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

	nCount = (RCC_Clocks.HCLK_Frequency / 10000) * nTime;
	for (; nCount != 0; nCount--);
}

void DelayManager::DelayUs(volatile uint32_t nTime)
{
	volatile uint32_t nCount;
	RCC_ClocksTypeDef RCC_Clocks;
	RCC_GetClocksFreq(&RCC_Clocks);

	nCount = (RCC_Clocks.HCLK_Frequency / 10) * nTime;
	for (; nCount != 0; nCount--);
}

