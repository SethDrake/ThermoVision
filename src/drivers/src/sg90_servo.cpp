
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_tim.h"
#include "sg90_servo.h"

using namespace std;

SG90Servo::SG90Servo() {

}

SG90Servo::~SG90Servo() {

}

void SG90Servo::setupPort(GPIO_TypeDef* port, uint16_t pwmPin) {
	/* Configure SERVO PWM pin */
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = pwmPin;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(port, &GPIO_InitStructure);
}

void SG90Servo::setup(TIM_TypeDef* TIMx, short channelNum) {
	this->TIMx = TIMx;
	this->channelNum = channelNum;

	/* „астота счЄта - 10 мкс, период - 20 мс */
	TIM_TimeBaseInitTypeDef base_timer;
	TIM_TimeBaseStructInit(&base_timer);
	base_timer.TIM_Prescaler = SystemCoreClock / 100000 - 1;
	base_timer.TIM_Period = 2000;
	TIM_TimeBaseInit(TIMx, &base_timer);

	TIM_BDTRInitTypeDef bdtr;
	TIM_BDTRStructInit(&bdtr);
	bdtr.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
	TIM_BDTRConfig(TIMx, &bdtr);

	TIM_Cmd(TIMx, ENABLE);

	setupChannel(this->channelNum);
	setServoAngle(90);
}

void SG90Servo::setupChannel(short channelNum) {
	/* edge-aligned PWM */
	TIM_OCInitTypeDef timer_oc;
	TIM_OCStructInit(&timer_oc);
	timer_oc.TIM_Pulse = SERVO_SHORTEST_PULSE;
	timer_oc.TIM_OCMode = TIM_OCMode_PWM1;
	timer_oc.TIM_OutputState = TIM_OutputState_Enable;

	switch (channelNum)
	{
		case 1: TIM_OC1Init(TIMx, &timer_oc); TIM_OC1PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
		case 2: TIM_OC2Init(TIMx, &timer_oc); TIM_OC2PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
		case 3: TIM_OC3Init(TIMx, &timer_oc); TIM_OC3PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
		case 4: TIM_OC4Init(TIMx, &timer_oc); TIM_OC4PreloadConfig(TIMx, TIM_OCPreload_Enable); break;
		default: break;
	}
}

void SG90Servo::setServoAngle(uint8_t angle) {
	//90 degrees - center
	uint8_t internalAngle = 210 - angle;
	if (internalAngle > SERVO_MAX_ANGLE) {
		internalAngle = SERVO_MAX_ANGLE;
	}
	switch (channelNum)
	{
		case 1: TIM_SetCompare1(TIMx, SERVO_SHORTEST_PULSE + internalAngle); break;
		case 2: TIM_SetCompare2(TIMx, SERVO_SHORTEST_PULSE + internalAngle); break;
		case 3: TIM_SetCompare3(TIMx, SERVO_SHORTEST_PULSE + internalAngle); break;
		case 4: TIM_SetCompare4(TIMx, SERVO_SHORTEST_PULSE + internalAngle); break;
		default: break;
	}
}