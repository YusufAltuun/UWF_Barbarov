#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stdlib.h"

/*gereken tanımlamalar burasının altında olacak*/

GPIO_InitTypeDef gpioStruct;
TIM_TimeBaseInitTypeDef timerStruct;
TIM_OCInitTypeDef pwmStruct;
USART_InitTypeDef usartConfig;

void gpioConfig(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //pwm pin çıkışı timer 3

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);		//B4 OC1 tek motor sürülecek

	gpioStruct.GPIO_Pin = GPIO_Pin_4; //  pin konfigürasyonu pwm için
	gpioStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioStruct.GPIO_OType = GPIO_OType_PP;
	gpioStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpioStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpioStruct);
}

void timerConfig()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	timerStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	timerStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerStruct.TIM_RepetitionCounter = 0;
	timerStruct.TIM_Period = 19999; //20 us
	timerStruct.TIM_Prescaler = 47;
	TIM_TimeBaseInit(TIM3, &timerStruct);
	TIM_Cmd(TIM3, ENABLE);

	pwmStruct.TIM_OCMode = TIM_OCMode_PWM1;
	pwmStruct.TIM_OutputState = ENABLE;
	pwmStruct.TIM_OCPolarity = TIM_OCPolarity_High;

}

void forward()
{

	pwmStruct.TIM_Pulse = 1900;	//b4  1.8us
	TIM_OC1Init(TIM3, &pwmStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void stop(){

	pwmStruct.TIM_Pulse = 1500;	//b4
	TIM_OC1Init(TIM3, &pwmStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void back(){

	pwmStruct.TIM_Pulse = 1100;	//b4
	TIM_OC1Init(TIM3, &pwmStruct);
	TIM_OC1PreloadC	onfig(TIM3, TIM_OCPreload_Enable);
}

void bekletme(uint32_t zaman){

	while(zaman--);

}

int main(void)
{
	gpioConfig();
	timerConfig();

  while (1)
  {

	  forward();
	/*  bekletme(3360000);
	  stop();
	  bekletme(3360000);
	  back();
	  bekletme(3360000);
*/

  }
}

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
