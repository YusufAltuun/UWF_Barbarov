#include "stm32f4xx.h"
#include "stm32f4_discovery.h"
#include "stdlib.h"

/*gereken tanýmlamalar burasýnýn altnda olacak
*/

GPIO_InitTypeDef gpioStruct;
TIM_TimeBaseInitTypeDef timerStruct;
TIM_OCInitTypeDef pwmStruct;
ADC_CommonInitTypeDef adcCommon;
ADC_InitTypeDef adcStruct;
USART_InitTypeDef usartConfig;

//deðiþkenler
uint16_t adcDeger,mapDeger;

void gpioConfig(){

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); //pwm pin çýkýþý timer 3
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); // ADC1 A7

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4, GPIO_AF_TIM3);		//B4 OC1 tek motor sürülecek

	gpioStruct.GPIO_Pin = GPIO_Pin_4; //  pin konfigürasyonu pwm için
	gpioStruct.GPIO_Mode = GPIO_Mode_AF;
	gpioStruct.GPIO_OType = GPIO_OType_PP;
	gpioStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpioStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOB, &gpioStruct);

	gpioStruct.GPIO_Pin = GPIO_Pin_7;
	gpioStruct.GPIO_Mode = GPIO_Mode_AN;
	gpioStruct.GPIO_OType = GPIO_OType_PP;
	gpioStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpioStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOA, &gpioStruct);
}

void timerConfig()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	timerStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	timerStruct.TIM_CounterMode = TIM_CounterMode_Up;
	timerStruct.TIM_RepetitionCounter = 0;
	timerStruct.TIM_Period = 19999; //20 us
	timerStruct.TIM_Prescaler = 83;
	TIM_TimeBaseInit(TIM3, &timerStruct);
	TIM_Cmd(TIM3, ENABLE);

	pwmStruct.TIM_OCMode = TIM_OCMode_PWM1;
	pwmStruct.TIM_OutputState = ENABLE;
	pwmStruct.TIM_OCPolarity = TIM_OCPolarity_High;

}

void adcConfig(){
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	adcCommon.ADC_Mode = ADC_Mode_Independent;
	adcCommon.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInit(&adcCommon);

	adcStruct.ADC_Resolution = ADC_Resolution_8b;
	ADC_Init(ADC1, &adcStruct);

	ADC_Cmd(ADC1,ENABLE);
}

uint16_t oku(){
	ADC_RegularChannelConfig(ADC1,ADC_Channel_7, 1, ADC_SampleTime_56Cycles);
	ADC_SoftwareStartConv(ADC1);
	while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)==RESET);
	return ADC_GetConversionValue(ADC1);
}

void genel(long gelen){
	pwmStruct.TIM_Pulse = gelen;	//b4
	TIM_OC1Init(TIM3, &pwmStruct);
	TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
}

void bekletme(uint32_t zaman){

	while(zaman--);

}

long map(long x,long in_min,long in_max,long out_min,long out_max)
{
	return (x-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

int main(void)
{
	gpioConfig();
	timerConfig();
	adcConfig();
  while (1)
  {
	  adcDeger = oku();
	  mapDeger = map(adcDeger, 1, 255, 1000, 2000);
	  genel(mapDeger);
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
