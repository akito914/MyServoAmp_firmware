

#include "peripheral.h"

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"



#define OCP_THRESH (7.0f)



uint32_t Iu_raw = 0;
uint32_t Iv_raw = 0;
uint32_t Iw_raw = 0;
uint32_t Vdc_raw = 0;


float V_Iu_offset = 1.6666f;
float V_Iv_offset = 1.6666f;
float V_Iw_offset = 1.6666f;

float gain_v2i = 50.0f / 1.33333f / 5.0; // 1/[V/AT] / Turn

float ADC_resolution = 4096.0f;


void ADC_Refresh(float* Iu, float* Iv, float* Iw)
{
	Iu_raw = HAL_ADC_GetValue(&hadc1);
	Iv_raw = HAL_ADC_GetValue(&hadc2);
	Iw_raw = HAL_ADC_GetValue(&hadc3);
//	Vdc_raw = HAL_ADC_GetValue(&hadc3);

	HAL_ADC_Start_IT(&hadc1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc3);

	*Iu = (Iu_raw / ADC_resolution * 3.3 - V_Iu_offset) * gain_v2i;
	*Iv = (Iv_raw / ADC_resolution * 3.3 - V_Iv_offset) * gain_v2i;
	*Iw = (Iw_raw / ADC_resolution * 3.3 - V_Iw_offset) * gain_v2i;

	if(fabs(*Iu) > OCP_THRESH || fabs(*Iv) > OCP_THRESH || fabs(*Iw) > OCP_THRESH)
	{
		StopPWM();
		Error_Handler();
	}

}




void StartPWM()
{
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);

	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_3);
}

void StopPWM()
{
	HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_3);
}



