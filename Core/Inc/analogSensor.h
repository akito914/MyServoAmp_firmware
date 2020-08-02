

#ifndef _ANALOG_SENSOR_H_
#define _ANALOG_SENSOR_H_


#include "adc.h"


#define MEDIAN_ORDER	0



typedef enum
{
	CS_Type_3shunt,
	CS_Type_1shunt,
	CS_Type_2DCCT_UW,

}CurrentSensor_Type;



typedef struct
{

	CurrentSensor_Type CS_Type;

	ADC_HandleTypeDef *hadc_Iu;
	ADC_HandleTypeDef *hadc_Iv;
	ADC_HandleTypeDef *hadc_Iw;
	ADC_HandleTypeDef *hadc_Idc;
	ADC_HandleTypeDef *hadc_Vdc;

	float Iu_Gain;
	float Iv_Gain;
	float Iw_Gain;
	float Vdc_Gain;

	float V_Iu_offset;
	float V_Iv_offset;
	float V_Iw_offset;
	float V_Idc_offset;
	float V_Vdc_offset;


}analogSensor_InitTypeDef;



typedef struct
{

	analogSensor_InitTypeDef Init;

	uint16_t AD_Iu[1];
	uint16_t AD_Iv[1];
	uint16_t AD_Iw[1];
	uint16_t AD_Vdc[1];

	int32_t pos_MEDF_I;

	int32_t AD_Iu_buf[MEDIAN_ORDER];
	int32_t AD_Iv_buf[MEDIAN_ORDER];
	int32_t AD_Iw_buf[MEDIAN_ORDER];
	int32_t AD_Vdc_buf[MEDIAN_ORDER];

	uint8_t SVM_sector;

	float V_Iu, V_Iv, V_Iw, V_Vdc;

	float Iu, Iv, Iw, Vdc;

	float Id, Iq;

	float cos_theta_re, sin_theta_re;

}analogSensor_TypeDef;



extern analogSensor_TypeDef analogSensor;




void AnalogSensor_Init();

void AnalogSensor_Start(analogSensor_TypeDef *hSensor);


/*
 * UVWの電流値を更新するだけ
 * 座標変換とかはやらない
 */
void AnalogSensor_Refresh(analogSensor_TypeDef *hSensor);


/*
 * 回転座標系における電流を算出
 */
void AnalogSensor_getIdq(analogSensor_TypeDef *hSensor);




#endif /* _ANALOG_SENSOR_H_ */



