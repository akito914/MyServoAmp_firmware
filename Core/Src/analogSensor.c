

#include "analogSensor.h"

#include "adc.h"
#include "string.h"
#include "stdlib.h"



const float Vref_AD = 3.3f;

const int32_t AD_Range = 4096;



analogSensor_TypeDef analogSensor;



int32_t median3(int32_t *buf);

int32_t median5(int32_t *buf);


void AnalogSensor_Init()
{

	memset(&analogSensor, 0x00, sizeof(analogSensor));

	analogSensor.Init.CS_Type = CS_Type_2DCCT_UW;

	analogSensor.Init.Iu_Gain = 7.5756f;	// 1 / ( R * OPAmpGain) [A / V]
	analogSensor.Init.Iw_Gain = 7.5756f;	// 1 / ( R * OPAmpGain) [A / V]
	analogSensor.Init.Vdc_Gain = 250.0f;	// 1 / ( R * OPAmpGain) [A / V]

	analogSensor.Init.V_Iu_offset = 1.6471;
	analogSensor.Init.V_Iw_offset = 1.64475;
	analogSensor.Init.V_Vdc_offset = 1.2628;

	analogSensor.Init.hadc_Iu = &hadc1;
	analogSensor.Init.hadc_Iw = &hadc2;
	analogSensor.Init.hadc_Vdc = &hadc3;

	analogSensor.pos_MEDF_I = 0;

}


void AnalogSensor_Start(analogSensor_TypeDef *hSensor)
{

	switch(hSensor->Init.CS_Type)
	{
	case CS_Type_3shunt:

		HAL_ADC_Start_IT(hSensor->Init.hadc_Iu);
		HAL_ADC_Start_IT(hSensor->Init.hadc_Iv);
		HAL_ADC_Start_IT(hSensor->Init.hadc_Iw);

		break;

	case CS_Type_2DCCT_UW:

		HAL_ADC_Start_IT(hSensor->Init.hadc_Iu);
		HAL_ADC_Start_IT(hSensor->Init.hadc_Iw);
		HAL_ADC_Start_IT(hSensor->Init.hadc_Vdc);

		break;

	}



}


/*
 * UVWの電流値を更新するだけ
 * 座標変換とかはやらない
 */
inline void AnalogSensor_Refresh(analogSensor_TypeDef *hSensor)
{

	static int32_t AD_Iu_MEDF = 0;
	static int32_t AD_Iv_MEDF = 0;
	static int32_t AD_Iw_MEDF = 0;
	static int32_t AD_Vdc_MEDF = 0;

	static analogSensor_InitTypeDef *hSensor_Init;

	hSensor_Init = &hSensor->Init;

	switch(hSensor->Init.CS_Type)
	{
	case CS_Type_3shunt:

		hSensor->AD_Iu[0] = HAL_ADC_GetValue(hSensor_Init->hadc_Iu);
		hSensor->AD_Iv[0] = HAL_ADC_GetValue(hSensor_Init->hadc_Iv);
		hSensor->AD_Iw[0] = HAL_ADC_GetValue(hSensor_Init->hadc_Iw);

		HAL_ADC_Start_IT(hSensor_Init->hadc_Iu);
		HAL_ADC_Start_IT(hSensor_Init->hadc_Iv);
		HAL_ADC_Start_IT(hSensor_Init->hadc_Iw);

		hSensor->AD_Iu_buf[hSensor->pos_MEDF_I] = (int32_t)hSensor->AD_Iu[0];
		hSensor->AD_Iv_buf[hSensor->pos_MEDF_I] = (int32_t)hSensor->AD_Iv[0];
		hSensor->AD_Iw_buf[hSensor->pos_MEDF_I] = (int32_t)hSensor->AD_Iw[0];

		// MedianFilter用バッファ書き込み位置更新
		hSensor->pos_MEDF_I += 1;
		if(hSensor->pos_MEDF_I >= MEDIAN_ORDER)
		{
			hSensor->pos_MEDF_I = 0;
		}

#if MEDIAN_ORDER == 3
		AD_Iu_MEDF = median3(hSensor->AD_Iu_buf);
		AD_Iv_MEDF = median3(hSensor->AD_Iv_buf);
		AD_Iw_MEDF = median3(hSensor->AD_Iw_buf);
#elif MEDIAN_ORDER == 5
		AD_Iu_MEDF = median5(hSensor->AD_Iu_buf);
		AD_Iv_MEDF = median5(hSensor->AD_Iv_buf);
		AD_Iw_MEDF = median5(hSensor->AD_Iw_buf);
#else
		AD_Iu_MEDF = hSensor->AD_Iu[0];
		AD_Iv_MEDF = hSensor->AD_Iv[0];
		AD_Iw_MEDF = hSensor->AD_Iw[0];
#endif

		// 端子電圧更新
		hSensor->V_Iu = (float)AD_Iu_MEDF / AD_Range * Vref_AD - hSensor_Init->V_Iu_offset;
		hSensor->V_Iv = (float)AD_Iv_MEDF / AD_Range * Vref_AD - hSensor_Init->V_Iv_offset;
		hSensor->V_Iw = (float)AD_Iw_MEDF / AD_Range * Vref_AD - hSensor_Init->V_Iw_offset;


		switch(hSensor->SVM_sector)
		{
		case 0: case 5:
			hSensor->Iv = hSensor->V_Iv * hSensor->Init.Iv_Gain;
			hSensor->Iw = hSensor->V_Iw * hSensor->Init.Iw_Gain;
			hSensor->Iu = - hSensor->Iv - hSensor->Iw;
			break;

		case 1: case 2:
			hSensor->Iw = hSensor->V_Iw * hSensor->Init.Iw_Gain;
			hSensor->Iu = hSensor->V_Iu * hSensor->Init.Iu_Gain;
			hSensor->Iv = - hSensor->Iw - hSensor->Iu;
			break;

		case 3: case 4:
			hSensor->Iu = hSensor->V_Iu * hSensor->Init.Iu_Gain;
			hSensor->Iv = hSensor->V_Iv * hSensor->Init.Iv_Gain;
			hSensor->Iw = - hSensor->Iu - hSensor->Iv;
			break;
		}

		break; /* CS_Type_3shunt */

	case CS_Type_2DCCT_UW:

		hSensor->AD_Iu[0] = HAL_ADC_GetValue(hSensor_Init->hadc_Iu);
		hSensor->AD_Iw[0] = HAL_ADC_GetValue(hSensor_Init->hadc_Iw);
		hSensor->AD_Vdc[0] = HAL_ADC_GetValue(hSensor_Init->hadc_Vdc);

		HAL_ADC_Start_IT(hSensor_Init->hadc_Iu);
		HAL_ADC_Start_IT(hSensor_Init->hadc_Iw);
		HAL_ADC_Start_IT(hSensor_Init->hadc_Vdc);

		hSensor->AD_Iu_buf[hSensor->pos_MEDF_I] = (int32_t)hSensor->AD_Iu[0];
		hSensor->AD_Iw_buf[hSensor->pos_MEDF_I] = (int32_t)hSensor->AD_Iw[0];
		hSensor->AD_Vdc_buf[hSensor->pos_MEDF_I] = (int32_t)hSensor->AD_Vdc[0];

		// MedianFilter用バッファ書き込み位置更新
		hSensor->pos_MEDF_I += 1;
		if(hSensor->pos_MEDF_I >= MEDIAN_ORDER)
		{
			hSensor->pos_MEDF_I = 0;
		}

#if MEDIAN_ORDER == 3
		AD_Iu_MEDF = median3(hSensor->AD_Iu_buf);
		AD_Iv_MEDF = median3(hSensor->AD_Iv_buf);
		AD_Iw_MEDF = median3(hSensor->AD_Iw_buf);
#elif MEDIAN_ORDER == 5
		AD_Iu_MEDF = median5(hSensor->AD_Iu_buf);
		AD_Iv_MEDF = median5(hSensor->AD_Iv_buf);
		AD_Iw_MEDF = median5(hSensor->AD_Iw_buf);
#else
		AD_Iu_MEDF = hSensor->AD_Iu[0];
		AD_Iw_MEDF = hSensor->AD_Iw[0];
		AD_Vdc_MEDF = hSensor->AD_Vdc[0];
#endif

		// pin voltage refresh
		hSensor->V_Iu = (float)AD_Iu_MEDF / AD_Range * Vref_AD - hSensor_Init->V_Iu_offset;
		hSensor->V_Iw = (float)AD_Iw_MEDF / AD_Range * Vref_AD - hSensor_Init->V_Iw_offset;
		hSensor->V_Vdc = (float)AD_Vdc_MEDF / AD_Range * Vref_AD - hSensor_Init->V_Vdc_offset;

		hSensor->Iu = hSensor->V_Iu * hSensor->Init.Iu_Gain;
		hSensor->Iw = hSensor->V_Iw * hSensor->Init.Iw_Gain;
		hSensor->Iv = -hSensor->Iu - hSensor->Iw;
		hSensor->Vdc = hSensor->V_Vdc * hSensor->Init.Vdc_Gain;

		break; /* CS_Type_2DCCT_UW */

		default:
			break;


	}




}



/*
 * 回転座標系における電流を算出
 */
inline void AnalogSensor_getIdq(analogSensor_TypeDef *hSensor)
{

	hSensor->Id = 0.8165f * (
			+ hSensor->Iu * hSensor->cos_theta_re
			+ hSensor->Iv * (-0.5f * hSensor->cos_theta_re + 0.855f * hSensor->sin_theta_re)
			+ hSensor->Iw * (-0.5f * hSensor->cos_theta_re - 0.855f * hSensor->sin_theta_re));

	hSensor->Iq = 0.8165f * (
			- hSensor->Iu * hSensor->sin_theta_re
			+ hSensor->Iv * (0.5f * hSensor->sin_theta_re + 0.855f * hSensor->cos_theta_re)
			+ hSensor->Iw * (0.5f * hSensor->sin_theta_re - 0.855f * hSensor->cos_theta_re));

}


/*
 * Length:3 のメディアンフィルタ
 */
inline int32_t median3(int32_t *buf)
{

	if(buf[0] < buf[1])
	{
		if(buf[2] < buf[0])			return buf[0];
		else if(buf[2] < buf[1])	return buf[2];
		else						return buf[1];
	}
	else
	{
		if(buf[2] < buf[1])			return buf[1];
		else if(buf[2] < buf[1])	return buf[2];
		else						return buf[0];
	}

	return 0;
}


inline int32_t median5(int32_t *buf)
{
	static uint32_t winCount[5] = {0};

	winCount[0] = 0;
	winCount[1] = 0;
	winCount[2] = 0;
	winCount[3] = 0;
	winCount[4] = 0;

	if (buf[0] > buf[1]) winCount[0]++; else winCount[1]++;
	if (buf[0] > buf[2]) winCount[0]++; else winCount[2]++;
	if (buf[0] > buf[3]) winCount[0]++; else winCount[3]++;
	if (buf[0] > buf[4]) winCount[0]++; else winCount[4]++;

	if (winCount[0] == 2) return buf[0];

	if (buf[1] > buf[2]) winCount[1]++; else winCount[2]++;
	if (buf[1] > buf[3]) winCount[1]++; else winCount[3]++;
	if (buf[1] > buf[4]) winCount[1]++; else winCount[4]++;

	if (winCount[1] == 2) return buf[1];

	if (buf[2] > buf[3]) winCount[2]++; else winCount[3]++;
	if (buf[2] > buf[4]) winCount[2]++; else winCount[4]++;

	if (winCount[2] == 2) return buf[2];

	if (buf[3] > buf[4]) winCount[3]++; else winCount[4]++;

	if (winCount[3] == 2) return buf[3];

	return buf[4];

}



