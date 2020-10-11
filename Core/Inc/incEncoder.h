
#ifndef _INC_ENCODER_H_
#define _INC_ENCODER_H_
#ifdef __cplusplus
extern "C" {
#endif



#include "tim.h"



typedef struct{

	TIM_HandleTypeDef *htim;

	uint32_t PPR;

	int32_t maxCount;

	float cycleTime;

    float theta_m_offset;
    float theta_re_offset;

    float Pn; // Pole pairs

}IncEnc_InitTypeDef;


typedef struct{

	IncEnc_InitTypeDef Init;

    uint8_t firstRun;

	uint8_t z_pulse_detected;

	int32_t raw_count; // raw count

	float raw_pos; // [rad]

	float p_raw_pos; // [rad] 1 sample previous position

    float diff_raw_pos;

    float theta_rm;
    float theta_re;
    float omega_rm;
    float omega_re;

}IncEnc_TypeDef;



void IncEnc_Init(IncEnc_TypeDef *hIncEnc, TIM_HandleTypeDef *htim, uint32_t PPR, float cycleTime, float theta_m_offset, float theta_re_offset, float Pn);

int IncEnc_Update(IncEnc_TypeDef *hIncEnc);

void IncEnc_Reset(IncEnc_TypeDef *hIncEnc);




/**********************************************************************************************************/

#if 0


typedef struct{

	uint32_t PPR;

	uint32_t count_max;

	uint8_t Pn; // Pole pairs

	float theta_m_offset;

	float theta_re_offset;

	float cycleTime;


}IncEnc_InitTypeDef;



typedef struct{

	TIM_HandleTypeDef *htim;

	IncEnc_InitTypeDef Init;

	uint32_t raw_count;

	uint32_t p_raw_count;

	int32_t count;

	int32_t count_diff;


	float raw_position; // [rad] // rotation angle of output shaft

	float p_raw_position; // [rad] // 1 sample previous position

	float theta_m; // [rad]

	float omega_m; // [rad / sec] // Rotation speed of output shaft

	float theta_re;

	float omega_re;

	uint8_t z_pulse_detected;

}IncEnc_TypeDef;


extern IncEnc_TypeDef incEnc;



void IncEnc_Init();



void refreshIncEnc(IncEnc_TypeDef *IncEnc);


void resetIncEnc(IncEnc_TypeDef *IncEnc);

#endif

#ifdef __cplusplus
}
#endif
#endif /* _INC_ENCODER_H_ */

