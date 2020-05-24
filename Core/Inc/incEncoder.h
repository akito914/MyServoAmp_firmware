
#ifndef _INC_ENCODER_H_
#define _INC_ENCODER_H_
#ifdef __cplusplus
extern "C" {
#endif



#include "tim.h"



typedef struct{

	float GearRatio_out2enc;

	uint32_t PPR;

	uint32_t count_max;

	float cycleTime;

	uint32_t prescale;


}IncEnc_InitTypeDef;



typedef struct{

	TIM_HandleTypeDef *htim;

	IncEnc_InitTypeDef Init;


	uint32_t raw_count;

	uint32_t p_raw_count;

	int32_t count;

	// rotation angle of output shaft
	float position; // [rad]

	// 1 sample previous position
	float p_position; // [rad]

	// Rotation speed of output shaft
	float speed; // [rad / sec]


	uint32_t prescaleCount;

}IncEnc_TypeDef;


extern IncEnc_TypeDef incEnc;



void IncEnc_Init();



void refreshIncEnc(IncEnc_TypeDef *IncEnc);


void resetIncEnc(IncEnc_TypeDef *IncEnc);



#ifdef __cplusplus
}
#endif
#endif /* _INC_ENCODER_H_ */

