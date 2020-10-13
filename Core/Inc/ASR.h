

#ifndef _ASR_H_
#define _ASR_H_

#include <stdint.h>


typedef struct
{

	float Kp;
	float Ki;

	float cycleTime;

	float omega_limit;
	
	float *Iq_limitError;
	
	float *omega;

}ASR_InitTypeDef;

typedef struct
{

	ASR_InitTypeDef Init;

	uint8_t enable;
	
	float omega_limitError;

	float omega_ref;

	float omega_error;

	float p_omega_error;

	float omega_error_integ;

	float Id_ref;
	float Iq_ref;

}ASR_TypeDef;


void ASR_CalcGain(ASR_TypeDef *hASR, float J, float D, float Kt, float omega);


void ASR_Start(ASR_TypeDef *hASR);


void ASR_Stop(ASR_TypeDef *hASR);


void ASR_Refresh(ASR_TypeDef *hASR);


void ASR_Reset(ASR_TypeDef *hASR);



#endif /* _ASR_H_ */
