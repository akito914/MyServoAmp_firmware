
#ifndef _ACR_H_
#define _ACR_H_

#include <stdint.h>

typedef struct
{

	float Kp;
	float Ki;

	float cycleTime;

	float Id_limit, Iq_limit;
	
	float *Id, *Iq;
	
}ACR_InitTypeDef;


typedef struct
{

	ACR_InitTypeDef Init;

	uint8_t enable;

	float Id_limitError, Iq_limitError;

	float Id_ref, Iq_ref;

	float Id_error, Iq_error;

	float p_Id_error, p_Iq_error;

	float Id_error_integ, Iq_error_integ;
	
	float Vd_ref, Vq_ref;

}ACR_TypeDef;


void ACR_CalcGain(ACR_TypeDef *hACR, float R, float L, float omega);


void ACR_Start(ACR_TypeDef *hACR);


void ACR_Stop(ACR_TypeDef *hACR);


void ACR_Refresh(ACR_TypeDef *hACR);


void ACR_Reset(ACR_TypeDef *hACR);



#endif /* _ACR_H_ */
