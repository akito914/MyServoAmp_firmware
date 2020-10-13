
#include <math.h>

#include "ACR.h"



void ACR_CalcGain(ACR_TypeDef *hACR, float R, float L, float omega_c)
{
	
	hACR->Init.Kp = omega_c * L;
	hACR->Init.Ki = omega_c * R;
	
}



void ACR_Start(ACR_TypeDef *hACR)
{

	hACR->enable = 1;
	ACR_Reset(hACR);

}


void ACR_Stop(ACR_TypeDef *hACR)
{

	hACR->enable = 0;
	ACR_Reset(hACR);

}



void ACR_Refresh(ACR_TypeDef *hACR)
{

	static float _Id_ref;
	static float _Iq_ref;

	static ACR_InitTypeDef *hACR_Init;

	hACR_Init = &hACR->Init;
	
	if(hACR->enable == 0)
	{
		hACR->Id_limitError = hACR->Id_ref;
		hACR->Iq_limitError = hACR->Iq_ref;
		
		return;
	}

	_Id_ref = hACR->Id_ref;
	_Iq_ref = hACR->Iq_ref;
	
	// Limitter
	if(_Id_ref < -hACR_Init->Id_limit)			_Id_ref = -hACR_Init->Id_limit;
	else if(_Id_ref > hACR_Init->Id_limit)		_Id_ref = hACR_Init->Id_limit;

	if(_Iq_ref < -hACR_Init->Iq_limit)			_Iq_ref = -hACR_Init->Iq_limit;
	else if(_Iq_ref > hACR_Init->Iq_limit)		_Iq_ref = hACR_Init->Iq_limit;
	
	// Limit Error
	hACR->Id_limitError = hACR->Id_ref - _Id_ref;
	hACR->Iq_limitError = hACR->Iq_ref - _Iq_ref;
	
	// Control Error
	hACR->Id_error = _Id_ref - *hACR_Init->Id;
	hACR->Iq_error = _Iq_ref - *hACR_Init->Iq;
	
	// Error Integral
	hACR->Id_error_integ += hACR_Init->cycleTime * 0.5f * (hACR->Id_error + hACR->p_Id_error);
	hACR->Iq_error_integ += hACR_Init->cycleTime * 0.5f * (hACR->Iq_error + hACR->p_Iq_error);
	
	// Save as previous value
	hACR->p_Id_error = hACR->Id_error;
	hACR->p_Iq_error = hACR->Iq_error;
	
	// controller output
	hACR->Vd_ref = hACR_Init->Kp * hACR->Id_error + hACR_Init->Ki * hACR->Id_error_integ;
	hACR->Vq_ref = hACR_Init->Kp * hACR->Iq_error + hACR_Init->Ki * hACR->Iq_error_integ;


	return;
}

void ACR_Reset(ACR_TypeDef *hACR)
{

	hACR->Id_error_integ = 0.0f;
	hACR->Iq_error_integ = 0.0f;

	hACR->Id_ref = 0.0f;
	hACR->Iq_ref = 0.0f;

	hACR->Vd_ref = 0.0f;
	hACR->Vq_ref = 0.0f;
	
	hACR->p_Id_error = 0.0;
	hACR->p_Iq_error = 0.0;

}











