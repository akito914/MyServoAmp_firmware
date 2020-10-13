

#include "ASR.h"

#include <math.h>




void ASR_CalcGain(ASR_TypeDef *hASR, float J, float D, float Kt, float omega)
{
	
	hASR->Init.Kp = 2.0 * J * omega / Kt;
	hASR->Init.Ki = (J * omega * omega) / Kt;
	
	return;
}

void ASR_Start(ASR_TypeDef *hASR)
{

	hASR->enable = 1;
	ASR_Reset(hASR);

}

void ASR_Stop(ASR_TypeDef *hASR)
{

	hASR->enable = 0;
	ASR_Reset(hASR);

}


void ASR_Refresh(ASR_TypeDef *hASR)
{

	static float d_theta;
	static float _omega_ref;
	static float torque_ref;
	static float integInput;

	static ASR_InitTypeDef *hASR_Init;
	
	hASR_Init = &hASR->Init;

	// 有効時のみ実行
	if(hASR->enable == 0)
	{
		
		hASR->omega_limitError = hASR->omega_ref;
		
		return;
	}

	// 速度制限
	if(hASR->omega_ref < -hASR_Init->omega_limit)		_omega_ref = -hASR_Init->omega_limit;
	else if(hASR->omega_ref > hASR_Init->omega_limit)	_omega_ref = hASR_Init->omega_limit;
	else												_omega_ref = hASR->omega_ref;

	// 速度偏差
	hASR->omega_error = _omega_ref - *hASR_Init->omega;

	// リミット偏差フィードバックによる
	integInput = hASR->omega_error - *hASR_Init->Iq_limitError / hASR_Init->Kp;

	hASR->omega_error_integ += hASR_Init->cycleTime * 0.5 * (integInput + hASR->p_omega_error);

	hASR->p_omega_error = integInput;


	hASR->Id_ref = 0.0f;
	hASR->Iq_ref = hASR_Init->Kp * hASR->omega_error + hASR_Init->Ki * hASR->omega_error_integ;

	return;
}



void ASR_Reset(ASR_TypeDef *hASR)
{

	hASR->omega_error_integ = 0.0f;

	hASR->p_omega_error = 0.0f;
	
}





