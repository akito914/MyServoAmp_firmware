/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "math.h"
#include "incEncoder.h"
#include "sin_t.h"
#include "ACR.h"
#include "ASR.h"
#include "peripheral.h"
#include "parameters.h"



/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define TS (100E-6)


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */


/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

IncEnc_TypeDef incEnc;

ACR_TypeDef acr;
ASR_TypeDef asr;

// Sensor


float Iu = 0.0f;
float Iv = 0.0f;
float Iw = 0.0f;
float Vdc = 0.0f;


const float VDC = 141.4f;

float Id_ref = 0.0f, Iq_ref = 0.0f;

float Id = 0.0f;
float Iq = 0.0f;

float omega = 0.0;
float omega_ref = 0.0;

// state variable

float Vd_ref = 0.0, Vq_ref = 0.0;

volatile float Vu_ref = 0, Vv_ref = 0, Vw_ref = 0;

volatile float amp_u = 0, amp_v = 0, amp_w = 0;

float cos_theta_re = 0.0f, sin_theta_re = 0.0f;

// Dump

#define SAMPLE_NUM	5000

float debug_dump[SAMPLE_NUM][4] = {{0}};

int sample_start = 0;
int sample_count = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */
void __io_putchar(uint8_t ch)
{
	HAL_UART_Transmit(&huart2, &ch, 1, 1);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_GPIO_EXTI_Callback (uint16_t GPIO_Pin)
{

	//printf("%d\n", incEnc.raw_count);

	if(incEnc.z_pulse_detected == 0)
	{
		IncEnc_Reset(&incEnc);
		incEnc.z_pulse_detected = 1;
	}

}



void uvw2dq(float *d, float *q, float u, float v, float w, float Cos, float Sin)
{
	*d = 0.816496580927726f * (
			+ u * Cos
			+ v * (-0.5f * Cos + 0.8660254037844386f * Sin)
			+ w * (-0.5f * Cos - 0.8660254037844386f * Sin));

	*q = 0.816496580927726f * (
			- u * Sin
			+ v * (0.5f * Sin + 0.8660254037844386f * Cos)
			+ w * (0.5f * Sin - 0.8660254037844386f * Cos));
}

void dq2uvw(float *u, float *v, float *w, float d, float q, float Cos, float Sin)
{
	*u = 1.224744871391589f * (d * Cos - q * Sin);

	*v = 1.224744871391589f * (
			+ d * (-0.5f * Cos + 0.8660254037844386f * Sin)
			+ q * (0.5f * Sin + 0.8660254037844386f * Cos));

	*w = - *u - *v;
}



void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{

	if(htim->Instance == TIM8 && !__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
	{

		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

		IncEnc_Update(&incEnc);

		ADC_Refresh(&Iu, &Iv, &Iw);

		cos_theta_re = cos_t(incEnc.theta_re);
		sin_theta_re = sin_t(incEnc.theta_re);

		uvw2dq(&Id, &Iq, Iu, Iv, Iw, cos_theta_re, sin_theta_re);

		// ASR
		if(asr.enable == 1)
		{
			asr.omega_ref = omega_ref;
			ASR_Refresh(&asr);
			acr.Id_ref = asr.Id_ref;
			acr.Iq_ref = asr.Iq_ref;
		}
		else
		{
			acr.Id_ref = Id_ref;
			acr.Iq_ref = Iq_ref;
		}

		// ACR
		if(acr.enable == 1)
		{
			ACR_Refresh(&acr);
			Vd_ref = acr.Vd_ref + incEnc.omega_re * -M_Lq * Iq;
			Vq_ref = acr.Vq_ref + incEnc.omega_re * (M_Psi + M_Ld * Id);
		}
		else
		{
			Vd_ref = 0.0;
			Vq_ref = 0.0;
		}

		dq2uvw(&Vu_ref, &Vv_ref, &Vw_ref, Vd_ref, Vq_ref, cos_theta_re, sin_theta_re);

		Vdc = 141.4;

		if(Vdc > 5.0f)
		{
			amp_u = Vu_ref / VDC + 0.5;
			amp_v = Vv_ref / VDC + 0.5;
			amp_w = Vw_ref / VDC + 0.5;
		}

		if(amp_u < 0.0){ amp_u = 0.0; }else if(amp_u > 1.0){ amp_u = 1.0; }
		if(amp_v < 0.0){ amp_v = 0.0; }else if(amp_v > 1.0){ amp_v = 1.0; }
		if(amp_w < 0.0){ amp_w = 0.0; }else if(amp_w > 1.0){ amp_w = 1.0; }

		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, htim->Init.Period * amp_u);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, htim->Init.Period * amp_v);
		__HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, htim->Init.Period * amp_w);

#if 0
		if(sample_start == 1 && freq >= 9.0f)
		{
			debug_dump[sample_count][0] = Iu;
			debug_dump[sample_count][1] = Iv;
			debug_dump[sample_count][2] = Iw;
			debug_dump[sample_count][3] = Vdc;

			if(sample_count < SAMPLE_NUM - 1)
			{
				sample_count++;
			}
			else
			{
				sample_start = 2;
				freq_ref = 0.0;
			}
		}

#endif


		HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

	}



}



/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();
  MX_TIM1_Init();
  MX_TIM8_Init();
  /* USER CODE BEGIN 2 */


  /******************** Initialization ********************/


//  AnalogSensor_Init();

//  AnalogSensor_Start(&analogSensor);


  // ADC Setting

  HAL_ADC_Start_IT(&hadc1);
  HAL_ADC_Start_IT(&hadc2);
  HAL_ADC_Start_IT(&hadc3);


  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  IncEnc_Init(&incEnc, &htim1, 2000, 100E-6, 0.0, 3.14159f, 4);


  // ACR Setting
  acr.Init.cycleTime = TS;
  acr.Init.Id_limit = 3.0f;
  acr.Init.Iq_limit = 3.0f;
  acr.Init.Id = &Id;
  acr.Init.Iq = &Iq;
  ACR_CalcGain(&acr, M_R, (M_Ld + M_Lq) / 2.0, 2000);

  // ASR Setting
  asr.Init.cycleTime = TS;
  asr.Init.Iq_limitError = &acr.Iq_limitError;
  asr.Init.omega_limit = 500;
  asr.Init.omega = &incEnc.omega_rm;
  ASR_CalcGain(&asr, M_Jm, M_Dm, M_Kt, 200);

  while(incEnc.z_pulse_detected == 0)
  {
	  HAL_Delay(100);
  }

  // PWM Setting
  __HAL_TIM_CLEAR_FLAG(&htim8, TIM_FLAG_UPDATE);
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);

  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_UPDATE);
  HAL_TIM_GenerateEvent(&htim8, TIM_EVENTSOURCE_TRIGGER);

  __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, 8990);

  StartPWM();

  sample_start = 1;

  HAL_Delay(1000);

  ACR_Start(&acr);

  ASR_Start(&asr);


  //printf("Hello myServoAmpProject. \n");





  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static int count = 0;

	  HAL_Delay(1);
#if 0
	  if(sample_start == 2 && freq <= 1.0)
	  {
		  int i;

		  StopPWM();

		  HAL_Delay(100);

		  for(i = 0; i < SAMPLE_NUM; i++)
		  {
			  printf("%f, %f, %f, %f\n", debug_dump[i][0], debug_dump[i][1], debug_dump[i][2], debug_dump[i][3]);
		  }

		  sample_start = 3;

	  }
#endif


	  //refreshIncEnc(&incEnc);

	  if(count < 10)
	  {
		  count++;
	  }
	  else
	  {
		  //printf("count = %d\n", incEnc.htim->Instance->CNT);
		  count = 0;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 360;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
