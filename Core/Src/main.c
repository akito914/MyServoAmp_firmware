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
#include "analogSensor.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

IncEnc_TypeDef incEnc;



volatile uint32_t Iu_raw = 0;
volatile uint32_t Iv_raw = 0;
volatile uint32_t Iw_raw = 0;
volatile uint32_t Vdc_raw = 0;

float Iu = 0.0f;
float Iv = 0.0f;
float Iw = 0.0f;
float Vdc = 0.0f;

volatile float V_Iu_offset = 1.6666f;
volatile float V_Iv_offset = 1.6666f;
volatile float V_Iw_offset = 1.6666f;

float gain_v2i = 50.0f / 1.33333f / 5.0; // 1/[V/AT] / Turn

float ADC_resolution = 4096.0f;


// V/f control Test

const float VDC = 141.4f;

const float V_F_Rate = 200.0f / 60.0f;

float freq_ref = 10.0;
float freq = 0.0;
float voltage = 0.0;

float phase = 0.0;

volatile float Vu = 0, Vv = 0, Vw = 0;

volatile float amp_u = 0, amp_v = 0, amp_w = 0;


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

	printf("%d\n", incEnc.raw_count);

	if(incEnc.z_pulse_detected == 0)
	{
		IncEnc_Reset(&incEnc);
		incEnc.z_pulse_detected = 1;
	}

}




void HAL_TIM_PeriodElapsedCallback (TIM_HandleTypeDef * htim)
{

	if(htim->Instance == TIM8 && !__HAL_TIM_IS_TIM_COUNTING_DOWN(htim))
	{

		HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

//		AnalogSensor_Refresh(&analogSensor);

		IncEnc_Update(&incEnc);

		Iu_raw = HAL_ADC_GetValue(&hadc1);
		Iv_raw = HAL_ADC_GetValue(&hadc2);
		Iw_raw = HAL_ADC_GetValue(&hadc3);
//		Vdc_raw = HAL_ADC_GetValue(&hadc3);

//		HAL_ADC_Start_IT(&hadc1);
//		HAL_ADC_Start_IT(&hadc2);
//		HAL_ADC_Start_IT(&hadc3);

		Iu = (Iu_raw / ADC_resolution * 3.3 - V_Iu_offset) * gain_v2i;
		Iv = (Iv_raw / ADC_resolution * 3.3 - V_Iv_offset) * gain_v2i;
		Iw = (Iw_raw / ADC_resolution * 3.3 - V_Iw_offset) * gain_v2i;

//		Vdc = (Vdc_raw / 4096.0f * 3.3 - 1.29) * 250.0f;

//		Iu = analogSensor.Iu;
//		Iv = analogSensor.Iv;
//		Iw = analogSensor.Iw;
//		Vdc = analogSensor.Vdc;

		if((freq - freq_ref) < -0.02)
		{
			freq += 0.01;
		}
		else if((freq - freq_ref) > 0.02)
		{
			freq -= 0.01;
		}
		else
		{
			freq = freq_ref;
		}

		voltage = V_F_Rate * freq;

		Vu = voltage / sqrt(3) * sin(phase);
		Vv = voltage / sqrt(3) * sin(phase - 2.0f * M_PI / 3.0f);
		Vw = voltage / sqrt(3) * sin(phase - 4.0f * M_PI / 3.0f);

		phase += 2 * M_PI * freq * 0.0001;
		if(phase > M_PI) phase -= 2 * M_PI;
		if(phase < -M_PI) phase += 2 * M_PI;


		Vdc = 141.4;

		if(Vdc > 5.0f)
		{
			amp_u = Vu / VDC + 0.5;
			amp_v = Vv / VDC + 0.5;
			amp_w = Vw / VDC + 0.5;
		}

		if(amp_u < 0.0){ amp_u = 0.0; }else if(amp_u > 1.0){ amp_u = 1.0; }
		if(amp_v < 0.0){ amp_v = 0.0; }else if(amp_v > 1.0){ amp_v = 1.0; }
		if(amp_w < 0.0){ amp_w = 0.0; }else if(amp_w > 1.0){ amp_w = 1.0; }

		/*
		amp_u = 0.9;
		amp_v = 0.6;
		amp_w = 0.5;
		*/

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


  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);

  IncEnc_Init(&incEnc, &htim1, 2000, 100E-6, 0.0, 0.0, 4);

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

  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start_IT(&htim8, TIM_CHANNEL_3);

  sample_start = 1;




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


		  HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_1);
		  HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_2);
		  HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_3);

		  HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
		  HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_2);
		  HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_3);

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
