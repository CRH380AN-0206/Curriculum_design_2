/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "dac.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "stdio.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
	int KEYState0,KEYState1;
	char txt1[32];
	char txt2[32];
	void setvolt(float volt)
	{
		uint32_t dacvalue=0;
		float gain=0.0;
		dacvalue=(volt/3.3)*4096;
		gain=40.062*volt-34.925;
		HAL_DAC_Start(&hdac, DAC_CHANNEL_1);
		HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dacvalue);
		sprintf(txt1,"%.2f",volt);
		OLED_ShowString(15,1, txt1);
		sprintf(txt2,"%.2f",gain);
		OLED_ShowString(15,3, txt2);
	}
	void short_delay(int time)
	{
		int i,j;
		for(i=0;i<time;i++)
		{
			for(j=0;j<72;j++)
			{
				__NOP();
			}
		}
	}
	void x_init(unsigned char num)
	{
		unsigned char i;
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(XCS_GPIO_Port,XCS_Pin,GPIO_PIN_RESET);
		short_delay(20);
		HAL_GPIO_WritePin(UD_GPIO_Port,UD_Pin,GPIO_PIN_RESET);
		short_delay(20);
		for(i=0;i<100;i++)
		{
			HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
			short_delay(250);
			HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_RESET);
			short_delay(250);
		}
		HAL_GPIO_WritePin(UD_GPIO_Port,UD_Pin,GPIO_PIN_SET);
		short_delay(20);
		for(i=0;i<num;i++)
		{
			HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
			short_delay(250);
			HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_RESET);
			short_delay(250);
		}
		/**save**/
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(XCS_GPIO_Port,XCS_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(UD_GPIO_Port,UD_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
	}
	void x_inc(unsigned char num,float show)
	{
		unsigned char i;
		char txt1[32];
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(XCS_GPIO_Port,XCS_Pin,GPIO_PIN_RESET);
		short_delay(20);
		HAL_GPIO_WritePin(UD_GPIO_Port,UD_Pin,GPIO_PIN_SET);
		short_delay(20);
		for(i=0;i<num;i++)
		{
			HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
			short_delay(250);
			HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_RESET);
			short_delay(250);
		}
		/**save**/
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(XCS_GPIO_Port,XCS_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(UD_GPIO_Port,UD_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
		/**show**/
		sprintf(txt1,"%.2f",show);
		OLED_ShowString(15,5, txt1);
	}
	void x_dec(unsigned char num,float show)
	{
		unsigned char i;
		char txt1[32];
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(XCS_GPIO_Port,XCS_Pin,GPIO_PIN_RESET);
		short_delay(20);
		HAL_GPIO_WritePin(UD_GPIO_Port,UD_Pin,GPIO_PIN_RESET);
		short_delay(20);
		for(i=0;i<num;i++)
		{
			HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
			short_delay(250);
			HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_RESET);
			short_delay(250);
		}
		/**save**/
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(XCS_GPIO_Port,XCS_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(UD_GPIO_Port,UD_Pin,GPIO_PIN_SET);
		short_delay(20);
		HAL_GPIO_WritePin(INC_GPIO_Port,INC_Pin,GPIO_PIN_SET);
		/**show**/
		sprintf(txt1,"%.2f",show);
		OLED_ShowString(15,5, txt1);
	}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	float volt=0.0;
	float resister=1.0;
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
  MX_DAC_Init();
  /* USER CODE BEGIN 2 */
	OLED_Init();			//³õÊ¼»¯OLED  
	OLED_Clear(); 
	OLED_ShowCHinese(0,0,0);//Â½
	OLED_ShowCHinese(18,0,1);//¿¡
	OLED_ShowCHinese(36,0,2);//ºê
	OLED_ShowString(0,3, "20182332002");
	HAL_Delay(3000);
	OLED_Clear();
	OLED_ShowString(0,0, "DAC_volt=");
	OLED_ShowString(15,1, "0.00V");
	OLED_ShowString(0,2, "Gain=");
	OLED_ShowString(15,3, "00.00 dB");
	OLED_ShowString(0,4, "X9C103S=");
	OLED_ShowString(15,5, " 1.00k Ohm");
	x_init(10);
	HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 if(KEYState0==1)
	 {
		 if(volt<2.0)
		 {
			 volt=volt+0.1;
		 }
		 else if(volt>=2.0&&resister<10.0)
		 {
			 resister=resister+0.5;
			 x_inc(5,resister);
		 }
		 setvolt(volt);
		 HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		 HAL_Delay(100);
		 HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
	 }
	 if(KEYState1==1)
	 {
		 if(volt>=2.0&&resister>1.0)
		 {
			 resister=resister-0.5;
			 x_dec(5,resister);
		 }
		 else if(volt>=0.1)
		 {
			 volt=volt-0.1;
		 }
		 else volt=0.0;
		 setvolt(volt);
		 HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		 HAL_Delay(100);
		 HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
