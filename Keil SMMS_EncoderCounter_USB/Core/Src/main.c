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
#include "tim.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct _paramType
{
	uint32_t encoderPulseCount;
	float wheelRadius;
	float targetDistance;
} __attribute__((aligned(1), packed)) WheelParam;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//Setting var at page 63
#define BACKUP_FLASH_ADDR 0x0801F800

#define TARGET_PULSE_NUMBER           1

#define DIVISOR                       1

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int A_PLS_CNT = 0;
int B_PLS_CNT = 0;

bool bFlag = false;
uint8_t xFlag = 0;
uint8_t lineFlag = 0;
extern char rxbuf[200];
char txbuf[300];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch (GPIO_Pin)
	{
	case GPIO_PIN_5:
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_0);
		if (bFlag)
		{
			A_PLS_CNT++;
		}
		break;
	case GPIO_PIN_6:
		HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_1);
		if (bFlag)
		{
			B_PLS_CNT++;
		}
		break;
	case GPIO_PIN_7:
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_8);
		//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,SET);
		//bFlag = true;
		break;
	}
	//HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,RESET);
}

float diameter(float radius)
{
	return (2 * 3.1415 * radius);
}

float rotationForShoot(float targetDistance, float wheelDiameter)
{
	return (targetDistance / wheelDiameter);
}

int targetPulseCount(float rotationCount, int encoderPulseCnt)
{
	return (int) ((((rotationCount * encoderPulseCnt) / TARGET_PULSE_NUMBER)
			/ DIVISOR));
}

void SaveWheelParam(WheelParam *wP)
{
	HAL_FLASH_Unlock();
	{
		FLASH_EraseInitTypeDef fler;
		uint32_t perr;
		fler.TypeErase = FLASH_TYPEERASE_PAGES;
		fler.Banks = 1;
		fler.Page = 63;
		fler.NbPages = 1;
		HAL_FLASHEx_Erase(&fler, &perr);
		register uint64_t *_targetAddr = (uint64_t*) (wP);
		for (uint8_t i = 0; i <= (sizeof(WheelParam)*2); i += sizeof(uint64_t))
		{
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,
					BACKUP_FLASH_ADDR + i, _targetAddr[i/sizeof(uint64_t)]);
		}
	}
	HAL_FLASH_Lock();
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char buf[150];
	WheelParam wP;

	//At 1st, memcpy from backup addr

	memcpy(&wP, (void*) (BACKUP_FLASH_ADDR), sizeof(WheelParam));

	//2nd, compare memory
	/*if (wP.encoderPulseCount == 0xFFFFFFFF)
	{
		//if flash not initialized, set value to default
		wP.encoderPulseCount = 2000;
		wP.targetDistance = 5.0;
		wP.wheelRadius = 0.324;
		SaveWheelParam(&wP);
	}*/

	int encoderTargetCount = -1;
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
  MX_USB_Device_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	LL_TIM_EnableIT_UPDATE(TIM3);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_Delay(100);
	encoderTargetCount = targetPulseCount(
			rotationForShoot(wP.targetDistance, diameter(wP.wheelRadius)),
			wP.encoderPulseCount);
	xFlag = 0;
	while (1)
	{
		switch (xFlag)
		{
		case 1:	//set wheel param
			CDC_Transmit_FS("Input wheel radius.. > ", 23);
			while (!lineFlag)
				;
			lineFlag = 0;
			wP.wheelRadius = atof(rxbuf);
			encoderTargetCount = targetPulseCount(
					rotationForShoot(wP.targetDistance,
							diameter(wP.wheelRadius)), wP.encoderPulseCount);
			xFlag = 0;
			break;
		case 2:	//set encoder pulse count;
			CDC_Transmit_FS("Input encoder pulse cnt.. > ", 28);
			while (!lineFlag)
				;
			lineFlag = 0;
			wP.encoderPulseCount = atoi(rxbuf);
			if (wP.encoderPulseCount == 0)
			{
				break;
			}
			encoderTargetCount = targetPulseCount(
					rotationForShoot(wP.targetDistance,
							diameter(wP.wheelRadius)), wP.encoderPulseCount);
			xFlag = 0;
			break;
		case 3:	//set trigger distance
			CDC_Transmit_FS("Input trigger distance(m) > ", 28);
			while (!lineFlag)
				;
			lineFlag = 0;
			wP.targetDistance = atof(rxbuf);
			encoderTargetCount = targetPulseCount(
					rotationForShoot(wP.targetDistance,
							diameter(wP.wheelRadius)), wP.encoderPulseCount);
			xFlag = 0;
			break;
		case 4:
			sprintf(txbuf, "EncoderPulseCnt= %lu\r\n"
					"Wheel Radius= %0.5f\r\n"
					"Trigger Distance= %0.2f\r\n"
					"Target pulse count= %d\r\n"
					"Auto triggering= %s\r\n", wP.encoderPulseCount,
					wP.wheelRadius, wP.targetDistance, encoderTargetCount,
					(bFlag == true) ? ("Started") : ("Disabled"));
			CDC_Transmit_FS(txbuf, strlen(txbuf));
			xFlag = 0;
			break;
		case 5:
			CDC_Transmit_FS("Saving current param...\r\n",25);
			SaveWheelParam(&wP);
			xFlag=0;
			break;
		default:
			xFlag = 0;
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		if (bFlag && A_PLS_CNT >= encoderTargetCount)
		{
			A_PLS_CNT = 0;
			B_PLS_CNT = 0;
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, SET);
			LL_TIM_ClearFlag_UPDATE(TIM3);
			LL_TIM_EnableCounter(TIM3);
			char __buf=0xEE;
			CDC_Transmit_FS(&__buf, 1);
		}
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 42;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
