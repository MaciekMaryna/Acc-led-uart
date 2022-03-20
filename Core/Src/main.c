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
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
//#include "MY_LIS3DSH.h"
#include "LIS302DL.h"
#include "tools.h"

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
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
inline void showUartTilt(void);
inline void showLedTilt(void);
inline void showErrorRaport(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LIS302DL_DataScaled myData;
LIS302DL_DataRaw myDataRaw;
LIS302DL_InitTypeDef myAccel;
float shiftHyst1, shiftHyst2;

uint8_t spiBuf;

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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_USB_HOST_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  myAccel.dataRate=LIS302DL_DATARATE_400;
  myAccel.powerDown=LIS302DL_ACTIVE;
  myAccel.fullScale=LIS302DL_FULLSCALE_2;
  myAccel.enableAxes=LIS302DL_XYZ_ENABLE;
  myAccel.serialInterfaceMode=LIS302DL_SERIAL_INTERFACE_4WIRE;
  myAccel.rebootMemory=LIS302DL_BOOT_NORMAL_MODE;
  myAccel.filterConfig=LIS302DL_FILTERING_NONE;
  myAccel.interruptConfig=LIS302DL_INTERRUPT_NONE;
  LIS302DL_Init(&hspi1, &myAccel);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

	shiftHyst1 = 0.5f;
	shiftHyst2= 2 * shiftHyst1;

	//pre-calibration
	LIS302DL_X_calibrate(0.98, -1.00);
	LIS302DL_Y_calibrate(1.04, -1.04);
	LIS302DL_Z_calibrate(1.25, -0.87);

  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();

    /* USER CODE BEGIN 3 */
    	if (LIS302DL_PollDRDY(1))
    	{
			myData=LIS302DL_GetDataScaled();
			showUartTilt();
			showLedTilt();
    	}
    	else
    	{
    		showErrorRaport();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void showUartTilt(void)
{
	sprintf(uartBuffer,"x[g]=%+4.1f \ty[g]=%+4.1f \tz[g]=%+4.1f\n\r", myData.x, myData.y, myData.z);
	uartLog(uartBuffer);
}

void showLedTilt(void)
{
	if (myData.y > 0)
	{
		if (myData.y > shiftHyst2)
		{
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, SET);
		}
		if (myData.y < shiftHyst1)
		{
			HAL_GPIO_WritePin(LD4_GPIO_Port, LD4_Pin, RESET);
		}
	}

	if (myData.y < 0)
	{
		if (myData.y < -shiftHyst2)
		{
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, SET);
		}
		if (myData.y > -shiftHyst1)
		{
			HAL_GPIO_WritePin(LD5_GPIO_Port, LD5_Pin, RESET);
		}
	}

	if (myData.x > 0)
	{
		if (myData.x > shiftHyst2)
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, SET);
		}
		if (myData.x < shiftHyst1)
		{
			HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, RESET);
		}
	}

	if (myData.x < 0)
	{
		if (myData.x < -shiftHyst2)
		{
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, SET);
		}
		if (myData.x > -shiftHyst1)
		{
			HAL_GPIO_WritePin(LD6_GPIO_Port, LD6_Pin, RESET);
		}
	}
}
void showErrorRaport(void)
{
	//no data message
	sprintf(uartBuffer,"Timeout. No data from g-sensor.\n\r");
	uartLog(uartBuffer);
	//push status registers do UART
	LIS302DL_ReadIO(LIS302DL_STATUS_ADDR, &spiBuf, 1);
	sprintf(uartBuffer,"STAUS_REG: 0x%0X\n\r", spiBuf);
	uartLog(uartBuffer);
	for (int i = 0; i < 3; i++)
	{
		LIS302DL_ReadIO(LIS302DL_CTRL_REG1_ADDR+i, &spiBuf, 1);
		sprintf(uartBuffer,"CTRL_REG%d: 0x%0X\n\r", i, spiBuf);
		uartLog(uartBuffer);
	}
	HAL_Delay(1000);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
