/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_gyroscope.h"
#include "MyLib.h"
#include "math.h"
#include "usbd_cdc_if.h"
#include "string.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

uint8_t cnt = 0,a_cnt = 0;
float pos[3],oldpos[3] = {0};
float angle[3];
uint8_t x_direction,y_direction;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi5;

/* USER CODE BEGIN PV */
char bufpos[5];
Point arrow_up[] = {{120,10},{100,25},{140,25}};
Point arrow_left[] = {{10,160},{25,140},{25,180}};
Point arrow_down[] = {{100,295},{120,310},{140,295}};
Point arrow_right[] = {{230,160},{215,180},{215,140}};
uint8_t NoiseFilter(float* oldvalue,float* newvalue,int offset){
	uint8_t res = 0;
	for(uint8_t i=0;i<3;i++){
		if(abs(newvalue[i] - oldvalue[i]) > offset) res = 1;
	}
	return res;
}

void ConcatenateStr(const char* firstStr, char* secondStr, char* res){
	uint8_t frlen = strlen(firstStr);
	uint8_t total_len = frlen + strlen(secondStr);
	for(uint8_t i=0;i<total_len;i++){
		if(i<frlen) res[i] = firstStr[i];
		else res[i] = secondStr[i-frlen];
	}
	res[total_len] = '\n';
	res[total_len+1] = '\0';
}

void ResetStr(char* str){
	uint8_t len = strlen(str);
	for(uint8_t i=0;i<len;i++){
		str[i] = '\0';
	}
}

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI5_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	if (BSP_GYRO_Init() != GYRO_OK){
		BSP_LCD_DisplayStringAtLine(2,(uint8_t*)"CAN'T FIND GYRO");
		while(1);
	}
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
  MX_USB_DEVICE_Init();
  MX_SPI5_Init();
  /* USER CODE BEGIN 2 */
  	BSP_LCD_Init();//init LCD
  		//set the layer buffer address into SDRAM
  	BSP_LCD_LayerDefaultInit(1, SDRAM_DEVICE_ADDR);
  	BSP_LCD_SelectLayer(1);//select on which layer we write
  	BSP_LCD_DisplayOn();//turn on LCD
  	BSP_LCD_Clear(LCD_COLOR_WHITE);//clear the LCD on blue color
  	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);//set text background color
  	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);//set text color
  		//write text
  		//BSP_LCD_FillPolygon(Points, 3);
  	BSP_LCD_DisplayStringAtLine(2,"x= 0");
  	BSP_LCD_DisplayStringAtLine(3,"y= 0");
  	BSP_LCD_DisplayStringAtLine(4,"z= 0");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  BSP_GYRO_GetXYZ(pos);
	 	  	for(uint8_t i = 0;i<3;i++){
	 	  		// tich phan van toc tren mien t: S = v.dt (S la goc nghieng, v la  van toc goc)
	 	  		// lenh dieu kien de chan cac gia tri gay nhieu, co the dan den go'c nga`y cang lon'
	 	  		if(abs(pos[i])>500) angle[i]+= pos[i]*0.005/1000;
	 	  		pos[i] /= 1000; 	// chuyen tu mdps qua dps
	 	  	}

	 	  	/* 	Cu moi 5*100 ms thuc hien kiem tra van toc goc 1 lan,
	 	  	/*	neu su thay doi > 1 dps thi cap nhat vao trong chuoi~ bufpos */
	 	  	if(cnt > 100){
	 	  		cnt = 0;
	 	  		if(NoiseFilter(oldpos, pos, 1)){
	 	  			char res[10];
	 	  			ResetStr(bufpos);
	 	  			ResetStr(res);
	 	  			ftoa(pos[0], bufpos, 1);
	 	  			//BSP_LCD_DisplayStringAtLine(2, bufpos);
	 	  			ConcatenateStr("x = ",bufpos, res);
	 	  			CDC_Transmit_HS(res,strlen(res));

	 	  			ResetStr(bufpos);
	 	  			ResetStr(res);
	 	  			ftoa(pos[1], bufpos, 1);
	 	  			//BSP_LCD_DisplayStringAtLine(3, bufpos);
	 	  			ConcatenateStr("y = ",bufpos, res);
	 	  			CDC_Transmit_HS(res,strlen(res));

	 	  			ResetStr(bufpos);
	 	  			ResetStr(res);
	 	  			ftoa(pos[2], bufpos, 1);
	 	  			//BSP_LCD_DisplayStringAtLine(4, bufpos);
	 	  			ConcatenateStr("z = ",bufpos, res);
	 	  			CDC_Transmit_HS(res,strlen(res));

	 	  			oldpos[0] = pos[0];
	 	  			oldpos[1] = pos[1];
	 	  			oldpos[2] = pos[2];
	 	  		}
	 	  	}

	 	  	/* moi~ (20*5 + tg delay giao tiep USB ) ms kiem tra su thay doi goc xoay 1 lan*/
	 	  	/* Neu sau (20*5 + tg delay giao tiep USB ), su thay doi goc > 3 do => gyro dang xoay, kiem tra trang thai xoay => dua ra ket qua */
	 	  	if(a_cnt > 20){
	 	  		a_cnt = 0;
	 	  			/* Su dung nhi phan {x_direction,y_direction} de to hop 4 trang thai*/
	 	  		if(angle[0] > 0) x_direction = 1;
	 	  		else x_direction = 0;
	 	  		if(angle[1] > 0) y_direction = 1;
	 	  		else y_direction = 0;
	 	  		switch(x_direction*2+y_direction){
	 	  			case 0:
	 	  				BSP_LCD_Clear(LCD_COLOR_WHITE);
	 	  				BSP_LCD_FillPolygon(arrow_up, 3);
	 	  				BSP_LCD_FillPolygon(arrow_left, 3);
	 	  				break;
	 	  			case 1:
	 	  				BSP_LCD_Clear(LCD_COLOR_WHITE);
	 	  				BSP_LCD_FillPolygon(arrow_up, 3);
	 	  				BSP_LCD_FillPolygon(arrow_right, 3);
	 	  				break;
	 	  			case 2:
	 	  				BSP_LCD_Clear(LCD_COLOR_WHITE);
	 	  				BSP_LCD_FillPolygon(arrow_down, 3);
	 	  				BSP_LCD_FillPolygon(arrow_left, 3);
	 	  				break;
	 	  			case 3:
	 	  				BSP_LCD_Clear(LCD_COLOR_WHITE);
	 	  				BSP_LCD_FillPolygon(arrow_down, 3);
	 	  				BSP_LCD_FillPolygon(arrow_right, 3);
	 	  				break;
	 	  			default:
	 	  				break;
	 	  		}
	 	  	}
	 	  	cnt++;
	 	  	a_cnt++;
	 HAL_Delay(5);
    /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
  }
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

/**
  * @brief SPI5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI5_Init(void)
{

  /* USER CODE BEGIN SPI5_Init 0 */

  /* USER CODE END SPI5_Init 0 */

  /* USER CODE BEGIN SPI5_Init 1 */

  /* USER CODE END SPI5_Init 1 */
  /* SPI5 parameter configuration*/
  hspi5.Instance = SPI5;
  hspi5.Init.Mode = SPI_MODE_MASTER;
  hspi5.Init.Direction = SPI_DIRECTION_2LINES;
  hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi5.Init.NSS = SPI_NSS_SOFT;
  hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi5.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI5_Init 2 */

  /* USER CODE END SPI5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

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
