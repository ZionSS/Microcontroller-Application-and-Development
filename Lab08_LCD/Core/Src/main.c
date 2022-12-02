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
#include "adc.h"
#include "i2c.h"
#include "rng.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"
#include "string.h"
#include "am2320.h"
#include "picture.h"
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
/* Private variables ---------------------------------------------------------*/
float h=30.0, t=40.0;
float dutyCycle = 0.0;
uint8_t step = 0;
HAL_StatusTypeDef status;
uint16_t color = 0;
uint16_t r_density = 0;
uint16_t g_density = 0;
uint16_t b_density = 0;
uint32_t count;
uint32_t lasted_count;
uint8_t state = 0;
volatile uint32_t adc_val = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
uint16_t CRC16_2(uint8_t *, uint8_t);
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
	char str[50];
	char str1[50];
	char r[50];
	char g[50];
	char b[50];
	//uint8_t cmdBuffer[3];
	//uint8_t dataBuffer[8];
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_USART3_UART_Init();
  MX_SPI5_Init();
  MX_TIM1_Init();
  MX_RNG_Init();
  MX_I2C1_Init();
  MX_TIM8_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  ILI9341_Init();//initial driver setup to drive ili9341
  //sprintf(str, "\n\rAM2320 I2C DEMO Starting . . . \n\r");
  Am2320_HandleTypeDef Am2320_;
  Am2320_ = am2320_Init(&hi2c1, AM2320_ADDRESS);
  HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str), 200);
  float temperature, humidity;
  uint8_t rx = 0;
  uint8_t gx = 0;
  uint8_t bx = 0;
  //cmdBuffer[0] = 0x03;
  //cmdBuffer[1] = 0x00;
  //cmdBuffer[2] = 0x04;
  char fname[32] = "Phacharaphol";
  char lname[32] = "Chokkhun";
  char std_id[32] = "63010631";
  char group_no[32] = "Group No.1";
  ILI9341_Fill_Screen(WHITE);
  ILI9341_Set_Rotation(SCREEN_HORIZONTAL_1);
  HAL_TIM_Base_Start_IT(&htim8);
  HAL_ADC_Start(&hadc1);
  htim2.Instance -> CCR4 = (10000-1) * dutyCycle;
   HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(state == 0)
	  {
	  	  sprintf(r,"%3d %%",(int)rx*100/120);
	  	  sprintf(g,"%3d %%",(int)gx*100/120);
	  	  sprintf(b,"%3d %%",(int)bx*100/120);


	  	  color = (r_density << 11) + (g_density << 5) + b_density;
	  	  ILI9341_Draw_Filled_Circle(125, 30, 25, color);

	  	  ILI9341_Draw_Filled_Circle(25, 85, 20, 0xF800);
	  	  ILI9341_Draw_Filled_Rectangle_Coord(70, 70, 190, 100, 0xFDB6);
	  	  ILI9341_Draw_Filled_Rectangle_Coord(70, 70, 70+rx, 100, 0xF800);
	  	  ILI9341_Draw_Text(r, 200, 70, BLACK, 2, WHITE);

	  	  ILI9341_Draw_Filled_Circle(25, 145, 20, 0x07E0);
	  	  ILI9341_Draw_Filled_Rectangle_Coord(70, 130, 190, 160, 0xCFFA);
	  	  ILI9341_Draw_Filled_Rectangle_Coord(70, 130, 70+gx, 160, 0x07E0);
	  	  ILI9341_Draw_Text(g, 200, 130, BLACK, 2, WHITE);

	  	  ILI9341_Draw_Filled_Circle(25, 205, 20, 0x001F);
	  	  ILI9341_Draw_Filled_Rectangle_Coord(70, 190, 190, 220, 0xCE5F);
	  	  ILI9341_Draw_Filled_Rectangle_Coord(70, 190, 70+bx, 220, 0x001F);
	  	  ILI9341_Draw_Text(b, 200, 190, BLACK, 2, WHITE);

	  	  am2320_GetTemperatureAndHumidity(&Am2320_, &temperature, &humidity);
	  	  sprintf(str, "%4.1f C",temperature);
	  	  sprintf(str1, "%4.1f %%RH" ,humidity);
	  	  ILI9341_Draw_Text(str, 20, 20, BLACK, 2, WHITE);
	  	  ILI9341_Draw_Text(str1, 160, 20, BLACK, 2, WHITE);


	  	  if(TP_Touchpad_Pressed())
	  	  {
	  		  uint16_t x_pos = 0;
	  		  uint16_t y_pos = 0;
	  		  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_SET);
	  		  uint16_t position_array[2];
	  		  if(TP_Read_Coordinates(position_array) == TOUCHPAD_DATA_OK)
	  		  {
	  		  	  	y_pos = 240 - position_array[0];
	  		  	  	x_pos = position_array[1];
	  		  	  	if((x_pos >= 5 && x_pos <= 45) && (y_pos >= 65 && y_pos <= 105))
	  		  	  	{
	  		  	  		//ILI9341_Draw_Hollow_Rectangle_Coord(5, 65, 45, 105, 0x001F);
	  		  	  		rx += 12;
	  		  	  		r_density += (int)31*10/100;
	  		  	  		if(rx > 120)
	  		  	  		{
	  		  	  			r_density = 0;
	  		  	  			rx = 0;
	  		  	  		}
	  		  	  	}
	  		  	  	if((x_pos >= 5 && x_pos <= 45) && (y_pos >= 125 && y_pos <= 165))
	  		  	  	{
	  		  	  		//ILI9341_Draw_Hollow_Rectangle_Coord(5, 125, 45, 165, 0x001F);
	  		  	  		gx += 12;
	  		  	  		g_density += (int)63*10/100;
	  		  	  		if(gx > 120)
	  		  	  		{
	  		  	  			g_density = 0;
	  		  	  			gx = 0;
	  		  	  		}
	  		  	  	}
	  		  	  	if((x_pos >= 5 && x_pos <= 45) && (y_pos >= 185 && y_pos <= 225))
	  		  	  	{
	  		  	  		//ILI9341_Draw_Hollow_Rectangle_Coord(5, 185, 45, 225, 0x00FF);
	  		  			bx += 12;
	  		  			b_density += (int)31*10/100;
	  		  			if(bx > 120)
	  		  			{
	  		  				b_density = 0;
	  		  				bx = 0;
	  		  			}
	  		  		}
	  		  	  	if((x_pos >= 100 && x_pos <= 150) && (y_pos >= 5 && y_pos <= 55))
	  		  	  	{
	  		  	  		//ILI9341_Draw_Hollow_Rectangle_Coord(100, 5, 150, 55, 0x00FF);
	  		  	  		HAL_Delay(200);
	  		  	  		ILI9341_Fill_Screen(WHITE);
	  		  	  		state = 1;
	  		  	  		count = 0;
	  		  	  	}
	  		  }
	  		  else
	  		  {
	  		  	  				HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);
	  		  }
	  	  }

	  	}
	  	else
	  	{
	  		ILI9341_Draw_Image((const char*)my_picture , SCREEN_HORIZONTAL_1);
	  		ILI9341_Draw_Text(group_no, 120, 60, BLACK, 2, WHITE);
	  		ILI9341_Draw_Text(fname, 120, 85, BLACK, 2, WHITE);
	  		ILI9341_Draw_Text(lname, 120, 110, BLACK, 2, WHITE);
	  		ILI9341_Draw_Text(std_id, 120, 135, BLACK, 2, WHITE);
	  		if (count >= 5 )
	  		{
	  			ILI9341_Fill_Screen(WHITE);
	  			state = 0;
	  		}
	  		else if(TP_Touchpad_Pressed() )
	  		{
	  			//HAL_Delay(1000);
	  			ILI9341_Fill_Screen(WHITE);
	  			state = 0;
	  		}
	  	}
	  	  while (HAL_ADC_PollForConversion(&hadc1, 100) != HAL_OK){}
	  	  	  adc_val = HAL_ADC_GetValue(&hadc1);
	  	  	  dutyCycle = 0.2 + (adc_val/4095.0 * 0.8);
	  	  htim2.Instance -> CCR4 = (10000-1) * dutyCycle;
	  	  /*while(__HAL_UART_GET_FLAG(&huart3,UART_FLAG_TC)== RESET){}
	  	  HAL_UART_Transmit(&huart3, (uint8_t*) str, strlen(str), 200);

	  	  HAL_Delay(5000);*/
	  	  /*HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);

	  	  HAL_I2C_Master_Transmit(&hi2c1, 0x5c << 1, cmdBuffer, 3, 200);
	  	  HAL_I2C_Master_Transmit(&hi2c1, 0x5c << 1, cmdBuffer, 3, 200);
	  	  HAL_Delay(1);

	  	  HAL_I2C_Master_Receive(&hi2c1, 0x5c<<1, dataBuffer, 8, 200);

	  	  uint16_t Rcrc = dataBuffer[7] << 8;
	  	  Rcrc += dataBuffer[6];
	  	  if (Rcrc == CRC16_2(dataBuffer, 6)){
	  		  uint16_t temperature = ((dataBuffer[4] & 0x7F) << 8) + dataBuffer[5];
	  		  t = temperature /10.0;
	  		  t = (((dataBuffer[4] & 0x80) >> 7)==1) ? (t * (-1)) : t ;
	  		  uint16_t humidity = (dataBuffer[2] << 8) + dataBuffer[3];
	  		  h = humidity /10.0;
	  	  }
	  	*/
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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 9;
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
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint16_t CRC16_2(uint8_t *ptr, uint8_t length)
{
	uint16_t crc = 0xFFFF;
	uint8_t s = 0x00;

	while(length--)
	{
		crc ^= *ptr++;
		for(s = 0; s < 8; s++){
			if ((crc & 0x01) != 0){
				crc >>= 1;
				crc ^= 0xA001;
			}else crc >>= 1 ;
		}
	}
	return crc;
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
  while(1)
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
