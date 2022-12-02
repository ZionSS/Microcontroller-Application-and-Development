/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "ILI9341_Touchscreen.h"

#include "ILI9341_STM32_Driver.h"
#include "ILI9341_GFX.h"

#include "snow_tiger.h"

#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DIR_PIN GPIO_PIN_8
#define DIR_PORT GPIOC
#define STEP_PIN GPIO_PIN_9
#define STEP_PORT GPIOC
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int isAuto = 0;

int RSteps = 25;
int LSteps = 25;
int grabSteps = 40;
int isGrip = 0;
int gripCanToggle = 0;
int stateCanToggle = 0;
int stepMotorDelay = 200;
int stepMotorSteps = 200;

uint32_t counter;
uint32_t sec;

uint32_t tick = 0;
uint32_t previousTick = 0;

int modePressed = 0;
int gripPressed = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void microDelay(uint16_t delay);
void stepMotorToOrigin();
void servoMotorToOrigin();
void automation();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
char str[50];
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
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_SPI5_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, grabSteps);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, RSteps);
  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, LSteps);

  HAL_TIM_Base_Stop(&htim4);

  HAL_TIM_Base_Start(&htim2);
  ILI9341_Set_Rotation(SCREEN_VERTICAL_1);
  ILI9341_Fill_Screen(WHITE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if(isAuto == 1)
	  {
		  sprintf(str,"AUTOMATIC...");
		  ILI9341_Draw_Text(str, 10, 10, BLACK, 2, WHITE);
//		  servoMotorToOrigin();
//		  if(stepMotorSteps > 200){
//			  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN,GPIO_PIN_RESET);
//			  for(int i = 0; i< abs(stepMotorSteps - 200);i++){
//				  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
//				  HAL_Delay(10);
//				  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
//				  HAL_Delay(10);
//			  }
//		  }
//		  else if(stepMotorSteps < 200){
//		  			  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN,GPIO_PIN_SET);
//		  			  for(int i = 0; i< abs(stepMotorSteps - 200);i++){
//		  				  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
//		  				  HAL_Delay(10);
//		  				  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
//		  				  HAL_Delay(10);
//		  			  }
//		  		  }
//		  stepMotorSteps = 200;
//		  HAL_Delay(100);

		  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN,GPIO_PIN_SET);
		  for(int i = 0; i <51;i = i+1){
			  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			  HAL_Delay(10);
			  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			  HAL_Delay(10);
		  }
		  HAL_Delay(500);

		  while( LSteps < 50){
			  LSteps++;
			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, LSteps);
			  HAL_TIM_Base_Stop(&htim4);
			  HAL_Delay(10);
		  }
		  HAL_Delay(800);

		  // Gripper OPEN
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 40);
		  HAL_TIM_Base_Stop(&htim4);
		  HAL_Delay(10);
		  HAL_Delay(800);

		  while( RSteps < 35) {
			  RSteps++;
			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, RSteps);
			  HAL_TIM_Base_Stop(&htim4);
			  HAL_Delay(10);
		  }
		  HAL_Delay(800);

		  // Gripper CLOSE
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 25);
		  HAL_TIM_Base_Stop(&htim4);
		  HAL_Delay(10);
		  HAL_Delay(800);

		  while( RSteps > 25) {
			  RSteps--;
			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, RSteps);
			  HAL_TIM_Base_Stop(&htim4);
			  HAL_Delay(10);
		  }
		  HAL_Delay(800);

		  while( LSteps > 25){
			  LSteps--;
			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, LSteps);
			  HAL_TIM_Base_Stop(&htim4);
			  HAL_Delay(10);
		  }
		  HAL_Delay(800);

		  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN,GPIO_PIN_RESET);
		  for(int i = 0; i <51;i = i+1){
			  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			  HAL_Delay(10);
			  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			  HAL_Delay(10);
		  }
		  HAL_Delay(500);

		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 40);
		  HAL_TIM_Base_Stop(&htim4);
		  HAL_Delay(10);
		  HAL_Delay(800);

		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 25);
		  HAL_TIM_Base_Stop(&htim4);
		  HAL_Delay(10);
		  HAL_Delay(800);

//		  for(int x = RSteps;x < 35;x++){
//			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, x);
//			  HAL_TIM_Base_Stop(&htim4);
//			  HAL_Delay(5);
//		  }
//		  RSteps = 35;
//		  HAL_Delay(500);

//		  for(int x = RSteps;x > 25;x++){
//			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
//			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, x);
//			  HAL_TIM_Base_Stop(&htim4);
//			  HAL_Delay(5);
//		  }
//
//		  RSteps = 25;
//		  HAL_Delay(500);
//
//		  for(int x = LSteps;x < 40;x++){
//			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, x);
//			  HAL_TIM_Base_Stop(&htim4);
//			  HAL_Delay(5);
//		  }
//		  LSteps = 40;
//		  HAL_Delay(500);
//
//		  for(int x = LSteps;x < 25;x++){
//			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
//			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, x);
//			  HAL_TIM_Base_Stop(&htim4);
//			  HAL_Delay(5);
//		  }
//		  LSteps = 25;
//		  HAL_Delay(500);
//
//
//
//		  HAL_Delay(1000);
	  }
	  if(isAuto == 0)
	  {
		  sprintf(str,"MANUAL...");
		  ILI9341_Draw_Text(str, 10, 10, BLACK, 2, WHITE);
		  // SERVO MOTOR RIGHT
		  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6) == GPIO_PIN_RESET && RSteps < 35)
		  {
			  RSteps++;
			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, RSteps);
			  HAL_TIM_Base_Stop(&htim4);

			  //HAL_UART_Transmit(&huart3, "test", 10, 100);
		  }
		  else if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_RESET && RSteps > 25)
		  {
			  RSteps--;
			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, RSteps);
			  HAL_TIM_Base_Stop(&htim4);

			  //HAL_UART_Transmit(&huart3, "stes", 10, 100);

		  // SERVO MOTOR LEFT
		  }
		  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_12) == GPIO_PIN_RESET && LSteps < 50)
		  {
			  LSteps++;
			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, LSteps);
			  HAL_TIM_Base_Stop(&htim4);
		  }
		  else if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_RESET && LSteps > 25)
		  {
			  LSteps--;
			  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
			  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, LSteps);
			  HAL_TIM_Base_Stop(&htim4);
		  }

		  // STEPPER MOTOR
		  if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_1) == GPIO_PIN_RESET && stepMotorSteps < 400)
		  {
			  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
			  stepMotorSteps++;
			  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			  microDelay(stepMotorDelay);
			  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			  microDelay(stepMotorDelay);
		  }
		  else if(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_1) == GPIO_PIN_RESET && stepMotorSteps > 0)
		  {
			  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
			  stepMotorSteps--;
			  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			  microDelay(stepMotorDelay);
			  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			  microDelay(stepMotorDelay);
		  }
		  HAL_Delay(20);
		  HAL_TIM_Base_Stop(&htim4);

		//	  Stepper Motor Rounding!!!!!
		//	  int stepper;
		//	  HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void microDelay(uint16_t delay)
{
	__HAL_TIM_SET_COUNTER(&htim3, 0);
	while(__HAL_TIM_GET_COUNTER(&htim3) < delay);
}

void stepMotorToOrigin()
{
	while(stepMotorSteps != 100)
	{
		if(stepMotorSteps > 100)
		{
			HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
			stepMotorSteps--;
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			microDelay(stepMotorDelay);
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			microDelay(stepMotorDelay);
		}
		else if(stepMotorSteps < 100)
		{
			HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
			stepMotorSteps++;
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
			microDelay(stepMotorDelay);
			HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
			microDelay(stepMotorDelay);
		}
	}
}

void servoMotorToOrigin()
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);

	LSteps = 25;
	RSteps = 25;
	grabSteps = 40;

	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, grabSteps);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, LSteps);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, RSteps);

	HAL_TIM_Base_Stop(&htim4);
}

void automation()
{

	servoMotorToOrigin();
    HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_SET);
    for(int steps = 100; steps < 200; steps++)
    {
      HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
      microDelay(stepMotorDelay);
      HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
      microDelay(stepMotorDelay);
	}
    HAL_Delay(500);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 50);
    HAL_TIM_Base_Stop(&htim4);
    HAL_Delay(500);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
    __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 35);
    HAL_TIM_Base_Stop(&htim4);
    HAL_Delay(500);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
    __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, 25);
    //HAL_TIM_Base_Stop(&htim4);
    HAL_Delay(500);

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 25);
	HAL_TIM_Base_Stop(&htim4);
	HAL_Delay(500);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 25);
	HAL_TIM_Base_Stop(&htim4);
	HAL_Delay(500);

	HAL_GPIO_WritePin(DIR_PORT, DIR_PIN, GPIO_PIN_RESET);
	for(int steps = 100; steps > 0; steps--)
	{
	  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_SET);
	  microDelay(stepMotorDelay);
	  HAL_GPIO_WritePin(STEP_PORT, STEP_PIN, GPIO_PIN_RESET);
	  microDelay(stepMotorDelay);
	}
	HAL_Delay(500);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_2, 35);
	HAL_TIM_Base_Stop(&htim4);
	HAL_Delay(500);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, 50);
	HAL_TIM_Base_Stop(&htim4);
	HAL_Delay(500);

	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, 40);
	HAL_TIM_Base_Stop(&htim4);
	HAL_Delay(500);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	HAL_Delay(50);
//	if (GPIO_Pin == GPIO_PIN_0){
//		for(int grabSteps = 25; grabSteps < 50; grabSteps++)
//		{
//			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, grabSteps);
//			HAL_Delay(5);
//		}
//		HAL_Delay(500);
//		for(int grabSteps = 50; grabSteps > 25; grabSteps--)
//		{
//			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
//			__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, grabSteps);
//			HAL_Delay(5);
//		}
//	}

	if (GPIO_Pin == GPIO_PIN_0)
	{
	  if(isGrip == 0){
		  isGrip = 1;
	  }
	  else
	  {
		  isGrip = 0;
	  }

	  if(isGrip == 0){
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, 25);
		  //isGrip = 1;
		  HAL_Delay(5);
  //		  {
  //		  for(grabSteps = 25; grabSteps < 50; grabSteps++)
  //		  {
  //			HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  //			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, grabSteps);
  //			HAL_Delay(5);
  //		  }
  //		  for(grabSteps = 50; grabSteps > 25; grabSteps--)
  //		  {
  //			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_1, grabSteps);
  //			HAL_Delay(5);
  //		  }
	  }
	  else if(isGrip == 1){
		  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		  __HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_1, 40);
		  //isGrip = 0;
		  HAL_Delay(5);

	  }
	  HAL_Delay(500);

	}

	if(GPIO_Pin == GPIO_PIN_3)
	{
		if(isAuto == 0)
		{
			isAuto = 1;
			stepMotorSteps = 200;
			servoMotorToOrigin();

		}
		else
		{
			isAuto = 0;


		}
		HAL_Delay(200);
	}
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
