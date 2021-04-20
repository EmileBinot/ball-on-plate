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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <String.h>
#include <stdio.h>
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
TIM_HandleTypeDef htim4;

USART_HandleTypeDef husart2;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_Init(void);
/* USER CODE BEGIN PFP */

//______________________________________________________________
// Touchscreen functions prototype
void Touch_Init(void);
uint16_t median(int n, uint16_t x[]);
uint16_t read_touchX(void);
uint16_t read_touchY(void);
uint8_t read_touch_cal (double *Xpos, double *Ypos);

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
  MX_TIM4_Init();
  MX_USART2_Init();
  /* USER CODE BEGIN 2 */
	//setup touch
  Touch_Init();
	double X_touch, Y_touch, X_last, Y_last;
	char txdata[50];
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		read_touch_cal(&X_touch, &Y_touch);
	  X_last = X_touch;
	  Y_last = Y_touch;
		
		sprintf(txdata,"x=%f, y=%f\n\r",X_touch,Y_touch);
		
		
		HAL_USART_Transmit(&husart2,(uint8_t*)txdata,strlen(txdata),HAL_MAX_DELAY);
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 27;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 60000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  husart2.Instance = USART2;
  husart2.Init.BaudRate = 115200;
  husart2.Init.WordLength = USART_WORDLENGTH_8B;
  husart2.Init.StopBits = USART_STOPBITS_1;
  husart2.Init.Parity = USART_PARITY_NONE;
  husart2.Init.Mode = USART_MODE_TX_RX;
  husart2.Init.CLKPolarity = USART_POLARITY_LOW;
  husart2.Init.CLKPhase = USART_PHASE_1EDGE;
  husart2.Init.CLKLastBit = USART_LASTBIT_DISABLE;
  if (HAL_USART_Init(&husart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */
//______________________________________________________________
// Definitions of TOUCHSCREEN FUNCTIONS : 

//#### init ####
void Touch_Init(void){
	  //enable GPIOA clock
	  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	  //enable ADC1 clock
	  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	  //set X- and y- pull down to make ADC read 0 when not pressed
		//  GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD2 | GPIO_PUPDR_PUPD3)) | GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD3_1;
		GPIOA->PUPDR = (GPIOA->PUPDR & ~(GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7)) | GPIO_PUPDR_PUPD6_1 | GPIO_PUPDR_PUPD7_1;
	  //set ODR for X- and Y- pins to low so when set up output the will be low
		//  GPIOA->ODR &= ~(GPIO_ODR_OD2 | GPIO_ODR_OD3);
		GPIOA->ODR &= ~(GPIO_ODR_OD6 | GPIO_ODR_OD7);
	  //set ODR for X+ and Y+ pins to high so whenever set as output they will be high
	  GPIOA->ODR |= GPIO_ODR_OD0 | GPIO_ODR_OD1;
	  //set both pins as push pull
	  GPIOA->OTYPER &= ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1);

	  //turn ADC1 on and set EOCS to regular conversion
	  ADC1->CR2 |= ADC_CR2_ADON |  ADC_CR2_EOCS;
	  //set shortest sample time
	  ADC1->SMPR1 |= 0x11;
	  //set resution to 10 bit
	  ADC1->CR1 = (ADC1->CR1 & ~(ADC_CR1_RES)) | ADC_CR1_RES_0;
}

//#### calculate the median value of 5 ADC readings ####
uint16_t median(int n, uint16_t x[]) {
    int temp;
    int i, j;
    // the following two loops sort the array x in ascending order
    for(i=0; i<n-1; i++) {
        for(j=i+1; j<n; j++) {
            if(x[j] < x[i]) {
                // swap elements
                temp = x[i];
                x[i] = x[j];
                x[j] = temp;
            }
        }
    }
    return x[n/2];
}

//#### Read X voltage divider ####
uint16_t read_touchX(void){

	//reset all modes of lowest 2 pins
	GPIOA->MODER &= ~0xF;
	GPIOA->MODER &= ~0xF000;
	//set Y+ and Y- to output, set X+ to analog and X- to imput
	GPIOA->MODER |= GPIO_MODER_MODE1_0 | GPIO_MODER_MODE7_0 | GPIO_MODER_MODE0;

	//set to ADC to channel 0
	ADC1->SQR3 = 0;

	//we need short delay and this is currently way longer than needed.
	HAL_Delay(2);

	//we will get 5 conversions then use the median 1
	uint16_t adcs[5];
	for(uint8_t number = 0; number < 5; number++){
		//start ADC convertion
		ADC1->CR2 |= ADC_CR2_SWSTART;
		//wait for convertion to finish
		while ((ADC1->SR & ADC_SR_EOC) == 0){
		}
		adcs[number] = ADC1->DR;
	}

	uint16_t med = median(5, adcs)/*/(1024/Xresolution)*/;
	return med;
}

//#### Read Y voltage divider ####
uint16_t read_touchY(void){

	//reset all modes of every pins
	GPIOA->MODER &= ~0xF00F;
	//set X+ and X- to output, set Y+ to analog and Y- to input
	GPIOA->MODER |= GPIO_MODER_MODE0_0 | GPIO_MODER_MODE6_0 | GPIO_MODER_MODE1;

	//set to ADC to channel 1
	ADC1->SQR3 = 1;
	
	//we need short delay and this is currently way longer than needed.
	HAL_Delay(2);

	//we will get 5 conversions then use the median 1
	uint16_t adcs[5];
	for(uint8_t number = 0; number < 5; number++){
		//start ADC convertion
		ADC1->CR2 |= ADC_CR2_SWSTART;
		//wait for convertion to finish
		while ((ADC1->SR & ADC_SR_EOC) == 0){
		}
		adcs[number] = ADC1->DR;
	}
	//printf("%d\r\n", median(5, adcs));
	//cut off at 120
	uint16_t med = median(5, adcs)/*/(1024/Xresolution)*/;
	/*if (med < 120){
		return 5000;
	}*/

	//return (med * 0.31) - 41; 	//return result
	return med;
}

//#### Formatting read values ####
uint8_t read_touch_cal (double *Xpos, double *Ypos){
	double x,y;
	
	static double xmin = 90.0;
	static double xmax = 939.0;
	double xm = (xmax - xmin) / 2.0; // x - Center 
	static double xLength = 182.0; //Width of Touchscreen in mm at 8.0"

	static double ymin = 102.0;    //380.0  
	static double ymax = 904.0;    //900.0
	double ym = (ymax - ymin) / 2.0; // y - Center
	static double yLength = 140.0; //Length of Touchscreen in mm at 8.0"

	double convertX = xLength / (xmax - xmin);   // converts raw x values to mm. found through manual calibration
	double convertY = yLength / (ymax - ymin);   // converts raw y values to mm. found through manual calibration
	
	x=read_touchX();
	y=read_touchY();
	
	//we do so x and y are roughly within [-100;100] range 
	x = (x - xmin - xm) * convertX;
	y = (y - ymin - ym) * convertY;
	
	if(y<-75 && x<-102 ){
		x=0.0;
		y=0.0;
	}
	
	*Xpos = x;
	*Ypos = y;
	return 1;
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
