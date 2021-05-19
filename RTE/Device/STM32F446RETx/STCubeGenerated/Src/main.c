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
#include <math.h>
#include "../PID_Controller-master/C/pid_controller.h"
#include "../PID_Controller-master/C/pid_controller.c"
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
TIM_HandleTypeDef htim6;

USART_HandleTypeDef husart2;

/* USER CODE BEGIN PV */
//______________________________________________________________
// Motors
int flat_X =4500;
int flat_Y =5000;


//______________________________________________________________
// Touchscreen variables
float X_touch, Y_touch, X_touch_last, Y_touch_last;
float unfilteredLP;
float unfilteredABG;

int statex=0;
int statey=0;

//______________________________________________________________
// UART
char txdata[100];


//______________________________________________________________
// PID
float SampleTime=0.005;// Will also change the timer IT Arr

float Kpx = 5;  //Kpx = 3;	//Proportional (P)                                                   
float Kix = 4; //Kix = 2	//Integral (I)                                                     
float Kdx = 1.9;  //Kdx = 1.1;	//Derivative (D)

float Kpy = 4;  //Kpy = 3;                                                       
float Kiy = 3;  //Kiy = 2;                                                      
float Kdy = 1.5;  //Kdy = 1.1;

PIDControl PIDx;
PIDControl PIDy;

bool PidFlag=false;
int PIDcount=0;

//______________________________________________________________
// FILTERS
float a=0.5;
float b=0.03;
float g=1;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_Init(void);
static void MX_TIM6_Init(void);


/* USER CODE BEGIN PFP */

//______________________________________________________________
// Touchscreen functions prototype
void Touch_Init(void);
uint16_t median(int n, uint16_t x[]);
uint16_t read_touchX(void);
uint16_t read_touchY(void);
uint8_t read_touch_cal (float *Xpos, float *Ypos);

float ADCfilterX(float current,float last);
float ADCfilterY(float current,float last);

float LPfilterX(float current);
float LPfilterY(float current);

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
	
	int stablecount=0;	//stablecount will increment everytime ball is in center of the plate
	int stable=0;				//stable is either 1 or 0, means either stable or not
	
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	
	// Touchscreen init
  Touch_Init();
	
	// PID init
	PIDInit(&PIDx,Kpx,Kix,Kdx,SampleTime*8,-1000,+1000,AUTOMATIC,DIRECT);
	PIDInit(&PIDy,Kpy,Kiy,Kdy,SampleTime*8,-1000,+1000,AUTOMATIC,REVERSE);
	PIDSetpointSet(&PIDx,0.0);
	PIDSetpointSet(&PIDy,0.0);

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART2_Init();
  MX_TIM6_Init();
	
  /* USER CODE BEGIN 2 */

	// Starting motor's PWMs
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_2);
	
	TIM4->CCR1=flat_X; 	// Make Plate flat in X-Direction
	TIM4->CCR2=flat_Y; 	// Make Plate flat in Y-Direction
	HAL_Delay(500);
	TIM4->CCR1=flat_X-150;
	TIM4->CCR2=flat_Y+150;
	HAL_Delay(500);
	
	// Start Compute/Position update IT
	HAL_TIM_Base_Start_IT(&htim6);// Starting motor position update timer IT
	
  /* USER CODE END 2 */

  /* Infinite loop   */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		if(PidFlag==true){		//PidFlag is updated in HAL_TIM_PeriodElapsedCallback() interruption
			read_touch_cal(&X_touch, &Y_touch);
			PIDcount++;					//Increment PIDcount counter 
			
			X_touch_last=X_touch;
			Y_touch_last=Y_touch;
			
			// When PID count > 8 = 8x5ms = 40ms && ball is on plate :
			if((PIDcount>8) &&((statex==1) || (statey==1))){
				
				PIDcount=0;				// Reset PIDcount counter
				
				//Fill up the buffer we're gonna send to PC, then send it via USART2
				sprintf(txdata,"%f, %f, %f\r\n",X_touch,unfilteredLP,unfilteredABG);
				HAL_USART_Transmit(&husart2,(uint8_t*)txdata,strlen(txdata),HAL_MAX_DELAY);
				
				if(stable==0){		// If ball is not stabilised
					PidFlag=false;	// Reset PIDFlag
					PIDcount=0;			// Reset PIDcount
					
					PIDInputSet(&PIDx,X_touch);			// Update PIDx->input
					PIDInputSet(&PIDy,Y_touch);			// Update PIDy->input
				
					PIDCompute(&PIDx);			// Computes P,I,D values then update PIDx->output
					PIDCompute(&PIDy);			// Computes P,I,D values then update PIDy->output
					
					TIM4->CCR1=flat_X-150+(int)PIDOutputGet(&PIDx);		//Update X motor position
					TIM4->CCR2=flat_Y+150+(int)PIDOutputGet(&PIDy);		//Update Y motor position
				}
				if(stable==1){					// If ball is stabilised on setpoint
					TIM4->CCR1=flat_X;		// Make Plate flat in X-Direction
					TIM4->CCR2=flat_Y;		// Make Plate flat in Y-Direction
				}
				
				// if ball is not stabilised:
				if(stable==0){
					// if ball is in the center of the plate
					if((X_touch<20 && X_touch>-20)&&(Y_touch<20 && Y_touch>-20)){
						stablecount++;			// Increment stablecount
						if(stablecount>20){	// If ball has been in the center of the board enough time
							stable=1;					// Stable flag switch to 1 (stable)
						}
					}
				}
				// if ball is stabilised:
				if(stable==1){
					// if ball goes out of the center of the plate
					if((X_touch>15 || X_touch<-15)||(Y_touch>15 || Y_touch<-15)){
						stablecount=0; 			// Stablecount reset
						stable=0;						// Stable flag switch to 0 (unstable)
					}
				}

			}
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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  sConfigOC.Pulse = 30000;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 10000-1;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = (SampleTime*8400)-1;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

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
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BUILT_IN_LED_GPIO_Port, BUILT_IN_LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : BUILT_IN_LED_Pin */
  GPIO_InitStruct.Pin = BUILT_IN_LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BUILT_IN_LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//______________________________________________________________
// Timers IT Callback : 
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	
	if (htim == &htim6){	//timer 6 full triggers IT then change motor position
			PidFlag=true;	
	}
}



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
	HAL_Delay(4);

	//we will get 5 conversions then use the median 1
	uint16_t adcs[5];
	for(uint16_t number = 0; number < 5; number++){
		//start ADC convertion
		ADC1->CR2 |= ADC_CR2_SWSTART;
		//wait for convertion to finish
		while ((ADC1->SR & ADC_SR_EOC) == 0){
		}
		adcs[number] = ADC1->DR;
	}

	uint16_t med = median(5, adcs);

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
	HAL_Delay(4);

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

	uint16_t med = median(5, adcs);

	return med;
}
//#### Formatting read values ####
uint8_t read_touch_cal (float *Xpos, float *Ypos){
	
	static float xmin = 90.0;
	static float xmax = 939.0;
	float xm = (xmax - xmin) / 2.0; // x - Center 
	static float xLength = 182.0; //Width of Touchscreen in mm at 8.0"

	static float ymin = 102.0;    //380.0  
	static float ymax = 904.0;    //900.0
	float ym = (ymax - ymin) / 2.0; // y - Center
	static float yLength = 140.0; //Length of Touchscreen in mm at 8.0"

	float convertX = xLength / (xmax - xmin);   // converts raw x values to mm. found through manual calibration
	float convertY = yLength / (ymax - ymin);   // converts raw y values to mm. found through manual calibration
	
	static float mesX;
	static float mesY;
	static float lastmesX;
	static float lastmesY;
	
	mesX=read_touchX();
	mesY=read_touchY();
	unfilteredABG = mesX;
	unfilteredABG = (unfilteredABG - xmin - xm) * convertX;
	mesX = ADCfilterX(mesX,lastmesX);
	mesY = ADCfilterY(mesY,lastmesY);
	unfilteredLP = mesX;
	unfilteredLP = (unfilteredLP - xmin - xm) * convertX;
	mesX = LPfilterX(mesX);
	mesY = LPfilterY(mesY);
	
	*Xpos= (mesX - xmin - xm) * convertX;
	*Ypos= (mesY - ymin - ym) * convertY;
	
	lastmesX=mesX;
	lastmesY=mesY;
	
	return 1;
}
//#### Filtering ADC readings ####
float ADCfilterX(float current,float last){
	static bool LPfilter=false;
	static int i=0;
	static int j=0;

	//LP filter
	float filtered;
	
	float dt=SampleTime;

	//init speed and acceleration to 0
	static float abgfiltvx=0;
	static float abgfiltax=0;
	static float abgfiltvxlast=0;
	static float abgfiltaxlast=0;
	//position variables
	static float abgfiltx=0;
	static float abgfiltxlast=0;	
	static float rk=0;
	
	if(statex==1){//ball on plate
			j++;
			filtered = current;
			if(current<(80)){
				i++;
				if(i>10){
					statex=0;
					//re-init
					abgfiltvx=0;
					abgfiltax=0;
					abgfiltvxlast=0;
					abgfiltaxlast=0;

					abgfiltx=0;
					abgfiltxlast=0;	
					rk=0;
				}
			}
			else{
				i=0;
			}
	}
	if(statex==0){//no ball on plate
		i=0;
		j=0;
		filtered = current;
		if (current >(80)){//if position different than zero
			statex=1;
		}
	}
	if(j>10 && statex==1){
		LPfilter=true;
		
		//ALPHA-BETA-GAMMA FILTER
		abgfiltx = abgfiltxlast+dt*abgfiltvxlast+(pow(dt,2)/2)*abgfiltaxlast;
    abgfiltvx = abgfiltvxlast+dt*abgfiltaxlast;
    abgfiltax = abgfiltvxlast;
		
		rk=current-abgfiltx;

		if(rk<-170){
			//sprintf(txdata,"PROBLEMy\r\n"); // Fill up the buffer we're gonna send to PC
			//HAL_USART_Transmit(&husart2,(uint8_t*)txdata,strlen(txdata),HAL_MAX_DELAY); // Send buffer via USART
			
      abgfiltx=abgfiltx;
      abgfiltvx=abgfiltvx;
      abgfiltax=abgfiltax;
		}
		else{
			abgfiltx=abgfiltx+a*rk;
      abgfiltvx=abgfiltvx+(b/dt)*rk;
      abgfiltax=abgfiltax+(g/dt)*rk;
		}
		abgfiltxlast = abgfiltx;
		abgfiltvxlast = abgfiltvx;
		abgfiltaxlast = abgfiltax;
	}
	else{
		LPfilter=false;
		
		abgfiltx=current;
		abgfiltxlast=last;	
		
		filtered=current;
		
		last=current;
	}
	
	return abgfiltx;
}


float ADCfilterY(float current,float last){
	static bool LPfilter=false;
	static int i=0;
	static int j=0;

	float filtered;

	float dt=SampleTime;

	//init speed and acceleration to 0
	static float abgfiltvy=0;
	static float abgfiltay=0;
	static float abgfiltvylast=0;
	static float abgfiltaylast=0;
	//position variables
	static float abgfilty=0;
	static float abgfiltylast=0;	
	static float rk=0;
	
	if(statey==1){//ball on plate
			j++;
			filtered = current;
			if(current<(80)){
				i++;
				if(i>10){
					statey=0;
				}
			}
			else{
				i=0;
			}
	}
	if(statey==0){//no ball on plate
		i=0;
		j=0;
		filtered = current;
		if (current >(80)){//if position different than zero
			statey=1;
		}
	}
	if(j>10 && statey==1){
		LPfilter=true;
		
		//ALPHA-BETA-GAMMA FILTER
		abgfilty = abgfiltylast+dt*abgfiltvylast+(pow(dt,2)/2)*abgfiltaylast;
    abgfiltvy = abgfiltvylast+dt*abgfiltaylast;
    abgfiltay = abgfiltvylast;
		
		rk=current-abgfilty;
		
		if(rk<-170){
			//sprintf(txdata,"PROBLEMx\r\n"); // Fill up the buffer we're gonna send to PC
			//HAL_USART_Transmit(&husart2,(uint8_t*)txdata,strlen(txdata),HAL_MAX_DELAY); // Send buffer via USART
			
      abgfilty=abgfilty;
      abgfiltvy=abgfiltvy;
      abgfiltay=abgfiltay;
		}
		else{
			abgfilty=abgfilty+a*rk;
      abgfiltvy=abgfiltvy+(b/dt)*rk;
      abgfiltay=abgfiltay+(g/dt)*rk;
		}
		abgfiltylast = abgfilty;
		abgfiltvylast = abgfiltvy;
		abgfiltaylast = abgfiltay;
	}
	else{
		LPfilter=false;
		
		abgfilty=current;
		abgfiltylast=last;	
		
		last=current;
	}
	
	return abgfilty;
}



float LPfilterX(float current){
	static float LPfiltered;
	static float a=0.08;

	LPfiltered=a*current+(1-a)*LPfiltered;
	
	return LPfiltered;
}

float LPfilterY(float current){
	static float LPfiltered;
	static float a=0.1;

	LPfiltered=a*current+(1-a)*LPfiltered;
	
	return LPfiltered;
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
