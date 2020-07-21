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
#include <math.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define PI 3.1416
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float GetTemp(void);
void first_order_lowpass(void) ;
void OnOff(void);
void Pid(void);
void AuToTuning(void);
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */
struct dulieugui{
	float nhietdo;
	float ki;
	float kp;
	float kd;
};
struct dulieunhan {
	uint16_t open;
	uint16_t mode; // mode phai luon duoc kieu tra
	float settemp;
	float ki;
	float kp;
	float kd;
	float time;
};
///
int T = 1; // thoi gian lay mau
float error[3]; // tinh sai so nhiet do
float temp_conf[]={0.1f,0.9f}; // he so bo lloc thong thap
float control[2] = {0,0}; // gia tri phan tram cong suat
uint8_t open=1; // kiem tra thermocouple co ho mach khong
uint32_t Timer =0, Ta =0;
// pwm
struct thongso{
	float temp0;
	float temp1;
	float ki;
	float kp;
	float kd;
	float settemp;
	float temp_auto;
	float pwm;
};
/* USER CODE END PV */
struct dulieugui dataout, *out;
struct dulieunhan datain, *in;
struct thongso thongso1= {0,0,0,0,0,0,0,0}, *thgso;
uint8_t *pointer, *pointer1; // pointer truyen vao dma de truyen du lieu
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
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
	out = &dataout ;
	in = &datain;
	thgso =&thongso1;
	pointer = (uint8_t*)out;
	pointer1 =(uint8_t*)in;
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
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_Delay(500);
	
	thongso1.temp0 = GetTemp();
  /* USER CODE END 2 */
	
	
	// bat dau ngat timer 1 de truyen du lieu lien tuc
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_UART_Receive_IT(&huart1, pointer1, 48);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
		switch(datain.mode)
			{
			case 0: OnOff();
			break;
			case 1: Pid();
			break;
			case 2: AuToTuning();
			break;
			default : thongso1.temp0 = GetTemp();
			}
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}
// ham doc nhiet do tu thermorcouple
float GetTemp(void){
	uint8_t buffer_bit[16];
	int16_t buffer =0;
	float temperature_instance;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
	for(int i=0;i<16;i++)
	{HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
		buffer_bit[i]=HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
		if (i>0 && i <13)
		{buffer=buffer<<1;
		if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6)==1)
			{buffer++;}
		}
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
	}
	open = buffer_bit[13];
	temperature_instance=buffer/4.0;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
	HAL_Delay(200);
	return temperature_instance;
}
////


	// cap nhat gia tri nhiet do 
	// nhap doi so la thongso1.temp1
	// de tinh nhiet do trung binh

void first_order_lowpass(void) 
{
	 thongso1.temp1= thongso1.temp0;
	thongso1.temp0 = (0.1*GetTemp()+0.9*thongso1.temp1);
}

//////


void OnOff(void){
		while (datain.open) {	
		first_order_lowpass(); // cap nhat gia tri 
		if (thongso1.temp0 > datain.settemp)
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,00);
		}
		else
		{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1600);//1600/2000=80%duty
		}	
	}
}
/////


void Pid(void) {
	////////////////////////Cap nhat thong so ban dau//////////////////
	HAL_TIM_Base_Start_IT(&htim2);
	error[2]=0;
	error[1]=0;
	error[0]=0;
	thongso1.temp0 = GetTemp();
	// cap nhat thong so PID dau tien
	thgso->kd = in->kd;
	thgso->ki = in->ki;
	thgso->kp = in->kp;
	//Kp=0.0276;Ki=0.000007;Kd=1.826;
	while (datain.open)
		{	
			first_order_lowpass();
			error[0]=datain.settemp - thongso1.temp0;
			// luat dieu khien pid
			control[0] = control[1]+thongso1.kp*(error[0] - error[1])
			+0.5*thongso1.ki*T*(error[0] + error[1])+thongso1.kd/T*( error[0] - 2*error[1]+ error[2]);
			///	
			/*
			P=Kp*(error - pre_error);
			I=0.5*Ki*(error + pre_error);
			D=Kd*( error - 2*pre_error+ pre_pre_error);
			*/
			// cap nhat thong so luat dieu khien
			error[2]=error[1];
			error[1] = error[0];
			control[1]= control[0];
			if (control[0]<0) control[0]=0;
			if (control[0]>1) control[0]=1;
			// tinh toan cong suat  va xuat ra 
			thongso1.pwm=sqrt(control[0])*1600;
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,(uint32_t)thongso1.pwm);
			HAL_Delay(1800);
		}
}
	

////
/////

void AuToTuning(void){
	////// cap nhat thong so ban dau /////
	HAL_TIM_Base_Start_IT(&htim2);
	uint8_t select = 0;
	error[2]=0;
	error[1]=0;
	error[0]=0;
	HAL_TIM_Base_Start_IT(&htim2); // tinh Tc
	int dem = 0;
	thongso1.temp0 = GetTemp();
	thongso1.temp_auto = 0.7f*datain.settemp;
	float ymax=thongso1.temp_auto;
	float ymin=thongso1.temp_auto;
	// bat dau
	while (datain.open==1) {	
		first_order_lowpass();
		if (select==0){// bat dau qua trinh ONOFF
			error[0]=thongso1.temp_auto-thongso1.temp0;
			if (error[0]<0)
			{
				__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,00);// gia tri pwm =0
			}
			else // nguoc lai thi maxgang
			{
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,1414);//1600/2000=80%duty
			}
			
			if ((error[0]>=0)&&(error[1]<0)&&(error[2]<0))
				{
				if(dem==1)
					{
					if ((Timer-Ta)>1000) dem++;
					}
				else if  (dem==0) dem++;
				}
			error[2]=error[1];
			error[1] = error[0];
			control[1]= control[0];
			if (dem==1){
				if(Ta==0) Ta=Timer;
				if(ymax<thongso1.temp0) ymax=thongso1.temp0;
				if(ymin>thongso1.temp0) ymin=thongso1.temp0;
			}
		}
			if (dem==2)
			{
				float Tc=Timer-Ta;
				thongso1.kp= cos(PI/2.4)*4*0.5/(PI*(ymax-ymin));
				thongso1.kd=thongso1.kp*Tc/(400*PI)*(tan(PI/2.4)+sqrt(4/5+tan(PI/2.4)*tan(PI/2.4)));
				thongso1.ki = thongso1.kp/(thongso1.kd/thongso1.kp*5.0f);
				select=1;
				error[2]=0;
				error[1]=0;
				error[0]=0;
				//Ti=Timer;
			}
		HAL_TIM_Base_Stop_IT(&htim2); // tat interrupt
		Timer = 0;
		}
		if (select==1){ // bat dau qua trinh pid
			Pid();
		}
}
/////////////////////////////////////////////////////Get_Temp//////////////////////////////////////

//////////////////////Calculate Average Time/////////////////////////////



////////////////////////////////////////////////////Iimer Interup////////////////////////////////////


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{if (htim->Instance==TIM2)
	{	Timer++;
	
	}
	if(htim->Instance == htim1.Instance){
		// cap nhat thong so goi di
		out->nhietdo = thongso1.temp0;
		out->ki = thongso1.ki;
		out->kp = thongso1.kp;
		out->kd = thongso1.kd;
		// dung dma nen khong ton tai nguyen cpu
		HAL_UART_Transmit_DMA(&huart1,pointer, 32);
	}
}
////
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//if(huart->Instance == huart1.Instance)
		;
}


////

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
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
  /** Initializes the CPU, AHB and APB busses clocks 
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 9999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 719;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 99;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 35999;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 3999;// pwm trong 2 s
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, EN_TRANS_TEMP_Pin|CLOCK_TRANS_TEMP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : EN_TRANS_TEMP_Pin CLOCK_TRANS_TEMP_Pin */
  GPIO_InitStruct.Pin = EN_TRANS_TEMP_Pin|CLOCK_TRANS_TEMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : INPUT_TRANS_TEMP_Pin */
  GPIO_InitStruct.Pin = INPUT_TRANS_TEMP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INPUT_TRANS_TEMP_GPIO_Port, &GPIO_InitStruct);

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
