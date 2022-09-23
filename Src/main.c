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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"
#include "stdint.h"
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

CAN_HandleTypeDef hcan;

CRC_HandleTypeDef hcrc;

DAC_HandleTypeDef hdac1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
unsigned char state='S';
uint8_t sent_data[16]={0x06,0x71,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},buffer_usart[13],rec_D[1],cn1;
uint8_t Enable_HC, ii, jj, kk, Avg_Cnt1, Avg_Cnt2, Avg_Cnt3;
uint32_t ADC_Calibration_Factor_1, ADC_Calibration_Factor_2, ADC_Buffer[5], ADC_Buffer_2[5];
uint16_t Current_Limit, Voltage_Limit, Power_Limit;
char Read_Current[5], Read_Voltage[5];
uint32_t ADC0_Buffer_Sum,Averaged_ADC0_Buffer,Moving_Average_Buffer_Current,Setpoint_Limit_Current,ADC_Calibration_Factor;
uint32_t ADC1_Buffer_Sum, Averaged_ADC1_Buffer, Moving_Average_Buffer_Voltage, Setpoint_Limit_Voltage;
uint32_t Power_Buffer, Averaged_Power, Moving_Average_Power, Output_Power,Pout=0;
float Iout_NotCalibrated,Iout;
float Current_Calibration_Coefficient = 1;//0.1185; //Current_HMI-Preview = 0.1185*ADC + 0.9473 
float Current_Calibration_Offset = 0;//0.9473;
float Vout_NotCalibrated,Vout;
float Voltage_Calibration_Coefficient = 1;//0.0056; // Voltage_HMI-Preview = 0.0056*ADC + 0.0481
float Voltage_Calibration_Offset = 0;//0.0481;

uint32_t Voltage_SP=0,Voltage_Setpoint;
uint32_t Current_SP=0,Current_Setpoint;
unsigned char DC_OK=0, PFC_OK=0, Inrush_OK=0;
unsigned int DC_OK_Cnt=0, PFC_Cnt=0, Inrush_Cnt=0;

uint16_t Heatsink_Overheat, Trans_Overheat, Fan_Enable;
uint16_t tacho1,tacho2,Fan_Timer,Fan_Timer_Enable,tacho1_Backup,tacho2_Backup,tacho1_Disable,tacho2_Disable,Usart_Counter,Com_Failure;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_CRC_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void i2s(unsigned long int ,char str[5]);                // Integer to string data conversion
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_ADC2_Init();
  MX_CRC_Init();
  MX_USART2_UART_Init();
  MX_CAN_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);
	
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_RXNE);	// Receive Data register not empty interrupt
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_TC);		// Transmission complete interrupt
	
	HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
	ADC_Calibration_Factor_1 = HAL_ADCEx_Calibration_GetValue(&hadc1, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc1, ADC_SINGLE_ENDED, ADC_Calibration_Factor_1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) ADC_Buffer, 3);
	
	HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
	ADC_Calibration_Factor_2 = HAL_ADCEx_Calibration_GetValue(&hadc2, ADC_SINGLE_ENDED);
	HAL_ADCEx_Calibration_SetValue(&hadc2, ADC_SINGLE_ENDED, ADC_Calibration_Factor_2);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t*) ADC_Buffer_2, 3);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET);							// DE/RE = 0 ----> ADM485 is in 
	HAL_TIM_Base_Start_IT(&htim1);
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);//Fan Enable

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		//--------------------- Data Gathering ---------------------//
		Current_Limit	= (buffer_usart[0]-48)*1000 + (buffer_usart[1]-48)*100 +(buffer_usart[2]-48)*10 +(buffer_usart[3]-48)*1	;
		Voltage_Limit	= (buffer_usart[4]-48)*1000 + (buffer_usart[5]-48)*100 +(buffer_usart[6]-48)*10 +(buffer_usart[7]-48)*1	;
		Power_Limit		= (buffer_usart[8]-48)*1000 + (buffer_usart[9]-48)*100 +(buffer_usart[10]-48)*10 +(buffer_usart[11]-48)*1	;
		Enable_HC			= (buffer_usart[12]-48)	;
		//----------------------------------------------------------//
		//Voltage_SP=(uint32_t)(Voltage_Limit*185.150-10.118); //DAC_Voltage = HMI_VSP*185.150-10.118
		//Current_SP=(uint32_t)(Current_Limit*15.364+675.450); //DAC_Current = HMI_ISP*15.364+675.450
		Voltage_SP=(uint32_t) Voltage_Limit;
		Current_SP=(uint32_t) Current_Limit;
		if (Voltage_SP<=0)
			Voltage_SP=0;
		if (Voltage_SP>=2750)
			Voltage_SP=2750;
		
		if (Current_SP<=0)
			Current_SP=0;
		if (Current_SP>=4095)
			Current_SP=4095;
		
		if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8))
		{
			DC_OK_Cnt++;
			if (DC_OK_Cnt>=40000)
			{
				DC_OK=1;
				DC_OK_Cnt=40000;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET); // DC_OK Status LED On
			}
		}
		else
		{
			DC_OK_Cnt=0;
			DC_OK=0;
			Voltage_SP=0;
			Current_SP=0;
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET); // DC_OK Status LED Off
		};
		
		if ((Com_Failure==0) & (Enable_HC==1) & (Trans_Overheat==0) & (Heatsink_Overheat==0) & (tacho1_Disable==0))
		{
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);							// Fan Enable
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);							// Relay AC Enable
			if (PFC_Cnt<1000)
			{
				PFC_Cnt++;
			}
			else
			{
				PFC_OK=1;
				PFC_Cnt=1000;
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);					// PFC Enable
			};
			
			Fan_Timer_Enable=0;
			Fan_Timer=0;
			if ((tacho2_Disable==0) & (DC_OK==1) & (PFC_OK==1) & (Current_SP>0 || Voltage_SP>0))
			{
				if (Inrush_Cnt<1000)//500
				{
					Inrush_Cnt++;
				}
				else
				{
					Inrush_Cnt=1000;//500
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);					// Inrush Relay Enable
					Inrush_OK=1;
				};
				if (Inrush_OK==1)
				{
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_SET);						// Enable HC
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_SET);					// Enable Status LED On
					// ----------------------------- Current ----------------------------- //
					if (Current_Setpoint < Current_SP && Current_Setpoint < 4095)
						Current_Setpoint = Current_Setpoint + 1;
					if (Current_Setpoint >= Current_SP && Current_Setpoint > 0)
						Current_Setpoint = Current_Setpoint - 1;
					//---------------------------------------------------------------------//
					// ----------------------------- Voltage ----------------------------- //
					if (Voltage_Setpoint < Voltage_SP  && Voltage_Setpoint < 4095)
						Voltage_Setpoint = Voltage_Setpoint + 1;
					if (Voltage_Setpoint >= Voltage_SP && Voltage_Setpoint > 0)
						Voltage_Setpoint = Voltage_Setpoint - 1;
					//---------------------------------------------------------------------//
				};
			}
			else
			{
				//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);						// Relay AC Disable
				if (DC_OK_Cnt>=40000)
					HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);					// PFC Disable
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);					// Inrush Relay Disable
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);						// Disable HC
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);					// Enable Status LED Off

				Current_Setpoint = 0;
				Voltage_Setpoint = 0;
			};
		}
		else
		{
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);							// Relay AC Disable
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);						// PFC Disable
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET);						// Inrush Relay Disable
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,GPIO_PIN_RESET);							// Disable HC
			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);						// Enable Status LED Off
			Fan_Timer_Enable=1;
			Inrush_Cnt=0;
			Inrush_OK=0;
			PFC_Cnt=0;
			PFC_OK=0;
			if (Fan_Timer>60)
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);						 //////// timer
				Fan_Timer_Enable=0;
				Fan_Timer=0;
			};
			Current_Setpoint = 0;
			Voltage_Setpoint = 0;
		};
		
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(Voltage_Setpoint));//(Arc_Level/(Current_Calibration_Coefficient*Nextion_Current_IL300_Coefficient)));
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)(Current_Setpoint));
		
		//--------------------- Voltage ADC ---------------------//
		ADC0_Buffer_Sum = 0;
		for (ii=0;ii<50;ii++)
			ADC0_Buffer_Sum = ADC0_Buffer_Sum + ADC_Buffer[0];
		Averaged_ADC0_Buffer = ADC0_Buffer_Sum/50;
		// Moving average filter //
		Moving_Average_Buffer_Voltage = Moving_Average_Buffer_Voltage + Averaged_ADC0_Buffer;
		if (Avg_Cnt1==10)
		{
			Vout_NotCalibrated = (uint32_t) ( ((Vout_NotCalibrated*9)+(Moving_Average_Buffer_Voltage/10))/10 );
			Vout = (Vout_NotCalibrated*Voltage_Calibration_Coefficient); //Voltage_HMI-Preview = 0.0056*ADC + 0.0481
			Moving_Average_Buffer_Voltage = 0;
			// Offset Calibration //
			if (Vout-Voltage_Calibration_Offset>0)
				Vout = Vout + Voltage_Calibration_Offset;   
			else
				Vout = 0;
			
			////////////////////////
			Avg_Cnt1 = 0;
		};
		Avg_Cnt1++;
		i2s(Vout,Read_Voltage);
		sent_data[6]=Read_Voltage[0];
		sent_data[7]=Read_Voltage[1];
		sent_data[8]=Read_Voltage[2];
		sent_data[9]=Read_Voltage[3];
		//-------------------------------------------------------//
		
		//--------------------- Current ADC ---------------------//
		ADC1_Buffer_Sum = 0;
		for (jj=0;jj<50;jj++)
			ADC1_Buffer_Sum = ADC1_Buffer_Sum + ADC_Buffer[1];
		Averaged_ADC1_Buffer = ADC1_Buffer_Sum/50;
		// Moving average filter //
		Moving_Average_Buffer_Current = Moving_Average_Buffer_Current + Averaged_ADC1_Buffer;
		if (Avg_Cnt2==10)
		{
			Iout_NotCalibrated = (uint32_t) ( ((Iout_NotCalibrated*9)+(Moving_Average_Buffer_Current/10))/10 );
			Iout = (uint32_t)(Iout_NotCalibrated*Current_Calibration_Coefficient); //Current_HMI-Preview = 0.1185*ADC + 0.9473 
			Moving_Average_Buffer_Current = 0;
			// Offset Calibration //
			if (Iout-Current_Calibration_Offset>0)
				Iout = Iout + Current_Calibration_Offset;
			else
				Iout = 0;
			////////////////////////
			Avg_Cnt2 = 0;
		};
		Avg_Cnt2++;
		i2s(Iout,Read_Current);
		sent_data[2]=Read_Current[0];
		sent_data[3]=Read_Current[1];
		sent_data[4]=Read_Current[2];
		sent_data[5]=Read_Current[3];
		//-------------------------------------------------------//
		sent_data[10]=DC_OK+'0';
		sent_data[11]=Trans_Overheat+'0';
		sent_data[12]=Heatsink_Overheat+'0';
		sent_data[13]=tacho1_Disable+'0';
		sent_data[14]=tacho2_Disable+'0';
		
		//--------------------- Power ADC ---------------------//
		Pout=Vout*Iout/1000;
		//------ Power Filter ------//
		Power_Buffer = 0;
		for (kk=0;kk<50;kk++)
			Power_Buffer = Power_Buffer + Pout;
		Averaged_Power = Power_Buffer/50;
		// Moving average filter //
		Moving_Average_Power = Moving_Average_Power + Averaged_Power;
		if (Avg_Cnt3==10)
		{
			Output_Power = (uint32_t) ( ((Output_Power*9)+(Moving_Average_Power/10))/10 );
			Moving_Average_Power = 0;
			Avg_Cnt3 = 0;
		};
		Avg_Cnt3++;
		//--------------------------//
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(0));//(Arc_Level/(Current_Calibration_Coefficient*Nextion_Current_IL300_Coefficient)));
		//HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)(0));
		
		if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9))	
			Heatsink_Overheat=1;											// overheat
		else
			Heatsink_Overheat=0;											// normal
		
		if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))	
			Trans_Overheat=1;													// overheat
		else
			Trans_Overheat=0;													// normal
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 16;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT1 config
  */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_OutputSwitch = DAC_OUTPUTSWITCH_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

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
  htim1.Init.Prescaler = 32000;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 2000;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 19200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14|GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5
                          |GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC6 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10
                           PB11 PB14 PB15 PB5
                           PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10
                          |GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_5
                          |GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void i2s(unsigned long int num,char str[5]){
    str[0] = (num/1000)%10 + '0';
    str[1] = (num/100)%10 + '0';
    str[2] = (num/10)%10 + '0';
    str[3] = (num/1)%10 + '0';
    str[4] = '0';
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
