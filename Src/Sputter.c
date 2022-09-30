/*
 * Sputter.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "Sputter.h"
#include "uart_mgr.h"
#include "main.h"

FailureStatus_t flags_status;
unsigned int DC_OK_Cnt=0, PFC_Cnt=0, Inrush_Cnt=0;
uint16_t Fan_Timer_Enable;
uint16_t Fan_Timer;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac1;

extern uint32_t VoltageLimit;
extern uint32_t Voltage_Setpoint;
extern uint32_t Current_SP;
extern uint32_t Current_Setpoint;
extern hmi_configuration_data_t hmi_config_data;


void SputterFunc(void)
{
  /* USER CODE BEGIN Sputter */

  /* Infinite loop */
  for(;;)
  {
	  if (VoltageLimit<=0)
		  VoltageLimit=0;
	  if (VoltageLimit>=2750)
		  VoltageLimit=2750;

	  if (Current_SP<=0)
	  	Current_SP=0;
	  if (Current_SP>=4095)
	  	Current_SP=4095;

	  if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8))
	  {
	  	DC_OK_Cnt++;
	  	if (DC_OK_Cnt>=40000)
	  	{
	  		flags_status.DC_OK=1;
	  		DC_OK_Cnt=40000;
	  		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET); // DC_OK Status LED On
	  	}
	  }
	  else
	  {
	  	DC_OK_Cnt=0;
	  	flags_status.DC_OK=0;
	  	VoltageLimit=0;
	  	Current_SP=0;
	  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET); // DC_OK Status LED Off
	  };

	  if ((flags_status.Com_Failure==0) & (hmi_config_data.Enable_HC==1) & (flags_status.Trans_Overheat==0) & (flags_status.Heatsink_Overheat==0) & (flags_status.tacho1_Disable==0))
	  {
	  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);							// Fan Enable
	  	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);							// Relay AC Enable
	  	if (PFC_Cnt<1000)
	  	{
	  		PFC_Cnt++;
	  	}
	  	else
	  	{
	  		flags_status.PFC_OK=1;
	  		PFC_Cnt=1000;
	  		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);					// PFC Enable
	  	};

	  	Fan_Timer_Enable=0;
	  	Fan_Timer=0;
	  	if ((flags_status.tacho2_Disable==0) & (flags_status.DC_OK==1) & (flags_status.PFC_OK==1) & (Current_SP>0 || VoltageLimit>0))
	  	{
	  		if (Inrush_Cnt<1000)//500
	  		{
	  			Inrush_Cnt++;
	  		}
	  		else
	  		{
	  			Inrush_Cnt=1000;//500
	  			HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET);					// Inrush Relay Enable
	  			flags_status.Inrush_OK=1;
	  		};
	  		if (flags_status.Inrush_OK==1)
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
	  			if (Voltage_Setpoint < VoltageLimit  && Voltage_Setpoint < 4095)
	  				Voltage_Setpoint = Voltage_Setpoint + 1;
	  			if (Voltage_Setpoint >= VoltageLimit && Voltage_Setpoint > 0)
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
	  	flags_status.Inrush_OK=0;
	  	PFC_Cnt=0;
	  	flags_status.PFC_OK=0;
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


		if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9))
			flags_status.Heatsink_Overheat=1;											// overheat
		else
			flags_status.Heatsink_Overheat=0;											// normal

		if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))
			flags_status.Trans_Overheat=1;													// overheat
		else
			flags_status.Trans_Overheat=0;													// normal
	}
	    osDelay(1);

  }

