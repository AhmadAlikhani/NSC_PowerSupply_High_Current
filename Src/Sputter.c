/*
 * Sputter.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "Sputter.h"
#include "uart_mgr.h"
#include "main.h"

FailureStatus_t failure_status;
FlagsStatus_t flags_status;
unsigned int DC_OK_Cnt=0, PFC_Cnt=0, Inrush_Cnt=0;
uint16_t Fan_Timer_Enable;
uint16_t Fan_Timer;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DAC_HandleTypeDef hdac1;

extern PowerSupplySetPoints_t set_points_data;
extern hmi_configuration_data_t hmi_config_data;


void SputterFunc(void)
{
  /* USER CODE BEGIN Sputter */

  /* Infinite loop */
  for(;;)
  {
	  if (set_points_data.VoltageLimit<=0)
		  set_points_data.VoltageLimit=0;
	  if (set_points_data.VoltageLimit>=2750)
		  set_points_data.VoltageLimit=2750;

	  if (set_points_data.Current_Target_SetPoint<=0)
		  set_points_data.Current_Target_SetPoint=0;
	  if (set_points_data.Current_Target_SetPoint>=4095)
		  set_points_data.Current_Target_SetPoint=4095;

	  if (!HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_8))
	  {
	  	DC_OK_Cnt++;
	  	if (DC_OK_Cnt>=40000)
	  	{
	  		flags_status.flag_DC_OK=1;
	  		DC_OK_Cnt=40000;
	  		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET); // DC_OK Status LED On
	  	}
	  }
	  else
	  {
	  	DC_OK_Cnt=0;
	  	flags_status.flag_DC_OK=0;
	  	set_points_data.VoltageLimit=0;
	  	set_points_data.Current_Target_SetPoint=0;
	  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET); // DC_OK Status LED Off
	  };

	  if ((failure_status.Com_Failure==0) & (hmi_config_data.Enable_HC==1) & (failure_status.Trans_Overheat==0) & (failure_status.Heatsink_Overheat==0) & (failure_status.tacho1_Disable==0))
	  {
	  	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);							// Fan Enable
	  	//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);							// Relay AC Enable
	  	if (PFC_Cnt<1000)
	  	{
	  		PFC_Cnt++;
	  	}
	  	else
	  	{
	  		flags_status.flag_PFC_OK=1;
	  		PFC_Cnt=1000;
	  		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);					// PFC Enable
	  	};

	  	Fan_Timer_Enable=0;
	  	Fan_Timer=0;
	  	if ((failure_status.tacho2_Disable==0) & (flags_status.flag_DC_OK==1) & (flags_status.flag_PFC_OK==1) & (set_points_data.Current_Target_SetPoint>0 || set_points_data.VoltageLimit>0))
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
	  			if (set_points_data.Current_Dac_Value < set_points_data.Current_Target_SetPoint)
	  				set_points_data.Current_Dac_Value = set_points_data.Current_Dac_Value + set_points_data.Current_Step;
	  			if (set_points_data.Current_Dac_Value >= set_points_data.Current_Target_SetPoint)
	  				set_points_data.Current_Dac_Value = set_points_data.Current_Dac_Value - set_points_data.Current_Step;
	  			//---------------------------------------------------------------------//
	  			// ----------------------------- Voltage ----------------------------- //
	  			if (set_points_data.Voltage_Setpoint < set_points_data.VoltageLimit)
	  				set_points_data.Voltage_Setpoint = set_points_data.Voltage_Setpoint + set_points_data.Voltage_Step;
	  			if (set_points_data.Voltage_Setpoint >= set_points_data.VoltageLimit)
	  				set_points_data.Voltage_Setpoint = set_points_data.Voltage_Setpoint - set_points_data.Voltage_Step;
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

	  		set_points_data.Current_Dac_Value = 0;
	  		set_points_data.Voltage_Setpoint = 0;
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
	  	flags_status.flag_PFC_OK=0;
	  	if (Fan_Timer>60)
	  	{
	  		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);						 //////// timer
	  		Fan_Timer_Enable=0;
	  		Fan_Timer=0;
	  	};
	  	set_points_data.Current_Dac_Value = 0;
	  	set_points_data.Voltage_Setpoint = 0;
	  };

	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, (uint32_t)(set_points_data.Voltage_Setpoint));//(Arc_Level/(Current_Calibration_Coefficient*Nextion_Current_IL300_Coefficient)));
	  HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint32_t)(set_points_data.Current_Dac_Value));


		if (HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_9))
			failure_status.Heatsink_Overheat=1;											// overheat
		else
			failure_status.Heatsink_Overheat=0;											// normal

		if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13))
			failure_status.Trans_Overheat=1;													// overheat
		else
			failure_status.Trans_Overheat=0;													// normal
	}
	    osDelay(1);

  }

