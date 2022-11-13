/*
 * ADC_Calculation.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "ADC_Calculation.h"
#include "Sputter.h"
#include "stdint.h"

uint32_t Power_Buffer, Averaged_Power, Moving_Average_Power, Output_Power,Pout=0;
uint32_t ADC0_Buffer_Sum,Averaged_ADC0_Buffer,Moving_Average_Buffer_Current,ADC_Calibration_Factor;
uint8_t Avg_Cnt1, Avg_Cnt2, Avg_Cnt3;
uint32_t ADC1_Buffer_Sum, Averaged_ADC1_Buffer, Moving_Average_Buffer_Voltage;
uint32_t ADC_Buffer[5];
uint8_t sent_data[16]={0x06,0x71,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30},rec_D[1],cn1;
char Read_Current[5], Read_Voltage[5];
float Vout_NotCalibrated,Vout;
float Voltage_Calibration_Offset = 0;//0.0481;
float Iout_NotCalibrated,Iout;
float Voltage_Calibration_Coefficient = 1;//0.0056; // Voltage_HMI-Preview = 0.0056*ADC + 0.0481
float Current_Calibration_Coefficient = 1;//0.1185; //Current_HMI-Preview = 0.1185*ADC + 0.9473
float Current_Calibration_Offset = 0;//0.9473;

extern FailureStatus_t failure_status;
extern FlagsStatus_t flags_status;

void ADC_CaculationFunc(void)
{
  /* USER CODE BEGIN ADC_Caculation */
  /* Infinite loop */
  for(;;)
  {
	  //--------------------- Voltage ADC ---------------------//
	  ADC0_Buffer_Sum = 0;
	  for (uint32_t i=0; i < 50 ;i++)
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
	  }

	  Avg_Cnt1++;
	  i2s(Vout,sent_data);

	  //-------------------------------------------------------//

	  //--------------------- Current ADC ---------------------//
	  ADC1_Buffer_Sum = 0;
	  for (uint32_t i=0; i < 50 ; i++)
	  	ADC1_Buffer_Sum = ADC1_Buffer_Sum + ADC_Buffer[1];

	  Averaged_ADC1_Buffer = ADC1_Buffer_Sum/50;
	  // Moving average filter //
	  Moving_Average_Buffer_Current = Moving_Average_Buffer_Current + Averaged_ADC1_Buffer;

	  if (Avg_Cnt2==10)
	  {
	  	Iout_NotCalibrated = (uint32_t) ((( Iout_NotCalibrated * 9 ) + ( Moving_Average_Buffer_Current / 10 )) / 10 );
	  	Iout = (uint32_t)(Iout_NotCalibrated*Current_Calibration_Coefficient); //Current_HMI-Preview = 0.1185*ADC + 0.9473
	  	Moving_Average_Buffer_Current = 0;
	  	// Offset Calibration //
	  	if (Iout-Current_Calibration_Offset>0)
	  		Iout = Iout + Current_Calibration_Offset;
	  	else
	  		Iout = 0;
	  	////////////////////////
	  	Avg_Cnt2 = 0;
	  }

	  Avg_Cnt2++;
	  i2s(Iout,sent_data);

	  //-------------------------------------------------------//
	  sent_data[10]=flags_status.flag_DC_OK+'0';
	  sent_data[11]=failure_status.Trans_Overheat+'0';
	  sent_data[12]=failure_status.Heatsink_Overheat+'0';
	  sent_data[13]=failure_status.tacho1_Disable+'0';
	  sent_data[14]=failure_status.tacho2_Disable+'0';

	  //--------------------- Power ADC ---------------------//
	  Pout = (Vout * Iout) / 1000;
	  //------ Power Filter ------//
	  Power_Buffer = 0;
	  for (uint32_t i=0; i < 50; i++)
	  	Power_Buffer = Power_Buffer + Pout;
	  Averaged_Power = Power_Buffer / 50;
	  // Moving average filter //
	  Moving_Average_Power = Moving_Average_Power + Averaged_Power;
	  if (Avg_Cnt3==10)
	  {
	  	Output_Power = (uint32_t) ( ((Output_Power * 9 )+(Moving_Average_Power / 10 )) / 10 );
	  	Moving_Average_Power = 0;
	  	Avg_Cnt3 = 0;
	  };
	  Avg_Cnt3++;
	  //--------------------------//

	  osDelay(1);
  }
}


void i2s(unsigned long int num,char str[5]){
    str[0] = (num/1000)%10 + '0';
    str[1] = (num/100)%10 + '0';
    str[2] = (num/10)%10 + '0';
    str[3] = (num/1)%10 + '0';
    str[4] = '0';
}
