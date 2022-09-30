/*
 * BoardComm.c
 *
 *  Created on: Aug 12, 2022
 *      Author: M.Mozafari
 */

#include "BoardComm.h"
#include "uart_mgr.h"

extern uint8_t buffer_usart2[24];
uint8_t Control_Mode;
float Setpoint_Limit_Offset_C1 = 0;//0.25;//0.4983;//0.2611;
float Setpoint_Limit_Coefficient_C =1;//1.0286;//1.329;
float Setpoint_Limit_Offset_V1 = 0;//1.6476;//0.5773;
float Setpoint_Limit_Offset_V2 = 0;
float Setpoint_Limit_Coefficient_V = 1;//0.6258;//0.6243;//0.5626; //DAC-SP=(Nextion-SP*0.5626P-1.5773)
uint32_t Setpoint_Limit_Current;
uint32_t Setpoint_Limit_Voltage;
uint32_t VoltageLimit=0,Voltage_Setpoint;
uint32_t Current_SP=0,Current_Setpoint;
uint8_t buffer_usart[13];
hmi_configuration_data_t hmi_config_data;

void BoardCommFunc(void)
{
	memset(&hmi_config_data, 0x0, sizeof(hmi_config_data));

  /* Infinite loop */
  for(;;)
  {
		//--------------------- Data Gathering ---------------------//
		hmi_config_data.Current_Setpoint	= (buffer_usart[0]-48)*1000 + (buffer_usart[1]-48)*100 +(buffer_usart[2]-48)*10 +(buffer_usart[3]-48)*1	;
		hmi_config_data.Voltage_Limit		= (buffer_usart[4]-48)*1000 + (buffer_usart[5]-48)*100 +(buffer_usart[6]-48)*10 +(buffer_usart[7]-48)*1	;
		hmi_config_data.Enable_HC			= (buffer_usart[12]-48)	;
		//----------------------------------------------------------//
		//Voltage_SP=(uint32_t)(Voltage_Limit*185.150-10.118); //DAC_Voltage = HMI_VSP*185.150-10.118
		//Current_SP=(uint32_t)(Current_Limit*15.364+675.450); //DAC_Current = HMI_ISP*15.364+675.450
		VoltageLimit=(uint32_t) hmi_config_data.Voltage_Limit;
		Current_SP=(uint32_t) hmi_config_data.Current_Setpoint;

	    osDelay(1);
  }
}
