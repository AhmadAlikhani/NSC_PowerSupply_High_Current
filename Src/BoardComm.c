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
uint8_t buffer_usart[13];
hmi_configuration_data_t hmi_config_data;
PowerSupplySetPoints_t set_points_data;

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
		set_points_data.VoltageLimit=(uint32_t) hmi_config_data.Voltage_Limit;
		set_points_data.Current_Target_SetPoint=(uint32_t) hmi_config_data.Current_Setpoint;

	    osDelay(1);
  }
}
