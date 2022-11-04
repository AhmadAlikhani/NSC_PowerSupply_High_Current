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

extern UART_HandleTypeDef huart2;

void BoardCommFunc(void)
{
	memset(&hmi_config_data, 0x0, sizeof(hmi_config_data));

  /* Infinite loop */
  for(;;)
  {
	  uart_receiver(huart2, (void*)&hmi_config_data);

	  //---------- Calcultaion of Setpoint Limit ------------//
	  set_points_data.VoltageLimit=(uint32_t) hmi_config_data.Voltage_Limit;
	  set_points_data.Current_Target_SetPoint=(uint32_t) hmi_config_data.Current_Setpoint;

	  osDelay(1);
  }
}
