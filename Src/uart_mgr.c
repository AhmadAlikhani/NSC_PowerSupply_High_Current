/*
 * uart_mgr.c
 *
 *  Created on: Aug 25, 2022
 *      Author: M.Mozafari
 */

#include "uart_mgr.h"

#define MAX_ENABLE_HC_VALUE 			4095
#define MAX_HMI_CURRENT_VALUE				4095
#define MAX_HMI_VOLTAGE_LIMIT_VALUE		4095
#define MAX_ARC_LEVEL_VALUE					4095
#define MAX_QUANCH_TIME_VALUE				5001

uint8_t buffer_usart2[24];
hmi_configuration_data_t * temporary_data;
extern UART_HandleTypeDef huart2;
extern uint8_t buffer_usart[13];

void uart_receiver(UART_HandleTypeDef uart_index, void* outgoing_data)
{
	if(uart_index.Instance == huart2.Instance)
	{
		//--------------------- Data Gathering ---------------------//
		//store incomming data if they are greater than 0 and lower than max values
		temporary_data = (hmi_configuration_data_t*) outgoing_data;
		temporary_data->Current_Setpoint	= ((((buffer_usart[0]-48)*1000 + (buffer_usart[1]-48)*100 +(buffer_usart[2]-48)*10 +
												(buffer_usart[3]-48)) < MAX_HMI_CURRENT_VALUE) && (((buffer_usart[0]-48)*1000 +
												(buffer_usart[1]-48)*100 +(buffer_usart[2]-48)*10 +(buffer_usart[3]-48)) > 0))
												? ((buffer_usart[0]-48)*1000 + (buffer_usart[1]-48)*100 +(buffer_usart[2]-48)*10 +
												(buffer_usart[3]-48)) : (temporary_data->Current_Setpoint);

		temporary_data->Voltage_Limit		= ((((buffer_usart[4]-48)*1000 + (buffer_usart[5]-48)*100 +(buffer_usart[6]-48)*10 +
												(buffer_usart[7]-48)) < MAX_HMI_VOLTAGE_LIMIT_VALUE ) && (((buffer_usart[4]-48)*1000 +
												(buffer_usart[5]-48)*100 +(buffer_usart[6]-48)*10 +(buffer_usart[7]-48)) > 0 ))
												? (((buffer_usart[4]-48)*1000 + (buffer_usart[5]-48)*100 +(buffer_usart[6]-48) * 10 +
												(buffer_usart[7]-48))) : (temporary_data->Voltage_Limit);

		temporary_data->Enable_HC			= (((buffer_usart[12]-48) < MAX_ENABLE_HC_VALUE) && ((buffer_usart[12]-48) > 0))
												? (buffer_usart[12]-48) : (temporary_data->Enable_HC);

	}
}
