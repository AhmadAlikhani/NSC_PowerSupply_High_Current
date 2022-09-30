#ifndef UART_MGR_COMM
#define UART_MGR_COMM

#include "main.h"
#include "stdint.h"

typedef struct hmi_configuration_data
{
	uint8_t Enable_HC;
	uint32_t Current_Setpoint;
	uint32_t Voltage_Limit;
}hmi_configuration_data_t;

typedef struct PowerSupplySetPoints
{
	uint32_t VoltageLimit;
	uint32_t Voltage_Setpoint;
	uint32_t Current_Target_SetPoint;
	uint32_t Current_Dac_Value;
	uint32_t Voltage_Step;
	uint32_t Current_Step;
}PowerSupplySetPoints_t;


#endif /* UART_MGR_COMM */
