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

#endif /* UART_MGR_COMM */
