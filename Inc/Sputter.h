#ifndef SPUTTER_HANDLE_DEFINE
#define SPUTTER_HANDLE_DEFINE

#include "stdint.h"

void SputterFunc(void);

typedef struct FailureStatus
{
	uint16_t Com_Failure;
	uint16_t tacho2_Disable;
	uint16_t tacho1_Disable;
	uint16_t Heatsink_Overheat;
	uint16_t Trans_Overheat;
}FailureStatus_t;

typedef struct FlagsStatus
{
	unsigned int flag_DC_OK;
	unsigned int flag_PFC_OK;
	unsigned int flag_PFC_OTP;
	unsigned int flag_FB_OTP;
	unsigned int flag_Hiccup;
	unsigned int flag_Fan;
	unsigned char Inrush_OK;
}FlagsStatus_t;

typedef struct FailureCounter
{
	unsigned int DC_OK_Cnt;
	unsigned int PFC_Cnt;
	unsigned int Inrush_Cnt;
}FailureCounter_t;

#endif /*SPUTTER_HANDLE_DEFINE*/
