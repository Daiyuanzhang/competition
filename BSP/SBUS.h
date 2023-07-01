#ifndef  __SBUS_H_
#define  __SBUS_H_
#include "stm32f4xx.h"

#define SBUS_RX_BUF_NUM_1 46u
#define RC_FRAME_LENGTH_1 21u
#define RC_SBUS_CH_VALUE_OFFSET 1024
#define RC_SBUS_CHANNAL_ERROR_VALUE 730


void SBUS_TO_RC_1(volatile const uint8_t *rec_sbus_data,  int *sbus_channel);
void SBUS_Init();
extern  int sbus_channel[16];
extern volatile unsigned char rc_sbus_data[2][SBUS_RX_BUF_NUM_1];
extern int SBUS_CH[16];












#endif