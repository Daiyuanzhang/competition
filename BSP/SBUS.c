#include "SBUS.h"
#include "stddef.h"

//volatile unsigned char rc_sbus_data[2][SBUS_RX_BUF_NUM_1] = {0,0,0,0,0,0,0,0,0,0,
//	                     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
//0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

volatile unsigned char rc_sbus_data[2][SBUS_RX_BUF_NUM_1] = {0,0,0,0,0,0,0,0,0,0,
	                     0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
0,0,0,0};

int sbus_channel[16];
int SBUS_CH[16];

uint8_t RC_SBUS_data_is_error(void);

void SBUS_Init()
{
 	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;	
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
     
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2 , ENABLE);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;	                                                                                                                                                                                                                                                                                                         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7 ,GPIO_AF_USART1);

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
  RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	                                                                                                                                                                                                                                                                                                         
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
	GPIO_Init(GPIOB,&GPIO_InitStructure);	
	
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6 ,GPIO_AF_USART1);	

	USART_DeInit(USART1);
	
	USART_InitStructure.USART_BaudRate = 100000;	//SBUS 100K baudrate
	USART_InitStructure.USART_WordLength = USART_WordLength_9b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_Even; //偶校验
	USART_InitStructure.USART_Mode = USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);	
	
	USART_ClearFlag(USART1, USART_FLAG_IDLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
											
	USART_DMACmd(USART1,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART1, ENABLE);	
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 6; 
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_Cmd(DMA2_Stream2, DISABLE);	
	 while (DMA2_Stream2->CR & DMA_SxCR_EN); //禁止数据流
	 
	DMA_DeInit(DMA2_Stream2);	
	DMA_InitStructure.DMA_Channel= DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);	
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rc_sbus_data[0];			
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;									
	DMA_InitStructure.DMA_BufferSize = 50;	//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界															
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;			
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;							
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; 		
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;												
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;							
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream2,&DMA_InitStructure);
	DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)rc_sbus_data[1], DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
	DMA_Cmd(DMA2_Stream2, DISABLE); //Add a disable
	DMA_Cmd(DMA2_Stream2, ENABLE);
		 
}

void SBUS_TO_RC_1(volatile const uint8_t *rec_sbus_data,  int *sbus_channel)
{
    if (rec_sbus_data == NULL || sbus_channel == NULL)
    {
        return;
    }


	sbus_channel[0]  = ((rec_sbus_data[1]|rec_sbus_data[2]<<8) & 0x07FF);
	sbus_channel[1]  = ((rec_sbus_data[2]>>3 |rec_sbus_data[3]<<5) & 0x07FF);
	sbus_channel[2]  = ((rec_sbus_data[3]>>6 |rec_sbus_data[4]<<2 |rec_sbus_data[5]<<10) & 0x07FF);
	sbus_channel[3]  = ((rec_sbus_data[5]>>1 |rec_sbus_data[6]<<7) & 0x07FF);
	sbus_channel[4]  = ((rec_sbus_data[6]>>4 |rec_sbus_data[7]<<4) & 0x07FF);
	sbus_channel[5]  = ((rec_sbus_data[7]>>7 |rec_sbus_data[8]<<1 |rec_sbus_data[9]<<9) & 0x07FF);
	sbus_channel[6]  = ((rec_sbus_data[9]>>2 |rec_sbus_data[10]<<6) & 0x07FF);
	sbus_channel[7]  = ((rec_sbus_data[10]>>5|rec_sbus_data[11]<<3) & 0x07FF);
	sbus_channel[8]  = ((rec_sbus_data[12]   |rec_sbus_data[13]<<8) & 0x07FF);
	sbus_channel[9]  = ((rec_sbus_data[13]>>3|rec_sbus_data[14]<<5) & 0x07FF);
	sbus_channel[10] = ((rec_sbus_data[14]>>6|rec_sbus_data[15]<<2|rec_sbus_data[16]<<10) & 0x07FF);
	sbus_channel[11] = ((rec_sbus_data[16]>>1|rec_sbus_data[17]<<7) & 0x07FF);
	sbus_channel[12] = ((rec_sbus_data[17]>>4|rec_sbus_data[18]<<4) & 0x07FF);
	sbus_channel[13] = ((rec_sbus_data[18]>>7|rec_sbus_data[19]<<1|rec_sbus_data[20]<<9)& 0x07FF);
	sbus_channel[14] = ((rec_sbus_data[20]>>2|rec_sbus_data[21]<<6) & 0x07FF);
	sbus_channel[15] = ((rec_sbus_data[21]>>5|rec_sbus_data[22]<<3) & 0x07FF);



	SBUS_CH[0] = 995 - sbus_channel[0];
	SBUS_CH[1] = 998 - sbus_channel[1];
	SBUS_CH[2] = 1010 - sbus_channel[2];
	SBUS_CH[3] = 1003 - sbus_channel[3];
	
//	if(SBUS_CH[4] < 400 && SBUS_CH[4] > 100) sbus_channel[4] = 3;
//	else if(SBUS_CH[4] > 600 && SBUS_CH[4] < 1400) sbus_channel[4] = 2;
//	else if(SBUS_CH[4] > 1600 && SBUS_CH[4] < 2000 ) sbus_channel[4] = 1;
//	else 
	SBUS_CH[4] = sbus_channel[4];
	
	
	SBUS_CH[5] = sbus_channel[5];
	SBUS_CH[6] = sbus_channel[6];
	SBUS_CH[7] = sbus_channel[7];
	SBUS_CH[8] = sbus_channel[8];
	SBUS_CH[9] = sbus_channel[9];
	SBUS_CH[10] = sbus_channel[10] - RC_SBUS_CH_VALUE_OFFSET;
	SBUS_CH[11] = sbus_channel[11] - RC_SBUS_CH_VALUE_OFFSET;
	
	SBUS_CH[12] = sbus_channel[12] - RC_SBUS_CH_VALUE_OFFSET;
	SBUS_CH[13] = sbus_channel[13] - RC_SBUS_CH_VALUE_OFFSET;
	SBUS_CH[14] = sbus_channel[14] - RC_SBUS_CH_VALUE_OFFSET;
  SBUS_CH[15] = sbus_channel[15] - RC_SBUS_CH_VALUE_OFFSET;
	
  RC_SBUS_data_is_error();
}

//取正函数
static int16_t RC_SBUS_abs(int16_t value)
{
    if (value > 0)
    {
        return value;
    }
    else
    {
        return -value;
    }
}



//判断遥控器数据是否出错，
uint8_t RC_SBUS_data_is_error(void)
{
    //使用了go to语句 方便出错统一处理遥控器变量数据归零
    if (RC_SBUS_abs(SBUS_CH[0]) > RC_SBUS_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_SBUS_abs(SBUS_CH[1]) > RC_SBUS_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_SBUS_abs(SBUS_CH[2]) > RC_SBUS_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
    if (RC_SBUS_abs(SBUS_CH[3]) > RC_SBUS_CHANNAL_ERROR_VALUE)
    {
        goto error;
    } 
			 if(SBUS_CH[4] < 400 && SBUS_CH[4] > 100) SBUS_CH[4] = 3;
	else if(SBUS_CH[4] > 600 && SBUS_CH[4] < 1400) SBUS_CH[4] = 2;
	else if(SBUS_CH[4] > 1600 && SBUS_CH[4] < 2000 ) SBUS_CH[4] = 1;
	else goto error;
		
			 if(SBUS_CH[5] < 600) SBUS_CH[5] = 1;
	else if(SBUS_CH[5] > 600 ) SBUS_CH[5] = 2;
	else goto error;
			 if(SBUS_CH[7] < 600) SBUS_CH[7] = 1;
	else if(SBUS_CH[7] > 600 ) SBUS_CH[7] = 2;
	else goto error;
		
//		if (RC_SBUS_abs(SBUS_CH[4]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }    
//		if (RC_SBUS_abs(SBUS_CH[5]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }    
//		if (RC_SBUS_abs(SBUS_CH[6]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }    
//		if (RC_SBUS_abs(SBUS_CH[7]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }    
//		if (RC_SBUS_abs(SBUS_CH[8]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }		
		if (RC_SBUS_abs(SBUS_CH[9]) > RC_SBUS_CHANNAL_ERROR_VALUE)
    {
        goto error;
    }
//		if (RC_SBUS_abs(SBUS_CH[10]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }
//		if (RC_SBUS_abs(SBUS_CH[11]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }
//		if (RC_SBUS_abs(SBUS_CH[12]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }		
//		if (RC_SBUS_abs(SBUS_CH[13]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }		
//		if (RC_SBUS_abs(SBUS_CH[14]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }		
//		if (RC_SBUS_abs(SBUS_CH[15]) > RC_SBUS_CHANNAL_ERROR_VALUE)
//    {
//        goto error;
//    }
		
		

    return 0;

error:
	SBUS_CH[0] = 0;
	SBUS_CH[1] = 0;
	SBUS_CH[2] = 0;
	SBUS_CH[3] = 0;
	SBUS_CH[4] = 0;
	SBUS_CH[5] = 0;
	SBUS_CH[6] = 0;
	SBUS_CH[7] = 0;
	SBUS_CH[8] = 0;
	SBUS_CH[9] = 0;
	SBUS_CH[10] = 0;
	SBUS_CH[11] = 0;
	SBUS_CH[12] = 0;
	SBUS_CH[13] = 0;
	SBUS_CH[14] = 0;
  SBUS_CH[15] = 0;
    return 1;
}



