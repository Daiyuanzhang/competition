#include "FrictionMoterPWM.h"
#include "stm32f4xx.h"


//PD12��PD13��PD14��PD15��PH10��PH11��PH12��PI0
void TIM4_PWM_Init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE);  
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE); 	


	GPIO_PinAFConfig(GPIOD,GPIO_PinSource12, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource13, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource14, GPIO_AF_TIM4);
	GPIO_PinAFConfig(GPIOD,GPIO_PinSource15, GPIO_AF_TIM4);


	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;            
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;       
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;      
	GPIO_Init(GPIOD,&GPIO_InitStructure);            


	TIM_TimeBaseStructure.TIM_Prescaler = 84-1;  //0.1MHZ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; 
	TIM_TimeBaseStructure.TIM_Period = 50000-1;  //50HZ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);


	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_OCInitStructure.TIM_Pulse = 1000;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);   
	TIM_OC2Init(TIM4, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);   
	TIM_OC4Init(TIM4, &TIM_OCInitStructure);   

	 
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable); 
	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable); 

	TIM_ARRPreloadConfig(TIM4,ENABLE);

	TIM_Cmd(TIM4, ENABLE);  	
}


void TIM5_PWM_Init()
{		 					 

	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIMʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH | RCC_AHB1Periph_GPIOI, ENABLE); 	//ʹ��PORThʱ��	
	
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5); 
	GPIO_PinAFConfig(GPIOI,GPIO_PinSource0,GPIO_AF_TIM5); 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;        //
	GPIO_Init(GPIOH,&GPIO_InitStructure);              //��ʼ��PF9
	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;          
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;        //
	GPIO_Init(GPIOI,&GPIO_InitStructure);              //��ʼ��PF9
	
	TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=50000-1;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//��ʼ����ʱ��14
	
	//��ʼ��TIM5 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
	TIM_OC2Init(TIM5, &TIM_OCInitStructure);  
	TIM_OC3Init(TIM5, &TIM_OCInitStructure);  
	TIM_OC4Init(TIM5, &TIM_OCInitStructure);  

	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  
  TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);
	
  TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM14	

}  

//PI5��PI6��PI7��PI2
void TIM8_PWM_Init()
{		 					 
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  	//TIMʱ��ʹ��    
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE); 	//ʹ��PORThʱ��	

GPIO_PinAFConfig(GPIOI, GPIO_PinSource5, GPIO_AF_TIM8); 
GPIO_PinAFConfig(GPIOI, GPIO_PinSource6, GPIO_AF_TIM8); 
GPIO_PinAFConfig(GPIOI, GPIO_PinSource7, GPIO_AF_TIM8); 
GPIO_PinAFConfig(GPIOI, GPIO_PinSource2, GPIO_AF_TIM8); 

GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;          
GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;        //
GPIO_Init(GPIOI,&GPIO_InitStructure);              //��ʼ��PF9

TIM_TimeBaseStructure.TIM_Prescaler=84-1;  //��ʱ����Ƶ
TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
TIM_TimeBaseStructure.TIM_Period=10000-1;   //�Զ���װ��ֵ
TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
TIM_TimeBaseStructure.TIM_RepetitionCounter=0;  //
TIM_TimeBaseInit(TIM8,&TIM_TimeBaseStructure);//��ʼ����ʱ��

//��ʼ��TIM5 PWMģʽ	 
TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2
TIM_OCInitStructure.TIM_Pulse=200;
TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ���
TIM_OCInitStructure.TIM_OCIdleState = TIM_OCNIdleState_Reset;
TIM_OCInitStructure.TIM_OutputNState = TIM_OutputState_Enable;
TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCPolarity_High;
TIM_OCInitStructure.TIM_OCNIdleState = TIM_OutputNState_Disable; 
TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������TIM1 4OC1
TIM_OC2Init(TIM8, &TIM_OCInitStructure);  
TIM_OC3Init(TIM8, &TIM_OCInitStructure);  
TIM_OC4Init(TIM8, &TIM_OCInitStructure);  

TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //ʹ��TIM5��CCR1�ϵ�Ԥװ�ؼĴ���
TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  
TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  
TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);

TIM_ARRPreloadConfig(TIM8,ENABLE);//ARPEʹ�� 
TIM_CtrlPWMOutputs(TIM8,ENABLE);
TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM14	
}  



void steering_engine_on(void)
{
    TIM_SetCompare1(TIM5, 20000-500);
    TIM_SetCompare2(TIM5, 1200-800);
}
void steering_engine_off(void)
{
    TIM_SetCompare1(TIM5,  20000-2000);
   TIM_SetCompare2(TIM5,  20000-500);
}

void fric_off(void)
{
    TIM_SetCompare1(TIM2, Fric_OFF);
    TIM_SetCompare2(TIM2, Fric_OFF);
}
void fric1_on(uint16_t cmd)
{
    TIM_SetCompare1(TIM2, cmd);
}
void fric2_on(uint16_t cmd)
{
    TIM_SetCompare2(TIM2, cmd);
}


