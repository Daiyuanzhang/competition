#include "IO_Function_Init.h"
#include "sys.h"
#include "hand_task.h"
/*************************
������������������PD12
�����������PI0
�������λ����Ϣ��ȡ����PA0
**************************/
/******************************************���ڹ�翪��***********************************************/


//PA5
//PC2��PC3��PC4��PC5��
//PE5��PE6
//PF1
//��ʼ�����ţ�����΢�����أ�
void IO_Read_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF, ENABLE);//ʹ��GPIOA,GPIOIʱ��
	GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
		
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
		
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6; // GPIO_Pin_0 |
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIO	
		
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1; //
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;//��ͨ����ģʽ
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
	GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIO
	
} 


/******************************************���ڿ�������***********************************************/
 //PD13��PD14��PD15
//��ʼ�����ţ����ڿ������ף�
/*************************************



**************************************/
void IO_Out_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOF, &GPIO_InitStructure);//��ʼ��GPIO
  GPIO_ResetBits(GPIOF, GPIO_Pin_0);//GPIO���õ͵�ƽ 

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOE, &GPIO_InitStructure);//��ʼ��GPIO
  GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_12);//GPIO���õ͵�ƽ

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_1 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��GPIO
  GPIO_ResetBits(GPIOB, GPIO_Pin_8 |GPIO_Pin_4|GPIO_Pin_1);//GPIO���õ͵�ƽ
	

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOC, &GPIO_InitStructure);//��ʼ��GPIO
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1);//GPIO���õ͵�ƽ
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);//GPIO���õ͵�ƽ
}


/****************************����У׼���ƽ��λ��ʱ�ı���ֵ************************************/
    //�ⲿ�ж�PA0
//void Set_Hand_Offset(void)
//{
//  //ʹ��GPIO�Ĺ�����
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//ʹ�� SYSCFG ʱ��
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);//PI2 ���ӵ��ж���2
//	
//	//��ΪNVIC�����ù���
//	NVIC_InitTypeDef   NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //EXTIx �ж�ͨ��
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //��ռ���ȼ�	
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; //��Ӧ���ȼ�	
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��	
//	NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��NVIC�Ĵ���
//	
//	//��Ϊ��ʼ��EXTI��ѡ�񴥷���ʽ
//	EXTI_InitTypeDef   EXTI_InitStructure;
//	EXTI_InitStructure.EXTI_Line    = EXTI_Line0; //����EXTI�ж���
//	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt; //����EXTIģʽΪ�ж�
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising;//EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling; //����EXTI������ʽ
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE; //ʹ��EXTI�ж���
//	EXTI_Init(&EXTI_InitStructure); //����ָ���Ĳ�����ʼ��EXTI�Ĵ���
//	
//}

/*********************����ж�У׼����ֵЧ�����ѣ�ʹ�ô˴���**********************
void Hand_Link_to_Balance()
{
	char IO_current_state = 0, IO_last_state = 0, end_if_flag = 0;
		IO_current_state = Hand_link_Level_Position;
	
		if(end_if_flag == 0)  IO_last_state = IO_current_state, end_if_flag = 1;
    if(IO_last_state != IO_current_state)
		{
		 hand_control.hand_link_motor.offset_ecd  =  hand_control.hand_link_motor.hand_motor_measure->ecd ;
		 IO_last_state = IO_current_state;//�����������ֵ����
		}

}
**********************************************************************************/









