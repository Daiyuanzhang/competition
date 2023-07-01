#include "IO_Function_Init.h"
#include "sys.h"
#include "hand_task.h"
/*************************
步进电机脉冲输出引脚PD12
方向控制引脚PI0
步进电机位置信息读取引脚PA0
**************************/
/******************************************用于光电开关***********************************************/


//PA5
//PC2、PC3、PC4、PC5、
//PE5、PE6
//PF1
//初始化引脚（用于微动开关）
void IO_Read_Init(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_GPIOF, ENABLE);//使能GPIOA,GPIOI时钟
	GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_5; 
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
		
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3| GPIO_Pin_4| GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
		
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_5 | GPIO_Pin_6; // GPIO_Pin_0 |
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO	
		
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_1; //
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN;//普通输入模式
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100M
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
	GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIO
	
} 


/******************************************用于控制气缸***********************************************/
 //PD13、PD14、PD15
//初始化引脚（用于控制气缸）
/*************************************



**************************************/
void IO_Out_Init(void)
{    	 
  GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB|RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF, ENABLE);

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOF, &GPIO_InitStructure);//初始化GPIO
  GPIO_ResetBits(GPIOF, GPIO_Pin_0);//GPIO设置低电平 

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4 | GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOE, &GPIO_InitStructure);//初始化GPIO
  GPIO_ResetBits(GPIOE, GPIO_Pin_4 | GPIO_Pin_12);//GPIO设置低电平

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8 | GPIO_Pin_1 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化GPIO
  GPIO_ResetBits(GPIOB, GPIO_Pin_8 |GPIO_Pin_4|GPIO_Pin_1);//GPIO设置低电平
	

  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOC, &GPIO_InitStructure);//初始化GPIO
	GPIO_ResetBits(GPIOC, GPIO_Pin_0 | GPIO_Pin_1);//GPIO设置低电平
	
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//普通输出模式
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//下拉
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
	GPIO_ResetBits(GPIOA, GPIO_Pin_4);//GPIO设置低电平
}


/****************************用于校准电机平衡位置时的编码值************************************/
    //外部中断PA0
//void Set_Hand_Offset(void)
//{
//  //使能GPIO的过程略
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);//使能 SYSCFG 时钟
//  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);//PI2 连接到中断线2
//	
//	//下为NVIC的配置过程
//	NVIC_InitTypeDef   NVIC_InitStructure;
//	NVIC_InitStructure.NVIC_IRQChannel = EXTI0_IRQn; //EXTIx 中断通道
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x02; //抢占优先级	
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x00; //响应优先级	
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能	
//	NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化NVIC寄存器
//	
//	//下为初始化EXTI，选择触发方式
//	EXTI_InitTypeDef   EXTI_InitStructure;
//	EXTI_InitStructure.EXTI_Line    = EXTI_Line0; //配置EXTI中断线
//	EXTI_InitStructure.EXTI_Mode    = EXTI_Mode_Interrupt; //配置EXTI模式为中断
//	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;//EXTI_Trigger_Rising;//EXTI_Trigger_Falling;//EXTI_Trigger_Rising_Falling; //配置EXTI触发方式
//	EXTI_InitStructure.EXTI_LineCmd = ENABLE; //使能EXTI中断线
//	EXTI_Init(&EXTI_InitStructure); //根据指定的参数初始化EXTI寄存器
//	
//}

/*********************如果中断校准编码值效果不佳，使用此代码**********************
void Hand_Link_to_Balance()
{
	char IO_current_state = 0, IO_last_state = 0, end_if_flag = 0;
		IO_current_state = Hand_link_Level_Position;
	
		if(end_if_flag == 0)  IO_last_state = IO_current_state, end_if_flag = 1;
    if(IO_last_state != IO_current_state)
		{
		 hand_control.hand_link_motor.offset_ecd  =  hand_control.hand_link_motor.hand_motor_measure->ecd ;
		 IO_last_state = IO_current_state;//结束保存编码值条件
		}

}
**********************************************************************************/









