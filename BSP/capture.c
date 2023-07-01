#include "capture.h"
#include "sys.h"
#include "timer.h"

//PA0 PA1 PA2 PA3输入捕获
void TIM2_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM5时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA0
	

 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2); //PA0复用位定时器5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
  TIM_ICInitTypeDef TIM2_ICInitStructure;
 
	//初始化TIM5输入捕获参数
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;//CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
 

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3;//CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4;//CC1S=01 	选择输入端 IC1映射到TI1上
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	
		
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
	TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_CC3,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_CC4,ENABLE);
  TIM_Cmd(TIM2,ENABLE ); 	//使能定时器5
 
 
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
}



//捕获状态
//[7]:0,没有成功的捕获;1,成功捕获到一次.
//[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
//[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
u8  TIM2CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM2CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
u8  TIM2CH2_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM2CH2_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
u8  TIM2CH3_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM2CH3_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
u8  TIM2CH4_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM2CH4_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
 
//定时器2中断服务程序	 
void TIM2_IRQHandler(void)
{ 		    
 			        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
              TIM_ClearFlag(TIM2, TIM_FLAG_Update);  
	
 	if((TIM2CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM2CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM2CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM2CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					TIM2CH1_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH1_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
		{	
			if(TIM2CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM2CH1_CAPTURE_VAL=TIM_GetCapture1(TIM2);//获取当前的捕获值.
	 			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH1_CAPTURE_STA=0;			//清空
				TIM2CH1_CAPTURE_VAL=0;
				TIM2CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM_Cmd(TIM2,DISABLE ); 	//关闭定时器5
	 			TIM_SetCounter(TIM2,0);
	 			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM2,ENABLE ); 	//使能定时器5
			}		    
		}			     	    					   
 	}
 
	 	if((TIM2CH2_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM2CH2_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM2CH2_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM2CH2_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					TIM2CH2_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH2_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
		{	
			if(TIM2CH2_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH2_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM2CH2_CAPTURE_VAL=TIM_GetCapture2(TIM2);//获取当前的捕获值.
	 			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH2_CAPTURE_STA=0;			//清空
				TIM2CH2_CAPTURE_VAL=0;
				TIM2CH2_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM_Cmd(TIM2,DISABLE ); 	//关闭定时器5
	 			TIM_SetCounter(TIM2,0);
	 			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM2,ENABLE ); 	//使能定时器5
			}		    
		}			     	    					   
 	}
	 	if((TIM2CH3_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM2CH3_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM2CH3_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM2CH3_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					TIM2CH3_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH3_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)//捕获1发生捕获事件
		{	
			if(TIM2CH3_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH3_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM2CH3_CAPTURE_VAL=TIM_GetCapture3(TIM2);//获取当前的捕获值.
	 			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH3_CAPTURE_STA=0;			//清空
				TIM2CH3_CAPTURE_VAL=0;
				TIM2CH3_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM_Cmd(TIM2,DISABLE ); 	//关闭定时器5
	 			TIM_SetCounter(TIM2,0);
	 			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM2,ENABLE ); 	//使能定时器5
			}		    
		}			     	    					   
 	}
	 	if((TIM2CH4_CAPTURE_STA&0X80)==0)//还未成功捕获	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//溢出
		{	     
			if(TIM2CH4_CAPTURE_STA&0X40)//已经捕获到高电平了
			{
				if((TIM2CH4_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
				{
					TIM2CH4_CAPTURE_STA|=0X80;		//标记成功捕获了一次
					TIM2CH4_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH4_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)//捕获1发生捕获事件
		{	
			if(TIM2CH4_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
			{	  			
				TIM2CH4_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
			  TIM2CH4_CAPTURE_VAL=TIM_GetCapture4(TIM2);//获取当前的捕获值.
	 			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
			}else  								//还未开始,第一次捕获上升沿
			{
				TIM2CH4_CAPTURE_STA=0;			//清空
				TIM2CH4_CAPTURE_VAL=0;
				TIM2CH4_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
				TIM_Cmd(TIM2,DISABLE ); 	//关闭定时器5
	 			TIM_SetCounter(TIM2,0);
	 			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				TIM_Cmd(TIM2,ENABLE ); 	//使能定时器5
			}		    
		}			     	    					   
 	}
	
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1|TIM_IT_Update|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4); //清除中断标志位
	
}




void GetCaptureVal(long long *CH1, long long *CH2, long long *CH3, long long *CH4)
{
long long temp1 = 0;
long long temp2 = 0;
long long temp3 = 0;
long long temp4 = 0;
	
//	*CH1 = temp1;
//	*CH2 = temp2;
//	*CH3 = temp3;
//	*CH4 = temp4;
	
	//通道1
		if(TIM2CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp1=TIM2CH1_CAPTURE_STA&0X3F; 
			temp1*=0XFFFFFFFF;		 		         //溢出时间总和
			temp1+=TIM2CH1_CAPTURE_VAL;		   //得到总的高电平时间
			TIM2CH1_CAPTURE_STA=0;			     //开启下一次捕获
			*CH1 = temp1;
		}
//通道2
		if(TIM2CH2_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp2=TIM2CH2_CAPTURE_STA&0X3F; 
			temp2*=0XFFFFFFFF;		 		         //溢出时间总和
			temp2+=TIM2CH2_CAPTURE_VAL;		   //得到总的高电平时间
			TIM2CH2_CAPTURE_STA=0;			     //开启下一次捕获
			*CH2 = temp2;
		}
//通道3
		if(TIM2CH3_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp3=TIM2CH3_CAPTURE_STA&0X3F; 
			temp3*=0XFFFFFFFF;		 		         //溢出时间总和
			temp3+=TIM2CH3_CAPTURE_VAL;		   //得到总的高电平时间
			TIM2CH3_CAPTURE_STA=0;			     //开启下一次捕获
			*CH3 = temp3;
		}
//通道4
		if(TIM2CH4_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
		{
			temp4=TIM2CH4_CAPTURE_STA&0X3F; 
			temp4*=0XFFFFFFFF;		 		         //溢出时间总和
			temp4+=TIM2CH4_CAPTURE_VAL;		   //得到总的高电平时间
			TIM2CH4_CAPTURE_STA=0;			     //开启下一次捕获
			*CH4 = temp4;
		}

}


