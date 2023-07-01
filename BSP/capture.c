#include "capture.h"
#include "sys.h"
#include "timer.h"

//PA0 PA1 PA2 PA3���벶��
void TIM2_Cap_Init(u32 arr,u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
 
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  	//TIM5ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
	
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3; //GPIOA0
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA0
	

 
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2); //PA0����λ��ʱ��5
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_TIM2);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_TIM2);
  
	  
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseStructure);
	
  TIM_ICInitTypeDef TIM2_ICInitStructure;
 
	//��ʼ��TIM5���벶�����
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2;//CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
 

	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_3;//CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	
	TIM2_ICInitStructure.TIM_Channel = TIM_Channel_4;//CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
  TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
  TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
  TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
  TIM_ICInit(TIM2, &TIM2_ICInitStructure);
	
	
		
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE);//��������ж� ,����CC1IE�����ж�	
	TIM_ITConfig(TIM2,TIM_IT_CC1,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_CC2,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_CC3,ENABLE);
	TIM_ITConfig(TIM2,TIM_IT_CC4,ENABLE);
  TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��5
 
 
  NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
}



//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
u8  TIM2CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM2CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
u8  TIM2CH2_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM2CH2_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
u8  TIM2CH3_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM2CH3_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
u8  TIM2CH4_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM2CH4_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
 
//��ʱ��2�жϷ������	 
void TIM2_IRQHandler(void)
{ 		    
 			        TIM_ClearITPendingBit(TIM2,TIM_IT_Update);
              TIM_ClearFlag(TIM2, TIM_FLAG_Update);  
	
 	if((TIM2CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM2CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM2CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM2CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					TIM2CH1_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH1_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM2CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM2CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM2CH1_CAPTURE_VAL=TIM_GetCapture1(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
	 			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH1_CAPTURE_STA=0;			//���
				TIM2CH1_CAPTURE_VAL=0;
				TIM2CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM_Cmd(TIM2,DISABLE ); 	//�رն�ʱ��5
	 			TIM_SetCounter(TIM2,0);
	 			TIM_OC1PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��5
			}		    
		}			     	    					   
 	}
 
	 	if((TIM2CH2_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM2CH2_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM2CH2_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM2CH2_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					TIM2CH2_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH2_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC2) != RESET)//����1���������¼�
		{	
			if(TIM2CH2_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM2CH2_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM2CH2_CAPTURE_VAL=TIM_GetCapture2(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
	 			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH2_CAPTURE_STA=0;			//���
				TIM2CH2_CAPTURE_VAL=0;
				TIM2CH2_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM_Cmd(TIM2,DISABLE ); 	//�رն�ʱ��5
	 			TIM_SetCounter(TIM2,0);
	 			TIM_OC2PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��5
			}		    
		}			     	    					   
 	}
	 	if((TIM2CH3_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM2CH3_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM2CH3_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM2CH3_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					TIM2CH3_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH3_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC3) != RESET)//����1���������¼�
		{	
			if(TIM2CH3_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM2CH3_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM2CH3_CAPTURE_VAL=TIM_GetCapture3(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
	 			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH3_CAPTURE_STA=0;			//���
				TIM2CH3_CAPTURE_VAL=0;
				TIM2CH3_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM_Cmd(TIM2,DISABLE ); 	//�رն�ʱ��5
	 			TIM_SetCounter(TIM2,0);
	 			TIM_OC3PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��5
			}		    
		}			     	    					   
 	}
	 	if((TIM2CH4_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{
		if(TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//���
		{	     
			if(TIM2CH4_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM2CH4_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM2CH4_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
					TIM2CH4_CAPTURE_VAL=0XFFFFFFFF;
				}else TIM2CH4_CAPTURE_STA++;
			}	 
		}
		if(TIM_GetITStatus(TIM2, TIM_IT_CC4) != RESET)//����1���������¼�
		{	
			if(TIM2CH4_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM2CH4_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
			  TIM2CH4_CAPTURE_VAL=TIM_GetCapture4(TIM2);//��ȡ��ǰ�Ĳ���ֵ.
	 			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM2CH4_CAPTURE_STA=0;			//���
				TIM2CH4_CAPTURE_VAL=0;
				TIM2CH4_CAPTURE_STA|=0X40;		//��ǲ�����������
				TIM_Cmd(TIM2,DISABLE ); 	//�رն�ʱ��5
	 			TIM_SetCounter(TIM2,0);
	 			TIM_OC4PolarityConfig(TIM2,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
				TIM_Cmd(TIM2,ENABLE ); 	//ʹ�ܶ�ʱ��5
			}		    
		}			     	    					   
 	}
	
		TIM_ClearITPendingBit(TIM2, TIM_IT_CC1|TIM_IT_Update|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4); //����жϱ�־λ
	
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
	
	//ͨ��1
		if(TIM2CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			temp1=TIM2CH1_CAPTURE_STA&0X3F; 
			temp1*=0XFFFFFFFF;		 		         //���ʱ���ܺ�
			temp1+=TIM2CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			TIM2CH1_CAPTURE_STA=0;			     //������һ�β���
			*CH1 = temp1;
		}
//ͨ��2
		if(TIM2CH2_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			temp2=TIM2CH2_CAPTURE_STA&0X3F; 
			temp2*=0XFFFFFFFF;		 		         //���ʱ���ܺ�
			temp2+=TIM2CH2_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			TIM2CH2_CAPTURE_STA=0;			     //������һ�β���
			*CH2 = temp2;
		}
//ͨ��3
		if(TIM2CH3_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			temp3=TIM2CH3_CAPTURE_STA&0X3F; 
			temp3*=0XFFFFFFFF;		 		         //���ʱ���ܺ�
			temp3+=TIM2CH3_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			TIM2CH3_CAPTURE_STA=0;			     //������һ�β���
			*CH3 = temp3;
		}
//ͨ��4
		if(TIM2CH4_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
		{
			temp4=TIM2CH4_CAPTURE_STA&0X3F; 
			temp4*=0XFFFFFFFF;		 		         //���ʱ���ܺ�
			temp4+=TIM2CH4_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
			TIM2CH4_CAPTURE_STA=0;			     //������һ�β���
			*CH4 = temp4;
		}

}


