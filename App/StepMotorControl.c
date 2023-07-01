#include "StepMotorControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RemotDbus.h"
#include "gimbal_task.h"
#include "BasicPeripherals.h"
#include "IO_Function_Init.h"
 /*****�����������������****/
 /*****���в�����ȫ��Ϊ��ˢֱ�����****/
	
	
u8  TIM5CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
fp32 step_motor_relative_degree = 0;  
uint32_t  position_pluse_set = 0, TIM_ARR_SET = 20000, MD_Pulse_Cnt = 0;
int pluses = 10, clear_MD_Pulse_Cnt = 0, TIM_arr = 600; 
uint8_t step_motor_init = 0;
step_motor_t Step_Motor_Set;

 fp32 DC_motor_position_test = 14.0f;
void StepMotorRelativeControl(fp32 step_motor_position_set, fp32 ecd)
{
	  Step_Motor_Set.DC_motor_stop = *DC_motor_stop_flag();  //��ȡ����ϵͳ�ĳ�ʼ״̬ ����dc�����ʼʱֹͣ���ȴ������ֲ�����

if(Step_Motor_Set.DC_motor_stop == 0) {TIM_SetCompare3(TIM5, 0);  return;}//��ʼ̬Ϊ�㣬��ֹ���ƶ���pwm���
if(rc_ctrl.rc.s[0] == 2 ) return; //����������ģʽ��ֱ�ӷ��أ����ڽ����ƶ�
if(step_motor_init == 0)
{
	Step_Motor_Set.setp_motor_pid.Kp = STEP_MOTOR_SPEED_PID_KP;
	Step_Motor_Set.setp_motor_pid.Ki = STEP_MOTOR_SPEED_PID_KI;
	Step_Motor_Set.setp_motor_pid.Kd = STEP_MOTOR_SPEED_PID_KD;
	Step_Motor_Set.setp_motor_pid.max_out  = STEP_MOTOR_SPEED_PID_MAX_OUT;
	Step_Motor_Set.setp_motor_pid.max_iout = STEP_MOTOR_SPEED_PID_MAX_IOUT;
	Step_Motor_Set.setp_motor_pid.mode     = PID_POSITION;
	Step_Motor_Set.relative_angle_set = 14.0f;
  step_motor_init = 1;
	Step_Motor_Set.min_angle = MIN_ANGLE;
}
//��¼115��	
//	Step_Motor_Set.relative_angle_set = DC_motor_position_test;  //���Դ��룬�����趨�ĽǶ�



	Step_Motor_Set.relative_angle_set = *return_DC_motor_position_set();  


	  Step_Motor_Set.relative_angel = step_motor_ecd_to_angle_change(ecd,0);
    Step_Motor_Set.speed = PID_Calc(&Step_Motor_Set.setp_motor_pid,
		Step_Motor_Set.relative_angel, Step_Motor_Set.relative_angle_set);
	  Step_Motor_Set.speed_out = (int)(degree_abs)(Step_Motor_Set.speed);


	  Step_Motor_Set.Real_DegreeError = Step_Motor_Set.relative_angle_set - Step_Motor_Set.relative_angel;
//�����޶�    min_angleΪ��ת����С���       MIN_ANGLEΪ������ת����С���
   	if(degree_abs(Step_Motor_Set.Real_DegreeError) > MIN_ANGLE)  Step_Motor_Set.min_angle = MIN_ANGLE_OFFSET;


      if(Step_Motor_Set.DC_motor_stop == 0) {TIM_SetCompare3(TIM5, 0);  return;}//��ʼ̬Ϊ�㣬��ֹ���ƶ���pwm���
			if(degree_abs(Step_Motor_Set.Real_DegreeError) > Step_Motor_Set.min_angle)
			{			
					if(Step_Motor_Set.speed_out < DC_MIN_OUTPUT)  Step_Motor_Set.speed_out = DC_MIN_OUTPUT;  //�����ת����С�������
					if(Step_Motor_Set.Real_DegreeError > 0)
					{
							#if StepMotorTurn		 //��������������������ֵ��180���ң�����Ϊ�����ˣ�
									 DC_MOTOR_UP;    //˳ʱ����ת
							#else		
									 DC_MOTOR_DOWN; //��ʱ����ת
							#endif

					}
					else if(Step_Motor_Set.Real_DegreeError < 0)
					{
							#if StepMotorTurn		
										DC_MOTOR_DOWN; //��ʱ����ת
							#else		
									 DC_MOTOR_UP;    //˳ʱ����ת
							#endif
								
					}	
          TIM_SetCompare3(TIM5,  Step_Motor_Set.speed_out);						
	 }
   else
	{
		  Step_Motor_Set.min_angle = MIN_ANGLE;
			TIM_SetCompare3(TIM5, 0);
			return;	
	}	 

}
/**********************************************************************************************************
u8  TIM5CH1_CAPTURE_STA=0;	//���벶��״̬		    				
u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
fp32 step_motor_relative_degree = 0;  
 uint32_t  position_pluse_set = 0, TIM_ARR_SET = 20000, MD_Pulse_Cnt = 0;
 int pluses = 10, clear_MD_Pulse_Cnt = 0, TIM_arr = 600; 
uint8_t step_motor_init = 0;
step_motor_t Step_Motor_Set;


//u16 temp_arr=1000000/frequency-1; 
void StepMotorRelativeControl(fp32 step_motor_position_set, fp32 ecd)
{
if(rc_ctrl.rc.s[0] == 2) return; //����������ģʽ��ֱ�ӷ��أ����ڽ����ƶ�
if(step_motor_init == 0)
{
	Step_Motor_Set.setp_motor_pid.Kp = STEP_MOTOR_SPEED_PID_KP;
	Step_Motor_Set.setp_motor_pid.Ki = STEP_MOTOR_SPEED_PID_KI;
	Step_Motor_Set.setp_motor_pid.Kd = STEP_MOTOR_SPEED_PID_KD;
	Step_Motor_Set.setp_motor_pid.max_out  = STEP_MOTOR_SPEED_PID_MAX_OUT;
	Step_Motor_Set.setp_motor_pid.max_iout = STEP_MOTOR_SPEED_PID_MAX_IOUT;
	Step_Motor_Set.setp_motor_pid.mode     = PID_POSITION;
	Step_Motor_Set.relative_angle_set = 18;
  step_motor_init = 1;
}

	fp32 step_motor_position = StepMotor_init_degreen;
	step_motor_position = *return_step_motor_position();
	Step_Motor_Set.relative_angle_set = step_motor_position;
	
//	if(set_arr_end == 0)  TIM_SetAutoreload(TIM4, 600), set_arr_end = 1;
 // degree_error = step_motor_position*Rad_TO_DEGRE - step_motor_relative_degree;
 // degree_error = step_motor_position_set - step_motor_relative_degree;
	  Step_Motor_Set.relative_angel = step_motor_ecd_to_angle_change(ecd,0);
	
//	 degree_error = step_motor_position_set - current_angle;
    Step_Motor_Set.speed = PID_Calc(&Step_Motor_Set.setp_motor_pid,
		Step_Motor_Set.relative_angel, Step_Motor_Set.relative_angle_set);
	  Step_Motor_Set.speed_out = (int)(Step_Motor_Set.speed);
	  Step_Motor_Set.frequency = (uint16_t)(1000000/Step_Motor_Set.speed_out-1); //���ٶ�ת��ΪƵ�����


	  Step_Motor_Set.Real_DegreeError = Step_Motor_Set.relative_angle_set - Step_Motor_Set.relative_angel;
	if(degree_abs(Step_Motor_Set.Real_DegreeError) < MIN_ANGLE)
		{
			TIM_SetCompare3(TIM5, 0);
			return;	
		}

if(Step_Motor_Set.frequency < MIN_FREQUENCY)  Step_Motor_Set.frequency = MIN_FREQUENCY;
//else if(Step_Motor_Set.frequency < -MIN_FREQUENCY)  Step_Motor_Set.frequency = -MIN_FREQUENCY;

		if(Step_Motor_Set.Real_DegreeError > 0)
		{
				#if StepMotorTurn		 //��������������������ֵ��180���ң�����Ϊ�����ˣ�
						 DC_MOTOR_UP;    //˳ʱ����ת
				#else		
						DC_MOTOR_DOWN; //��ʱ����ת
				#endif

		}
		else if(Step_Motor_Set.Real_DegreeError < 0)
		{
				#if StepMotorTurn		
							DC_MOTOR_DOWN; //��ʱ����ת
				#else		
						 DC_MOTOR_UP;    //˳ʱ����ת
				#endif
					
		}

    TIM_SetAutoreload(TIM5, Step_Motor_Set.frequency);
    TIM_SetCompare3(TIM5,  Step_Motor_Set.frequency/2);	

	}
*********************************************************************************************************/



/*****************************************************
	  degree_error = Step_Motor_Set.frequency;

	if(degree_error > 0.0f && degree_error < MIN_ANGLE) //�Ƕ�Ϊ��ֵ����������Χ��
	{
	  degree_error = 0;
	}
	else if(degree_error < 0.0f && degree_error > (-MIN_ANGLE))//�Ƕ�Ϊ��ֵ����������Χ��
	{
	  degree_error = 0;
	}
	pluses = (int)(degree_error*DegreeToPluse);  //���趨λ���뷴��λ�õ����ת��Ϊ����



	if(degree_error == 0)  
	{
		TIM_SetCompare1(TIM4, 0);	
	}
	else if(degree_error > 0.0f)  //�ж�˳ʱ����ת
	{ 
#if StepMotorTurn		//��������������������ֵ��180���ң�����Ϊ�����ˣ�
		 StepMotor_Clockwise;    //˳ʱ����ת
#else		
		StepMotor_Anticlockwise; //��ʱ����ת
#endif
		
    if(clear_MD_Pulse_Cnt == 0) MD_Pulse_Cnt = 0, clear_MD_Pulse_Cnt = 1;
		if(MD_Pulse_Cnt < pluses + over_pluse)
		{
		  TIM_SetCompare1(TIM4, TIM_arr);		
		}
		else 
		{  
			clear_MD_Pulse_Cnt = 0;
			TIM_SetCompare1(TIM4, 0);	
		}
	}
	else if(degree_error < 0.0f) //�ж���ʱ����ת
	{
		
#if StepMotorTurn		
			StepMotor_Anticlockwise; //��ʱ����ת
#else		
	   StepMotor_Clockwise;    //˳ʱ����ת
#endif
		
    if(clear_MD_Pulse_Cnt == 0) MD_Pulse_Cnt = 0, clear_MD_Pulse_Cnt = 1;
		if(MD_Pulse_Cnt < (-pluses - over_pluse))
		{
		  TIM_SetCompare1(TIM4, TIM_arr);	
		}
		else 
		{  
			clear_MD_Pulse_Cnt = 0;
			TIM_SetCompare1(TIM4, 0);	
		}
	}
	else 
	{
		TIM_SetCompare1(TIM4, 0);
	}
	
	*********************************************/
  // TIM_SetAutoreload(TIM4,Step_Motor_ARR-1);                       




double degree_abs( double value)
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

static fp32 step_motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Extra_Half_ecd_range)
    {
        relative_ecd -= Extra_ecd_range;
    }
    else if (relative_ecd < -Extra_Half_ecd_range)
    {
        relative_ecd += Extra_ecd_range;
    }

    return relative_ecd *EXTRA_ECD_TO_DEGREE;
}



void StepMotor_PWM_Init(u32 arr , u32 psc) 
		{
/*****************************���ڿ��Ʋ�������ķ���*******************************************/		
//	GPIO_InitTypeDef           GPIOI_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
//  GPIOI_InitStructure.GPIO_Pin   = GPIO_Pin_0;
//  GPIOI_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//��ͨ���ģʽ
//  GPIOI_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIOI_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
//  GPIO_Init(GPIOI, &GPIOI_InitStructure);//��ʼ��GPIO
//			 
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//  GPIOI_InitStructure.GPIO_Pin   = GPIO_Pin_13|GPIO_Pin_14;
//  GPIOI_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//��ͨ���ģʽ
//  GPIOI_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
//  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIOI_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
//  GPIO_Init(GPIOD, &GPIOI_InitStructure);//��ʼ��GPIO
	
			
/***********************************************************************************************/			
			
	GPIO_InitTypeDef           GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	TIM_OCInitTypeDef          TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
			
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);//| GPIO_PinSource13 | GPIO_PinSource14|GPIO_PinSource15,GPIO_AF_TIM4); 

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;//|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;           //����GPIOD12
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOD,&GPIO_InitStructure);   

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//��ʼ����ʱ��1

	//��ʼ��TIM4 Channel 2/3 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2 //ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��,Ҳ����ʹ�� PWM ������˿�
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); //����Tָ���Ĳ�����ʼ������TIM4 OC1
	
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //����ʱ��4�����ж�
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
/******************************************************************************************************/

		}

void PWM_GPIOH_Init(u32 arr, u32 psc)
{
	/*****************************���ڿ��Ʋ�������ķ���*******************************************/		
	GPIO_InitTypeDef           GPIOI_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
  GPIOI_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIOI_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//��ͨ���ģʽ
  GPIOI_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIOI_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//����
  GPIO_Init(GPIOI, &GPIOI_InitStructure);//��ʼ��GPIO
	
	GPIO_InitTypeDef          GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
/********************************************************************************************************/
    //PH10 ��11��12 PWM��ʼ��     PH11��PH12Ϊ����
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM1ʱ��ʹ��    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); //ʹ��GPIOAʱ��	
//  GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5); 
//	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5); 	//Ԥ������
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5); 	//Ԥ������
			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           //����GPIOD12  GPIO_Pin_10|GPIO_Pin_11|
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //����
	GPIO_Init(GPIOH,&GPIO_InitStructure);  //��ʼ��PA9	
		
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//��ʼ����ʱ��1
	

	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//��ʼ����ʱ��5
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2 //ģʽ1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��,Ҳ����ʹ�� PWM ������˿�
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ե�
//	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //PA1����ʹ�ö�ʱ��5��ͨ��1//����Tָ���Ĳ�����ʼ������TIM1 OC2
//	TIM_OC2Init(TIM5, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM5, &TIM_OCInitStructure); 

	
//	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //ʹ��TIM1��CCR1�ϵ�Ԥװ�ؼĴ���
//	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //Ԥ������
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //Ԥ������
	
	TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPEʹ�� 
	TIM_Cmd(TIM5, ENABLE);  //ʹ��TIM5
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);//ʹ��GPIOFʱ��
	
/*************************************************************************************************************/

}	
	

//void StepMotorRelativeControl(fp32 step_motor_position_set)
//{
// // degree_error = step_motor_position*Rad_TO_DEGRE - step_motor_relative_degree;
//  degree_error = step_motor_position_set - step_motor_relative_degree;
//	if(degree_error > 0.0f && degree_error < ONE_CIRCLE)  //�ж�˳ʱ����ת
//	{ 
//		StepMotor_Anticlockwise; //��ʱ����ת
////	   StepMotor_Clockwise;    //˳ʱ����ת
//		if(degree_error > MIN_ANGLE)                          
//		{
////			 TIM_SetAutoreload(TIM4, TIM_ARR_SET);    
//			 TIM_SetCompare1(TIM4, 600);
//			 run1++;
//		}       
//		else if(degree_error < MIN_ANGLE)
//		{
////			 TIM_SetAutoreload(TIM4, 0);    
//       TIM_SetCompare1(TIM4, 0);
//		} 
//	}
//	else if(degree_error < 0.0f && degree_error > -ONE_CIRCLE) //�ж���ʱ����ת
//	{
//		 StepMotor_Clockwise;    //˳ʱ����ת
////	   StepMotor_Anticlockwise; //��ʱ����ת
//	   if(degree_error < -MIN_ANGLE)                          
//		{
////			 TIM_SetAutoreload(TIM4, TIM_ARR_SET);    
//			 TIM_SetCompare1(TIM4, 600);
//			 run2++;
//		}       
//		else if(degree_error > -MIN_ANGLE)
//		{
////			 TIM_SetAutoreload(TIM4, 0);    
//       TIM_SetCompare1(TIM4, 0);
//		}                     
//	}
//	else 
//	{
//		TIM_SetCompare1(TIM4, 0);
//	}
//	
//	
//  // TIM_SetAutoreload(TIM4,Step_Motor_ARR-1);                         
//}



//int last_position = 0, count = 0, tutal = 0;
//void motor_position(void)
//{ 
//	long long temp=0;  
//	TIM5_CH1_Cap_Init(0XFFFFFFFF,84-1); //��1Mhz��Ƶ�ʼ���
//  	if(TIM5CH1_CAPTURE_STA&0X80)        //�ɹ�������һ�θߵ�ƽ
//		{
//				temp=TIM5CH1_CAPTURE_STA&0X3F; 
//				temp*=0XFFFFFFFF;		 		         //���ʱ���ܺ�
//				temp+=TIM5CH1_CAPTURE_VAL;		   //�õ��ܵĸߵ�ƽʱ��
//				TIM5CH1_CAPTURE_STA=0;			     //������һ�β���
//  			step_motor_relative_degree =  temp*PLUSE_TO_DEGREE;
//	      count++;
//			  tutal += temp;
//				if(count == 3)
//				{
//					step_motor_relative_degree =  (tutal/3)*PLUSE_TO_DEGREE;
//					count = 0;
//					tutal = 0;
//				}
////   step_motor_relative =  temp;
//		}

//}

//  int Step_Motor_ARR = 0, Step_Motor_CCR1 = 0;	//˳ʱ��ץ������
/*************************�����㷨 ���������������*************************/
//int  arr_pluse_motor = ARR_STEP_MOTOR,  step_motor_circles = PULSE_NUM, pulse_count = 0;
//char current_state = 0, last_state = 0, End_If_flag = 0, Start_MD_Pulse_Cnt;

//void StepMotor_Control(char  step_motor_position)
//{	   
//	if(Start_MD_Pulse_Cnt == 0) MD_Pulse_Cnt = 0, Start_MD_Pulse_Cnt = 1;//���д˺����ǽ���������
//		      LED_GREEN = !LED_GREEN;  //��ʾ��

///******************��ʼ��ʱ��У׼ץ�ָ���λ��******************************************************************/
//	     if(End_If_flag == 0) 
//			 {
//				 if(Read_Step_Motor_State == 1) //΢����������Ĭ��������״̬������΢������ʱΪ�͵�ƽ
//				 {
//					 StepMotor_Clockwise;//������  
//					 TIM_SetCompare1(TIM4, arr_pluse_motor);
//					 if(MD_Pulse_Cnt > CALE_PULSE_NUM) //����У׼ʱ�������ת�Ƕȣ���ֹ��΢�����س��������һֱ��ת
//					 {
//					 TIM_SetCompare1(TIM4, 0);
//					 last_state  = 1, End_If_flag = 1, MD_Pulse_Cnt = PULSE_NUM, pulse_count = PULSE_NUM;
//					 }
//					 return;   
//				 }
//				 else  last_state  = 1, End_If_flag = 1, MD_Pulse_Cnt = PULSE_NUM, pulse_count = PULSE_NUM;  //last_stateֵ��¼һ��
//			 }
///******************ץ�ָ���λ��У׼��ϣ�֮��ִ�д���֮��Ĵ���******************************************************************/ 
//			 else if(End_If_flag == 1)
//			 {
//	   //�����������ת�ж�
//			 if(step_motor_position == 1)      	StepMotor_Clockwise,     current_state = 1;
//  else if(step_motor_position == 0)  			StepMotor_Anticlockwise, current_state = 0;      //��ʱ������

//       if(last_state != current_state)  //�жϷ����Ƿ��л����������Ƿ����л�
//			 {
//				 if(pulse_count < step_motor_circles)  step_motor_circles =  pulse_count;
//				 else if(pulse_count >= step_motor_circles)  step_motor_circles = PULSE_NUM;
//			   arr_pluse_motor = ARR_STEP_MOTOR;//�����л������¸�arr_pluse_motor��ֵ	
//				 MD_Pulse_Cnt = 0;
//				 last_state  = current_state;//�����ϴη���״̬�����ڷ����л�ʱ���¸�arr_pluse_motor��ֵ	
//			 }
//	//��������ƣ�����MD_Pulse_Cnt��¼�˲������������
//       if (MD_Pulse_Cnt >= step_motor_circles)   arr_pluse_motor = 0;  	
//	         TIM_SetCompare1(TIM4, arr_pluse_motor);   
//			     pulse_count = MD_Pulse_Cnt;
//		  }
//}




////��ʱ��5ͨ��1���벶������
////arr���Զ���װֵ(TIM2,TIM5��32λ��!!)
////psc��ʱ��Ԥ��Ƶ��
//void TIM5_CH1_Cap_Init(u32 arr,u16 psc)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef   NVIC_InitStructure;
//  TIM_ICInitTypeDef  TIM5_ICInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5ʱ��ʹ��    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //GPIOA0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA0

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); //PA0����λ��ʱ��5
//  
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
//	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
//	

//	//��ʼ��TIM5���벶�����
//	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
//  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
//  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
//  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
//  TIM5_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
//  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
//		
//	TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC2,ENABLE);//��������ж� ,����CC1IE�����ж�	
//	
//  TIM_Cmd(TIM5,ENABLE ); 	//ʹ�ܶ�ʱ��5

// 
//  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//	
//	
//}

////��ʱ��5�жϷ������	 
//void TIM5_IRQHandler(void)
//{ 		    

// 	if((TIM5CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
//	{
//		if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//���
//		{	     
//			if(TIM5CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
//			{
//				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
//				{
//					TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
//					TIM5CH1_CAPTURE_VAL=0XFFFFFFFF;
//				}else TIM5CH1_CAPTURE_STA++;
//			}	 
//		}
//		
//		if(TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)//����1���������¼�
//		{	
//			if(TIM5CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
//			{	  			
//				
//				TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//			  TIM5CH1_CAPTURE_VAL=TIM_GetCapture2(TIM5);//��ȡ��ǰ�Ĳ���ֵ.
//	 			TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//			}else  								//��δ��ʼ,��һ�β���������
//			{
//				TIM5CH1_CAPTURE_STA=0;			//���
//				TIM5CH1_CAPTURE_VAL=0;
//				TIM5CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
//				TIM_Cmd(TIM5,DISABLE ); 	//�رն�ʱ��5
//	 			TIM_SetCounter(TIM5,0);
//	 			TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
//				TIM_Cmd(TIM5,ENABLE ); 	//ʹ�ܶ�ʱ��5
//			}		    
//		}			     	    					   
// 	}
//	TIM_ClearITPendingBit(TIM5, TIM_IT_CC2|TIM_IT_Update); //����жϱ�־λ
//}







//TIM_ICInitTypeDef  TIM5_ICInitStructure;
////��ʱ��5ͨ��1���벶������
////arr���Զ���װֵ(TIM2,TIM5��32λ��!!)
////psc��ʱ��Ԥ��Ƶ��
//void TIM5_CH1_Cap_Init(u32 arr,u16 psc)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5ʱ��ʹ��    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//ʹ��PORTAʱ��	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //GPIOA0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //����
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA0

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); //PA0����λ��ʱ��5
//  
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
//	TIM_TimeBaseStructure.TIM_Period=arr;   //�Զ���װ��ֵ
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
//	

//	//��ʼ��TIM5���벶�����
//	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
//  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
//  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
//  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
//  TIM5_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
//  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
//		
//	TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	
//	
//  TIM_Cmd(TIM5,ENABLE ); 	//ʹ�ܶ�ʱ��5

// 
//  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ�����
//	
//	
//}
////����״̬3F = 0011 1111
////[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.0x40
////[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
////[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
//u8  TIM5CH1_CAPTURE_STA=0;	//���벶��״̬		    				
//u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
////��ʱ��5�жϷ������	 
//void TIM5_IRQHandler(void)
//{ 		    

// 	if((TIM5CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
//	{
//		if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//���
//		{	     
//			if(TIM5CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
//			{
//				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
//				{
//					TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
//					TIM5CH1_CAPTURE_VAL=0XFFFFFFFF;
//				}else TIM5CH1_CAPTURE_STA++;
//			}	 
//		}
//		
//		if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)//����1���������¼�
//		{	
//			if(TIM5CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
//			{	  			
//				TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//			  TIM5CH1_CAPTURE_VAL=TIM_GetCapture1(TIM5);//��ȡ��ǰ�Ĳ���ֵ.
//	 			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
//			}else  								//��δ��ʼ,��һ�β���������
//			{
//				TIM5CH1_CAPTURE_STA=0;			//���
//				TIM5CH1_CAPTURE_VAL=0;
//				TIM5CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
//				TIM_Cmd(TIM5,DISABLE ); 	//�رն�ʱ��5
//	 			TIM_SetCounter(TIM5,0);
//	 			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
//				TIM_Cmd(TIM5,ENABLE ); 	//ʹ�ܶ�ʱ��5
//			}		    
//		}			     	    					   
// 	}
//	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ
//}




//void Serve_motor_Clockwise_90()
//{
//    TIM_SetCompare3(TIM5,1950); 
//}
//void	Serve_motor_Clockwise_180()
//{
//    TIM_SetCompare3(TIM5,2600); 
//}
//void	Serve_motor_Anticlockwise_90()
//{
//    TIM_SetCompare3(TIM5,550); 
//}
//void	Serve_motor_Run_To_Balance_Init()
//{
//    TIM_SetCompare3(TIM5,1250); 
//}
		


/***********************************���Դ���****************************************/
//void StepMotor_Control()
//{		
//							switch(  rc_ctrl.rc.s[1])//�󲦸�1
//			{
//				case RC_SW_UP   :  TIM_SetCompare1(TIM4, arr_pluse_motor/2);  	if (MD_Pulse_Cnt > step_motor_circles)  arr_pluse_motor = 0;   
//				break;
//				case RC_SW_MID  :  TIM_SetCompare1(TIM4, Step_Motor_CCR1);  MD_Pulse_Cnt = 0; arr_pluse_motor = 1200; break;
//				case RC_SW_DOWN :  TIM_SetCompare1(TIM4,0);break;
//			}
//							switch(  rc_ctrl.rc.s[0])//�Ҳ���0
//			{
//				case RC_SW_UP   :   StepMotor_Clockwise;
//				break;
//				case RC_SW_MID  :   StepMotor_Anticlockwise;
//				break;
//				case RC_SW_DOWN :  
//				break;
//			}
//	}