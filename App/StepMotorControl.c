#include "StepMotorControl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "RemotDbus.h"
#include "gimbal_task.h"
#include "BasicPeripherals.h"
#include "IO_Function_Init.h"
 /*****步进电机方案放弃了****/
 /*****带有步进的全部为有刷直流电机****/
	
	
u8  TIM5CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM5CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
fp32 step_motor_relative_degree = 0;  
uint32_t  position_pluse_set = 0, TIM_ARR_SET = 20000, MD_Pulse_Cnt = 0;
int pluses = 10, clear_MD_Pulse_Cnt = 0, TIM_arr = 600; 
uint8_t step_motor_init = 0;
step_motor_t Step_Motor_Set;

 fp32 DC_motor_position_test = 14.0f;
void StepMotorRelativeControl(fp32 step_motor_position_set, fp32 ecd)
{
	  Step_Motor_Set.DC_motor_stop = *DC_motor_stop_flag();  //获取整个系统的初始状态 （让dc电机初始时停止。等待操作手操作）

if(Step_Motor_Set.DC_motor_stop == 0) {TIM_SetCompare3(TIM5, 0);  return;}//初始态为零，禁止控制端有pwm输出
if(rc_ctrl.rc.s[0] == 2 ) return; //拨杆是无力模式，直接返回，用于紧急制动
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
//检录115度	
//	Step_Motor_Set.relative_angle_set = DC_motor_position_test;  //测试代码，测需设定的角度



	Step_Motor_Set.relative_angle_set = *return_DC_motor_position_set();  


	  Step_Motor_Set.relative_angel = step_motor_ecd_to_angle_change(ecd,0);
    Step_Motor_Set.speed = PID_Calc(&Step_Motor_Set.setp_motor_pid,
		Step_Motor_Set.relative_angel, Step_Motor_Set.relative_angle_set);
	  Step_Motor_Set.speed_out = (int)(degree_abs)(Step_Motor_Set.speed);


	  Step_Motor_Set.Real_DegreeError = Step_Motor_Set.relative_angle_set - Step_Motor_Set.relative_angel;
//死区限定    min_angle为旋转的最小误差       MIN_ANGLE为触发旋转的最小误差
   	if(degree_abs(Step_Motor_Set.Real_DegreeError) > MIN_ANGLE)  Step_Motor_Set.min_angle = MIN_ANGLE_OFFSET;


      if(Step_Motor_Set.DC_motor_stop == 0) {TIM_SetCompare3(TIM5, 0);  return;}//初始态为零，禁止控制端有pwm输出
			if(degree_abs(Step_Motor_Set.Real_DegreeError) > Step_Motor_Set.min_angle)
			{			
					if(Step_Motor_Set.speed_out < DC_MIN_OUTPUT)  Step_Motor_Set.speed_out = DC_MIN_OUTPUT;  //电机旋转的最小脉宽输出
					if(Step_Motor_Set.Real_DegreeError > 0)
					{
							#if StepMotorTurn		 //如果不进电机抖动（且数值在180左右）是因为方向反了，
									 DC_MOTOR_UP;    //顺时针旋转
							#else		
									 DC_MOTOR_DOWN; //逆时针旋转
							#endif

					}
					else if(Step_Motor_Set.Real_DegreeError < 0)
					{
							#if StepMotorTurn		
										DC_MOTOR_DOWN; //逆时针旋转
							#else		
									 DC_MOTOR_UP;    //顺时针旋转
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
u8  TIM5CH1_CAPTURE_STA=0;	//输入捕获状态		    				
u32	TIM5CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
fp32 step_motor_relative_degree = 0;  
 uint32_t  position_pluse_set = 0, TIM_ARR_SET = 20000, MD_Pulse_Cnt = 0;
 int pluses = 10, clear_MD_Pulse_Cnt = 0, TIM_arr = 600; 
uint8_t step_motor_init = 0;
step_motor_t Step_Motor_Set;


//u16 temp_arr=1000000/frequency-1; 
void StepMotorRelativeControl(fp32 step_motor_position_set, fp32 ecd)
{
if(rc_ctrl.rc.s[0] == 2) return; //拨杆是无力模式，直接返回，用于紧急制动
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
	  Step_Motor_Set.frequency = (uint16_t)(1000000/Step_Motor_Set.speed_out-1); //将速度转化为频率输出


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
				#if StepMotorTurn		 //如果不进电机抖动（且数值在180左右）是因为方向反了，
						 DC_MOTOR_UP;    //顺时针旋转
				#else		
						DC_MOTOR_DOWN; //逆时针旋转
				#endif

		}
		else if(Step_Motor_Set.Real_DegreeError < 0)
		{
				#if StepMotorTurn		
							DC_MOTOR_DOWN; //逆时针旋转
				#else		
						 DC_MOTOR_UP;    //顺时针旋转
				#endif
					
		}

    TIM_SetAutoreload(TIM5, Step_Motor_Set.frequency);
    TIM_SetCompare3(TIM5,  Step_Motor_Set.frequency/2);	

	}
*********************************************************************************************************/



/*****************************************************
	  degree_error = Step_Motor_Set.frequency;

	if(degree_error > 0.0f && degree_error < MIN_ANGLE) //角度为正值且在死区范围内
	{
	  degree_error = 0;
	}
	else if(degree_error < 0.0f && degree_error > (-MIN_ANGLE))//角度为负值且在死区范围内
	{
	  degree_error = 0;
	}
	pluses = (int)(degree_error*DegreeToPluse);  //将设定位置与反馈位置的误差转换为脉冲



	if(degree_error == 0)  
	{
		TIM_SetCompare1(TIM4, 0);	
	}
	else if(degree_error > 0.0f)  //判断顺时针旋转
	{ 
#if StepMotorTurn		//如果不进电机抖动（且数值在180左右）是因为方向反了，
		 StepMotor_Clockwise;    //顺时针旋转
#else		
		StepMotor_Anticlockwise; //逆时针旋转
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
	else if(degree_error < 0.0f) //判断逆时针旋转
	{
		
#if StepMotorTurn		
			StepMotor_Anticlockwise; //逆时针旋转
#else		
	   StepMotor_Clockwise;    //顺时针旋转
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
/*****************************用于控制步进电机的方向*******************************************/		
//	GPIO_InitTypeDef           GPIOI_InitStructure;
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
//  GPIOI_InitStructure.GPIO_Pin   = GPIO_Pin_0;
//  GPIOI_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//普通输出模式
//  GPIOI_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIOI_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//上拉
//  GPIO_Init(GPIOI, &GPIOI_InitStructure);//初始化GPIO
//			 
//  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
//  GPIOI_InitStructure.GPIO_Pin   = GPIO_Pin_13|GPIO_Pin_14;
//  GPIOI_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//普通输出模式
//  GPIOI_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
//  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
//  GPIOI_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//上拉
//  GPIO_Init(GPIOD, &GPIOI_InitStructure);//初始化GPIO
	
			
/***********************************************************************************************/			
			
	GPIO_InitTypeDef           GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef    TIM_TimeBaseStructure;
	TIM_OCInitTypeDef          TIM_OCInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
			
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);  	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
  GPIO_PinAFConfig(GPIOD, GPIO_PinSource12, GPIO_AF_TIM4);//| GPIO_PinSource13 | GPIO_PinSource14|GPIO_PinSource15,GPIO_AF_TIM4); 

	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_12;//|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;           //调用GPIOD12
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd 	= GPIO_PuPd_UP;        //上拉
	GPIO_Init(GPIOD,&GPIO_InitStructure);   

	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		
	TIM_TimeBaseInit(TIM4,&TIM_TimeBaseStructure);//初始化定时器1

	//初始化TIM4 Channel 2/3 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2 //模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能,也就是使能 PWM 输出到端口
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
	TIM_OC1Init(TIM4, &TIM_OCInitStructure); //根据T指定的参数初始化外设TIM4 OC1
	
	
	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE); //允许定时器4更新中断
	NVIC_InitStructure.NVIC_IRQChannel=TIM4_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x00; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_ARRPreloadConfig(TIM4,ENABLE);//ARPE使能 
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
// RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
/******************************************************************************************************/

		}

void PWM_GPIOH_Init(u32 arr, u32 psc)
{
	/*****************************用于控制步进电机的方向*******************************************/		
	GPIO_InitTypeDef           GPIOI_InitStructure;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI, ENABLE);
  GPIOI_InitStructure.GPIO_Pin   = GPIO_Pin_0;
  GPIOI_InitStructure.GPIO_Mode  = GPIO_Mode_OUT;//普通输出模式
  GPIOI_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
  GPIOI_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
  GPIOI_InitStructure.GPIO_PuPd  = GPIO_PuPd_DOWN;//上拉
  GPIO_Init(GPIOI, &GPIOI_InitStructure);//初始化GPIO
	
	GPIO_InitTypeDef          GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef   TIM_TimeBaseStructure;
	TIM_OCInitTypeDef         TIM_OCInitStructure;
/********************************************************************************************************/
    //PH10 、11、12 PWM初始化     PH11和PH12为备用
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM1时钟使能    
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE); //使能GPIOA时钟	
//  GPIO_PinAFConfig(GPIOH,GPIO_PinSource10,GPIO_AF_TIM5); 
//	GPIO_PinAFConfig(GPIOH,GPIO_PinSource11,GPIO_AF_TIM5); 	//预留备用
	GPIO_PinAFConfig(GPIOH,GPIO_PinSource12,GPIO_AF_TIM5); 	//预留备用
			
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;           //调用GPIOD12  GPIO_Pin_10|GPIO_Pin_11|
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;        //复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;      //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;        //下拉
	GPIO_Init(GPIOH,&GPIO_InitStructure);  //初始化PA9	
		
	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 		
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//初始化定时器1
	

	
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);//初始化定时器5
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2 //模式1
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能,也就是使能 PWM 输出到端口
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性低
//	TIM_OC1Init(TIM5, &TIM_OCInitStructure);  //PA1引脚使用定时器5的通道1//根据T指定的参数初始化外设TIM1 OC2
//	TIM_OC2Init(TIM5, &TIM_OCInitStructure); 
	TIM_OC3Init(TIM5, &TIM_OCInitStructure); 

	
//	TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);  //使能TIM1在CCR1上的预装载寄存器
//	TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);  //预留备用
	TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);  //预留备用
	
	TIM_ARRPreloadConfig(TIM5,ENABLE);//ARPE使能 
	TIM_Cmd(TIM5, ENABLE);  //使能TIM5
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);//使能GPIOF时钟
	
/*************************************************************************************************************/

}	
	

//void StepMotorRelativeControl(fp32 step_motor_position_set)
//{
// // degree_error = step_motor_position*Rad_TO_DEGRE - step_motor_relative_degree;
//  degree_error = step_motor_position_set - step_motor_relative_degree;
//	if(degree_error > 0.0f && degree_error < ONE_CIRCLE)  //判断顺时针旋转
//	{ 
//		StepMotor_Anticlockwise; //逆时针旋转
////	   StepMotor_Clockwise;    //顺时针旋转
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
//	else if(degree_error < 0.0f && degree_error > -ONE_CIRCLE) //判断逆时针旋转
//	{
//		 StepMotor_Clockwise;    //顺时针旋转
////	   StepMotor_Anticlockwise; //逆时针旋转
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
//	TIM5_CH1_Cap_Init(0XFFFFFFFF,84-1); //以1Mhz的频率计数
//  	if(TIM5CH1_CAPTURE_STA&0X80)        //成功捕获到了一次高电平
//		{
//				temp=TIM5CH1_CAPTURE_STA&0X3F; 
//				temp*=0XFFFFFFFF;		 		         //溢出时间总和
//				temp+=TIM5CH1_CAPTURE_VAL;		   //得到总的高电平时间
//				TIM5CH1_CAPTURE_STA=0;			     //开启下一次捕获
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

//  int Step_Motor_ARR = 0, Step_Motor_CCR1 = 0;	//顺时针抓手向下
/*************************废弃算法 步进电机单环控制*************************/
//int  arr_pluse_motor = ARR_STEP_MOTOR,  step_motor_circles = PULSE_NUM, pulse_count = 0;
//char current_state = 0, last_state = 0, End_If_flag = 0, Start_MD_Pulse_Cnt;

//void StepMotor_Control(char  step_motor_position)
//{	   
//	if(Start_MD_Pulse_Cnt == 0) MD_Pulse_Cnt = 0, Start_MD_Pulse_Cnt = 1;//运行此函数是将计数置零
//		      LED_GREEN = !LED_GREEN;  //提示灯

///******************初始化时，校准抓手俯仰位置******************************************************************/
//	     if(End_If_flag == 0) 
//			 {
//				 if(Read_Step_Motor_State == 1) //微动开关引脚默认是拉高状态，触发微动开关时为低电平
//				 {
//					 StepMotor_Clockwise;//定方向  
//					 TIM_SetCompare1(TIM4, arr_pluse_motor);
//					 if(MD_Pulse_Cnt > CALE_PULSE_NUM) //限制校准时的最大旋转角度，防止因微动开关出现问题而一直旋转
//					 {
//					 TIM_SetCompare1(TIM4, 0);
//					 last_state  = 1, End_If_flag = 1, MD_Pulse_Cnt = PULSE_NUM, pulse_count = PULSE_NUM;
//					 }
//					 return;   
//				 }
//				 else  last_state  = 1, End_If_flag = 1, MD_Pulse_Cnt = PULSE_NUM, pulse_count = PULSE_NUM;  //last_state值记录一次
//			 }
///******************抓手俯仰位置校准完毕，之后将执行此行之后的代码******************************************************************/ 
//			 else if(End_If_flag == 1)
//			 {
//	   //步进电机正反转判断
//			 if(step_motor_position == 1)      	StepMotor_Clockwise,     current_state = 1;
//  else if(step_motor_position == 0)  			StepMotor_Anticlockwise, current_state = 0;      //逆时针向上

//       if(last_state != current_state)  //判断方向是否切换，不等于是方向切换
//			 {
//				 if(pulse_count < step_motor_circles)  step_motor_circles =  pulse_count;
//				 else if(pulse_count >= step_motor_circles)  step_motor_circles = PULSE_NUM;
//			   arr_pluse_motor = ARR_STEP_MOTOR;//方向切换，重新给arr_pluse_motor赋值	
//				 MD_Pulse_Cnt = 0;
//				 last_state  = current_state;//保存上次方向状态，用于方向切换时重新给arr_pluse_motor赋值	
//			 }
//	//单脉冲控制，变量MD_Pulse_Cnt记录了产生的脉冲次数
//       if (MD_Pulse_Cnt >= step_motor_circles)   arr_pluse_motor = 0;  	
//	         TIM_SetCompare1(TIM4, arr_pluse_motor);   
//			     pulse_count = MD_Pulse_Cnt;
//		  }
//}




////定时器5通道1输入捕获配置
////arr：自动重装值(TIM2,TIM5是32位的!!)
////psc：时钟预分频数
//void TIM5_CH1_Cap_Init(u32 arr,u16 psc)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef   NVIC_InitStructure;
//  TIM_ICInitTypeDef  TIM5_ICInitStructure;
//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1; //GPIOA0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA0

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource1,GPIO_AF_TIM5); //PA0复用位定时器5
//  
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
//	

//	//初始化TIM5输入捕获参数
//	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
//  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
//  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
//  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
//  TIM5_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
//  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
//		
//	TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
//	
//  TIM_Cmd(TIM5,ENABLE ); 	//使能定时器5

// 
//  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
//	
//	
//}

////定时器5中断服务程序	 
//void TIM5_IRQHandler(void)
//{ 		    

// 	if((TIM5CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
//	{
//		if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//溢出
//		{	     
//			if(TIM5CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
//			{
//				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
//				{
//					TIM5CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次
//					TIM5CH1_CAPTURE_VAL=0XFFFFFFFF;
//				}else TIM5CH1_CAPTURE_STA++;
//			}	 
//		}
//		
//		if(TIM_GetITStatus(TIM5, TIM_IT_CC2) != RESET)//捕获1发生捕获事件
//		{	
//			if(TIM5CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
//			{	  			
//				
//				TIM5CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
//			  TIM5CH1_CAPTURE_VAL=TIM_GetCapture2(TIM5);//获取当前的捕获值.
//	 			TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//			}else  								//还未开始,第一次捕获上升沿
//			{
//				TIM5CH1_CAPTURE_STA=0;			//清空
//				TIM5CH1_CAPTURE_VAL=0;
//				TIM5CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
//				TIM_Cmd(TIM5,DISABLE ); 	//关闭定时器5
//	 			TIM_SetCounter(TIM5,0);
//	 			TIM_OC2PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//				TIM_Cmd(TIM5,ENABLE ); 	//使能定时器5
//			}		    
//		}			     	    					   
// 	}
//	TIM_ClearITPendingBit(TIM5, TIM_IT_CC2|TIM_IT_Update); //清除中断标志位
//}







//TIM_ICInitTypeDef  TIM5_ICInitStructure;
////定时器5通道1输入捕获配置
////arr：自动重装值(TIM2,TIM5是32位的!!)
////psc：时钟预分频数
//void TIM5_CH1_Cap_Init(u32 arr,u16 psc)
//{
//	GPIO_InitTypeDef GPIO_InitStructure;
//	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

//	
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);  	//TIM5时钟使能    
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 	//使能PORTA时钟	
//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0; //GPIOA0
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度100MHz
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN; //下拉
//	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA0

//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource0,GPIO_AF_TIM5); //PA0复用位定时器5
//  
//	  
//	TIM_TimeBaseStructure.TIM_Prescaler=psc;  //定时器分频
//	TIM_TimeBaseStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
//	TIM_TimeBaseStructure.TIM_Period=arr;   //自动重装载值
//	TIM_TimeBaseStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
//	
//	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseStructure);
//	

//	//初始化TIM5输入捕获参数
//	TIM5_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 IC1映射到TI1上
//  TIM5_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
//  TIM5_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
//  TIM5_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频 
//  TIM5_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
//  TIM_ICInit(TIM5, &TIM5_ICInitStructure);
//		
//	TIM_ITConfig(TIM5,TIM_IT_Update|TIM_IT_CC1,ENABLE);//允许更新中断 ,允许CC1IE捕获中断	
//	
//  TIM_Cmd(TIM5,ENABLE ); 	//使能定时器5

// 
//  NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器、
//	
//	
//}
////捕获状态3F = 0011 1111
////[7]:0,没有成功的捕获;1,成功捕获到一次.0x40
////[6]:0,还没捕获到低电平;1,已经捕获到低电平了.
////[5:0]:捕获低电平后溢出的次数(对于32位定时器来说,1us计数器加1,溢出时间:4294秒)
//u8  TIM5CH1_CAPTURE_STA=0;	//输入捕获状态		    				
//u32	TIM5CH1_CAPTURE_VAL;	//输入捕获值(TIM2/TIM5是32位)
////定时器5中断服务程序	 
//void TIM5_IRQHandler(void)
//{ 		    

// 	if((TIM5CH1_CAPTURE_STA&0X80)==0)//还未成功捕获	
//	{
//		if(TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//溢出
//		{	     
//			if(TIM5CH1_CAPTURE_STA&0X40)//已经捕获到高电平了
//			{
//				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//高电平太长了
//				{
//					TIM5CH1_CAPTURE_STA|=0X80;		//标记成功捕获了一次
//					TIM5CH1_CAPTURE_VAL=0XFFFFFFFF;
//				}else TIM5CH1_CAPTURE_STA++;
//			}	 
//		}
//		
//		if(TIM_GetITStatus(TIM5, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
//		{	
//			if(TIM5CH1_CAPTURE_STA&0X40)		//捕获到一个下降沿 		
//			{	  			
//				TIM5CH1_CAPTURE_STA|=0X80;		//标记成功捕获到一次高电平脉宽
//			  TIM5CH1_CAPTURE_VAL=TIM_GetCapture1(TIM5);//获取当前的捕获值.
//	 			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
//			}else  								//还未开始,第一次捕获上升沿
//			{
//				TIM5CH1_CAPTURE_STA=0;			//清空
//				TIM5CH1_CAPTURE_VAL=0;
//				TIM5CH1_CAPTURE_STA|=0X40;		//标记捕获到了上升沿
//				TIM_Cmd(TIM5,DISABLE ); 	//关闭定时器5
//	 			TIM_SetCounter(TIM5,0);
//	 			TIM_OC1PolarityConfig(TIM5,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
//				TIM_Cmd(TIM5,ENABLE ); 	//使能定时器5
//			}		    
//		}			     	    					   
// 	}
//	TIM_ClearITPendingBit(TIM5, TIM_IT_CC1|TIM_IT_Update); //清除中断标志位
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
		


/***********************************测试代码****************************************/
//void StepMotor_Control()
//{		
//							switch(  rc_ctrl.rc.s[1])//左拨杆1
//			{
//				case RC_SW_UP   :  TIM_SetCompare1(TIM4, arr_pluse_motor/2);  	if (MD_Pulse_Cnt > step_motor_circles)  arr_pluse_motor = 0;   
//				break;
//				case RC_SW_MID  :  TIM_SetCompare1(TIM4, Step_Motor_CCR1);  MD_Pulse_Cnt = 0; arr_pluse_motor = 1200; break;
//				case RC_SW_DOWN :  TIM_SetCompare1(TIM4,0);break;
//			}
//							switch(  rc_ctrl.rc.s[0])//右拨杆0
//			{
//				case RC_SW_UP   :   StepMotor_Clockwise;
//				break;
//				case RC_SW_MID  :   StepMotor_Anticlockwise;
//				break;
//				case RC_SW_DOWN :  
//				break;
//			}
//	}