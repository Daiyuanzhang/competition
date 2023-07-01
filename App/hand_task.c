/**
  ******************************************************************************
  *    此处定义了抓手处的电机和抓手旋转处的电机
  *    函数校准motor_link编码值和中断校准motor_link编码值需要验证
  *          
  *             
  *           
  *      
  *    
  *     
  *     
  *     
  *    遥控器无输入一段时间，电机自锁关闭
  *
  ********************************************************************************/
//#include "hand_task.h"
#include "gimbal_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h" 
#include "INS_task.h"
#include "detect_task.h"
#include "gimbal_behaviour.h"
#include "user_lib.h"
#include "MotorCAN.h"
#include "chassis_task.h"
#include "RemotDbus.h"
#include "delay.h"
#include "IO_Function_Init.h"
#include "StepMotorControl.h"



int flag_io = 0;
//旋转处校准电机变量
char IO_current_state = 0, IO_last_state = 0, end_if_flag = 0;
//抓手旋转处变量
int clockwise_180 = RUN_VALUE, clockwise_90 = RUN_VALUE/2, anticlockwise_90 = -RUN_VALUE/2-20,TutolRunCount3508 = 0, RunToBalanceCountFlag = 0;
//抓手校准标志位
uint8_t	end_turn = 0, RunToBalanceCaleFinish = 0,  ClockwiseRunToBalance = 0, 
AnticlockwiseRunToBalance = 0;
uint8_t SecondCalebrate = 0;
//, SecondCalebrate_end_turn = 0, SecondCalebrate_ClockwiseRunToBalance = 0, SecondCalebrate_AnticlockwiseRunToBalance = 0;  
//读取抓手计算后的电流值
int left_current_data = 0, right_current_data = 0, hand_link_current_data = 0;//查看发送设定值数据
//测试代码处的变量
int  flag_i = 0, flag_max = 1024,  run_finish = 0,opposite_flag_i = 0, opposite_flag_max = 1024,opposite_run_finish = 0, run_count = 0, run_count_max = 500;
//PID处的变量
double Hand_RC_Send = 0.001f,add_hand_angle = 2000, hand_relative_add = 76, speed_rate = 0.0093f;//0.011f;//0.0093;//152;
//发送的电流值数组
int16_t hand_set_current[4]; 
#define hand_total_pid_clear(gimbal_clear)                                                   \
    {                                                                                        \
        Hand_PID_clear(&(gimbal_clear)->hand_left_motor.hand_motor_relative_speed_pid );     \
        Hand_PID_clear(&(gimbal_clear)->hand_left_motor.hand_motor_relative_angle_pid);      \
        PID_clear(&(gimbal_clear)->hand_left_motor.hand_motor_speed_pid);                    \
                                                                                             \
        Hand_PID_clear(&(gimbal_clear)->hand_right_motor.hand_motor_relative_speed_pid);     \
        Hand_PID_clear(&(gimbal_clear)->hand_right_motor.hand_motor_relative_angle_pid);     \
        PID_clear(&(gimbal_clear)->hand_right_motor.hand_motor_speed_pid);                   \
			                                                                                       \
        Hand_PID_clear(&(gimbal_clear)->hand_link_motor.hand_motor_relative_speed_pid);      \
        Hand_PID_clear(&(gimbal_clear)->hand_link_motor.hand_motor_relative_angle_pid);      \
        PID_clear(&(gimbal_clear)->hand_link_motor.hand_motor_speed_pid);                    \
    }

////云台控制所有相关数据
 Hand_Control_t hand_control;

//初始化云台 
static void Hand_Init(Hand_Control_t *gimbal_init);
 //初始化云台位置环PID参数
static void Hand_PID_Init(Hand_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
//将pid误差值，设定值，输出值清零
static void Hand_PID_clear(Hand_PID_t *gimbal_pid_clear);
////根据拨杆选择控制模式，再根据状态机选择电机状态机
static void  Hand_Set_Mode(Hand_Control_t *hand_motor_get); 		
static void	 Hand_Feedback_Update(Hand_Control_t *hand_motor_feedback_update);               //抓手电机数据反馈
static void	 Hand_Set_Contorl( Hand_Control_t *gimbal_set_control );                 //设置云台控制量
static void  Hand_3508_Control_loop(Hand_Control_t *gimbal_control_loop);
static void  hand_motor_set(Hand_Control_t *hand_motor);

//云台控制量设置
static void Hand_Set_Contorl(Hand_Control_t *gimbal_set_control);
void Hand_Init_in_gimbal()
{
 Hand_Init(&hand_control);
}

void Hand_task(int16_t *left_hand_set_current, int16_t * right_hand_set_current, int16_t * hand_link_set_current)
{
      fp32 get_DC_motor_position;
	    get_DC_motor_position = *return_DC_motor_position_set();
		  StepMotorRelativeControl(get_DC_motor_position, hand_control.hand_pitch_motor->ecd);
//     	 Hand_Init(&hand_control);
	     Hand_Set_Mode(&hand_control); 
			 Hand_Feedback_Update(&hand_control);               //抓手电机反馈数据更新
			 Hand_Set_Contorl(&hand_control);                   //设置云台控制量
			 Hand_3508_Control_loop(&hand_control);
		   hand_motor_set(&hand_control);
	
	#if LEFT_TURN
		 hand_set_current[2] = - hand_control.hand_left_motor.given_current; 
	#else
		 hand_set_current[2] = hand_control.hand_left_motor.given_current; 
	#endif
	
	#if RIGHT_TURN
		hand_set_current[3] = - hand_control.hand_right_motor.given_current;
	#else 
		hand_set_current[3] =   hand_control.hand_right_motor.given_current;
	#endif	
	#if HAND_LINK_TURN   //抓手关节处用CAN2接收和发送，
		hand_set_current[1] = - hand_control.hand_link_motor.given_current;
	#else 
		hand_set_current[1] =   hand_control.hand_link_motor.given_current;
	#endif
	//CAN接收和发送函数在云天任务内，在抓手任务内循环CAN函数会出现电机反馈调节数据不及时问题
     *hand_link_set_current = hand_set_current[1];
     *left_hand_set_current = hand_set_current[2];
		*right_hand_set_current = hand_set_current[3]; 

		//三个电机数据对比
		hand_link_current_data = hand_set_current[1];
		left_current_data      = hand_set_current[2];
		right_current_data     = hand_set_current[3];
//		CanSendMess(CAN1,SEND_ID205_207,hand_set_current); 
}

static void hand_motor_set(Hand_Control_t *hand_motor)
{
    if (hand_motor == NULL)
    {
        return;
    }
        hand_motor->hand_left_motor.relative_angle_set += -*hand_control.tow_hand_motors_add*0.01;
			 hand_motor->hand_right_motor.relative_angle_set +=  *hand_control.tow_hand_motors_add*0.01;
		
	     hand_motor->hand_left_motor.speed_set  = -*hand_control.tow_hand_motors_add;
			 hand_motor->hand_right_motor.speed_set =  *hand_control.tow_hand_motors_add;
	    
}



static void hand_motor_raw_angle_control(Hand_Motor_t *hand_motor)
{
    if (hand_motor == NULL)
    {
        return;
    }
    hand_motor->current_set = hand_motor->raw_cmd_current;
    hand_motor->given_current = (int16_t)(hand_motor->current_set);
}

static fp32 Hand_PID_Calc(Hand_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
{
    fp32 err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

static void hand_motor_3508_speed_control(Hand_Motor_t *hand_motor)
{
    if (hand_motor == NULL)
    {
        return;
    }
//    hand_motor->motor_position_set =  Hand_PID_Calc(&hand_motor->hand_motor_relative_angle_pid , hand_motor->relative_angle, hand_motor->relative_angle_set, 0);
	  hand_motor->current_set = PID_Calc(&hand_motor->hand_motor_speed_pid,
		hand_motor->hand_motor_measure->speed_rpm*Hand_RC_Send, hand_motor->speed_set);
    hand_motor->given_current = (int16_t)(hand_motor->current_set);
}

static void hand_motor_relative_angle_control(Hand_Motor_t *hand_motor)
{
    if (hand_motor == NULL)
    {
        return;
    }
    hand_motor->motor_position_set =  Hand_PID_Calc(&hand_motor->hand_motor_relative_angle_pid , hand_motor->relative_angle, hand_motor->relative_angle_set, 0);
		hand_motor->current_set = PID_Calc(&hand_motor->hand_motor_speed_pid, hand_motor->hand_motor_measure->speed_rpm*speed_rate, hand_motor->motor_position_set);
    hand_motor->given_current = (int16_t)(hand_motor->current_set);
}

static void Hand_3508_Control_loop(Hand_Control_t *gimbal_control_loop)
{
	   if (gimbal_control_loop == NULL)   return; 
    //yaw不同模式对于不同的控制函数
	   if (	gimbal_control_loop->hand_left_motor.hand_mode == HAND_MOTOR_ENCONDE)
    {
        //enconde角度控制zx
 //       hand_motor_relative_angle_control(&gimbal_control_loop->hand_left_motor);
       hand_motor_3508_speed_control(&gimbal_control_loop->hand_left_motor);
    }
		 else if (gimbal_control_loop->hand_left_motor.hand_mode == HAND_MOTOR_SPEED)
    {
        //gyro角度控制
       hand_motor_3508_speed_control(&gimbal_control_loop->hand_left_motor);
    }
		//    //yaw不同模式对于不同的控制函数
     else if (gimbal_control_loop->hand_left_motor.hand_mode == HAND_MOTOR_RAW)
    {
        //raw控制
        hand_motor_raw_angle_control(&gimbal_control_loop->hand_left_motor);
    }
		
/***************************************************************************************************/
		
		   //pitch不同模式对于不同的控制函数
     if (	gimbal_control_loop->hand_right_motor.hand_mode == HAND_MOTOR_ENCONDE)
    {
        //gyro角度控制
//        hand_motor_relative_angle_control(&gimbal_control_loop->hand_right_motor);
			   hand_motor_3508_speed_control(&gimbal_control_loop->hand_right_motor);
    }
		 else if (gimbal_control_loop->hand_right_motor.hand_mode == HAND_MOTOR_SPEED)
    {
        //gyro角度控制
         hand_motor_3508_speed_control(&gimbal_control_loop->hand_right_motor);
    }
		  else if (gimbal_control_loop->hand_right_motor.hand_mode == HAND_MOTOR_RAW)
    {
        //raw控制
        hand_motor_raw_angle_control(&gimbal_control_loop->hand_right_motor);
    }
/***************************************************************************************************/
		//依然从云台任务获取模式，编码值模式和速度模式调换
     if (	gimbal_control_loop->hand_right_motor.hand_mode == HAND_MOTOR_ENCONDE)//编码值模式
    {
        hand_motor_relative_angle_control(&gimbal_control_loop->hand_link_motor);
    }
		 else if (gimbal_control_loop->hand_right_motor.hand_mode ==  HAND_MOTOR_SPEED)//速度模式
    {
         hand_motor_3508_speed_control(&gimbal_control_loop->hand_link_motor);
    }
		  else if (gimbal_control_loop->hand_right_motor.hand_mode == HAND_MOTOR_RAW)
    {
        hand_motor_raw_angle_control(&gimbal_control_loop->hand_link_motor);
    }
}

static void Hand_relative_angle_limit(Hand_Motor_t *hand_motor, fp32 add)
{
    if (hand_motor == NULL)
    {
        return;
    }
//	 hand_motor->relative_angle_set += 0;//add*40;//+ *hand_control.tow_hand_motors_add*0.01;

//		 hand_motor->relative_angle_set +=  *hand_control.tow_hand_motors_add*0.01;	
//    hand_motor->relative_angle_set = 0;//hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//		switch(rc_ctrl.rc.s[1]) //左拨杆
//	{
//			case RC_SW_MID   : 
//			{
////		hand_motor->relative_angle_set -= hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
////				if( opposite_flag_i < opposite_flag_max )
////				{
////					opposite_run_finish = 1;
//					hand_motor->relative_angle_set += add *Motor_3508_Ecd_to_ANGLE_compensate * 30 ;
////					opposite_flag_i++;
////				}else opposite_run_finish = 0;
//			}	break;
//				case RC_SW_UP   :   ////抓手旋转测试代码
//			{
//			 hand_motor->relative_angle_set = 0;
//       flag_i = 0, opposite_flag_i = 0;
//      }break;
//				case RC_SW_DOWN   : 
//			{
//		  hand_control.hand_left_motor.offset_ecd  =  hand_control.hand_left_motor.hand_motor_measure->ecd ;
//			hand_control.hand_right_motor.offset_ecd =  hand_control.hand_right_motor.hand_motor_measure->ecd;				
//			}break;
//	}	

}

static void Hand_3508_speed_limit(Hand_Motor_t *hand_motor, fp32 add)
{
    if (hand_motor == NULL)
    {
        return;
    }
   hand_motor->relative_angle_set = 0;//add*add_hand_angle;
}

 
//云台控制量设置
//uint8_t errors=0;
static void Hand_Set_Contorl( Hand_Control_t *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }
    fp32 add_yaw_angle   =  *gimbal_set_control->hand_left_motor.add_angle ;
    fp32 add_pitch_angle =  *gimbal_set_control->hand_right_motor.add_angle;
		
		
//		if(gimbal_set_control->gimbal_rc_ctrl->rc.ch[2]==0&&gimbal_set_control->gimbal_rc_ctrl->mouse.x==0)
//		{
//			errors=1;
//		}
/*************************************left********************************************************************/
		
    if (gimbal_set_control->hand_left_motor.hand_mode  == HAND_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_set_control->hand_left_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (gimbal_set_control->hand_left_motor.hand_mode  == HAND_MOTOR_SPEED)  //中档
    {
        //gyro模式下，陀螺仪角度控制
   		  Hand_3508_speed_limit(&gimbal_set_control->hand_left_motor, -add_yaw_angle );
    }
    else if (gimbal_set_control->hand_left_motor.hand_mode  == HAND_MOTOR_ENCONDE)  //上档
    {
        //enconde模式下，电机编码角度控制
       Hand_relative_angle_limit(&gimbal_set_control->hand_left_motor,  -add_yaw_angle );
    }
		
/*************************************right********************************************************************/
		
    if (gimbal_set_control->hand_right_motor.hand_mode  == HAND_MOTOR_RAW)
    {
        //raw模式下，直接发送控制值
        gimbal_set_control->hand_right_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_set_control->hand_right_motor.hand_mode == HAND_MOTOR_SPEED)
    {
        //gyro模式下，陀螺仪角度控制
			 Hand_3508_speed_limit(&gimbal_set_control->hand_right_motor, add_yaw_angle );
    }
    else if (gimbal_set_control->hand_right_motor.hand_mode == HAND_MOTOR_ENCONDE)
    {
        //enconde模式下，电机编码角度控制
  		  Hand_relative_angle_limit(&gimbal_set_control->hand_right_motor,  add_yaw_angle );
    }
/******************************************link********************************************************************/
//关节处电机连接can2，模式依然使用云台任务的模式
		  if (gimbal_set_control->hand_right_motor.hand_mode  == HAND_MOTOR_RAW)
    {
        gimbal_set_control->hand_link_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (gimbal_set_control->hand_right_motor.hand_mode == HAND_MOTOR_SPEED)
    {
			  Hand_3508_speed_limit(&gimbal_set_control->hand_link_motor, add_yaw_angle );
    }
    else if (gimbal_set_control->hand_right_motor.hand_mode == HAND_MOTOR_ENCONDE)
    {
			  Hand_relative_angle_limit(&gimbal_set_control->hand_link_motor,  add_yaw_angle );
    }
}

//初始化pid 数据指针
static void Hand_Init(Hand_Control_t *gimbal_init)
{
		static const fp32 LeftHand_speed_pid[3]  = {LEFT_HAND_SPEED_PID_KP, LEFT_HAND_SPEED_PID_KI, LEFT_HAND_SPEED_PID_KD};
		static const fp32 RightHand_speed_pid[3] = {RIGHT_HAND_SPEED_PID_KP, RIGHT_HAND_SPEED_PID_KI, RIGHT_HAND_SPEED_PID_KD};
		static const fp32 Hand_link_speed_pid[3] = {HAND_LINK_SPEED_PID_KP, HAND_LINK_SPEED_PID_KI, HAND_LINK_SPEED_PID_KD};
		
    //电机数据指针获取
		gimbal_init->hand_left_motor.hand_motor_measure   =  get_LeftHand_Motor_Measure_Point();
		gimbal_init->hand_right_motor.hand_motor_measure  =  get_RightHand_Motor_Measure_Point();	
		gimbal_init->hand_link_motor.hand_motor_measure   =  get_Hand_Motor_3508_Measure_Point();	
		gimbal_init->hand_pitch_motor =  get_Hand_Pitch_Measure_Point();	
		gimbal_init->hand_left_motor.hand_mode   =  *get_yaw_motor_mode_point();//获取云台模式给抓手做判断
    gimbal_init->hand_right_motor.hand_mode  =  *get_pitch_motor_mode_point();
    gimbal_init->hand_left_motor.add_angle   = get_add_yaw_angle_pointer();//获取遥控器的输入值
		gimbal_init->hand_right_motor.add_angle  = get_add_pitch_angle_pointer();
	  gimbal_init->hand_link_motor.add_angle   = get_add_pitch_angle_pointer();
		gimbal_init->tow_hand_motors_add         = send_hand_motor_set();
    //遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();

//  初始化左右电机的pid
    Hand_PID_Init(&gimbal_init->hand_right_motor.hand_motor_relative_speed_pid, RIGHT_HAND_GYRO_ABSOLUTE_PID_MAX_OUT, RIGHT_HAND_GYRO_ABSOLUTE_PID_MAX_IOUT, RIGHT_HAND_GYRO_ABSOLUTE_PID_KP, RIGHT_HAND_GYRO_ABSOLUTE_PID_KI, RIGHT_HAND_GYRO_ABSOLUTE_PID_KD);
    Hand_PID_Init(&gimbal_init->hand_right_motor.hand_motor_relative_angle_pid, RIGHT_HAND_ENCODE_RELATIVE_PID_MAX_OUT, RIGHT_HAND_ENCODE_RELATIVE_PID_MAX_IOUT, RIGHT_HAND_ENCODE_RELATIVE_PID_KP, RIGHT_HAND_ENCODE_RELATIVE_PID_KI, RIGHT_HAND_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->hand_right_motor.hand_motor_speed_pid, PID_POSITION, LeftHand_speed_pid, RIGHT_HAND_SPEED_PID_MAX_OUT, RIGHT_HAND_SPEED_PID_MAX_IOUT);
  
	  Hand_PID_Init(&gimbal_init->hand_left_motor.hand_motor_relative_speed_pid, LEFT_HAND_GYRO_ABSOLUTE_PID_MAX_OUT, LEFT_HAND_GYRO_ABSOLUTE_PID_MAX_IOUT, LEFT_HAND_GYRO_ABSOLUTE_PID_KP, LEFT_HAND_GYRO_ABSOLUTE_PID_KI, LEFT_HAND_GYRO_ABSOLUTE_PID_KD);
    Hand_PID_Init(&gimbal_init->hand_left_motor.hand_motor_relative_angle_pid, LEFT_HAND_ENCODE_RELATIVE_PID_MAX_OUT, LEFT_HAND_ENCODE_RELATIVE_PID_MAX_IOUT, LEFT_HAND_ENCODE_RELATIVE_PID_KP, LEFT_HAND_ENCODE_RELATIVE_PID_KI, LEFT_HAND_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->hand_left_motor.hand_motor_speed_pid, PID_POSITION, RightHand_speed_pid, LEFT_HAND_SPEED_PID_MAX_OUT, LEFT_HAND_SPEED_PID_MAX_IOUT);
  //初始化关节处电机
		Hand_PID_Init(&gimbal_init->hand_link_motor.hand_motor_relative_speed_pid, HAND_LINK_GYRO_ABSOLUTE_PID_MAX_OUT, HAND_LINK_GYRO_ABSOLUTE_PID_MAX_IOUT, HAND_LINK_GYRO_ABSOLUTE_PID_KP, HAND_LINK_GYRO_ABSOLUTE_PID_KI, HAND_LINK_GYRO_ABSOLUTE_PID_KD);
    Hand_PID_Init(&gimbal_init->hand_link_motor.hand_motor_relative_angle_pid, HAND_LINK_ENCODE_RELATIVE_PID_MAX_OUT, HAND_LINK_ENCODE_RELATIVE_PID_MAX_IOUT, HAND_LINK_ENCODE_RELATIVE_PID_KP, HAND_LINK_ENCODE_RELATIVE_PID_KI, HAND_LINK_ENCODE_RELATIVE_PID_KD);
    PID_Init(&gimbal_init->hand_link_motor.hand_motor_speed_pid, PID_POSITION, Hand_link_speed_pid, HAND_LINK_SPEED_PID_MAX_OUT, HAND_LINK_SPEED_PID_MAX_IOUT);

    //清除所有PID
    hand_total_pid_clear(gimbal_init);
    //更新云台反馈数据
    Hand_Feedback_Update(gimbal_init);
}

static void  Hand_Set_Mode(Hand_Control_t *hand_motor_get)
{
		hand_motor_get->hand_left_motor.hand_mode   =  *get_yaw_motor_mode_point();//获取云台模式给抓手做判断
		hand_motor_get->hand_right_motor.hand_mode  =  *get_pitch_motor_mode_point();
}
static void Hand_PID_Init(Hand_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

////pid数据清理
static void Hand_PID_clear(Hand_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get   = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

////计算相对角度
static fp32 hand_motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd *Motor_3508_Ecd_to_ANGLE;
}

//抓手数据更新
static void Hand_Feedback_Update(Hand_Control_t *hand_motor_feedback_update)
{
    if (hand_motor_feedback_update == NULL)
    {
        return;
    }
    //3508电机数据更新
    hand_motor_feedback_update->hand_left_motor.absolute_angle  = 0;// -*(hand_motor_feedback_update->gimbal_INT_angle_point + INS_ROLL_ADDRESS_OFFSET);
    hand_motor_feedback_update->hand_left_motor .relative_angle = hand_motor_ecd_to_angle_change(hand_motor_feedback_update->hand_left_motor.hand_motor_measure->ecd,
                                                                                          hand_motor_feedback_update->hand_left_motor .offset_ecd);
    hand_motor_feedback_update->hand_left_motor.motor_gyro = 0;//0 *(hand_motor_feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET);

    hand_motor_feedback_update->hand_right_motor.absolute_angle = 0;// *(hand_motor_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    hand_motor_feedback_update->hand_right_motor.relative_angle = hand_motor_ecd_to_angle_change(hand_motor_feedback_update->hand_right_motor.hand_motor_measure->ecd,
                                                                                        hand_motor_feedback_update->hand_right_motor.offset_ecd );
    hand_motor_feedback_update->hand_right_motor.motor_gyro = 0;// -*(hand_motor_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
		//关节处3508电机数据更新
		hand_motor_feedback_update->hand_link_motor.absolute_angle = 0;// *(hand_motor_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    hand_motor_feedback_update->hand_link_motor.relative_angle = hand_motor_ecd_to_angle_change(hand_motor_feedback_update->hand_link_motor.hand_motor_measure->ecd,
                                                                                        hand_motor_feedback_update->hand_link_motor.offset_ecd );
    hand_motor_feedback_update->hand_link_motor.motor_gyro = 0;// -*(hand_motor_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
    	
//		hand_motor_feedback_update->hand_pitch_motor.absolute_angle = 0;// *(hand_motor_feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
//    hand_motor_feedback_update->hand_pitch_motor.relative_angle = hand_motor_ecd_to_angle_change(hand_motor_feedback_update->hand_pitch_motor.hand_motor_measure->ecd,
//                                                                                        hand_motor_feedback_update->hand_pitch_motor.offset_ecd );
//    hand_motor_feedback_update->hand_pitch_motor.motor_gyro = 0;// -*(hand_motor_feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET);
    
}



//	//EXTI中断校准motor_link的编码值
//	void EXTI0_IRQHandler(void)
//  {
//	//		  delay_ms(10);	//消抖
//			if(EXTI_GetITStatus(EXTI_Line0) == 1 ) //读取中断标志位，确定中断真的发生
//			{   //此处编写中断函数
//				flag_io++;  //计数，查看进入了多少次中断
//				LED_GREEN = !LED_GREEN;  //提示灯
//				TutolRunCount3508 = 0, ClockwiseRunToBalance = 0, AnticlockwiseRunToBalance = 0;
//				//触发中断保存编码值
//				hand_control.hand_link_motor.offset_ecd  =  hand_control.hand_link_motor.hand_motor_measure->ecd ;
//				//以下两行是测试代码
//				hand_control.hand_left_motor.offset_ecd  =  hand_control.hand_left_motor.hand_motor_measure->ecd ;
//				hand_control.hand_right_motor.offset_ecd =  hand_control.hand_right_motor.hand_motor_measure->ecd;					
//			}
//			EXTI_ClearITPendingBit(EXTI_Line0); //清除中断标志位
//	}


	
/******************************************
*	                                        *
*    下方为关节处3508电机的任务函数       *
*	                                        *
*******************************************/
void Run_To_Balance_Init()  //初始化时让抓手回归平衡位置
{
	/*******光电为1在右边的旋转区域内，********/
 	     if(Hand_link_Level_Position == 1 && end_turn == 0)    AnticlockwiseRunToBalance = 1, end_turn = 1;      //微动开关处于触发状态
	else if(Hand_link_Level_Position == 0 && end_turn == 0)    ClockwiseRunToBalance = 1, end_turn = 1;  //微动开关处于非触发状态
	
//顺时针旋转超过半圈，抵消可能存在的误差
		if(RunToBalanceCountFlag < CLOCKWISE_RUN_TO_BALANCE_TIMES &&  ClockwiseRunToBalance == 1 ) 
	{
		hand_control.hand_link_motor.relative_angle_set += 0.15*hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
		RunToBalanceCountFlag++;
	}
	//逆时针旋转超过四分之一圈，抵消可能存在的误差
	else if(RunToBalanceCountFlag > ANTICLOCKWISE_RUN_TO_BALANCE_TIMES &&  AnticlockwiseRunToBalance == 1 ) 
	{
		hand_control.hand_link_motor.relative_angle_set -= 0.15*hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
		RunToBalanceCountFlag--;
	}
	else 
	{
		RunToBalanceCaleFinish = 1;
	  RunToBalanceCountFlag = 0;
		AnticlockwiseRunToBalance = 0;
		ClockwiseRunToBalance = 0;
	}
	Cale_Hand_Link_to_Balance();
}

//void Cale_Run_To_Balance()  //初始化时让抓手回归平衡位置
//{
//	/*******光电为1在右边的旋转区域内，********/
// 	     if(Hand_link_Level_Position == 1 && end_turn == 0)    AnticlockwiseRunToBalance = 1, end_turn = 1;      //微动开关处于触发状态
//	else if(Hand_link_Level_Position == 0 && end_turn == 0)    ClockwiseRunToBalance = 1, end_turn = 1;  //微动开关处于非触发状态
//	
////顺时针旋转超过半圈，抵消可能存在的误差
//		if(RunToBalanceCountFlag < CLOCKWISE_RUN_TO_BALANCE_TIMES &&  ClockwiseRunToBalance == 1 ) 
//	{
//		hand_control.hand_link_motor.relative_angle_set += 0.1*hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//		RunToBalanceCountFlag++;
//	}
//	//逆时针旋转超过四分之一圈，抵消可能存在的误差
//	else if(RunToBalanceCountFlag > ANTICLOCKWISE_RUN_TO_BALANCE_TIMES &&  AnticlockwiseRunToBalance == 1 ) 
//	{
//		hand_control.hand_link_motor.relative_angle_set -= 0.1*hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//		RunToBalanceCountFlag--;
//	}
//	else 
//	{
////		RunToBalanceCaleFinish = 1;
//	  RunToBalanceCountFlag = 0;
//		AnticlockwiseRunToBalance = 0;
//		ClockwiseRunToBalance = 0;
//	}
//	Cale_Hand_Link_to_Balance();
//}

void  Link_Motor_Self_lock()
{
  hand_control.hand_link_motor.relative_angle_set = 0;
}


//void Run_To_Balance()  //初始化时让抓手回归平衡位置
//{
//	     if(Hand_link_Level_Position == 1 && end_turn == 0) SecondCalebrate_ClockwiseRunToBalance = 1,  ClockwiseRunToBalance = 1, end_turn = 1;  //微动开关处于触发状态
//	else if(Hand_link_Level_Position == 0 && end_turn == 0) SecondCalebrate_AnticlockwiseRunToBalance = 1,  AnticlockwiseRunToBalance = 1, end_turn = 1;  //微动开关处于非触发状态

////顺时针旋转超过半圈，抵消可能存在的误差
//		if(TutolRunCount3508 < HAND_CLOCKWISE_RUN_TO_BALANCE_TIMES &&  ClockwiseRunToBalance == 1 ) 
//	{
//		hand_control.hand_link_motor.relative_angle_set -= hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//		TutolRunCount3508++;
//	}
//	//逆时针旋转超过四分之一圈，抵消可能存在的误差
//	else if(TutolRunCount3508 > HAND_ANTICLOCKWISE_RUN_TO_BALANCE_TIMES &&  AnticlockwiseRunToBalance == 1 ) 
//	{
//		hand_control.hand_link_motor.relative_angle_set += hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//		TutolRunCount3508--;
//	}
//	else if((ClockwiseRunToBalance == 0 || AnticlockwiseRunToBalance == 0) && end_turn == 1)
//	{
//	  TutolRunCount3508 = 0;
////		AnticlockwiseRunToBalance = 0;
////		ClockwiseRunToBalance = 0;
//		//开启第二次校准
//		SecondCalebrate = 1;
//		SecondCalebrate_end_turn = 0;
//		end_turn = 2;
//	}
//	
//	if(SecondCalebrate == 1)
//	{
//			//第二次校准
//		   if(Hand_link_Level_Position == 1 && SecondCalebrate_end_turn == 0) end_if_flag = 0, SecondCalebrate_ClockwiseRunToBalance = 1,   SecondCalebrate_end_turn = 1;  //微动开关处于触发状态
//	else if(Hand_link_Level_Position == 0 && SecondCalebrate_end_turn == 0) end_if_flag = 0, SecondCalebrate_AnticlockwiseRunToBalance = 1, SecondCalebrate_end_turn = 1;  //微动开关处于非触发状态
//	
//			if(TutolRunCount3508 < HAND_CLOCKWISE_RUN_TO_BALANCE_TIMES &&  SecondCalebrate_ClockwiseRunToBalance == 1 ) 
//			{
//				hand_control.hand_link_motor.relative_angle_set -= 0.05*hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//				TutolRunCount3508++;
//			}
//			//逆时针旋转超过四分之一圈，抵消可能存在的误差
//			else if(TutolRunCount3508 > HAND_ANTICLOCKWISE_RUN_TO_BALANCE_TIMES &&  SecondCalebrate_AnticlockwiseRunToBalance == 1 ) 
//			{
//				hand_control.hand_link_motor.relative_angle_set += 0.05*hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//				TutolRunCount3508--;
//			}
//			else 
//			{
//			 //保存当前编码值
//				hand_control.hand_link_motor.offset_ecd  =  hand_control.hand_link_motor.hand_motor_measure->ecd;
//				SecondCalebrate = 0;
//				TutolRunCount3508 = 0;
////				AnticlockwiseRunToBalance = 0;
////				ClockwiseRunToBalance = 0;
//				SecondCalebrate_ClockwiseRunToBalance = 0, SecondCalebrate_AnticlockwiseRunToBalance = 0;
//			}
//	}
//	Cale_Hand_Link_to_Balance();
//}


void Run_To_Balance()  //初始化时让抓手回归平衡位置
{
	 
		if(TutolRunCount3508 < 0)  
	{
		hand_control.hand_link_motor.relative_angle_set  += hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
		TutolRunCount3508++;
	}
	else if(TutolRunCount3508 > 0)
	{
//		hand_control.hand_link_motor.relative_angle_set  -= hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//		TutolRunCount3508--;
		hand_control.hand_link_motor.relative_angle_set  -= hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
    TutolRunCount3508--;
	}	
	else//(TutolRunCount3508 == 5)
	{
		return;

	}
//	if(SecondCalebrate == 1)
//	{
////					 if(Hand_link_Level_Position == 1 && end_turn == 0)   AnticlockwiseRunToBalance = 1, end_turn = 1;  //微动开关处于触发状态
////			else if(Hand_link_Level_Position == 0 && end_turn == 0)   ClockwiseRunToBalance  = 1, end_turn = 1;  //微动开关处于非触发状态
//					 if(Hand_link_Level_Position == 1 )   AnticlockwiseRunToBalance = 1;  //微动开关处于触发状态
//			else if(Hand_link_Level_Position == 0 )   ClockwiseRunToBalance  = 1;  //微动开关处于非触发状态

//		//顺时针旋转超过半圈，抵消可能存在的误差
//				if(TutolRunCount3508 < HAND_CLOCKWISE_RUN_TO_BALANCE_TIMES &&  ClockwiseRunToBalance == 1 ) 
//			{
//				hand_control.hand_link_motor.relative_angle_set += 0.1*hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//				TutolRunCount3508++;
//			}
//			//逆时针旋转超过四分之一圈，抵消可能存在的误差
//			else if(TutolRunCount3508 > HAND_ANTICLOCKWISE_RUN_TO_BALANCE_TIMES &&  AnticlockwiseRunToBalance == 1 ) 
//			{
//				hand_control.hand_link_motor.relative_angle_set -= 0.1*hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//				TutolRunCount3508--;
//			}
//				Cale_Hand_Link_to_Balance();
//			 if((ClockwiseRunToBalance == 0 || AnticlockwiseRunToBalance == 0))
//			{
//				SecondCalebrate = 2;
//				TutolRunCount3508 = 0;	
//				 
//			}
//			
//	}
}

void Clockwise_90()  //顺时针旋转90度函数
{
		if(TutolRunCount3508 < 512)  
	{
		hand_control.hand_link_motor.relative_angle_set  += hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
		TutolRunCount3508++;
	}
	else if(TutolRunCount3508 > 512)
	{
//		hand_control.hand_link_motor.relative_angle_set  -= hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//		TutolRunCount3508--;
		hand_control.hand_link_motor.relative_angle_set  -= hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
    TutolRunCount3508--;
	}	
}

void Clockwise_180()  //顺时针旋转180度
{
		if(TutolRunCount3508 < clockwise_180) 
	{
//		hand_control.hand_link_motor.relative_angle_set  += hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
//		TutolRunCount3508++;
		hand_control.hand_link_motor.relative_angle_set  += hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
    TutolRunCount3508++;
	}
	else	if(TutolRunCount3508 > clockwise_180)
	{
		hand_control.hand_link_motor.relative_angle_set  -= hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
		TutolRunCount3508--;
	}
}
void Anticlockwise_90() //逆时针旋转90度
{
	  if (TutolRunCount3508 > anticlockwise_90)
	{
		hand_control.hand_link_motor.relative_angle_set  -= hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
    TutolRunCount3508--;
	}
		else  if (TutolRunCount3508 < anticlockwise_90)
	{
		hand_control.hand_link_motor.relative_angle_set  += hand_relative_add * Motor_3508_Ecd_to_ANGLE*Motor_3508_Ecd_to_ANGLE_compensate;
    TutolRunCount3508++;
	}
}

void Cale_Hand_Link_to_Balance()
{
		IO_current_state = Hand_link_Level_Position;
		if(end_if_flag == 0)  IO_last_state = IO_current_state, end_if_flag = 1;
    if(IO_last_state != IO_current_state)
		{
//		flag_io++;  //计数，查看进入了多少次中断
//		LED_GREEN = !LED_GREEN;  //提示灯
		TutolRunCount3508 = 0, ClockwiseRunToBalance = 0, AnticlockwiseRunToBalance = 0;
   	SecondCalebrate = 0;  *retur_hand_to_balance() = 0;
//		hand_control.hand_link_motor.offset_ecd  =  hand_control.hand_link_motor.hand_motor_measure->ecd ;
	 				//以下两行是测试代码
//		hand_control.hand_left_motor.offset_ecd  =  hand_control.hand_left_motor.hand_motor_measure->ecd ;
//		hand_control.hand_right_motor.offset_ecd =  hand_control.hand_right_motor.hand_motor_measure->ecd;
		IO_last_state = IO_current_state;//结束保存编码值条件
		}
}

int16_t *get_link_motor_current(void)
{
   return &hand_set_current[1];
}

////将校准值传回
//void set_cali_key_hand_hook(const uint16_t left_offset, const uint16_t right_offset)
//{
////    hand_control.hand_left_motor.offset_ecd  = left_offset;
////    hand_control.hand_right_motor.offset_ecd = right_offset;
//	
//}

//bool_t cmd_cali_hand_3508_hook( uint16_t *left_offset, uint16_t *right_offset )
//{
//	
//     *left_offset = hand_control.hand_left_motor.hand_motor_measure->ecd;
//    *right_offset = hand_control.hand_right_motor.hand_motor_measure->ecd;

//	return  1;
//}




//	double value_abs( double value)
//	{
//  if (value > 0)
//    {
//        return value;
//    }
//    else
//    {
//        return -value;
//    }
//	
//	}



////int current_ecd = 0, last_ecd = 0, ecd_flag = 0, error_ecd = 0, error_flag = 0;
////fp64 relative_ecd;	
////double motor_3508_speed_rpm;
////static fp64 motor_3508_ecd_to_angle_change(uint16_t ecd, double motor_speed)
////{

////	hand_motor_t *motor_speed;
//// 	motor_3508_speed_rpm = motor_speed;
////	current_ecd = ecd;
////	if (ecd_flag == 0) 
////	{
////		last_ecd = current_ecd;
////		 ecd_flag = 1;
////	}
////	error_flag = current_ecd - last_ecd;
////	if(error_flag > 100 || error_flag < -100 ) 
////	{
////		 error_ecd  = current_ecd - last_ecd;
////		if(motor_3508_speed_rpm > 0)
////		{
////	          error_ecd =	value_abs( error_ecd );
////		}
////   else  	error_ecd =	-value_abs( error_ecd );	
////		 last_ecd   = current_ecd;
////	}
////	else error_ecd = 0;
////问题：转速  转的过快，编码值会反着减小
////电机正传：电机采样时间内 慢于一圈，8192-error	
////    relative_ecd +=  error_ecd* Motor_Ecd_to_Rad;
////	  last_ecd = current_ecd;
//// return relative_ecd;  
////}