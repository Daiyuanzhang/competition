/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      完成底盘控制任务。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
	
#include "chassis_task.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "arm_math.h"
#include "MotorCan.h"
#include "Detect_Task.h"
#include "pid.h"
#include "RemotDbus.h"
#include "INS_Task.h"
#include "chassis_behaviour.h"
#include "Referee_DispatchTask.h"
#include "SBUS.h"



uint8_t CHASSIS_BACK_KEY_flag = 0;
uint8_t given_init_x = 0, given_init_y = 0;
fp32 front_wheels_increase_0 = 1.4f,front_wheels_increase_1 = 1.4f, back_wheels_increase_2 = 1.0f,back_wheels_increase_3 = 1.0f;
#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

fp32 Chassis_Speed_Change_Unit = 5.0f;	
fp32 Chassis_Current_Change_Unit = 0.001f;
fp32 chassis_speed_add = 1.0f;//0.5f;		
fp32 chassis_add_set   = 0.007f;		
//底盘运动数据
chassis_move_t chassis_move;
fp32 vx_set_channel_init = 0, vy_set_channel_init = 0;
fp32 vx_set_channel_add = 0.0f, vy_set_channel_add = 0.0f;	
fp32 add_set_channel = 0.008f;
		
void chassis_total_pid_clear(chassis_move_t *chassis_clear);   
static void Chassis_PID_clear(Chassis_PID_t *gimbal_pid_clear);		
//底盘初始化，主要是pid初始化
static void chassis_init(chassis_move_t *chassis_move_init);
//底盘状态机选择，通过遥控器的开关
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘状态改变后处理控制量的改变static
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//底盘设置根据遥控器控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//底盘PID计算以及运动分解
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t chassis_high_water;
#endif






static void CalcChassisMaxOutput(void);
int chassis_run = 0;

//主任务
void chassis_task(void *pvParameters)
{
	static uint64_t cnt = 0;
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);
    //底盘初始化
    chassis_init(&chassis_move);
    //判断底盘电机是否都在线
//    while (toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE) || toe_is_error(DBUSTOE))
//    {
//        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
//    }

    while (1)
    {
			
			
			chassis_run++;

        //遥控器设置状态
        chassis_set_mode(&chassis_move);
        //遥控器状态切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);


 
		 int16_t chassis_set_current[4];

				for(;chassis_run<300;chassis_run++)
				{	
					for(char i = 0;i<4;i++)
					{
							chassis_set_current[i] = 0;
					}
				}
			
			
            {
				for(char i = 0;i<4;i++)
				{
					chassis_set_current[i] = chassis_move.motor_chassis[i].give_current;
					
//					if(rc_ctrl.rc.s[0] == 3 || rc_ctrl.rc.s[0] == 2 || toe_is_error(DBUSTOE) == 1)
//  				  chassis_set_current[i] = 0;
					
				}
               // CAN_CMD_CHASSIS(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,\
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            }
			
			 CanSendMess(CAN1,SEND_ID201_204,chassis_set_current); 
//        }
        //系统延时
        vTaskDelay(20);

//#if INCLUDE_uxTaskGetStackHighWaterMark
//        chassis_high_water = uxTaskGetStackHighWaterMark(NULL);
//#endif
    }
}

static void chassis_init(chassis_move_t *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //底盘速度环pid值
    const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
    //底盘旋转环pid值
    const static fp32 chassis_yaw_pid[3] = {CHASSIS_FOLLOW_GIMBAL_PID_KP, CHASSIS_FOLLOW_GIMBAL_PID_KI, CHASSIS_FOLLOW_GIMBAL_PID_KD};
    const static fp32 chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static fp32 chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};
		//小陀螺旋转PID值
		const static fp32 revolve_pid[3]={ CHASSIS_REVOLCE_PID_KP,CHASSIS_REVOLCE_PID_KI,CHASSIS_REVOLCE_PID_KD};
		
    uint8_t i;

    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_VECTOR_RAW;
    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();
    //获取陀螺仪姿态角指针
    chassis_move_init->chassis_INS_angle = get_INS_angle_point();
    //获取云台电机数据指针
    chassis_move_init->chassis_yaw_motor = get_yaw_motor_point();
    chassis_move_init->chassis_pitch_motor = get_pitch_motor_point();
    //初始化PID 运动
    for (i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_measure = get_Chassis_Motor_Measure_Point(i); 
				Chassis_PID_Init(&chassis_move_init->motor_relative_angle_pid[i], M3505_MOTOR_RELATIVE_PID_MAX_OUT, M3505_MOTOR_RELATIVE_PID_MAX_IOUT, M3505_MOTOR_RELATIVE_PID_KP, M3505_MOTOR_RELATIVE_PID_KI,M3505_MOTOR_RELATIVE_PID_KD);
        PID_Init(&chassis_move_init->motor_speed_pid[i], PID_POSITION, motor_speed_pid, M3505_MOTOR_SPEED_PID_MAX_OUT, M3505_MOTOR_SPEED_PID_MAX_IOUT);
    }
    //初始化旋转PID
    PID_Init(&chassis_move_init->chassis_angle_pid, PID_POSITION, chassis_yaw_pid, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT);
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME, chassis_y_order_filter);

    //最大 最小速度
    chassis_move_init->vx_max_speed2 =  SLOW_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed2 = -SLOW_MIN_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed2 =  SLOW_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed2 = -SLOW_MAX_CHASSIS_SPEED_Y;

		chassis_move_init->vx_max_speed =  NORMAL_MAX_CHASSIS_SPEED_X;
    chassis_move_init->vx_min_speed = -NORMAL_MIN_CHASSIS_SPEED_X;

    chassis_move_init->vy_max_speed = NORMAL_MAX_CHASSIS_SPEED_Y;
    chassis_move_init->vy_min_speed = -NORMAL_MAX_CHASSIS_SPEED_Y;
		
		
    //更新一下数据
    chassis_feedback_update(chassis_move_init);
	  chassis_total_pid_clear(chassis_move_init); 
}

static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
}

static void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //切入跟随云台模式
    if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        chassis_move_transit->chassis_relative_angle_set = 0.0f;
    }
    //切入跟随底盘角度模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }
    //切入不跟随云台模式
    else if ((chassis_move_transit->last_chassis_mode != CHASSIS_VECTOR_NO_FOLLOW_YAW) && chassis_move_transit->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW)
    {
        chassis_move_transit->chassis_yaw_set = chassis_move_transit->chassis_yaw;
    }

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}

static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    if (chassis_move_update == NULL)
    {
        return;
    }
	
	chassis_move_update->real_power = GetRealPower();

    uint8_t i = 0;
    for (i = 0; i < 4; i++)
    {
        //更新电机速度，加速度是速度的PID微分
				chassis_move_update->motor_chassis[i].chassis_motor_relative_angle  = chassis_ecd_to_angle_change(chassis_move_update->motor_chassis[i].chassis_motor_measure->ecd, 0);
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * chassis_move_update->motor_chassis[i].chassis_motor_measure->speed_rpm;
        chassis_move_update->motor_chassis[i].accel = chassis_move_update->motor_speed_pid[i].Dbuf[0] * CHASSIS_CONTROL_FREQUENCE;
    }

    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系                                                                                                                                                                                                                                                                                        
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
                                                                                                                                                                                                                                                                                                                       
		                                                                                                                                                                                                                                           
    //计算底盘姿态角度, 如果底盘上有陀螺仪请更改这部分代码                                                                                                                                                                                                                                           
    chassis_move_update->chassis_yaw = rad_format(*(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET));// - chassis_move_update->chassis_yaw_motor->relative_angle);
    chassis_move_update->chassis_pitch = rad_format(*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET));// - chassis_move_update->chassis_pitch_motor->relative_angle);
    chassis_move_update->chassis_roll = *(chassis_move_update->chassis_INS_angle + INS_ROLL_ADDRESS_OFFSET);

//底盘有陀螺仪时		
//	  chassis_move_update->chassis_pitch= *(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET);
//		chassis_move_update->chassis_pitch =*(chassis_move_update->chassis_INS_angle + INS_PITCH_ADDRESS_OFFSET);
//		
		//小陀螺仪旋转前进时用与计算Vx和Vy的二维矩阵的角度,角度用陀螺仪和编码器得到的云台参考系与底盘参考系的夹角
//		chassis_move_update->revolve_angle=rad_format(chassis_move_update->chassis_yaw- *(chassis_move_update->chassis_INS_angle + INS_YAW_ADDRESS_OFFSET) );

}
int check_low_speed_mode;
//遥控器的数据处理成底盘的前进vx速度，vy速度
void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }
		uint8_t low_speed_mode;
		low_speed_mode =  *return_low_speed_mode();
		check_low_speed_mode = low_speed_mode;
    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
    fp32 vx_set_channel, vy_set_channel;
    //死区限制，因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);
		
		
    vx_set_channel =  vx_channel * CHASSIS_VX_RC_SEN;
    vy_set_channel = -vy_channel * CHASSIS_VY_RC_SEN;
	

//		vx_set_channel=chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL]* CHASSIS_VX_RC_SEN;
//		vy_set_channel=chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL]* -CHASSIS_VY_RC_SEN;
		
//		chassis_movex=vx_set_channel;
//		chassis_movey=vy_set_channel;

    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
			CHASSIS_BACK_KEY_flag = 0;

			  if(vx_set_channel_init == 0) vx_set_channel_add = 0.2f, vx_set_channel_init = 1; //赋初始值
			     vx_set_channel_add = vx_set_channel_add + add_set_channel;
			     vx_set_channel = vx_set_channel_add;
        if(vx_set_channel > chassis_move_rc_to_vector->vx_max_speed) vx_set_channel_add = vx_set_channel = chassis_move_rc_to_vector->vx_max_speed;
			
			  if(low_speed_mode==1) 
					{
					  if(vx_set_channel_init == 0) vx_set_channel_add = 0.1f, vx_set_channel_init = 1; //赋初始值
						   vx_set_channel_add = vx_set_channel_add + add_set_channel;
			         vx_set_channel = vx_set_channel_add;
						if(vx_set_channel > chassis_move_rc_to_vector->vx_max_speed2)  vx_set_channel_add = vx_set_channel = chassis_move_rc_to_vector->vx_max_speed2;
					}						
    }
/********************************************************************************************************************/
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
			CHASSIS_BACK_KEY_flag = 1;

			  if(vx_set_channel_init == 0) vx_set_channel_add = -0.1f, vx_set_channel_init = 1; //赋初始值
			     vx_set_channel_add = vx_set_channel_add - CHASSIS_ADD_MIN_VALUE;
			     vx_set_channel = vx_set_channel_add;
        if(vx_set_channel < chassis_move_rc_to_vector->vx_min_speed) vx_set_channel_add = vx_set_channel = chassis_move_rc_to_vector->vx_min_speed;
			
			  if(low_speed_mode==1) 
					{
					  if(vx_set_channel_init == 0) vx_set_channel_add = -0.1f, vx_set_channel_init = 1; //赋初始值
						   vx_set_channel_add = vx_set_channel_add - CHASSIS_ADD_MIN_VALUE;
			         vx_set_channel = vx_set_channel_add;
						if(vx_set_channel < chassis_move_rc_to_vector->vx_min_speed2)  vx_set_channel_add = vx_set_channel = chassis_move_rc_to_vector->vx_min_speed2;
					}	
    }
		else vx_set_channel_init = 0;

/****************************************************************************************************************/
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
			CHASSIS_BACK_KEY_flag = 1;

			  if(vy_set_channel_init == 0) vy_set_channel_add = 0.3f, vy_set_channel_init = 1; //赋初始值
			     vy_set_channel_add = vy_set_channel_add + CHASSIS_ADD_MIN_VALUE;
			     vy_set_channel = vy_set_channel_add;
        if(vy_set_channel > chassis_move_rc_to_vector->vy_max_speed)  vy_set_channel_add = vy_set_channel = chassis_move_rc_to_vector->vy_max_speed;
			
			  if(low_speed_mode == 1) 
					{
					  if(vy_set_channel_init == 0) vy_set_channel_add = 0.1f, vy_set_channel_init = 1; //赋初始值
			         vy_set_channel_add = vy_set_channel_add + CHASSIS_ADD_MIN_VALUE;
			         vy_set_channel = vy_set_channel_add;
						if(vy_set_channel > chassis_move_rc_to_vector->vy_max_speed2)  vy_set_channel_add = vy_set_channel = chassis_move_rc_to_vector->vy_max_speed2;
					}		
    }
/*****************************************************************************************************************/
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
			CHASSIS_BACK_KEY_flag = 1;
	
			  if(vy_set_channel_init == 0) vy_set_channel_add = -0.3f, vy_set_channel_init = 1; //赋初始值
			     vy_set_channel_add = vy_set_channel_add - CHASSIS_ADD_MIN_VALUE;
			     vy_set_channel = vy_set_channel_add;
        if(vy_set_channel < chassis_move_rc_to_vector->vy_min_speed)  vy_set_channel_add = vy_set_channel = chassis_move_rc_to_vector->vy_min_speed;
			
			  if(low_speed_mode == 1) 
					{
					  if(vy_set_channel_init == 0) vy_set_channel_add = -0.1f, vy_set_channel_init = 1; //赋初始值
			     vy_set_channel_add = vy_set_channel_add - CHASSIS_ADD_MIN_VALUE;
			     vy_set_channel = vy_set_channel_add;
						if(vy_set_channel < chassis_move_rc_to_vector->vy_min_speed2)  vy_set_channel_add = vy_set_channel = chassis_move_rc_to_vector->vy_min_speed2;
					}		    
		}
		else vy_set_channel_init = 0;

    //一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);

//    //停止信号，不需要缓慢加速，直接减速到零
//    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
//    {
//        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
//    }

//    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
//    {
//        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
//    }

    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}

extern uint8_t IsGimbalMotionless(void);

extern float chassis_speed, chassis_dir, chassis_level;
//设置遥控器输入控制量
int chassis_set_contorl_RUN;
float gain = 5.2f;

static void chassis_set_contorl(chassis_move_t *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }
 
		chassis_set_contorl_RUN++;
    //设置速度
    fp32 vx_set = 0.0f, vy_set = 0.0f, angle_set = 0.0f;
		
		chassis_speed = SBUS_CH[1]/RC_SBUS_CHANNAL_MAX_VALUE;
		chassis_level = SBUS_CH[0]/RC_SBUS_CHANNAL_MAX_VALUE;
		chassis_dir = SBUS_CH[3]/RC_SBUS_CHANNAL_MAX_VALUE;
		
		
//	  chassis_speed = SBUS_CH[1];
//		chassis_level = SBUS_CH[0];
//		chassis_dir = SBUS_CH[3];	
		
//		if(chassis_speed > 3.0 ||chassis_speed < -3.0 || chassis_dir > 3.0 || chassis_dir < -3.0)
//			vx_set = 0, angle_set = 0;
//		else
//			vx_set = -chassis_speed*1.5, angle_set = -chassis_dir*1.5;
		vx_set = chassis_speed*gain;
		vy_set = chassis_level*gain;
		angle_set = -+chassis_dir*18.0f;
		
		
		
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = angle_set;
        chassis_move_control->chassis_cmd_slow_set_vx.out = 0.0f;
        chassis_move_control->chassis_cmd_slow_set_vy.out = 0.0f;

}

float wheel_0 = 0,wheel_1 = 0,wheel_2 = 0,wheel_3 = 0;

static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, fp32 wheel_speed[4])
//static void chassis_vector_to_mecanum_wheel_speed(const fp32 vx_set, const fp32 vy_set, const fp32 wz_set, Chassis_Motor_t *wheel_speed)
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0 ，1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
//	  wheel_speed[0] = -(vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[1] = -(-vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[2] = -(vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);
//    wheel_speed[3] = -(-vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set);

    wheel_speed[0] =  vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = -vx_set - vy_set + (CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] =  vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (-CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;

//     wheel_speed[0] = wheel_0;
//     wheel_speed[1] = wheel_1;
//     wheel_speed[2] = wheel_2;
//     wheel_speed[3] = wheel_3;
	
}

fp32 MAX_V = 3.2f;
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    fp32 max_vector = 0.0f, vector_rate = 0.0f;
    fp32 temp = 0.0f;
		uint8_t low_speed_mode;
		low_speed_mode =  *return_low_speed_mode();
    fp32 wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};
		
    uint8_t i = 0;
    //麦轮运动分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, 
		                                      chassis_move_control_loop->wz_set, 
		                                      wheel_speed);

		        //赋值电流值
#if  DOUBLE_LOOP_CONTROL		
        for (i = 0; i < 4; i++)
        {
           chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_angle_set += 0.1*chassis_add_set;//chassis_add_set*chassis_move_control_loop->motor_chassis[i].wheel;
        }
#else
		/************************单环控制****************************/
        for (i = 0; i < 4; i++)
        {
           chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_speed_set = wheel_speed[i];
        }
#endif        
	    //计算轮子控制最大速度，并限制其最大速度
    for (i = 0; i < 4; i++)
    {
        temp = fabs(chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_speed_set);
        if ( temp >= max_vector)
        {
            max_vector = temp;
        }
    }
		
					if (max_vector > MAX_V)
					{
							vector_rate = MAX_V / max_vector;
							for (i = 0; i < 4; i++)
							{
									chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_speed_set *= vector_rate;
							}
					}


		
		
    for (i = 0; i < 4; i++)
    {
			
//					 chassis_move_control_loop->motor_chassis[i].chassis_cale_angle = 0;
#if  DOUBLE_LOOP_CONTROL					
			//fp32 PID_Calc(PidTypeDef *pid, fp32 ref, fp32 set)
			chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_angle_set = chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_speed_set;//遥控器的输入值
        chassis_move_control_loop->motor_chassis[i].chassis_cale_angle =  Chassis_PID_Calc(
			                                                    &chassis_move_control_loop->motor_relative_angle_pid[i], 
			                                                     chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_angle, 
			                                                     chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_angle_set, 0);    
		 	/***************************双环控制*************************/
        PID_Calc(&chassis_move_control_loop->motor_speed_pid[i],
			            chassis_move_control_loop->motor_chassis[i].speed*Chassis_Speed_Change_Unit,
		              chassis_move_control_loop->motor_chassis[i].chassis_cale_angle); 

#else	
	   /***************************单环控制*************************/

       PID_Calc(&chassis_move_control_loop->motor_speed_pid[i],
			           -chassis_move_control_loop->motor_chassis[i].speed,//speed*Chassis_Speed_Change_Unit,
		             chassis_move_control_loop->motor_chassis[i].chassis_motor_relative_speed_set);			

#endif		
 }                                                                                                                                                                                          

    //赋值电流值
    for (i = 0; i < 4; i++)
    {
			//最终
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }
			
}

static void CalcChassisMaxOutput()
{
	float error_out;	
	float maxout;
	error_out = (50.0f - GetPowerBuffer()) * 0.0195f;//0.02560;//0.0195f;	
	maxout = 16000.0f * ( 1.0f - error_out);
	

	if(maxout >= M3505_MOTOR_SPEED_PID_MAX_OUT)
		maxout = M3505_MOTOR_SPEED_PID_MAX_OUT;
	else if(maxout <= 600)
		maxout = 600;
	
	for(char i = 0;i<4;i++)
	{
	   chassis_move.motor_speed_pid[i].max_out =5000;// maxout;
	}
	
	
}

float  GetChassisMaxOutput()
{
	return    chassis_move.motor_speed_pid[0].max_out; 
}

Chassis_Motor_t *getChassisGive_current()
{
  return chassis_move.motor_chassis;
}




//计算相对角度
static fp32 chassis_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
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

    return relative_ecd * Motor_Ecd_to_Rad;
}


static void Chassis_PID_Init(Chassis_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd)
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


static fp32 Chassis_PID_Calc(Chassis_PID_t *pid, fp32 get, fp32 set, fp32 error_delta)
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



////pid数据清理
static void Chassis_PID_clear(Chassis_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
	
      gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get   = 0.0f;
      gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;                         
                                                                                                                                                                      
}                                                                                                                                                                                               

void chassis_total_pid_clear(chassis_move_t *chassis_clear)                                                
{              
		for (char i = 0; i < 4; i++)
		{			
				Chassis_PID_clear(&(chassis_clear)->motor_relative_angle_pid[i]);     
				PID_clear(&(chassis_clear)->motor_speed_pid[i]);   
		}	
}


