/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       chassis.c/h
  * @brief      ��ɵ��̿�������
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef CHASSISTASK_H
#define CHASSISTASK_H
#include "main.h"
#include "MotorCAN.h"
#include "Gimbal_Task.h"
#include "pid.h"
#include "RemotDbus.h"
#include "user_lib.h"

#define RC_SBUS_CHANNAL_MAX_VALUE   700.0f

//����ʼ����һ��ʱ��
#define CHASSIS_TASK_INIT_TIME 357
#define MOUSE_TO_WZ 0.02f
//ǰ���ң����ͨ������
#define CHASSIS_X_CHANNEL 1
//���ҵ�ң����ͨ������
#define CHASSIS_Y_CHANNEL 0
//������ģʽ�£�����ͨ��ң����������ת
#define CHASSIS_WZ_CHANNEL 2



//ѡ�����״̬ ����ͨ����
#define MODE_CHANNEL 0
//ң����ǰ��ҡ�ˣ�max 660��ת���ɳ���ǰ���ٶȣ�m/s���ı���
#define CHASSIS_VX_RC_SEN 0.0053f
//ң��������ҡ�ˣ�max 660��ת���ɳ��������ٶȣ�m/s���ı���
#define CHASSIS_VY_RC_SEN 0.0035f
//�������yawģʽ�£�ң������yawң�ˣ�max 660�����ӵ�����Ƕȵı���
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//��������̨��ʱ�� ң������yawң�ˣ�max 660��ת���ɳ�����ת�ٶȵı���
#define CHASSIS_WZ_RC_SEN 0.006f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f

#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//����������Ƽ�� 2ms
#define CHASSIS_CONTROL_TIME_MS 2
//����������Ƽ�� 0.002s
#define CHASSIS_CONTROL_TIME 0.002
//�����������Ƶ�ʣ���δʹ�������
#define CHASSIS_CONTROL_FREQUENCE 500.0f
//����3508���can���͵���ֵ
#define MAX_MOTOR_CAN_CURRENT 16000.0f

//����360��ת����
#define REVOLVE_START_KEY   KEY_PRESSED_OFFSET_SHIFT
#define REVOLVE_STOP_KEY    KEY_PRESSED_OFFSET_CTRL
//����ҡ�ڰ���
//#define SWING_START_KEY KEY_PRESSED_OFFSET_SHIFT
//#define SWING_STOP_KEY KEY_PRESSED_OFFSET_CTRL
//����ǰ�����ҿ��ư���
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY  KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY  KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508ת���ɵ����ٶ�(m/s)�ı������������� ����Ϊ���ܻ������Ҫ��������
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN    M3508_MOTOR_RPM_TO_VECTOR










//�����˶��������ǰ���ٶ�
#define SLOW_MAX_CHASSIS_SPEED_X 0.5f
#define SLOW_MIN_CHASSIS_SPEED_X 0.5f
#define CHASSIS_X_ADD_VALUE   0.001f;

//���̵������ٶ�
#define MAX_WHEEL_SPEED 12.0f
//���̵����������ٶ�
#define MAX_WHEEL_right_left_SPEED 8.0f

//�����˶��������ǰ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_X 1.5f
#define NORMAL_MIN_CHASSIS_SPEED_X 1.5f

#define CHASSIS_ADD_MIN_VALUE 0.008f

//�����˶��������ƽ���ٶ�
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.4f


//�����˶��������ƽ���ٶ�
#define SLOW_MAX_CHASSIS_SPEED_Y 1.0f
#define CHASSIS_Y_ADD_VALUE   0.001f;

//����������ת�ٶȣ�����ǰ�������ֲ�ͬ�趨�ٶȵı�����Ȩ 0Ϊ�ڼ������ģ�����Ҫ����
#define CHASSIS_WZ_SET_SCALE 0.1f
//������תǰ������ٶ�
#define REVOLVE_MAX_CHASSIS_SPEED_X1 428
#define REVOLVE_MAX_CHASSIS_SPEED_X  1.0f

//������תƽ�Ƶ�����ٶ�
#define REVOLVE_MAX_CHASSIS_SPEED_Y1 200
#define REVOLVE_MAX_CHASSIS_SPEED_Y  0.8f









//				if(given_init_x == 0) vx_set_channel = 0.4f;
//			     vx_set_channel += CHASSIS_X_ADD_VALUE;
//			  if( vx_set_channel == REVOLVE_MAX_CHASSIS_SPEED_X)
//					
//ҡ��ԭ�ز���ҡ�����Ƕ�(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//ҡ�ڹ��̵����˶����Ƕ�(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f

 //���̵���ǶȻ�PID
#define M3505_MOTOR_RELATIVE_PID_KP 3.0f
#define M3505_MOTOR_RELATIVE_PID_KI 0.0f
#define M3505_MOTOR_RELATIVE_PID_KD 0.0f
#define M3505_MOTOR_RELATIVE_PID_MAX_OUT  20.0f
#define M3505_MOTOR_RELATIVE_PID_MAX_IOUT  0.0f

//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 2500.0f//5000.0f//4000.0f//4000.0f//6500.0f//4000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT    3000.0f//10000.0f//10000.0f  //16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT  0.0f// 1000.0f// 1000.0f//10000.0f

/********˫��*********
λ�û�kp 9.0f
//���̵���ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 1500.0f
#define M3505_MOTOR_SPEED_PID_KI 50f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
#define M3505_MOTOR_SPEED_PID_MAX_OUT 16000.0f
#define M3505_MOTOR_SPEED_PID_MAX_IOUT 4000.0f
************************/
//������ת����PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 18.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

//С������תPID���ڶ��ַ���ʱ��
#define CHASSIS_REVOLCE_PID_KP 40.0f
#define CHASSIS_REVOLCE_PID_KI 0.0f
#define CHASSIS_REVOLCE_PID_KD 0.0f
#define CHASSIS_REVOLCE_PID_MAX_OUT 6.0f
#define CHASSIS_REVOLCE_PID_MAX_IOUT 0.2f

#define Chassis_Motor_Ecd_to_Rad 0.000766990394f //      2*  PI  /8192
#define DOUBLE_LOOP_CONTROL      0

typedef enum
{
  CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
  CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
  CHASSIS_VECTOR_NO_FOLLOW_YAW,
  CHASSIS_VECTOR_RAW,

  //  CHASSIS_AUTO,
  //  CHASSIS_FOLLOW_YAW,
  //  CHASSIS_ENCODER,
  //  CHASSIS_NO_ACTION,
  //  CHASSIS_RELAX,
} chassis_mode_e;

typedef struct
{
  const motor_measure_t *chassis_motor_measure;
  fp32 accel;
	fp32 chassis_motor_relative_speed_set;
	fp32 chassis_cale_angle;
  fp32 speed;
	fp32 chassis_motor_relative_angle;
	fp32 chassis_motor_relative_angle_set;
  int16_t give_current;
} Chassis_Motor_t;

 typedef struct
{
    fp32 kp;
    fp32 ki;
    fp32 kd;

    fp32 set;
    fp32 get;
    fp32 err;

    fp32 max_out;
    fp32 max_iout;

    fp32 Pout;
    fp32 Iout;
    fp32 Dout;

    fp32 out;
} Chassis_PID_t;



typedef struct
{
  const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
  const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
  const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  chassis_mode_e chassis_mode;               //���̿���״̬��
  chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
  Chassis_Motor_t motor_chassis[4];          //���̵������
  Chassis_PID_t motor_relative_angle_pid[4];    	
  PidTypeDef motor_speed_pid[4];             //���̵���ٶ�pid
  PidTypeDef chassis_angle_pid;              //���̸���Ƕ�pid
	PidTypeDef revolve_pid;                    //С������תpid

  first_order_filter_type_t chassis_cmd_slow_set_vx;
  first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad/s
  fp32 chassis_relative_angle_set; //���������̨���ƽǶ�
  fp32 chassis_yaw_set;
	fp32 chassis_yaw_motorerror;

  fp32 vx_max_speed2;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed2;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed2;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed2;  //���ҷ�����С�ٶ� ��λm/s

  fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�	
	fp32 chassis_angle_set;//С���ݵڶ��ַ���ʱ������ĽǶ�
	fp32 revolve_angle;//���ڼ���С������תʱǰ��ʱ��ά�������Ǻ���
  
	
  fp32 real_power;
} chassis_move_t;


 /*********������ļ�**********/
void chassis_total_pid_clear(chassis_move_t *chassis_clear);
static void Chassis_PID_Init(Chassis_PID_t *pid, fp32 maxout, fp32 max_iout, fp32 kp, fp32 ki, fp32 kd);
static fp32 Chassis_PID_Calc(Chassis_PID_t *pid, fp32 get, fp32 set, fp32 error_delta);
static fp32 chassis_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);


extern void chassis_task(void *pvParameters);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
extern float  GetChassisMaxOutput(void);
extern Chassis_Motor_t *getChassisGive_current(void );
#endif
