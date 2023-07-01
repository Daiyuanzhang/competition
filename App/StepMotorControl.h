#include "sys.h"
#include "main.h"
#include "pid.h"

 /*****�����������������****/
 /*****���в�����ȫ��Ϊ��ˢֱ�����****/
	
#define STEP_MOTOR_SPEED_PID_KP  140.0f//2000.0f
#define STEP_MOTOR_SPEED_PID_KI  0.0f
#define STEP_MOTOR_SPEED_PID_KD  0.0f
#define STEP_MOTOR_SPEED_PID_MAX_OUT  2000.0f//2000.0f
#define STEP_MOTOR_SPEED_PID_MAX_IOUT 0.0f//5000.0f


//#define ARR_STEP_MOTOR  600   //��ʼ����װ��ֵ
//#define PULSE_NUM  1422//1390//2700      ��ת��Ҫ�ĽǶ�
//#define CALE_PULSE_NUM  1800//1390//2700 У׼ʱ��Ҫ��ת�ĽǶ� 
#define MIN_FREQUENCY   260
#define DC_MIN_OUTPUT   800.0f//860f
#define PLUSE_TO_DEGREE  0.36180905f//����������ת��Ϊ�Ƕ�  995/360
#define PLUSE_TO_Rad   0.003378056616763f//����������ת��Ϊ�Ƕ�
#define Rad_TO_DEGRE  57.29577951308232f
#define MIN_ANGLE    5.0f
#define MIN_ANGLE_OFFSET  1.4f//1.0f
#define ONE_CIRCLE   360.0f
#define DegreeToPluse  17.77f
#define EXTRA_ECD_TO_DEGREE  0.01098633f   //   360��/32768
//#define over_pluse     0    //�ò����������һ���֣�ʹ����ȫ��������
#define StepMotorTurn  1 //�жϲ����������
#define StepMotor_init_degreen  14.0f  //���������ʼλ�ýǶ�ֵ
#define StepMotor_get_cube_angle  34.0f  //ȡ��ʱ��Ҫ�趨�ĽǶ�

#define StepMotor_end_degreen   97.0f  //���������ʼλ�ýǶ�ֵ
#define StepMotor_check_degreen   103.0f  //���������ʼλ�ýǶ�ֵ

//�������ֵ����Լ���ֵ
#define Extra_Half_ecd_range 16384    //15bit�ı�����һ��ı���ֵ
#define Extra_ecd_range 32767        //15bit�ı������ı���ֵ32768

typedef struct 
{
	PidTypeDef setp_motor_pid;
	fp32 relative_angel;
  fp32 relative_angle_set;
  fp32 speed;
	fp32 speed_out;
	int frequency;
	fp32 Real_DegreeError;
	fp32 Compensate_DegreeError;
	uint16_t min_angle;
	u8 DC_motor_stop;
}step_motor_t;


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
} Step_PID_t;

void StepMotor_PWM_Init(u32 arr , u32 psc);
//void StepMotor_Control(char  step_motor_position);
void PWM_GPIOH_Init(u32 arr, u32 psc);
void TIM5_CH1_Cap_Init(u32 arr,u16 psc);
void StepMotorRelativeControl(fp32 step_motor_position_set, fp32 ecd);
static fp32 step_motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
double degree_abs( double value);
void motor_position(void);
//void Serve_motor_Clockwise_90();
//void Serve_motor_Clockwise_180();
//void Serve_motor_Anticlockwise_90();
//void Serve_motor_Run_To_Balance_Init(); 
#define StepMotor_Clockwise       GPIO_ResetBits(GPIOI, GPIO_Pin_0)//GPIO���õ�    ˳ʱ��
#define StepMotor_Anticlockwise   GPIO_SetBits(GPIOI,	GPIO_Pin_0)//GPIO���ø�      ��ʱ��


