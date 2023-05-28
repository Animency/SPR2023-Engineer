
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __GIMBAL_TASK_H__
#define __GIMBAL_TASK_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

	
/* ******************************************************************
				              	AGV_CHASIS
****************************************************************** */	
//pitch speed close-loop PID params, max out and max iout
//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_SPEED_PID_KP        2900.0f
#define PITCH_SPEED_PID_KI        60.0f
#define PITCH_SPEED_PID_KD        0.0f
#define PITCH_SPEED_PID_MAX_OUT   30000.0f
#define PITCH_SPEED_PID_MAX_IOUT  10000.0f

//yaw speed close-loop PID params, max out and max iout
//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_SPEED_PID_KP        3600.0f
#define YAW_SPEED_PID_KI        20.0f
#define YAW_SPEED_PID_KD        0.0f
#define YAW_SPEED_PID_MAX_OUT   30000.0f
#define YAW_SPEED_PID_MAX_IOUT  5000.0f

//pitch gyro angle close-loop PID params, max out and max iout
//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_GYRO_ABSOLUTE_PID_KP 15.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f

#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyro angle close-loop PID params, max out and max iout
//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_GYRO_ABSOLUTE_PID_KP        26.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch encode angle close-loop PID params, max out and max iout
//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_ENCODE_RELATIVE_PID_KP 15.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 0.00f
#define PITCH_ENCODE_RELATIVE_PID_KD 0.0f

#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 10.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 0.0f

//yaw encode angle close-loop PID params, max out and max iout
//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_ENCODE_RELATIVE_PID_KP        8.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.0f
#define YAW_ENCODE_RELATIVE_PID_KD        0.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   10.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  0.0f


//任务初始化 空闲一段时间
#define GIMBAL_TASK_INIT_TIME 201
//yaw,pitch控制通道以及状态开关通道
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_LEFT_CHANNEL 1
#define GIMBAL_MODE_RIGHT_CHANNEL 0
#define GIMBAL_ZUOCE_CHANNEL 4
#define GIMBAL_ZUOCE_CHANNEL_SUBSTITUTE 3
#define GIMBAL_ZUOCE_CHANNEL_LEFT_RIGHT 2

//turn 180°
//掉头180 按键
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn speed
//掉头云台速度
#define TURN_SPEED    0.04f
//测试按键尚未使用
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R
//rocker value deadband
//遥控器输入死区，因为遥控器存在差异，摇杆在中间，其值不一定为零
#define RC_DEADBAND   10


//电机最大速度
#define NORMAL_MAX_GIMBAL_SPEED 10.0f

#define YAW_RC_SEN    -0.000005f
#define PITCH_RC_SEN  -0.000006f //0.005

#define YAW_MOUSE_SEN   0.00005f
#define PITCH_MOUSE_SEN 0.00015f

#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

#define GIMBAL_CONTROL_TIME 1

//test mode, 0 close, 1 open
//云台测试模式 宏定义 0 为不使用测试模式
#define GIMBAL_TEST_MODE 0

#define PITCH_TURN  1
#define YAW_TURN    0

//电机码盘值最大以及中值
#define HALF_ECD_RANGE  4096
#define ECD_RANGE       8191
//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
#define GIMBAL_INIT_STOP_TIME       100
#define GIMBAL_INIT_TIME            6000
#define GIMBAL_CALI_REDUNDANT_ANGLE 0.1f
//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED     0.004f
#define GIMBAL_INIT_YAW_SPEED       0.005f

#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

//云台校准中值的时候，发送原始电流值，以及堵转时间，通过陀螺仪判断堵转
#define GIMBAL_CALI_MOTOR_SET   8000
#define GIMBAL_CALI_STEP_TIME   2000
#define GIMBAL_CALI_GYRO_LIMIT  0.1f

#define GIMBAL_CALI_PITCH_MAX_STEP  1
#define GIMBAL_CALI_PITCH_MIN_STEP  2
#define GIMBAL_CALI_YAW_MAX_STEP    3
#define GIMBAL_CALI_YAW_MIN_STEP    4

#define GIMBAL_CALI_START_STEP  GIMBAL_CALI_PITCH_MAX_STEP
#define GIMBAL_CALI_END_STEP    5

//判断遥控器无输入的时间以及遥控器无输入判断，设置云台yaw回中值以防陀螺仪漂移
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//2006电机rpm转换成齿轮速度m/s
#define MOTOR_RPM_TO_SPEED_2006 0.000493706f

#define GIMBAL_PI 3.1415926535898
#define GIMBAL_DEC (GIMBAL_PI/180)
//6020电机rpm转换成齿轮速度m/s
#define GIMBAL_RPM_TO_SPEED_6020  0.003492400f
//3508电机rpm转换成齿轮速度m/s
#define GIMBAL_RPM_TO_SPEED_3508 0.00219905f
//电机编码值转化成角度值
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif
//云台遥控器与速度转换比例
#define GIMBAL_VEL 0.001
#define GIMBAL_ZHUAN_VEL 0.0005


//2023工程电子限位角度值
#define TARGET_CAN2_201_MAX 480
#define TARGET_CAN2_202_MAX -10
#define TARGET_CAN2_204_MAX 110
#define TARGET_CAN2_205_MAX 1100
#define TARGET_CAN2_206_MAX -20
#define TARGET_CAN2_207_6020_MAX 720
#define TARGET_CAN2_208_MAX 600
#define PWM_SERVO_MAX 2500

#define TARGET_CAN2_201_MIN -480
#define TARGET_CAN2_202_MIN -720
#define TARGET_CAN2_204_MIN -110
#define TARGET_CAN2_205_MIN -20
#define TARGET_CAN2_206_MIN -1100
#define TARGET_CAN2_207_6020_MIN -720
#define TARGET_CAN2_208_MIN -8
#define PWM_SERVO_MIN	500

typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //电机原始值控制
    GIMBAL_MOTOR_GYRO,    //电机陀螺仪角度控制
    GIMBAL_MOTOR_ENCONDE, //电机编码值角度控制
} gimbal_motor_mode_e;

typedef struct
{
    float kp;
    float ki;
    float kd;

    float set;
    float get;
    float err;

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    pid_type_def gimbal_motor_angle_pid;
	  pid_type_def gimbal_motor_speed_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;
    float max_relative_angle; //rad
    float min_relative_angle; //rad
    
	  float absolute_angle_current;
    float relative_angle;     //rad
    float relative_angle_set; //rad
    float absolute_angle;     //rad
    float absolute_angle_set; //rad
    float motor_gyro;         //rad/s
    float motor_gyro_set;
    float motor_speed_current;
    float raw_cmd_current;
    float current_set;
		float speed_set;
		float speed_max;
		float speed_min;
    int16_t given_current;
	  int16_t give_current;

} gimbal_motor_t;

typedef struct
{
    float max_yaw;
    float min_yaw;
    float max_pitch;
    float min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;
//用于本次工程所有的四个传感器
typedef struct
{
	//air_pump 是位于爪子吸盘处两个传感器，用于判断是否吸住矿石
	uint8_t air_pump_flag_left;
	uint8_t air_pump_flag_right;
	//laser 是位于爪子两侧的传感器，用于判断是否对准矿石
	uint8_t laser_flag_left;
	uint8_t laser_flag_right;
}ore_flag_t;
typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const float *gimbal_INT_angle_point;
    const float *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
	  gimbal_motor_t horizontal_scroll_motor[7];
	  gimbal_motor_t gimbal_6020_motor;
    gimbal_step_cali_t gimbal_cali;
		ore_flag_t ore_flag;
} gimbal_control_t;

//增加电子限位
#define electric_limit(input, output, maxdealine, mindealine)   \
  {                                                  						\
    if (((input) >= (mindealine)) && ((input) <= (maxdealine))) \
    {                                                						\
      (output) = (input);                          							\
    }                                                						\
    else if ((input) > (maxdealine))                            \
    {                                                						\
      (output) = (maxdealine);                            			\
    }                                                						\
		else if((input) < (mindealine))                             \
		{                                                           \
			(output) = (mindealine);                                  \
		}                                                           \
  }                                                             \
/**
  * @brief          返回yaw 电机数据指针
  * @param[in]      none
  * @retval         yaw电机指针
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          return pitch motor data point
  * @param[in]      none
  * @retval         pitch motor data point
  */
/**
  * @brief          返回pitch 电机数据指针
  * @param[in]      none
  * @retval         pitch
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

extern void gimbal_task(void const *pvParameters);


#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
