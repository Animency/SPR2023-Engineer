 /**************************************/
 /******      ID使用说明：        ******/
 /******   0: 横轴平移电机        ******/
 /******   1：竖轴平移电机        ******/
 /******   2：机械臂水平旋转电机  ******/   //已修改为6020且发送ID为3，标识符为0x1ff,接收ID为0x207
 /******   3：机械臂竖直俯仰电机  ******/
 /******   4：侧方抬升电机        ******/
 /******   5：侧方抬升电机        ******/
 /**************************************/
 
 
 /******     2:向上角度增加     ********/
 /******    3:逆时针角度增加    ********/
 /******       4:电机正装       *******/
 /******       5:电机反装       *******/
#ifndef __ENGINEER_HEHAVIOR_H__
#define __ENGINEER_HEHAVIOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdio.h"
  /*-----------------------------------------------------键盘控制灵敏度宏定义--------------------------------------------------------*/
#define KEYBOARD_CONTROL_ANGLE_CAN2_201_CHANGE 5
#define KEYBOARD_CONTROL_ANGLE_CAN2_202_CHANGE 5
#define KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE 1
#define KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE 8
#define KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE 1
#define KEYBOARD_CONTROL_ANGLE_CAN2_208_CHANGE 2
#define KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE 5
#define KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE 4
#define MOUSE_CONTROL_ANGLE_CAN2_201_CHANGE  0.4 /*按键为4*/
#define MOUSE_CONTROL_ANGLE_CAN2_202_CHANGE  0

#define ANGLE_TO_ECD 0.017453292519943f
#define RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE 1.255813953488f  //此处由减速比得到
 /*-----------------------------------------------------空接宏定义--------------------------------------------------------*/
/**********初始化宏定义**************/
#define TARGET_CAN2_205_206_ANGLE_FORMATTING 2
#define TARGET_CAN2_207_6020_ANGLE_FORMATTING 213
#define TARGET_CAN2_208_ANGLE_FORMATTING 2

/**********初始化宏定义**************/
#define TARGET_AG_CAN2_202_ANGLE_INIT 1
#define TARGET_AG_CAN2_204_ANGLE_INIT 4
#define TARGET_AG_CAN2_205_ANGLE_INIT 4
#define TARGET_AG_CAN2_206_ANGLE_INIT 1
#define TARGET_AG_CAN2_207_ANGLE_6020_INIT 123
/**********抬升宏定义**************/
#define TARGET_AG_CAN2_205_ANGLE_RISING 1061
#define TARGET_AG_CAN2_206_ANGLE_RISING 1061
/**********中置宏定义**************/
#define TARGET_AG_CAN2_207_ANGLE_6020_CENTER 213
/**********前伸宏定义**************/
#define TARGET_AG_CAN2_202_ANGLE_PROTRACTING 360
/**********翻转宏定义**************/
#define TARGET_AG_CAN2_204_ANGLE_OVERTURN 90
#define TARGET_AG_CAN2_204_ANGLE 0x00
#define TARGET_AG_CAN2_205_ANGLE 0x00
#define TARGET_AG_CAN2_206_ANGLE 0x00

 /*-----------------------------------------------------槽内取矿宏定义--------------------------------------------------------*/
/**********初始化宏定义**************/
#define TARGET_BTM_CAN2_202_ANGLE_INIT 1.0f
#define TARGET_BTM_CAN2_204_ANGLE_INIT 4.0f
#define TARGET_BTM_CAN2_205_ANGLE_INIT 4.0f
#define TARGET_BTM_CAN2_206_ANGLE_INIT 1.0f
#define TARGET_BTM_CAN2_207_ANGLE_6020_INIT 123.0f
/**********抬升宏定义**************/
#define TARGET_BTM_CAN2_205_ANGLE_RISING 300.0f
#define TARGET_BTM_CAN2_206_ANGLE_RISING 300.0f
/**********前伸宏定义**************/
#define TARGET_BTM_CAN2_202_ANGLE_Protract -100.0f
/**********回正宏定义**************/
#define TARGET_BTM_CAN2_207_ANGLE_6020_RETURN 213.0f
/**********再次上升宏定义**************/
#define TARGET_BTM_CAN2_205_ANGLE_RISING_STEP_2 900.0f
#define TARGET_BTM_CAN2_206_ANGLE_RISING_STEP_2 900.0f
/**********前伸与回存矿收回宏定义***********/
#define TARGET_BTM_CAN2_202_ANGLE_RETRACR 200.0f
#define TARGET_STM_CAN2_207_6020_ANGLE_ORE 33.0f
#define TARGET_BTM_CAN2_204_ANGLE_OVERTURN 98.0f
/**********抬升下降宏定义***********/
#define TARGET_BTM_CAN2_205_ANGLE_FALLING 10.0f
#define TARGET_BTM_CAN2_206_ANGLE_FALLING 10.0f
/**********存矿与机械臂回正下降宏定义***********/
#define TARGET_BTM_CAN2_204_ANGLE_OVERTURN_SECOND -20.0f
#define TARGET_BTM_CAN2_208_ANGLE_DOWN 0.0f
#define TARGET_BTM_CAN2_208_ANGLE_UP 80.0f
/**********存矿处取矿宏定义***********/
#define TARGET_BTM_CAN2_202_ANGLE_RETRACT_ORE 2.0f
#define TARGET_BTM_CAN2_204_ANGLE_OVERTURN_ORE 98.0f
#define TARGET_BTM_CAN2_202_ANGLE_PROTRACT_ORE 500.0f
#define TARGET_BTM_CAN2_207_6020_ANGLE_ARM_RETURN_ORE 213.0f

/*-----------------------------------------------------自动存矿宏定义--------------------------------------------------------*/
/**********抬升宏定义**************/
#define TARGET_AOS_CAN2_205_ANGLE_DOWN 5.0f
#define TARGET_AOS_CAN2_206_ANGLE_DOWN -5.0f
/**********横移回正宏定义**************/
#define TARGET_AOS_CAN2_202_HORIZON_CENTER 5.0f
/**********机械臂回正宏定义**************/
#define TARGET_AOS_CAN2_207_6020_ARM_TURNAROUND 213.0f
/**********舵机中置宏定义**************/
#define TARGET_CAN2_203_SERVO_INIT 90.0f
/**********小臂翻转宏定义**************/
#define TARGET_AOS_CAN2_204_OVERTURN_HORIZON 60.0f
/**********前伸收回宏定义**************/
#define TARGET_AOS_CAN2_202_RETRACT_FIRST 15.0f
/**********存矿宏定义**************/
#define TARGET_AOS_CAN2_208_STORAGE_UP 0
/**********前伸收回宏定义**************/
#define TARGET_AOS_CAN2_202_RETRACT 0
/*-----------------------------------------------------自动置位宏定义--------------------------------------------------------*/
/**********row轴定义**************/
#define TARGET_AMI_CAN2_203_MIDDLE 0
#define TARGET_AMI_CAN2_204_HORIZON 0
#define TARGET_AMI_CAN2_207_6020_HORIZON 57.0f
#define TARGET_AMI_CAN2_202_PROTRACT 10
#define TARGET_AMI_CAN2_201_MIDDLE 2
#define TARGET_AMI_CAN2_205_ANGLE_DOWN 20
#define TARGET_AMI_CAN2_206_ANGLE_DOWN 20
/*-----------------------------------------------------相关结构体定义--------------------------------------------------------*/
typedef struct
{
	uint8_t former_state;
	uint8_t current_state;
}keyboard_state_flag_t;
//用于确定与键盘状态
typedef struct
{
	keyboard_state_flag_t		key_F;
	keyboard_state_flag_t		key_C;
	keyboard_state_flag_t		key_B;
	uint8_t 				former_mouse_state;
	uint8_t 				current_mouse_state;

}mouse_keyboard_state_flag_t;
/*-----------------------------------------------------函数取回--------------------------------------------------------*/
extern void engineer_gimbal_behaviour_keyboard_control(void);


#endif 