 /**************************************/
 /******      IDʹ��˵����        ******/
 /******   0: ����ƽ�Ƶ��        ******/
 /******   1������ƽ�Ƶ��        ******/
 /******   2����е��ˮƽ��ת���  ******/   //���޸�Ϊ6020�ҷ���IDΪ3����ʶ��Ϊ0x1ff,����IDΪ0x207
 /******   3����е����ֱ�������  ******/
 /******   4���෽̧�����        ******/
 /******   5���෽̧�����        ******/
 /**************************************/
 
 
 /******     2:���ϽǶ�����     ********/
 /******    3:��ʱ��Ƕ�����    ********/
 /******       4:�����װ       *******/
 /******       5:�����װ       *******/
#ifndef __ENGINEER_HEHAVIOR_H__
#define __ENGINEER_HEHAVIOR_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdio.h"
  /*-----------------------------------------------------���̿��������Ⱥ궨��--------------------------------------------------------*/
#define KEYBOARD_CONTROL_ANGLE_CAN2_201_CHANGE 5
#define KEYBOARD_CONTROL_ANGLE_CAN2_202_CHANGE 5
#define KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE 3
#define KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE 8
#define KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE 1

#define KEYBOARD_CONTROL_ANGLE_MECHANICAL_ARM_COORDINATES_CHANGE 5
#define KEYBOARD_CONTROL_ANGLE_SERVO_COORDINATES_CHANGE 4
#define MOUSE_CONTROL_ANGLE_CAN2_201_CHANGE  0.4 /*����Ϊ4*/
#define MOUSE_CONTROL_ANGLE_CAN2_202_CHANGE  0

#define ANGLE_TO_ECD 0.017453292519943f
#define RM_2023_ENGINEER_LIFTING_TO_HENG_SCALE 1.255813953488f
 /*-----------------------------------------------------�սӺ궨��--------------------------------------------------------*/
/**********��ʼ���궨��**************/
#define TARGET_CAN2_205_206_ANGLE_FORMATTING 2
#define TARGET_CAN2_207_6020_ANGLE_FORMATTING 213
#define TARGET_CAN2_208_ANGLE_FORMATTING 2

/**********��ʼ���궨��**************/
#define TARGET_AG_CAN2_202_ANGLE_INIT 1
#define TARGET_AG_CAN2_204_ANGLE_INIT 4
#define TARGET_AG_CAN2_205_ANGLE_INIT 4
#define TARGET_AG_CAN2_206_ANGLE_INIT 1
#define TARGET_AG_CAN2_207_ANGLE_6020_INIT 123
/**********̧���궨��**************/
#define TARGET_AG_CAN2_205_ANGLE_RISING 1061
#define TARGET_AG_CAN2_206_ANGLE_RISING 1061
/**********���ú궨��**************/
#define TARGET_AG_CAN2_207_ANGLE_6020_CENTER 213
/**********ǰ��궨��**************/
#define TARGET_AG_CAN2_202_ANGLE_PROTRACTING 360
/**********��ת�궨��**************/
#define TARGET_AG_CAN2_204_ANGLE_OVERTURN 90
#define TARGET_AG_CAN2_204_ANGLE 0x00
#define TARGET_AG_CAN2_205_ANGLE 0x00
#define TARGET_AG_CAN2_206_ANGLE 0x00

 /*-----------------------------------------------------����ȡ��궨��--------------------------------------------------------*/
/**********��ʼ���궨��**************/
#define TARGET_BTM_CAN2_202_ANGLE_INIT 1.0f
#define TARGET_BTM_CAN2_204_ANGLE_INIT 4.0f
#define TARGET_BTM_CAN2_205_ANGLE_INIT 4.0f
#define TARGET_BTM_CAN2_206_ANGLE_INIT 1.0f
#define TARGET_BTM_CAN2_207_ANGLE_6020_INIT 123.0f
/**********̧���궨��**************/
#define TARGET_BTM_CAN2_205_ANGLE_RISING 900.0f
#define TARGET_BTM_CAN2_206_ANGLE_RISING 900.0f
/**********ǰ��궨��**************/
#define TARGET_BTM_CAN2_202_ANGLE_Protract -100.0f
/**********�����궨��**************/
#define TARGET_BTM_CAN2_207_ANGLE_6020_RETURN 213.0f
/**********�ٴ������궨��**************/
#define TARGET_BTM_CAN2_205_ANGLE_RISING_STEP_2 1020.0f
#define TARGET_BTM_CAN2_206_ANGLE_RISING_STEP_2 1020.0f
/**********ǰ���ջغ궨��***********/
#define TARGET_BTM_CAN2_202_ANGLE_RETRACR 2.0f
/**********̧���½��궨��***********/
#define TARGET_BTM_CAN2_205_ANGLE_FALLING 100.0f
#define TARGET_BTM_CAN2_206_ANGLE_FALLING 100.0f
/**********������е�ۻ����½��궨��***********/
#define TARGET_BTM_CAN2_208_ANGLE_DOWN 0x00f
#define TARGET_BTM_CAN2_207_6020_ARMTURNAROUND 0.0f

/*-----------------------------------------------------�Զ����궨��--------------------------------------------------------*/
/**********̧���궨��**************/
#define TARGET_AOS_CAN2_205_ANGLE_RISING 0
#define TARGET_AOS_CAN2_206_ANGLE_RISING 0
/**********��ת�궨��**************/
#define TARGET_AOS_CAN2_204_ARM_OVERTURN 0
/**********��е�ۻ����궨��**************/
#define TARGET_AOS_CAN2_207_6020_ARM_TURNAROUND 0
/**********���궨��**************/
#define TARGET_AOS_CAN2_208_STORAGE_UP 0
/**********ǰ���ջغ궨��**************/
#define TARGET_AOS_CAN2_202_RETRACT 0
/*-----------------------------------------------------�Զ���λ�궨��--------------------------------------------------------*/
/**********row�ᶨ��**************/
#define TARGET_AMI_CAN2_203_MIDDLE 0
#define TARGET_AMI_CAN2_204_HORIZON 0
#define TARGET_AMI_CAN2_207_6020_HORIZON 213
#define TARGET_AMI_CAN2_202_PROTRACT 10
#define TARGET_AMI_CAN2_201_MIDDLE 2
#define TARGET_AMI_CAN2_205_ANGLE_DOWN 20
#define TARGET_AMI_CAN2_206_ANGLE_DOWN 20
/*-----------------------------------------------------��ؽṹ�嶨��--------------------------------------------------------*/
typedef struct
{
	uint8_t former_state;
	uint8_t current_state;
}keyboard_state_flag_t;
//����ȷ�������״̬
typedef struct
{
	keyboard_state_flag_t		key_F;
	keyboard_state_flag_t		key_C;
	keyboard_state_flag_t		key_B;
	uint8_t 				former_mouse_state;
	uint8_t 				current_mouse_state;

}mouse_keyboard_state_flag_t;
/*-----------------------------------------------------����ȡ��--------------------------------------------------------*/
extern void engineer_gimbal_behaviour_keyboard_control(void);


#endif 