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
  /*-----------------------------------------------------键盘控制宏定义--------------------------------------------------------*/
#define KEYBOARD_CONTROL_ANGLE_CAN2_201_CHANGE  0
#define KEYBOARD_CONTROL_ANGLE_CAN2_207_6020_CHANGE 0
#define KEYBOARD_CONTROL_ANGLE_CAN2_204_CHANGE 0
#define KEYBOARD_CONTROL_ANGLE_CAN2_205_206_CHANGE 0

#define MOUSE_CONTROL_ANGLE_CAN2_201_CHANGE  0
#define MOUSE_CONTROL_ANGLE_CAN2_202_CHANGE  0

 
 /*-----------------------------------------------------空接宏定义--------------------------------------------------------*/
/**********初始化宏定义**************/
#define TARGET_AG_CAN2_202_ANGLE_INIT 1
#define TARGET_AG_CAN2_204_ANGLE_INIT 4
#define TARGET_AG_CAN2_205_ANGLE_INIT 4
#define TARGET_AG_CAN2_206_ANGLE_INIT 1
#define TARGET_AG_CAN2_207_ANGLE_6020_INIT 1
/**********抬升宏定义**************/
#define TARGET_AG_CAN2_205_ANGLE_RISING 1061
#define TARGET_AG_CAN2_206_ANGLE_RISING 1061
/**********中置宏定义**************/
#define TARGET_AG_CAN2_207_ANGLE_6020_CENTER 85
/**********前伸宏定义**************/
#define TARGET_AG_CAN2_202_ANGLE_PROTRACTING 360
/**********翻转宏定义**************/
#define TARGET_AG_CAN2_204_ANGLE_OVERTURN 95
#define TARGET_AG_CAN2_204_ANGLE 0x00
#define TARGET_AG_CAN2_205_ANGLE 0x00
#define TARGET_AG_CAN2_206_ANGLE 0x00

 /*-----------------------------------------------------槽内取矿宏定义--------------------------------------------------------*/
/**********抬升宏定义**************/
#define TARGET_BTM_CAN2_205_ANGLE_RISING 0x00
#define TARGET_BTM_CAN2_206_ANGLE_RISING 0x00
/**********回正宏定义**************/
#define TARGET_BTM_CAN2_207_ANGLE_6020_RETURN 0x00
/**********前伸收回宏定义***********/
#define TARGET_BTM_CAN2_202_ANGLE_RETRACR 0x00
/**********存矿下降宏定义***********/
#define TARGET_BTM_CAN2_208_ANGLE_DOWN 0x00