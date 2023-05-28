#include "servo_task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "Engineer_behaviour.h"
//被征用于读取键盘状态
extern mouse_keyboard_state_flag_t mouse_keyboard_state_flag;
extern RC_ctrl_t rc_ctrl;
extern uint8_t coordinates_control_flag;
extern int low_speed;

void keyboardReadTask(void const *pvParameters)
{

    while (1)
    {
			mouse_keyboard_state_flag.key_C.former_state 	= ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) >> 13);	//位移为了使数据变成1变为标识符
			mouse_keyboard_state_flag.key_B.former_state  = ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) >> 15);
			//读取鼠标状态 用于判断当且仅当按下鼠标时 改变舵机状态
			mouse_keyboard_state_flag.former_mouse_state	=	(rc_ctrl.mouse.press_l	<< 1);
			mouse_keyboard_state_flag.former_mouse_state 	= (rc_ctrl.mouse.press_r | mouse_keyboard_state_flag.former_mouse_state);
      vTaskDelay(100);
			mouse_keyboard_state_flag.key_C.current_state = ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) >> 13); //位移为了使数据变成1变为标识符
			mouse_keyboard_state_flag.key_B.current_state = ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) >> 15);
			mouse_keyboard_state_flag.current_mouse_state	=	(rc_ctrl.mouse.press_l	<< 1);
			mouse_keyboard_state_flag.current_mouse_state = (rc_ctrl.mouse.press_r | mouse_keyboard_state_flag.current_mouse_state);
			
			//切换键盘控制方式标识符更新 C用来控制云台控制模式
			if(mouse_keyboard_state_flag.key_B.former_state != mouse_keyboard_state_flag.key_B.current_state)
			{
				coordinates_control_flag++;
				if(coordinates_control_flag > 1)
				{
					coordinates_control_flag = 0;
				}
			}
			
			//切换键盘控制方式标识符更新 C用来控制底盘控制模式
			if(mouse_keyboard_state_flag.key_C.former_state != mouse_keyboard_state_flag.key_C.current_state)
			{
				low_speed++;
				if(low_speed > 2)
				{
					low_speed = 0;
				}
			}
		}
}
