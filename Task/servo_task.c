#include "servo_task.h"
#include "cmsis_os.h"
#include "remote_control.h"
#include "Engineer_behaviour.h"
//�������ڶ�ȡ����״̬
extern mouse_keyboard_state_flag_t mouse_keyboard_state_flag;
extern RC_ctrl_t rc_ctrl;
extern uint8_t coordinates_control_flag;
extern int low_speed;

void keyboardReadTask(void const *pvParameters)
{

    while (1)
    {
			mouse_keyboard_state_flag.key_C.former_state 	= ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) >> 13);	//λ��Ϊ��ʹ���ݱ��1��Ϊ��ʶ��
			mouse_keyboard_state_flag.key_B.former_state  = ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) >> 15);
			//��ȡ���״̬ �����жϵ��ҽ����������ʱ �ı���״̬
			mouse_keyboard_state_flag.former_mouse_state	=	(rc_ctrl.mouse.press_l	<< 1);
			mouse_keyboard_state_flag.former_mouse_state 	= (rc_ctrl.mouse.press_r | mouse_keyboard_state_flag.former_mouse_state);
      vTaskDelay(100);
			mouse_keyboard_state_flag.key_C.current_state = ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_C) >> 13); //λ��Ϊ��ʹ���ݱ��1��Ϊ��ʶ��
			mouse_keyboard_state_flag.key_B.current_state = ((rc_ctrl.key.v & KEY_PRESSED_OFFSET_B) >> 15);
			mouse_keyboard_state_flag.current_mouse_state	=	(rc_ctrl.mouse.press_l	<< 1);
			mouse_keyboard_state_flag.current_mouse_state = (rc_ctrl.mouse.press_r | mouse_keyboard_state_flag.current_mouse_state);
			
			//�л����̿��Ʒ�ʽ��ʶ������ C����������̨����ģʽ
			if(mouse_keyboard_state_flag.key_B.former_state != mouse_keyboard_state_flag.key_B.current_state)
			{
				coordinates_control_flag++;
				if(coordinates_control_flag > 1)
				{
					coordinates_control_flag = 0;
				}
			}
			
			//�л����̿��Ʒ�ʽ��ʶ������ C�������Ƶ��̿���ģʽ
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
