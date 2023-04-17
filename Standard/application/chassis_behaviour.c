/**
 * @file chassis_behaviour.c/h
 * @author ���廪
 * @brief ����ң������ֵ������������Ϊ��
 * @version 0.1
 * @date 2022-03-08
 * @verbatim
  ==============================================================================
    ���Ҫ���һ���µ���Ϊģʽ
    1.����,��chassis_behaviour.h�ļ��У� ���һ������Ϊ������ chassis_behaviour_e
    erum
    {
        ...
        ...
        CHASSIS_XXX_XXX, // ����ӵ�
    }chassis_behaviour_e,

    2. ʵ��һ���µĺ��� chassis_xxx_xxx_control(float *vx, float *vy, float *wz, chassis_move_t * chassis )
        "vx,vy,wz" �����ǵ����˶�����������
        ��һ������: 'vx' ͨ�����������ƶ�,��ֵ ǰ���� ��ֵ ����
        �ڶ�������: 'vy' ͨ�����ƺ����ƶ�,��ֵ ����, ��ֵ ����
        ����������: 'wz' �����ǽǶȿ��ƻ�����ת�ٶȿ���
        ������µĺ���, ���ܸ� "vx","vy",and "wz" ��ֵ��Ҫ���ٶȲ���
    3.  ��"chassis_behaviour_mode_set"��������У�����µ��߼��жϣ���chassis_behaviour_mode��ֵ��CHASSIS_XXX_XXX
        �ں���������"else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)" ,Ȼ��ѡ��һ�ֵ��̿���ģʽ
        4��:
        CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ��̨�͵��̵���ԽǶ�
        �����������"xxx_angle_set"������'wz'
        CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'�ǽǶȿ��� ���̵������Ǽ�����ľ��ԽǶ�
        �����������"xxx_angle_set"
        CHASSIS_VECTOR_NO_FOLLOW_YAW : 'vx' and 'vy'���ٶȿ��ƣ� 'wz'����ת�ٶȿ���
        CHASSIS_VECTOR_RAW : ʹ��'vx' 'vy' and 'wz'ֱ�����Լ�������ֵĵ���ֵ������ֵ��ֱ�ӷ��͵�can ������
    4.  ��"chassis_behaviour_control_set" ������������
        else if(chassis_behaviour_mode == CHASSIS_XXX_XXX)
        {
            chassis_xxx_xxx_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
        }
  ==============================================================================
 *
 * @copyright Copyright (c) 2022 SPR
 *
 */
#include "chassis_behaviour.h"
#include "cmsis_os.h"
#include "chassis_task.h"
// #include "arm_math.h"
#include "math.h"
#include "gimbal_behaviour.h"

extern 	float vx_set_zhuanghuan,vy_set_zhuanghuan,wz_set_zhuanghuan;
/**
 * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
 * @author         RM
 * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_zero_force_control(float *vx_can_set, float *vy_can_set, float *wz_can_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
 * @author         RM
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
 * @author         RM
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
 * @author         RM
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);

/**
 * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         none
 */

static void chassis_open_set_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);

// highlight, the variable chassis behaviour mode
//���⣬���������Ϊģʽ����
//chassis_behaviour_e chassis_behaviour_mode = CHASSIS_ZERO_FORCE;
static void chassis_Debug_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
static void chassis_remote_control_normal(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector);
/**
 * @brief          ͨ���߼��жϣ���ֵ"chassis_behaviour_mode"������ģʽ
 * @param[in]      chassis_move_mode: ��������
 * @retval         none
 */
void chassis_behaviour_mode_set(chassis_move_t *chassis_move_mode)
{
  if (chassis_move_mode == NULL)
  {
    return;
  }

//  //ң��������ģʽ
//  if (SWITCH_LEFT_IS_MID(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]))
//  {
//    //�������˶�
//    chassis_behaviour_mode = CHASSIS_NO_FOLLOW_YAW;
//  }
//  else if (SWITCH_LEFT_IS_DOWN(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]))
//  {
//    //ֹͣ�˶�
//    chassis_behaviour_mode = CHASSIS_NO_MOVE;
//  }
//  else if (SWITCH_LEFT_IS_UP(chassis_move_mode->chassis_RC->rc.s[CHASSIS_MODE_LEFT_CHANNEL]))
//  {
//    //������̨�˶�
//    chassis_behaviour_mode = CHASSIS_FOLLOW_GIMBAL_YAW;
//  }

//  //����̨��ĳЩģʽ�£����ʼ���� ���̲���
//  if (gimbal_cmd_to_chassis_stop())
//  {
//    chassis_behaviour_mode = CHASSIS_NO_MOVE;
//  }

  //����Լ����߼��жϽ�����ģʽ

  //������Ϊģʽѡ��һ�����̿���ģʽ
//  if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
//  }
//  else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
//  }
//  else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
//  }
//  else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_NO_FOLLOW_YAW;
//  }
//  else if (chassis_behaviour_mode == CHASSIS_OPEN)
//  {
//    chassis_move_mode->chassis_mode = CHASSIS_VECTOR_RAW;
//  }
    
		
}

/**
 * @brief          ���ÿ�����.���ݲ�ͬ���̿���ģʽ��������������Ʋ�ͬ�˶�.������������棬����ò�ͬ�Ŀ��ƺ���.
 * @param[out]     vx_set, ͨ�����������ƶ�.
 * @param[out]     vy_set, ͨ�����ƺ����ƶ�.
 * @param[out]     wz_set, ͨ��������ת�˶�.
 * @param[in]      chassis_move_rc_to_vector,  ��������������Ϣ.
 * @retval         none
 */
void chassis_behaviour_control_set(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{

  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

//  if (chassis_behaviour_mode == CHASSIS_ZERO_FORCE)
//  {
//    chassis_zero_force_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
//  else if (chassis_behaviour_mode == CHASSIS_NO_MOVE)
//  {
//    chassis_no_move_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
//  else if (chassis_behaviour_mode == CHASSIS_FOLLOW_GIMBAL_YAW)
//  {
//    chassis_follow_gimbal_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
//  else if (chassis_behaviour_mode == CHASSIS_NO_FOLLOW_YAW)
//  {
//    chassis_no_follow_yaw_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
//  else if (chassis_behaviour_mode == CHASSIS_OPEN)
//  {
//    chassis_open_set_control(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
//  }
  chassis_remote_control_normal(vx_set, vy_set, angle_set, chassis_move_rc_to_vector);
	
}

/**
 * @brief          ������������Ϊ״̬���£�����ģʽ��raw���ʶ��趨ֵ��ֱ�ӷ��͵�can�����Ϲʶ����趨ֵ������Ϊ0
 * @author         RM
 * @param[in]      vx_setǰ�����ٶ� �趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      vy_set���ҵ��ٶ� �趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      wz_set��ת���ٶ� �趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_zero_force_control(float *vx_can_set, float *vy_can_set, float *wz_can_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_can_set == NULL || vy_can_set == NULL || wz_can_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  *vx_can_set = 0.0f;
  *vy_can_set = 0.0f;
  *wz_can_set = 0.0f;
}

/**
 * @brief          ���̲��ƶ�����Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�
 * @author         RM
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      wz_set��ת���ٶȣ���ת�ٶ��ǿ��Ƶ��̵ĵ��̽��ٶ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_move_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
  *vx_set = 0.0f;
  *vy_set = 0.0f;
  *wz_set = 0.0f;
}

/**
 * @brief          ���̸�����̨����Ϊ״̬���£�����ģʽ�Ǹ�����̨�Ƕȣ�������ת�ٶȻ���ݽǶȲ���������ת�Ľ��ٶ�
 * @author         RM
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      angle_set��������̨���Ƶ�����ԽǶ�
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_follow_gimbal_yaw_control(float *vx_set, float *vy_set, float *angle_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || angle_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }
}

/**
 * @brief          ���̲�����Ƕȵ���Ϊ״̬���£�����ģʽ�ǲ�����Ƕȣ�������ת�ٶ��ɲ���ֱ���趨
 * @author         RM
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶ�,��ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      wz_set�������õ���ת�ٶ�,��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         ���ؿ�
 */
static void chassis_no_follow_yaw_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  chassis_rc_to_control_vector(vx_set, vy_set, chassis_move_rc_to_vector);
  *wz_set = -CHASSIS_WZ_RC_SEN * chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL];
}

/**
 * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         none
 */
static void chassis_open_set_control(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  *wz_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_OPEN_RC_SCALE;
  return;
}
/**
 * @brief          ���̿�������Ϊ״̬���£�����ģʽ��rawԭ��״̬���ʶ��趨ֵ��ֱ�ӷ��͵�can������
 * @param[in]      vx_setǰ�����ٶ�,��ֵ ǰ���ٶȣ� ��ֵ �����ٶ�
 * @param[in]      vy_set���ҵ��ٶȣ���ֵ �����ٶȣ� ��ֵ �����ٶ�
 * @param[in]      wz_set ��ת�ٶȣ� ��ֵ ��ʱ����ת����ֵ ˳ʱ����ת
 * @param[in]      chassis_move_rc_to_vector��������
 * @retval         none
 */
static void chassis_remote_control_normal(float *vx_set, float *vy_set, float *wz_set, chassis_move_t *chassis_move_rc_to_vector)
{
  if (vx_set == NULL || vy_set == NULL || wz_set == NULL || chassis_move_rc_to_vector == NULL)
  {
    return;
  }

  *vx_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL] * CHASSIS_REMOTE_CONTROL_CHANGE_TO_VEL;
  *vy_set = -chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_REMOTE_CONTROL_CHANGE_TO_VEL;//-chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL] * CHASSIS_REMOTE_CONTROL_CHANGE_TO_VEL
	*wz_set = chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_WZ_CHANNEL] * CHASSIS_WZ_RC_SEN;
  return;
}
