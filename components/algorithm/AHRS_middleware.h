
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      ��̬�����м�㣬Ϊ��̬�����ṩ��غ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef AHRS_MIDDLEWARE_H
#define AHRS_MIDDLEWARE_H

//���¶�Ӧ����������
typedef signed char int8_t;
typedef signed short int int16_t;
typedef signed int int32_t;
typedef signed long long int64_t;

/* exact-width unsigned integer types */
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;

//���� NULL
#ifndef NULL
#define NULL 0
#endif

//����PI ֵ
#ifndef PI
#define PI 3.14159265358979f
#endif

//���� �Ƕ�(��)ת���� ���ȵı���
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//���� ���� ת���� �Ƕȵı���
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

extern void AHRS_get_height(float *high);
extern void AHRS_get_latitude(float *latitude);
extern float AHRS_invSqrt(float num);
extern float AHRS_sinf(float angle);
extern float AHRS_cosf(float angle);
extern float AHRS_tanf(float angle);
extern float AHRS_asinf(float sin);
extern float AHRS_acosf(float cos);
extern float AHRS_atan2f(float y, float x);
#endif
