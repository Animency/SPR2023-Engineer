/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       INS_task.c/h
  * @brief      use bmi088 to calculate the euler angle. no use ist8310, so only
  *             enable data ready pin to save cpu time.enalbe bmi088 data ready
  *             enable spi DMA to save the time spi transmit
  *             主要利用陀螺仪bmi088，磁力计ist8310，完成姿态解算，得出欧拉角，
  *             提供通过bmi088的data ready 中断完成外部触发，减少数据等待延迟
  *             通过DMA的SPI传输节约CPU时间.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Nov-11-2019     RM              1. support bmi088, but don't support mpu6500
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "INS_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "bsp_imu_pwm.h"
#include "bsp_spi.h"
#include "bsp_buzzer.h"
#include "bsp_flash.h"
#include "bmi088driver.h"
#include "ist8310driver.h"
#include "string.h"
#include "pid.h"
#include "ahrs.h"
#include "detect_task.h"
#include "remote_control.h"

#define FLASH_WRITE_BUF_LENGHT (sizeof(imu_cali_t) * 3)

#define IMU_temp_PWM(pwm) imu_pwm_set(pwm) // pwm给定

#define BMI088_BOARD_INSTALL_SPIN_MATRIX \
  {0.0f, 1.0f, 0.0f},                    \
      {-1.0f, 0.0f, 0.0f},               \
  {                                      \
    0.0f, 0.0f, 1.0f                     \
  }

#define IST8310_BOARD_INSTALL_SPIN_MATRIX \
  {1.0f, 0.0f, 0.0f},                     \
      {0.0f, 1.0f, 0.0f},                 \
  {                                       \
    0.0f, 0.0f, 1.0f                      \
  }

/**
 * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
 * @param[out]     gyro: 加上零漂和旋转
 * @param[out]     accel: 加上零漂和旋转
 * @param[out]     mag: 加上零漂和旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @param[in]      ist8310: 磁力计数据
 * @retval         none
 */
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310);

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(float temp);

/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void);

/**
 * @brief 获取imu数据
 *
 * @param data：imu数据结构指针
 */
void get_INS_data(INS_data_t *data);

/**
  * @brief          陀螺仪设备校准
  * @param[in][out] cali:指针指向陀螺仪数据,当cmd为CALI_FUNC_CMD_INIT, 参数是输入,CALI_FUNC_CMD_ON,参数是输出
  * @param[in]      cmd:
                    CALI_FUNC_CMD_INIT: 代表用校准数据初始化原始数据
                    CALI_FUNC_CMD_ON: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd); // gyro device cali function

/**
 * @brief          从flash读取校准数据
 * @param[in]      none
 * @retval         none
 */
static void gyro_cali_data_read(void);

/**
 * @brief          往flash写入校准数据
 * @param[in]      none
 * @retval         none
 */
static void gyro_cali_data_write(void);

//static void return_INS_data(void);

static void get_key(void);

static uint8_t flash_write_buf[FLASH_WRITE_BUF_LENGHT];
static uint8_t cali_done;
static uint8_t gyro_cali_cmd;
static uint8_t key_gpio_state;
static uint8_t exit_flag;

extern SPI_HandleTypeDef hspi1;

static TaskHandle_t INS_task_local_handler;

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

bmi088_real_data_t bmi088_real_data;
float gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float gyro_offset[3];
float gyro_cali_offset[3];

float accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
float accel_offset[3];
float accel_cali_offset[3];

ist8310_real_data_t ist8310_real_data;
float mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
float mag_offset[3];
float mag_cali_offset[3];

INS_data_t INS_data;

static uint8_t first_temperate;
static pid_type_def imu_temp_pid;

static const float timing_time = 0.001f; // tast run time , unit s.任务运行的时间 单位 s

//加速度计低通滤波
static float accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static float accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const float fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};

static float INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static float INS_accel[3] = {0.0f, 0.0f, 0.0f};
static float INS_mag[3] = {0.0f, 0.0f, 0.0f};
static float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f}; // euler angle, unit rad.欧拉角 单位 rad

/**
 * @brief          imu任务, 初始化 bmi088, ist8310, 计算欧拉角
 * @param[in]      pvParameters: NULL
 * @retval         none
 */

void INS_task(void const *pvParameters)
{
  // wait a time
  osDelay(INS_TASK_INIT_TIME);
  while (BMI088_init())
  {
    osDelay(100);
  }
  while (ist8310_init())
  {
    osDelay(100);
  }

  BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);

  gyro_cali_data_read();
  // rotate and zero drift
  imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

  PID_init(&imu_temp_pid, TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD, TEMPERATURE_PID_MAX_OUT, TEMPERATURE_PID_MAX_IOUT);
  AHRS_init(INS_quat, INS_accel, INS_mag);

  accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
  accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
  accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
  // get the handle of task
  //获取当前任务的任务句柄，
  INS_task_local_handler = xTaskGetHandle(pcTaskGetName(NULL));

  // set spi frequency
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;

  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }

  SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);

  imu_start_dma_flag = 1;

  while (1)
  {
    //等待SPI DMA传输
    while (ulTaskNotifyTake(pdTRUE, portMAX_DELAY) != pdPASS)
    {
    }

    if (gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
    {
      gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
      BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
    }

    if (accel_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
    }

    if (accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
      imu_temp_control(bmi088_real_data.temp);
    }

    // rotate and zero drift
    imu_cali_slove(INS_gyro, INS_accel, INS_mag, &bmi088_real_data, &ist8310_real_data);

    //加速度计低通滤波
    accel_fliter_1[0] = accel_fliter_2[0];
    accel_fliter_2[0] = accel_fliter_3[0];

    accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

    accel_fliter_1[1] = accel_fliter_2[1];
    accel_fliter_2[1] = accel_fliter_3[1];

    accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

    accel_fliter_1[2] = accel_fliter_2[2];
    accel_fliter_2[2] = accel_fliter_3[2];

    accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];

    AHRS_update(INS_quat, timing_time, INS_gyro, accel_fliter_3, INS_mag);

    get_key();
    if (gyro_cali_cmd == CALI_FUNC_CMD_ON)
    {
      if (cali_gyro_hook((uint32_t *)&gyro_cali, CALI_FUNC_CMD_ON))
      {
        gyro_cali_cmd = 0;
      }
    }

    get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
    get_INS_data(&INS_data);

    // because no use ist8310 and save time, no use
    if (mag_update_flag &= 1 << IMU_DR_SHFITS)
    {
      mag_update_flag &= ~(1 << IMU_DR_SHFITS);
      mag_update_flag |= (1 << IMU_SPI_SHFITS);
      // ist8310_read_mag(ist8310_real_data.mag);
    }
  }
}

/**
 * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
 * @param[out]     gyro: 加上零漂和旋转
 * @param[out]     accel: 加上零漂和旋转
 * @param[out]     mag: 加上零漂和旋转
 * @param[in]      bmi088: 陀螺仪和加速度计数据
 * @param[in]      ist8310: 磁力计数据
 * @retval         none
 */
static void imu_cali_slove(float gyro[3], float accel[3], float mag[3], bmi088_real_data_t *bmi088, ist8310_real_data_t *ist8310)
{
  for (uint8_t i = 0; i < 3; i++)
  {
    gyro[i] = bmi088->gyro[0] * gyro_scale_factor[i][0] + bmi088->gyro[1] * gyro_scale_factor[i][1] + bmi088->gyro[2] * gyro_scale_factor[i][2] + gyro_offset[i];
    accel[i] = bmi088->accel[0] * accel_scale_factor[i][0] + bmi088->accel[1] * accel_scale_factor[i][1] + bmi088->accel[2] * accel_scale_factor[i][2] + accel_offset[i];
    mag[i] = ist8310->mag[0] * mag_scale_factor[i][0] + ist8310->mag[1] * mag_scale_factor[i][1] + ist8310->mag[2] * mag_scale_factor[i][2] + mag_offset[i];
  }
}

/**
 * @brief          控制bmi088的温度
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_temp_control(fp32 temp)
{
  uint16_t tempPWM;
  static uint8_t temp_constant_time = 0;
  if (first_temperate)
  {
    PID_calc(&imu_temp_pid, temp, 45.0f);
    if (imu_temp_pid.out < 0.0f)
    {
      imu_temp_pid.out = 0.0f;
    }
    tempPWM = (uint16_t)imu_temp_pid.out;
    IMU_temp_PWM(tempPWM);
  }
  else
  {
    //在没有达到设置的温度，一直最大功率加热
    if (temp > 45.0f)
    {
      temp_constant_time++;
      if (temp_constant_time > 200)
      {
        //达到设置温度，将积分项设置为一半最大功率，加速收敛
        first_temperate = 1;
        imu_temp_pid.Iout = MPU6500_TEMP_PWM_MAX / 2.0f;
      }
    }

    IMU_temp_PWM(MPU6500_TEMP_PWM_MAX - 1);
  }
}

/**
 * @brief          计算陀螺仪零漂
 * @param[out]     gyro_offset:计算零漂
 * @param[in]      gyro:角速度数据
 * @param[out]     offset_time_count: 自动加1
 * @retval         none
 */
void gyro_offset_calc(float gyro_offset[3], float gyro[3], uint16_t *offset_time_count)
{
  if (gyro_offset == NULL || gyro == NULL || offset_time_count == NULL)
  {
    return;
  }

  gyro_offset[0] = gyro_offset[0] - 0.0003f * gyro[0];
  gyro_offset[1] = gyro_offset[1] - 0.0003f * gyro[1];
  gyro_offset[2] = gyro_offset[2] - 0.0003f * gyro[2];
  (*offset_time_count)++;
}

/**
 * @brief          校准陀螺仪
 * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
 * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
 * @retval         none
 */
void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count)
{
  if (*time_count == 0)
  {
    gyro_offset[0] = gyro_cali_offset[0];
    gyro_offset[1] = gyro_cali_offset[1];
    gyro_offset[2] = gyro_cali_offset[2];
  }
  gyro_offset_calc(gyro_offset, INS_gyro, time_count);

  cali_offset[0] = gyro_offset[0];
  cali_offset[1] = gyro_offset[1];
  cali_offset[2] = gyro_offset[2];
  cali_scale[0] = 1.0f;
  cali_scale[1] = 1.0f;
  cali_scale[2] = 1.0f;
}

/**
 * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
 * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
 * @param[in]      陀螺仪的零漂
 * @retval         none
 */
void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3])
{
  gyro_cali_offset[0] = cali_offset[0];
  gyro_cali_offset[1] = cali_offset[1];
  gyro_cali_offset[2] = cali_offset[2];
  gyro_offset[0] = gyro_cali_offset[0];
  gyro_offset[1] = gyro_cali_offset[1];
  gyro_offset[2] = gyro_cali_offset[2];
}

/**
 * @brief          获取四元数
 * @param[in]      none
 * @retval         INS_quat的指针
 */
const float *get_INS_quat_point(void)
{
  return INS_quat;
}

/**
 * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
 * @param[in]      none
 * @retval         INS_angle的指针
 */
const float *get_INS_angle_point(void)
{
  return INS_angle;
}

/**
 * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
 * @param[in]      none
 * @retval         INS_gyro的指针
 */
extern const float *get_gyro_data_point(void)
{
  return INS_gyro;
}

/**
 * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
 * @param[in]      none
 * @retval         INS_accel的指针
 */
extern const float *get_accel_data_point(void)
{
  return INS_accel;
}

/**
 * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
 * @param[in]      none
 * @retval         INS_mag的指针
 */
extern const float *get_mag_data_point(void)
{
  return INS_mag;
}

const INS_data_t *return_INS_data(void)
{
  return &INS_data;
}

/**
 * @brief 获取imu数据
 *
 * @param data：imu数据结构指针
 */
void get_INS_data(INS_data_t *data)
{
  data->angle_yaw = *(INS_angle + INS_YAW_ADDRESS_OFFSET);
  data->angle_pitch = *(INS_angle + INS_PITCH_ADDRESS_OFFSET);
  data->angle_roll = *(INS_angle + INS_ROLL_ADDRESS_OFFSET);

  data->wx = *(INS_gyro + INS_GYRO_X_ADDRESS_OFFSET);
  data->wy = *(INS_gyro + INS_GYRO_Y_ADDRESS_OFFSET);
  data->wz = *(INS_gyro + INS_GYRO_Z_ADDRESS_OFFSET);

  data->ax = *(INS_accel + INS_ACCEL_X_ADDRESS_OFFSET);
  data->ay = *(INS_accel + INS_ACCEL_Y_ADDRESS_OFFSET);
  data->az = *(INS_accel + INS_ACCEL_Z_ADDRESS_OFFSET);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == INT1_ACCEL_Pin)
  {
    detect_hook(BOARD_ACCEL_TOE);
    accel_update_flag |= 1 << IMU_DR_SHFITS;
    accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag)
    {
      imu_cmd_spi_dma();
    }
  }
  else if (GPIO_Pin == INT1_GYRO_Pin)
  {
    detect_hook(BOARD_GYRO_TOE);
    gyro_update_flag |= 1 << IMU_DR_SHFITS;
    if (imu_start_dma_flag)
    {
      imu_cmd_spi_dma();
    }
  }
  else if (GPIO_Pin == DRDY_IST8310_Pin)
  {
    detect_hook(BOARD_MAG_TOE);
    mag_update_flag |= 1 << IMU_DR_SHFITS;
  }
  else if (GPIO_Pin == KEY_Pin)
  {
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
      static BaseType_t xHigherPriorityTaskWoken;
      vTaskNotifyGiveFromISR(INS_task_local_handler, &xHigherPriorityTaskWoken);
      portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
  }
}

/**
 * @brief          根据imu_update_flag的值开启SPI DMA
 * @param[in]      temp:bmi088的温度
 * @retval         none
 */
static void imu_cmd_spi_dma(void)
{
  UBaseType_t uxSavedInterruptStatus;
  uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();

  //开启陀螺仪的DMA传输
  if ((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
    gyro_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }
  //开启加速度计的DMA传输
  if ((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    accel_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }

  if ((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN) && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
  {
    accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
    accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
    taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
    return;
  }
  taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
}

void DMA2_Stream2_IRQHandler(void)
{

  if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
  {
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

    // gyro read over
    //陀螺仪读取完毕
    if (gyro_update_flag & (1 << IMU_SPI_SHFITS))
    {
      gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
      gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    }

    // accel read over
    //加速度计读取完毕
    if (accel_update_flag & (1 << IMU_SPI_SHFITS))
    {
      accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }
    // temperature read over
    //温度读取完毕
    if (accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
    {
      accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
      accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

      HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }

    imu_cmd_spi_dma();

    if (gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
      gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
      gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
      __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
    }
  }
}

/**
  * @brief          陀螺仪设备校准
  * @param[in]      cmd:
                    0: 代表用校准数据初始化原始数据
                    1: 代表需要校准
  * @retval         0:校准任务还没有完
                    1:校准任务已经完成
  */
static bool_t cali_gyro_hook(uint32_t *cali, bool_t cmd)
{
  imu_cali_t *local_cali_t = (imu_cali_t *)cali;
  if (cmd == CALI_FUNC_CMD_INIT)
  {
    INS_set_cali_gyro(local_cali_t->scale, local_cali_t->offset);
    return 0;
  }
  else if (cmd == CALI_FUNC_CMD_ON)
  {
    static uint16_t count_time = 0;
    INS_cali_gyro(local_cali_t->scale, local_cali_t->offset, &count_time);
    if (count_time > GYRO_CALIBRATE_TIME)
    {
      count_time = 0;
      buzzer_off();                //关闭蜂鸣器
      RC_restart(SBUS_RX_BUF_NUM); //重启遥控
      cali_done = 0x55;
      gyro_cali_data_write();
      return 1;
    }
    else
    {
      RC_unable();          //失能遥控器
      buzzer_on(95, 10000); //蜂鸣器响
      return 0;
    }
  }

  return 0;
}

/**
 * @brief          往flash写入校准数据
 * @param[in]      none
 * @retval         none
 */
static void gyro_cali_data_write(void)
{
  uint16_t offset = 0;

  //拷贝陀螺仪校准数据
  memcpy((void *)(flash_write_buf + offset), (void *)&gyro_cali, sizeof(imu_cali_t));
  offset += sizeof(imu_cali_t);

  // copy the name and "CALI_FLAG" of device
  memcpy((void *)(flash_write_buf + offset), (void *)&cali_done, 1);

  //擦除数据
  flash_erase_address(ADDR_FLASH_SECTOR_9, 1);
  //写入数据
  flash_write_single_address(ADDR_FLASH_SECTOR_9, (uint32_t *)flash_write_buf, (FLASH_WRITE_BUF_LENGHT + 3) / 4);
}

/**
 * @brief          从flash读取校准数据
 * @param[in]      none
 * @retval         none
 */
static void gyro_cali_data_read(void)
{
  uint8_t flash_read_buf;
  uint16_t offset = 0;

  // read the data in flash,
  flash_read(ADDR_FLASH_SECTOR_9 + offset, (uint32_t *)&gyro_cali, sizeof(imu_cali_t));
  offset += sizeof(imu_cali_t);

  // read the name and cali flag,
  flash_read(ADDR_FLASH_SECTOR_9 + offset, (uint32_t *)&flash_read_buf, 1);

  cali_done = flash_read_buf;

  if (cali_done != CALIED_FLAG)
  {
    gyro_cali_cmd = CALI_FUNC_CMD_ON;
  }
  else
  {
    cali_gyro_hook((uint32_t *)&gyro_cali, CALI_FUNC_CMD_INIT);
  }
}

static void get_key(void)
{
  static int key_reset_time;
  static int key_set_time;
  {
    if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET)
    {
      key_set_time = 0;
      //消抖
      key_reset_time++;
      if (key_reset_time > 500)
      {
        key_reset_time = 500;
        gyro_cali_cmd = CALI_FUNC_CMD_ON;
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
      }
    }
    else if (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_SET)
    {
      key_reset_time = 0;
      //消抖
      key_set_time++;
      if (key_set_time > 50)
      {
        key_set_time = 50;
        HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
      }
    }
    else
    {
      exit_flag = 0;
    }
  }
}