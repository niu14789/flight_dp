#ifndef _ICM_20600_H_
#define _ICM_20600_H_



#include "gd32f30x.h"

#ifndef U8
#define U8 unsigned char
#endif

#ifndef U16
#define U16 unsigned short
#endif


#ifndef U32
#define U32 unsigned int
#endif

#ifndef I8
#define I8 char
#endif

#ifndef I16
#define I16 short
#endif


#ifndef I32
#define I32 int
#endif


#define  ICM20600_SLAVE_ADDR     0XD0

#define ICM20600_SAMPLE_RATE_1000HZ    0
#define ICM20600_SAMPLE_RATE_500HZ     1
#define ICM20600_SAMPLE_RATE_333HZ     2
#define ICM20600_SAMPLE_RATE_250HZ     3
#define ICM20600_SAMPLE_RATE_200HZ     4
#define ICM20600_SAMPLE_RATE_167HZ     5
#define ICM20600_SAMPLE_RATE_143HZ     6
#define ICM20600_SAMPLE_RATE_125HZ     7
#define ICM20600_SAMPLE_RATE_111HZ     8
#define ICM20600_SAMPLE_RATE_100HZ     9
#define ICM20600_SAMPLE_RATE_91HZ     10
#define ICM20600_SAMPLE_RATE_83HZ     11
#define ICM20600_SAMPLE_RATE_77HZ     12
#define ICM20600_SAMPLE_RATE_71HZ     13
#define ICM20600_SAMPLE_RATE_67HZ     14
#define ICM20600_SAMPLE_RATE_63HZ     15
#define ICM20600_SAMPLE_RATE_59HZ     16
#define ICM20600_SAMPLE_RATE_56HZ     17
#define ICM20600_SAMPLE_RATE_53HZ     18
#define ICM20600_SAMPLE_RATE_50HZ     19



//#define  ADDR_MPU6050  0XD0





#define IIC_SCL_PERIPH_CLOCK  RCU_GPIOB
#define IIC_SDA_PERIPH_CLOCK  RCU_GPIOB

#define IIC_SCL_GPIO_PORT  GPIOB
#define IIC_SCL_GPIO_PIN   GPIO_PIN_10
#define IIC_SDA_GPIO_PORT  GPIOB
#define IIC_SDA_GPIO_PIN   GPIO_PIN_11

#define IIC_PERIPH_CLOCK   RCU_I2C1
#define IMU_IIC_PORT       I2C_PORT1



#define IMU_IIC_SPEED        100000
#define MASTER_ADDR  (0X0D)





#define WHO_AM_I         0X75
#define PWR_MGMT_1       0x6b
#define SMPLRT_DIV       0x19
#define CONFIG           0x1a
#define GYRO_CONFIG     0x1b
#define ACCEL_CONFIG    0x1c
#define ACCEL_CONFIG2   0x1d 
#define ACCEL_XOUT_H    0x3b
#define TEMP_OUT_H      0x41
#define GYRO_XOUT_H     0x43


#define CALIBRATION_TIMES_MAX 2000
#define CALIBRATION_TIMES_MIN 100

// ShenZhen Gravity acceleration is 9.7925f


#define GAL_VALUE           (9.8f)    //重力加速度值
#define ACCEL_RATIO_VALUE   (8192)   //分辨率，分辨率由ACCEL_FS_SEL_DEFAULT决定，ACCEL_FS_SEL_DEFAULT=2G时分辨率为2048*8; ACCEL_FS_SEL_DEFAULT=4G时分辨率为2048*4; ACCEL_FS_SEL_DEFAULT=8G时分辨率为2048*2; ACCEL_FS_SEL_DEFAULT=16G时分辨率为2048
#define GYRO_RATIO_VALUE    (32.767f)
#define DEGTORAD   (57.29578f)  // 360/(2*pi)


#define MPU_CLK_SEL_PLLGYROX     0x01
#define MPU_CLK_SEL_PLLGYROZ     0x03
#define IMU_ENABLE_TEMPERATURE   0x00
#define IMU_DISABLE_TEMPERATURE  0x08

#define GYRO_FS_SEL_250DPS      0
#define GYRO_FS_SEL_500DPS      1
#define GYRO_FS_SEL_1000DPS     2
#define GYRO_FS_SEL_2000DPS     3
#define GYRO_FS_SEL_DEFAULT     GYRO_FS_SEL_1000DPS

#define ACCEL_FS_SEL_2G        0
#define ACCEL_FS_SEL_4G        1
#define ACCEL_FS_SEL_8G        2
#define ACCEL_FS_SEL_16G       3
#define ACCEL_FS_SEL_DEFAULT   ACCEL_FS_SEL_8G

#define OFFSENT_THRESHOLD  1.0f   //加速度和角速度校准时的误差阈值


typedef struct _IMU_DATA_
{
    float m_afAccel[3]; // x, y, z
    float m_fTempature;
    float m_afGyro[3];  // x, y, z  unit:radian
}IMU_DATA;




int ICM20600_Init(void);

int ICM20600_GetData(IMU_DATA *poImuData);


#endif










