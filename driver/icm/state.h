/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : notify.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
	* BEEP TIM3 CHANNEL1 PWM Gerente
	* LED is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/

#ifndef __STATE_H__
#define __STATE_H__

/* SBUS_CHANNEL_DEFAULT*/
#define SBUS_CHANNEL_DEFAULT  10
/* transfer to rad and deg */
#define DEG2RAD		(0.017453293f)	/* deg to rad */
#define RAD2DEG		(57.29578f)		  /* rad to deg */

/* mpu9250 ins sensor default */
typedef struct
{
	float accel[3];
	float gyro[3];
	float icm206_temperature;
}ICM206_INS_DEF;

/* mpu9250 mag def */
typedef struct
{
	float mag[3];
}ST480_MAG_DEF;

/* attitude define */
typedef struct
{
	float roll;
	float pitch;
	float yaw;
}ATTITUDE_DEF;

/* define s */
typedef struct _BARO_METER_DATA_
{
	float m_fTempature;
	float m_fPress;
	float m_fAltitude;
}BARO_METER_DATA;

/* redef */
typedef struct
{
	/* analog channel */
  unsigned short channel[SBUS_CHANNEL_DEFAULT];
  /* digital channel */
  unsigned short  flag;
  /*-------------------------*/
}rcs_user_s;

/* gps def */
typedef struct
{
	/* position */
	double		lon;			/**&lt; Longitude [deg] */
	double		lat;			/**&lt; Latitude [deg] */
	float		  height;		/**&lt; Height above mean sea level [m] */
	/* time stamp */
	unsigned int	position_timestamp;
  /* velocity */
	float		velN;		/**&lt; NED north velocity [m/s]*/
	float		velE;		/**&lt; NED east velocity [m/s]*/
	float		velU;		/**&lt; NED up velocity [m/s]*/
  /* pos dop */
	float		pDOP;		/**&lt; Position DOP [0.01] */
	/* rev */
	unsigned int rev1;
	unsigned int rev2;
	unsigned int rev3;
	unsigned int rev4;	
  /* fixtype */
	unsigned char		fixType;		/**&lt; GNSSfix type: 0 = No fix, 1 = Dead Reckoning only, 2 = 2D fix, 3 = 3d-fix, 4 = GNSS + dead reckoning, 5 = time only fix */
	unsigned char		numSV;			/**&lt; Number of SVs used in Nav Solution */
	/* end of data */
}GPS_User_t;

/* motor default define */
typedef enum
{
	MOTOR_M1    = 0,
	MOTOR_M2    = 1,
	MOTOR_M3    = 2,
	MOTOR_M4    = 3, 
	SERVO       = 4,     
}motor_type_enum;

typedef struct
{
  float bat_voltage;
}power_user_s;
/* system math def */
typedef struct
{
	ICM206_INS_DEF ins;
	GPS_User_t gps;
}state_def;

#endif
















