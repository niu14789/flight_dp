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

#ifndef __SBUS_H__
#define __SBUS_H__
/* interface */
#include "fs.h"
/* default value */
#define SBUS_DEFAULT_VALUE  (1500)
/* some nessciry scale */
#define SBUS_SCALE_FACTOR ((2000.0f - 1000.0f) / (1800.0f - 200.0f))
#define SBUS_SCALE_OFFSET ((unsigned int)(1000.0f - (((2000.0f - 1000.0f) / (1800.0f - 200.0f)) * 200.0f + 0.5f)))
/* end of func */
static int sbus_default_config(void);
static int sbus_heap_init(void);
static void sbus_callback(void);
static int sbus_decode(unsigned short *values, unsigned char *frame_cache);
static void sbus_reset_channels(unsigned short * rc,unsigned int len);
/* file inteface */
static struct file * sbus_fopen (FAR struct file *filp);
static unsigned int sbus_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
static int sbus_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);
/* end of file */
#endif












