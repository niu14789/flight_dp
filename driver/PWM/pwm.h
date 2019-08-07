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
	* common is TIM4 CH3 and CH4
  */
/* USER CODE END Header */
#ifndef __PWM_H__
#define __PWM_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "state.h"
/* function declare */

#define MOTOR_RADIO_MAX 2000

static int pwm_heap_init(void);
static int pwm_default_config(void);
static void pwm_basic_init(void);
static void pwm_set_one_value(unsigned short motor,unsigned short pwmvalue );
static struct file * pwm_fopen (FAR struct file *filp);
static void pwm_set_value(unsigned short * pwmvalue , unsigned int len);
static int pwm_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);

#endif

/* end of file */


