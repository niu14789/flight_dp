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
#ifndef __bat_H__
#define __bat_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "state.h"
/* function declare */

static int bat_heap_init(void);
static int bat_default_config(void);
static unsigned int bat_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
static struct file * bat_fopen (FAR struct file *filp);
static int bat_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);
void battery_thread(void);

#endif

/* end of file */


