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
#ifndef __ADC_H__
#define __ADC_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "state.h"
/* function declare */

static int adc_heap_init(void);
static int adc_default_config(void);
static void adc_basic_init(void);
static struct file * adc_fopen (FAR struct file *filp);
static unsigned int adc_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
static int adc_get_value(power_user_s * m_power);
static int adc_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);

#endif

/* end of file */


