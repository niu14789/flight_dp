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
#ifndef __wifi_H__
#define __wifi_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "state.h"
/* function declare */

static int wifi_heap_init(void);
static int wifi_default_config(void);
static unsigned int wifi_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
static struct file * wifi_fopen (FAR struct file *filp);
static int wifi_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);
void wifi_send_bytes(const void * data,unsigned int len);
int wifi_receive_bytes(void * data,unsigned int len);
void wifi_link_data_send(void);
void wifi_link_data_receive(void);
static void wifi_send_thread(void);
static void wifi_receive_thread(void);

#endif

/* end of file */


