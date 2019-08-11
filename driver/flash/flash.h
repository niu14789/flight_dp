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
#ifndef __flash_H__
#define __flash_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "state.h"
/* function declare */

static int flash_heap_init(void);
static int flash_default_config(void);
static int flash_program_word(unsigned int addr,unsigned int dat);
static int flash_write(unsigned int base_addr,const void * src,unsigned int len);
static int flash_read(unsigned int base_addr,void * src,unsigned int len);
static struct file * flash_fopen (FAR struct file *filp);
static int flash_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);
static unsigned int flash_check_sum(const void * src,unsigned int len);
static int flash_erase_page(unsigned int page_addr);

#endif

/* end of file */


