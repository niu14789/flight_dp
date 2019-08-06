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
#ifndef __ST480_H__
#define __ST480_H__
/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "state.h"
/* function declare */

#define SELF_MASTER_ADDR   (0x0D)
#define MAG_I2C_ADDRESS    (0x18)

/* timeout detecter */
#define TIMEOUT_DETECTER(a,b) { if( a ++ > 20000 ){ return b ;} }
/* function declares */

static int st480_heap_init(void);
static int st480_default_config(void);
static void st480_i2c_hd_init(void);
static int st480_i2c_wr_byte(unsigned char * w_buf, unsigned char w_cnt, 
  unsigned char *r_buf, unsigned char r_cnt);
static int st480_basic_init(void);
static int st480_read_mag( ST480_MAG_DEF * st480_d );
static int st480_i2c_read_byte( unsigned char* r_buf, 
	unsigned char r_offset, unsigned short r_cnt);
static unsigned int st480_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen);
static struct file * st480_fopen (FAR struct file *filp);
static int st480_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);

#endif

/* end of file */


