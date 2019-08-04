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

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "f_shell.h"
#include "gd32f30x.h"
#include "runtime.h"
#include "string.h"
#include "fs_config.h"
/* declares */
static int common_default_config(void);
static int common_heap_init(void);
static void system_run_thread(void);
/* fs inode system register */
FS_INODE_REGISTER("/common.o",common,common_heap_init,0);
/* system time define */
static unsigned int system_run_ms = 0;
/* regrister a system task */
FS_SHELL_STATIC(system_run_thread,system_run_thread,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD0_ID));	
/* heap init */
static int common_heap_init(void)
{
  /* full of zero */
	memset(&common,0,sizeof(common));
	/* shell base */
	common.shell_i = shell_sched_getfiles();
	/* driver config */
	common.config = common_default_config;
	/* file interface */
	common.flip.f_inode = &common;
	common.flip.f_path = "/common.o";
	/* heap */
	
	/* add your own code here */
  common.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* common_default_config led init */
static int common_default_config(void)
{
	return FS_OK;
}
/* static get system time */
static void system_run_thread(void)
{
	system_run_ms++;
}
/* common function */
/* get system time */
unsigned long long read_sys_time_us(void)
{
	return ( system_run_ms * 1000 + timer_counter_read(TIMER2) ) ;
}












