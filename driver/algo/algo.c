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
#include "string.h"
#include "fs_config.h"
#include "f_ops.h"
#include "state.h"
#include "common.h"
#include "algo.h"
/* fs inode system register */
FS_INODE_REGISTER("algo.o",algo,algo_heap_init,0);
/* regrister some system task */
FS_SHELL_STATIC(algo_task_1ms,algo_task_1ms,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD0_ID));
FS_SHELL_STATIC(algo_task_4ms,algo_task_4ms,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD1_ID));
FS_SHELL_STATIC(algo_task_10ms,algo_task_10ms,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD2_ID));
FS_SHELL_STATIC(algo_task_20ms,algo_task_20ms,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD3_ID));
FS_SHELL_STATIC(algo_task_100ms,algo_task_100ms,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD4_ID));
/* register a idle task */
FS_SHELL_STATIC(algo_task_idle,algo_task_idle,4,_CB_TIMER_|_CB_IT_IRQN_(_CB_IDLE_));
/* define a template struct to get data */
ICM206_INS_DEF  icm206_d;
GPS_User_t      gps_user;
ST480_MAG_DEF   st480_user; /* user mag data */
BARO_METER_DATA baro_user;  /* user baro data*/ 
power_user_s    bat_user;   /* user battery voltage */
rcs_user_s      sbus_user;  /* user sbus data */
/* heap init */
static int algo_heap_init(void)
{
  /* full of zero */
	memset(&algo,0,sizeof(algo));
	/* shell base */
	algo.shell_i = shell_sched_getfiles();
	/* driver config */
	algo.config = algo_default_config;
	/* file interface */
	algo.flip.f_inode = &algo;
	algo.flip.f_path = "algo.o";
	/* heap */
	
	/* add your own code here */
  algo.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* algo_default_config algo init */
static int algo_default_config(void)
{
	/* something need init . write here */
	/* add your own code here */
	
	/* end of file */
	return FS_OK;
}
/* task list */
static void algo_task_1ms(void)
{
	/* system app . read imu data */
	user_read_imu(&icm206_d);
#if 1 /* read baro at 20 ms */
	static unsigned int baro_freq = 0;
	/* read */
	if( ! ( baro_freq++ % 20 ) )
	{
	  /* read */
		if( user_read_baro(&baro_user) != FS_OK )
		{
			/* good . looks like that we've got some data */
			/* end of func */			
		}
	}
#endif	
	/* add your own code here */
	
	/* end of func */
}
/* task list */
static void algo_task_4ms(void)
{
	/* add your own code here */
	
	/* end of func */		
}
/* task list */
static void algo_task_10ms(void)
{
	/* system app . read gps data */
	if( user_read_gps(&gps_user) != FS_OK )
	{
		/* good . looks like that we've got some data */
		/* end of func */
	}
	/* add your own code here */

	/* end of func */	
}
/* task list */
static void algo_task_20ms(void)
{
#if 1//LOG_TEST
	static short log_data[128];
	/* create a log */
	log_data[0] ++;
	/* send log */
	user_update_log(log_data,sizeof(log_data));
	/*----------*/
#endif	
/* read mag dta */
#if 1 /* configable */
 /* system app . read mag data */
 if( user_read_msg(&st480_user) != FS_OK )
 {
	/* good . looks like that we've got some data */
	/* end of func */		 
 }
#endif	
#if 1 /* configable */
 if( user_read_sbus(&sbus_user) != FS_OK )
 {
	/* good . looks like that we've got some data */
	 if( sbus_user.flag == 0 )
	 {
		 /* good . remote is on line . add your own code here */
		 /* end of func */
	 }
	 else
	 {
		 /* oh on . frame lost . add some tag */
		 /* end of func */
	 }
	/* end of func */			 
 }
#endif
	/* add your own code here */
	
	/* end of func */		
}
/* task list */
static void algo_task_100ms(void)
{
	/* create a 500ms task */
	static unsigned int task_ctrl = 0;
	/* freq crrl */
	if( ! ( task_ctrl ++ % 5 ) )
	{
		algo_task_500ms();
	}
#if 1
	/* get battery voltage */
	if( user_read_battery_voltage(&bat_user) == FS_OK )
	{
		/* good . looks like that we've got some data */
		/* end of func */			
	}
	/*---------------------*/
#endif	
	/* add you own code here at 100ms */

	/* end of func */
}
static void algo_task_500ms(void)
{
	
}
/* task list */
static void algo_task_idle(void)
{
	
}
	
	
	
	
	
	
	
	
	
	
	
	
	
	


