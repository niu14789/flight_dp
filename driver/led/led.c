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
#include "fs.h"
#include "f_shell.h"
#include "string.h"
#include "gd32f30x.h"
/* functions declare */
static int led_heap_init(void);
static int led_default_config(void);
static void led_thread(void);
static int led_write(FAR struct file *filp, FAR const void * buffer, unsigned int buflen);
static struct file * led_fopen (FAR struct file *filp);
static int led_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data);
/* fs inode system register */
FS_INODE_REGISTER("led.o",led,led_heap_init,0);
/* as default , define a led task in 100ms */
FS_SHELL_STATIC(led_thread,led_thread,4,_CB_TIMER_|_CB_IT_IRQN_(TASK_PERIOD4_ID));
/* led hardware default */
const unsigned int leds_hd[4][3] = 
{
   { RCU_GPIOB , GPIOB , GPIO_PIN_14 },
   { RCU_GPIOB , GPIOB , GPIO_PIN_15 },
   { RCU_GPIOC , GPIOC , GPIO_PIN_1  },
   { RCU_GPIOC , GPIOC , GPIO_PIN_0  }
};
/* heap init */
static int led_heap_init(void)
{
  /* full of zero */
	memset(&led,0,sizeof(led));
	/* shell base */
	led.shell_i = shell_sched_getfiles();
	/* driver config */
	led.config = led_default_config;
	/* file interface */
	led.flip.f_inode = &led;
	led.flip.f_path = "led.o";
	/* file interface */
	led.ops.open  = led_fopen;
	led.ops.ioctl = led_ioctrl;
	led.ops.write = led_write;
	/* heap */
	led.flip.f_user0 = 5;
	/* add your own code here */
  led.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* led_default_config led init */
static int led_default_config(void)
{
	/* config GPIOS */
	for( unsigned int i = 0 ; i < sizeof(leds_hd) / sizeof(leds_hd[0]) ; i ++ )
	{
		/* enable gpio 's clock */
		rcu_periph_clock_enable((rcu_periph_enum)leds_hd[i][0]);
		/* configration gpios */
		gpio_init(leds_hd[i][1], GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, leds_hd[i][2]);
		/* open leds */
		gpio_bit_set(leds_hd[i][1], leds_hd[i][2]);
		/* end of file */
	}
	/* return a ok */
	return FS_OK;
}
/* file interface */
/* file & driver 's interface */
static struct file * led_fopen (FAR struct file *filp)
{
	return &led.flip;
}
/* file & driver 's interface write */
static int led_write(FAR struct file *filp, FAR const void * buffer, unsigned int buflen)
{
	/* led dtatus */
	filp->f_user0 = *((unsigned short *)buffer);
	/* reuturn OK */
	return FS_OK;
}
/* ioctrl for control led mode */
static int led_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
  /* nothing diffrent */
	/* led ctrl */
	static unsigned int led_ctrl = 0;	
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
			/* default config */
		break;
		case 1:
			/* close all leds */
			gpio_bit_reset(leds_hd[0][1], leds_hd[0][2]);
			gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
			gpio_bit_reset(leds_hd[2][1], leds_hd[2][2]);
			gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);		  
		break;
		case 2:
			/* open all leds */
			gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
			gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);
			gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
			gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
		break;
		case 3:
			if( (led_ctrl++ % 50) < 25 )
			{
				gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
			}
			else
			{
				gpio_bit_reset(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_reset(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);
			}			
		break;	
		default:
			break;
	}
	/* return */
	return ret;
}
/* for a task test . run as 100ms */
static void led_thread(void)
{
	/* led ctrl */
	static unsigned int led_ctrl = 0;
	/* switch cmd */
	switch(led.flip.f_user0)
	{
		case 0:
			/* default config */
		break;
		case 1:
			/* close all leds */
			gpio_bit_reset(leds_hd[0][1], leds_hd[0][2]);
			gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
			gpio_bit_reset(leds_hd[2][1], leds_hd[2][2]);
			gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);		  
		break;
		case 2:
			/* open all leds */
			gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
			gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);
			gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
			gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
		break;		
		case 3:
			/* slow flash */
			if( (led_ctrl++ % 10) < 5 )
			{
				gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
			}
			else
			{
				gpio_bit_reset(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_reset(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);
			}		
		break;
		case 4:
			/* fast flash */
			if( (led_ctrl++ % 4) < 2 )
			{
				gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
			}
			else
			{
				gpio_bit_reset(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_reset(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);
			}		
		break;
		case 5:
			/* fast flash */
			if( (led_ctrl++ % 15) < 1 )
			{
				gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
			}
			else
			{
				gpio_bit_reset(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_reset(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);
			}		
		break;			
		case 6:
			/* greens are always on . red slow */
			gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
		  gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
	    /* slow flash */
			if( (led_ctrl++ % 10) < 5 )
			{				
				gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);			
				gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
			}
			else
			{		
				gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
				gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);
			}					
		break;
		case 7:
			gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
			gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);			
			/* slow flash */
			if( (led_ctrl++ % 10) < 5 )
			{
				gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);
			}
			else
			{
				gpio_bit_reset(leds_hd[0][1], leds_hd[0][2]);
				gpio_bit_reset(leds_hd[1][1], leds_hd[1][2]);
			}				
		break;
		case 8:
			gpio_bit_set(leds_hd[0][1], leds_hd[0][2]);
			gpio_bit_set(leds_hd[1][1], leds_hd[1][2]);			
			/* slow flash */
			if( (led_ctrl++ % 10) < 5 )
			{
				gpio_bit_set(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_set(leds_hd[3][1], leds_hd[3][2]);
			}
			else
			{
				gpio_bit_reset(leds_hd[2][1], leds_hd[2][2]);
				gpio_bit_reset(leds_hd[3][1], leds_hd[3][2]);
			}					
		break;
		default:
		break;		
	}
}
















