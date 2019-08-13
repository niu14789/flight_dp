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
	* pwm is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "f_shell.h"
#include "string.h"
#include "gd32f30x.h"
#include "pwm.h"
/* fs inode system register */
FS_INODE_REGISTER("pwm.o",pwm,pwm_heap_init,0);
/* heap init */
static int pwm_heap_init(void)
{
  /* full of zero */
	memset(&pwm,0,sizeof(pwm));
	/* shell base */
	pwm.shell_i = shell_sched_getfiles();
	/* driver config */
	pwm.config = pwm_default_config;
	/* file interface */
	pwm.flip.f_inode = &pwm;
	pwm.flip.f_path = "pwm.o";
	/* file interface */
	pwm.ops.open  = pwm_fopen;
	pwm.ops.ioctl = pwm_ioctrl;
	/* heap */
	pwm_basic_init();
	/* add your own code here */
  pwm.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* pwm_default_config pwm init */
static int pwm_default_config(void)
{
	/* nothing to do */
	return FS_OK;
}	
/* pwm hardware init */
static void pwm_basic_init(void)
{
	/* enable clock first */
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_TIMER7);
	rcu_periph_clock_enable(RCU_TIMER0);
	/*Configure PA0 PA1 PA2(TIMER7 CH0 CH1 CH2) as alternate function*/
	gpio_init(GPIOC,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_6);
	gpio_init(GPIOC,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_7);
	gpio_init(GPIOC,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_8);
	gpio_init(GPIOC,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_9);
	/* configure PB0 to TIME0 ch2 ON */
	gpio_pin_remap_config(GPIO_TIMER0_PARTIAL_REMAP,ENABLE);
	/* remap */
	gpio_init(GPIOB,GPIO_MODE_AF_PP,GPIO_OSPEED_50MHZ,GPIO_PIN_0);
	/* -----------------------------------------------------------
	 time7 init
	-------------------------------------------------------------*/
	timer_oc_parameter_struct timer_ocintpara;
	timer_parameter_struct timer_initpara;
	/* deinit TIMER7 */
	timer_deinit(TIMER7);
	timer_deinit(TIMER0);
	/* TIMER7 configuration */
	timer_initpara.prescaler         = 120 - 1 ; /* 1Mhz */
	timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
	timer_initpara.counterdirection  = TIMER_COUNTER_UP;
	timer_initpara.period            = 2500 - 1; /* 2500 us */
	timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
	timer_initpara.repetitioncounter = 0;
	timer_init(TIMER7,&timer_initpara);
	/* config time 0 */
	timer_init(TIMER0,&timer_initpara);
	/* CH0,CH1,CH2 and CH3 configuration in PWM mode */
	timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
	timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
	timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
	timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
	timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_HIGH;
	timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
	/* CH0,CH1,CH2 and CH3 configuration in PWM mode */
	timer_channel_output_config(TIMER7,TIMER_CH_0,&timer_ocintpara);
	timer_channel_output_config(TIMER7,TIMER_CH_1,&timer_ocintpara);
	timer_channel_output_config(TIMER7,TIMER_CH_2,&timer_ocintpara);
	timer_channel_output_config(TIMER7,TIMER_CH_3,&timer_ocintpara);
  /* set timer0*/
	timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
	timer_ocintpara.outputnstate = TIMER_CCXN_ENABLE;
	/* CH0 in PWM mode */
	timer_channel_output_config(TIMER0,TIMER_CH_1,&timer_ocintpara);
	/*------------------------------------------*/
	/* CH0 configuration in PWM mode0,duty cycle 40% */
	timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_0,1000 - 1 );
	timer_channel_output_mode_config(TIMER7,TIMER_CH_0,TIMER_OC_MODE_PWM0);

	/* CH1 configuration in PWM mode0,duty cycle 40% */
	timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_1,1000 - 1 );
	timer_channel_output_mode_config(TIMER7,TIMER_CH_1,TIMER_OC_MODE_PWM0);

	/* CH2 configuration in PWM mode0,duty cycle 40% */
	timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_2,1000 - 1 );
	timer_channel_output_mode_config(TIMER7,TIMER_CH_2,TIMER_OC_MODE_PWM0);

	/* CH3 configuration in PWM mode0,duty cycle 40% */
	timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_3,1000 - 1 );
	timer_channel_output_mode_config(TIMER7,TIMER_CH_3,TIMER_OC_MODE_PWM0);
	
	/* TIMER0 CH0 in PWM mode0 duty cycle 40% */
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,2500-1000);
	timer_channel_output_mode_config(TIMER0,TIMER_CH_1,TIMER_OC_MODE_PWM0);	
		
	/* TIMER7 primaryoutput function enable */
	timer_primary_output_config(TIMER7,ENABLE);
	timer_primary_output_config(TIMER0,ENABLE);
	/* auto-reload preload enable */
	timer_enable(TIMER7);	
	timer_enable(TIMER0);
	/* end of func */
}
/* file interface */
/* file & driver 's interface */
static struct file * pwm_fopen (FAR struct file *filp)
{
	return &pwm.flip;
}
/* ioctrl for control pwm mode */
static int pwm_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
			/* write */
			pwm_basic_init();
			/* end of cmd */
			break;
		case 1:
			/* pwm set value */
			pwm_set_value(pri_data,arg);
			/* end of cmd */
			break;
		case 2:
			/* pwm set one value */
			pwm_set_one_value(arg>>16,arg&0xffff);
		  /* end of cmd */
			break;
		case 3:
			/* pwm set servo */
		  pwm_set_servo(arg);
		  /* end of func */
		  break;
		default:
			break;
	}
	/* return OK */
	return ret;
}
/* static set pwm value rang 0~2000 */
static void pwm_set_value(unsigned short * pwmvalue , unsigned int len)
{
	/* length judge */
	if( len != 8 || pwmvalue == NULL )
	{
		/* bad data */
		return;
	}
	/* get pwm value */
  /* channel 1 */	
	timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_0,( pwmvalue[0] > MOTOR_RADIO_MAX ? MOTOR_RADIO_MAX : pwmvalue[0] ));
	/* channel 2 */
	timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_1,( pwmvalue[1] > MOTOR_RADIO_MAX ? MOTOR_RADIO_MAX : pwmvalue[1] ));
	/* channel 3 */
	timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_2,( pwmvalue[2] > MOTOR_RADIO_MAX ? MOTOR_RADIO_MAX : pwmvalue[2] ));
	/* channel 4 */
	timer_channel_output_pulse_value_config(TIMER7,TIMER_CH_3,( pwmvalue[3] > MOTOR_RADIO_MAX ? MOTOR_RADIO_MAX : pwmvalue[3] ));	
	/* end of func */
}
/* static set pwm value rang 0~2000 */
static void pwm_set_one_value(unsigned short motor,unsigned short pwmvalue )
{
	/* get pwm value */
	if( motor > 3 ) /* 0 , 1 , 2 , 3 */
	{
		/* bad data */
		return;
	}
  /* channel mptor */	
	timer_channel_output_pulse_value_config(TIMER7,motor,( pwmvalue > MOTOR_RADIO_MAX ? MOTOR_RADIO_MAX : pwmvalue ));
	/* end of func */
}
/* pwm set servo */
static void pwm_set_servo(unsigned short pwmvalue)
{
	/* transfer to N out */
	unsigned short pwm_rea = ( pwmvalue > MOTOR_RADIO_MAX ? MOTOR_RADIO_MAX : pwmvalue );
	/* transfer */
	pwm_rea = 5000 - pwm_rea;
  /* channel mptor */	
	timer_channel_output_pulse_value_config(TIMER0,TIMER_CH_1,pwm_rea);
	/* end of func */	
}


















