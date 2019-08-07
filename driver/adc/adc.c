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
	* BEEP TIM3 CHANNEL1 adc Gerente
	* adc is TIM4 CH3 and CH4
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "fs.h"
#include "f_shell.h"
#include "string.h"
#include "gd32f30x.h"
#include "adc.h"
/* fs inode system register */
FS_INODE_REGISTER("adc.o",adc,adc_heap_init,0);
/* heap init */
static int adc_heap_init(void)
{
  /* full of zero */
	memset(&adc,0,sizeof(adc));
	/* shell base */
	adc.shell_i = shell_sched_getfiles();
	/* driver config */
	adc.config = adc_default_config;
	/* file interface */
	adc.flip.f_inode = &adc;
	adc.flip.f_path = "adc.o";
	/* file interface */
	adc.ops.open  = adc_fopen;
	adc.ops.read  = adc_fread;
	adc.ops.ioctl = adc_ioctrl;
	/* heap */
	adc_basic_init();
	/* add your own code here */
  adc.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT; 	
	/* return ok */
	return FS_OK;
}
/* adc_default_config adc init */
static int adc_default_config(void)
{
	/* nothing to do */
	return FS_OK;
}	
/* file interface */
/* file & driver 's interface */
static struct file * adc_fopen (FAR struct file *filp)
{
	return &adc.flip;
}
/* data read */
static unsigned int adc_fread(FAR struct file *filp, FAR void * buffer, unsigned int buflen)
{
	/* ignore the complier warring */
	(void)filp;
	/* read */
	if( buflen != sizeof(power_user_s) || buffer == NULL )
	{
		/* can not supply this format */
		return 0;
	}
	/* read and transfer data */
	int ret = adc_get_value(buffer);
	/* read mag data */
	return ret;
	/* end of data */
}
/* ioctrl for control adc */
static int adc_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
			/* write */
		  adc_basic_init();
		  /* end of cmd */
			break;
		case 1:
			/* write */
		  if( arg != sizeof(power_user_s) || pri_data == NULL )
			{
				return FS_ERR;/* can not supply */
			}
			/* read data */
			ret = adc_get_value(pri_data);
			/* end of data */
			break;
		case 2:
			/* read data */
			/* end of data */			
			break;
		default :
			break;
	}
	/* return ij */
	return ret;
}
/* adc hardware init */
static void adc_basic_init(void)
{
	/* enable GPIOA clock */
	rcu_periph_clock_enable(RCU_GPIOB);
	/* enable ADC1 clock */
	rcu_periph_clock_enable(RCU_ADC1);
	/* config ADC clock */
	rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
	/* config the GPIO as analog mode */
	gpio_init(GPIOB, GPIO_MODE_AIN, GPIO_OSPEED_MAX, GPIO_PIN_1);

	/* ADC continuous function enable */
	adc_special_function_config(ADC1, ADC_CONTINUOUS_MODE, ENABLE);
	adc_special_function_config(ADC1, ADC_SCAN_MODE, DISABLE);    
	/* ADC trigger config */
	adc_external_trigger_source_config(ADC1, ADC_REGULAR_CHANNEL, ADC0_1_2_EXTTRIG_REGULAR_NONE); 
	/* ADC data alignment config */
	adc_data_alignment_config(ADC1, ADC_DATAALIGN_RIGHT);
	/* ADC mode config */
	adc_mode_config(ADC_MODE_FREE); 
	/* ADC channel length config */
	adc_channel_length_config(ADC1, ADC_REGULAR_CHANNEL, 1);

	/* ADC regular channel config */
	adc_regular_channel_config(ADC1, 0, ADC_CHANNEL_9, ADC_SAMPLETIME_55POINT5);
	adc_external_trigger_config(ADC1, ADC_REGULAR_CHANNEL, ENABLE);

	/* ADC resolusion 12B */
	adc_resolution_config(ADC1, ADC_RESOLUTION_12B);  

	/* enable ADC interface */
	adc_enable(ADC1);
	
	/* ADC calibration and reset calibration */
	adc_calibration_enable(ADC1);
	/* start adc convert */
	adc_software_trigger_enable(ADC1, ADC_REGULAR_CHANNEL);
	/* clear adc flag end of conversion */
	adc_flag_clear(ADC1, ADC_FLAG_EOC);
	/* end of func */
}
/* static adc get voltage */
static int adc_get_value(power_user_s * m_power)
{
	/* some defines */
	float temp ; 
	/* detes */
	if( SET == adc_flag_get(ADC1, ADC_FLAG_EOC) )
	{   
		/* read adc value */			
		temp = adc_regular_data_read(ADC1);
		/* transfer to voltage */
    m_power->bat_voltage = (float)temp / 4096.0f * 18.70f;
		/* clear adc flag end of conversion */
		adc_flag_clear(ADC1, ADC_FLAG_EOC);
		/* return OK */
		return FS_OK;
	}
	/* return FS_ERR*/    
	return FS_ERR;
}
















