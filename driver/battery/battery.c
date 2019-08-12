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
#include "fs_config.h"
#include "string.h"
#include "battery.h"
#include "f_ops.h"
#include "f_drv.h"
#include "state.h"
/* file interface */
struct file * bat_usart2 = NULL;
/* bat reciver type */
/* gps define */
static unsigned char bat_dma_cache[128];
/* define the inode */
FS_INODE_REGISTER("bat.d",bat,bat_heap_init,0);
/* heap init */
static int bat_heap_init(void)
{
	 /* full of zero */
	memset(&bat,0,sizeof(bat));
	/* shell base */
	bat.shell_i = shell_sched_getfiles();
	/* driver config */
	bat.config = bat_default_config;
	/* file interface */
	bat.flip.f_inode = &bat;
	bat.flip.f_path = "bat.d";
	/* file interface */
	bat.ops.open = bat_fopen;
	bat.ops.ioctl = bat_ioctrl;
	/* heap */

	/* add your own code here */
  bat.i_flags = __FS_IS_INODE_OK|__FS_IS_INODE_INIT|__FS_IS_INODE_NODATA;
	/* ------end of file------ */
	return FS_OK;
}
/* fdg */
static int bat_default_config(void)
{
	/* open usart 3 */
	bat_usart2 = open("/UART/2",__FS_OPEN_ALWAYS);
	/* config */
	if( bat_usart2 == NULL )
	{
		/* can not find some uart */
		return FS_ERR;
	}
	/* config msg */
	uart_config_msg msg;
	/* init param */
	msg.mode = 0;//none 8 1
	msg.index = 1;
	msg.rx_dma_buffer = (unsigned int)bat_dma_cache;
	msg.rx_dma_deepth = sizeof(bat_dma_cache);
	msg.baudrate = 115200;
	msg.tx_mode = _UART_TX_DMA;
	msg.rx_mode = _UART_RX_DMA;
	/* init */
	fs_ioctl(bat_usart2,0,sizeof(msg),&msg);	
	/* create a thread here */
	/*return*/
	return FS_OK;
}
/* file interface */
/* file & driver 's interface */
static struct file * bat_fopen (FAR struct file *filp)
{
	return &bat.flip;
}
/* ioctrl for control sbus */
static int bat_ioctrl(FAR struct file *filp, int cmd, unsigned long arg,void *pri_data)
{
	/* nothing diffrent */
	int ret = FS_OK;
	/* select a cmd */
	switch(cmd)
	{
		case 0:
			/* write */
		  /* end of cmd */
	 	  break;
		case 1:
			/* decode */
		  /* end of cmd */
		  break;
		case 2:
			/* copy data */
		  /* end of cmd */
		  break;	
		default :
      break;
	}
	/* return OK */
	return ret;
}






















